#!/usr/bin/env python3
"""
ROS 2 node wrapper around the original combined_face_hand.py.

It subscribes to a color image topic (default: /camera/camera/color/image_raw),
runs MediaPipe pose + hand tracking, renders the animated eyes window, and
optionally relays joint angles over TCP. All runtime configuration is exposed
as ROS parameters (see config/combined_face_hand_defaults.yaml).
"""

import json
import math
import os
import random
import socket
import subprocess
import tempfile
import threading
import time
from typing import Optional

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Float32MultiArray, String
from std_srvs.srv import Trigger

from .audio_utils import chunk_level, iter_chunks, load_wav_mono, prepend_silence, write_wav_mono16
from .face_id import FaceIdentifier
from .gl_eye_renderer import GlEyeRenderer
from .mouth_gestures import gesture_duration, gesture_frame, pick_gesture
from .tts_engines import EspeakTTS, KokoroTTS
from .face_utils import box_top_center, pick_face, pose_box_from_landmarks, schedule_blink
from .hand_processing import JOINT_OPEN_OFFSETS, format_joint_command, generate_joint_offsets
from .window_utils import (
    is_global_key_pressed,
    is_hotkey_pressed,
    set_windows_window_frame_color,
    supports_global_hotkeys,
)

# ---------- MediaPipe Hands setup ----------
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose


class FaceHandNode(Node):
    def __init__(self):
        super().__init__("robot_face")

        # Declare ROS parameters (defaults mirror config/combined_face_hand_defaults.yaml)
        self.declare_parameters(
            "",
            [
                ("image_topic", "/camera/camera/color/image_raw"),
                ("face_only", False),
                ("hand_in_eyes", False),
                ("mirror", False),
                ("width", 1280),
                ("height", 720),
                ("fps", 30.0),
                ("relay_hand", False),
                ("hand_hotkey", "ctrl+shift+h"),
                ("hand_host", "192.168.1.194"),
                ("hand_port", 8765),
                ("hand_rate", 20.0),
                ("hand_speed", 1000),
                ("hand_panel_height", 400),
                ("hand_panel_fraction", 0.5),
                ("hand_panel_scale", 1.0),
                ("blink_min", 2.5),
                ("blink_max", 15.5),
                ("blink_duration", 0.18),
                ("blink_closed_hold", 0.04),
                ("gaze_speed", 0.3),
                ("gaze_idle_speed", 0.01),
                ("mouth_enabled", True),
                ("mouth_level_topic", "~/mouth_level"),
                ("mouth_audio_topic", "~/mouth_audio"),
                ("mouth_gap", 550.0),
                ("mouth_amplitude", 70.0),
                ("mouth_width", 0.0),
                ("mouth_gain", 1.0),
                ("mouth_attack", 0.6),
                ("mouth_release", 0.12),
                ("mouth_timeout", 0.4),
                ("mouth_smile", 25.0),
                ("mouth_smile_happy", 70.0),
                ("mouth_gestures", True),
                ("mouth_gesture_min_interval", 4.0),
                ("mouth_gesture_max_interval", 12.0),
                ("greeting_enabled", True),
                ("greeting_text", "hello"),
                ("greeting_known_text", "hello {name}"),
                ("greeting_late_text", "oh, hi {name}"),
                ("greeting_wait", 2.0),
                ("greeting_name_window", 10.0),
                ("greeting_happy_seconds", 30.0),
                ("speech_lead_silence", 1.5),
                ("speech_lead_level", 0.01),
                ("tts_engine", "auto"),            # auto | kokoro | coqui | espeak
                ("tts_voice", "af_heart"),         # Kokoro voice id
                ("tts_speed", 1.0),
                ("tts_lang", "auto"),              # auto = from voice prefix; or en-us, en-gb, es, fr-fr, hi, it, ja, pt-br, cmn
                ("kokoro_python", "~/.robot_face/kokoro-venv/bin/python"),
                ("kokoro_model_dir", "~/.robot_face/kokoro"),
                ("face_id_enabled", True),
                ("face_id_models_dir", ""),
                ("face_id_gallery", "~/.robot_face/people.json"),
                ("face_id_interval", 3),
                ("face_id_match_threshold", 0.363),
                ("face_id_auto_enroll", True),
                ("face_id_min_face_px", 40),
                ("face_id_unknown_threshold", 0.28),
                ("face_id_enroll_min_face_px", 64),
                ("face_id_enroll_frames", 6),
                ("save_raw_video", "/tmp/raw_camera.mp4"),
                ("save_video", "/tmp/combined_output.mp4"),
            ],
        )

        def p(name):
            return self.get_parameter(name).value

        self.bridge = CvBridge()

        # Face-only mode fully disables hand tracking and joint streaming.
        self.face_only = bool(p("face_only"))
        if self.face_only and (bool(p("relay_hand")) or bool(p("hand_in_eyes"))):
            self.get_logger().info(
                "face_only set: ignoring relay_hand / hand_in_eyes; hand tracking disabled."
            )

        # Camera/image state
        self.camera_width = int(p("width"))
        self.camera_height = int(p("height"))
        self.capture_fps = float(p("fps")) if float(p("fps")) > 0 else 30.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, p("image_topic"), self.image_callback, qos)

        # Hand relay state
        self.hand_stream_enabled = bool(p("relay_hand")) and not self.face_only
        self.hand_stream_hotkey = str(p("hand_hotkey")) or "s"
        self.global_hotkey_supported = supports_global_hotkeys()
        self.hand_stream_toggle_active = False
        self.hotkey_prev_state = False
        self.hand_target_speed = int(min(max(int(p("hand_speed")), 50), 2000))
        self.hand_socket: Optional[socket.socket] = None
        self.last_hand_send = 0.0
        self.hand_send_interval = 1.0 / max(float(p("hand_rate")), 1e-5)
        self.hand_idle_timeout = 0.75
        self.hand_last_detection_time = time.monotonic()
        self.hand_open_sent = False

        if self.hand_stream_enabled:
            if self.global_hotkey_supported:
                self.get_logger().info(
                    f"Hand relay armed. Press '{self.hand_stream_hotkey}' to toggle streaming "
                    "(works even when the window is unfocused)."
                )
            else:
                self.hand_stream_toggle_active = True
                self.get_logger().info(
                    "Hand relay armed but global hotkeys unavailable; streaming continuously."
                )

        # Eye geometry (used by GL renderer and gaze math)
        self.eye_radius = 77        # 90 × 0.85
        self.eye_white_radius = 187  # 220 × 0.85
        self.eye_top_margin = 280
        self.eye_side_margin = 80
        self.eyelash_count = 0  # mood: 0=neutral,1=tired,2=angry,3=happy,4=suspicious

        hand_panel_frac = (
            float(p("hand_panel_fraction"))
            if bool(p("hand_in_eyes")) and not self.face_only
            else 0.0
        )
        # Mouth waveform: driven by a loudness level and/or raw audio samples
        self.mouth_enabled = bool(p("mouth_enabled"))
        self.mouth_gain = float(p("mouth_gain"))
        self.mouth_attack = self.clamp_speed(float(p("mouth_attack")))
        self.mouth_release = self.clamp_speed(float(p("mouth_release")))
        self.mouth_timeout = max(float(p("mouth_timeout")), 0.0)
        self.mouth_level = 0.0          # smoothed level actually rendered
        self.mouth_target = 0.0         # most recent level from a subscriber
        self.mouth_samples: Optional[np.ndarray] = None
        self.mouth_last_input = -1e9
        mouth_width = float(p("mouth_width"))

        # Resting smile (ends curl up), wider when a person is in view
        self.mouth_smile = float(p("mouth_smile"))
        self.mouth_smile_happy = float(p("mouth_smile_happy"))
        self.mouth_smile_current = self.mouth_smile

        # Random idle gestures while nobody is around
        self.mouth_gestures_enabled = bool(p("mouth_gestures"))
        self.gesture_min_interval = max(float(p("mouth_gesture_min_interval")), 0.5)
        self.gesture_max_interval = max(float(p("mouth_gesture_max_interval")), self.gesture_min_interval)
        self.gesture_active: Optional[tuple] = None     # (name, start_mono, duration)
        self.gesture_next_time = time.monotonic() + random.uniform(
            self.gesture_min_interval, self.gesture_max_interval
        )

        self.gl_renderer = GlEyeRenderer(
            width=1080,
            height=1900,
            title="Robot Disco",
            eye_hw=float(self.eye_white_radius),
            eye_hh=float(self.eye_radius * 2),
            eye_side_margin=float(self.eye_side_margin),
            eye_top_margin=float(self.eye_top_margin),
            hand_panel_frac=hand_panel_frac,
            mouth_enabled=self.mouth_enabled,
            mouth_gap=float(p("mouth_gap")),
            mouth_hw=(mouth_width / 2.0) if mouth_width > 0 else None,
            mouth_amp=float(p("mouth_amplitude")),
        )

        if self.mouth_enabled:
            mouth_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                Float32, str(p("mouth_level_topic")), self._mouth_level_cb, mouth_qos
            )
            self.create_subscription(
                Float32MultiArray, str(p("mouth_audio_topic")), self._mouth_audio_cb, mouth_qos
            )

        # Mood services
        _MOODS = {"neutral": 0, "tired": 1, "angry": 2, "happy": 3, "suspicious": 4}
        for mood_name, mood_id in _MOODS.items():
            self.create_service(
                Trigger,
                f"~/set_mood/{mood_name}",
                lambda req, res, m=mood_id, n=mood_name: self._set_mood_cb(req, res, m, n),
            )

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        raw_video_path = p("save_raw_video")
        if raw_video_path in (None, "", "null"):
            raw_video_path = None
        self.raw_video_writer = (
            cv2.VideoWriter(
                raw_video_path, fourcc, self.capture_fps, (self.camera_width, self.camera_height)
            )
            if raw_video_path
            else None
        )
        if raw_video_path and self.raw_video_writer is not None and not self.raw_video_writer.isOpened():
            self.get_logger().warning(f"Could not open raw video writer at {raw_video_path}")
            self.raw_video_writer = None

        # TTS greeting — model initialised lazily in a thread so startup isn't
        # blocked by a model download. Speech playback drives the mouth waveform.
        self.greeting_enabled = bool(p("greeting_enabled"))
        self.greeting_text = str(p("greeting_text"))
        self.greeting_known_text = str(p("greeting_known_text"))
        self.greeting_late_text = str(p("greeting_late_text"))
        self.greeting_wait = max(float(p("greeting_wait")), 0.0)
        self.greeting_name_window = max(float(p("greeting_name_window")), 0.0)
        self.greeting_happy_seconds = float(p("greeting_happy_seconds"))
        # Silence prepended to every utterance so a sleepy Bluetooth speaker doesn't clip the start
        self.speech_lead_silence = max(float(p("speech_lead_silence")), 0.0)
        self.speech_lead_level = min(max(float(p("speech_lead_level")), 0.0), 0.2)
        self.greeting_due: Optional[float] = None       # monotonic deadline for a pending greeting
        self.generic_greeted_at: Optional[float] = None  # when we said the generic hello (awaiting a name)

        # TTS engines, best first: Kokoro-82M (own venv) → Coqui → espeak.
        self.tts_engine_pref = str(p("tts_engine")).lower()
        self._kokoro: Optional[KokoroTTS] = None
        self._tts = None                    # Coqui model, if it loads
        self._espeak = EspeakTTS()
        self._tts_lock = threading.Lock()
        if self.tts_engine_pref in ("auto", "kokoro"):
            self._kokoro = KokoroTTS(
                python=str(p("kokoro_python")),
                model_dir=str(p("kokoro_model_dir")),
                voice=str(p("tts_voice")),
                speed=float(p("tts_speed")),
                lang=str(p("tts_lang")),
                log=lambda msg: self.get_logger().warning(msg),
            )
        threading.Thread(target=self._init_tts, daemon=True).start()
        self.create_subscription(String, "~/say", self._say_cb, 10)

        # Live-tunable parameters (ros2 param set /robot_face tts_voice bm_george)
        self.add_on_set_parameters_callback(self._on_params_changed)

        # Face identification — learns to tell people apart (see face_id.py)
        self.face_id: Optional[FaceIdentifier] = None
        self.face_id_interval = max(int(p("face_id_interval")), 1)
        self.current_person = None          # latest face_id.Match for the tracked person
        self.last_person_label: Optional[str] = None
        self.last_person_time = 0.0
        self.frame_counter = 0
        if bool(p("face_id_enabled")):
            models_dir = str(p("face_id_models_dir")) or self._default_models_dir()
            try:
                self.face_id = FaceIdentifier(
                    models_dir=models_dir,
                    gallery_path=str(p("face_id_gallery")) or None,
                    match_threshold=float(p("face_id_match_threshold")),
                    auto_enroll=bool(p("face_id_auto_enroll")),
                    min_face_px=int(p("face_id_min_face_px")),
                    unknown_threshold=float(p("face_id_unknown_threshold")),
                    enroll_min_face_px=int(p("face_id_enroll_min_face_px")),
                    enroll_frames=int(p("face_id_enroll_frames")),
                )
                self.get_logger().info(
                    f"Face ID ready: {len(self.face_id.people())} known people "
                    f"(gallery {os.path.expanduser(str(p('face_id_gallery')))})"
                )
            except Exception as e:
                self.get_logger().warning(f"Face ID disabled: {e}")
        self.person_pub = self.create_publisher(String, "~/person", 10)
        self.create_subscription(String, "~/set_person_name", self._set_person_name_cb, 10)
        self.create_subscription(String, "~/forget_person", self._forget_person_cb, 10)
        self.create_service(Trigger, "~/list_people", self._list_people_cb)

        # Face tracking state
        self.tracked_face = None
        self.last_seen_time = time.time()
        self.face_stick_seconds = 1.0
        self.happy_until = 0.0          # epoch time when human-detected happy mood expires
        self.idle_scan_delay_seconds = 4.0
        self.idle_scan_period_seconds = 7.0
        self.idle_scan_horizontal_fraction = 0.33
        self.idle_scan_vertical_fraction = 0.17
        self.idle_wander_fraction = 0.06          # small secondary drift layered on the sweep
        self.smoothed_center = None

        # Blink state
        self.blink_min_interval_seconds = float(p("blink_min"))
        self.blink_max_interval_seconds = float(p("blink_max"))
        self.blink_duration_seconds = float(p("blink_duration"))
        self.blink_closed_hold = float(p("blink_closed_hold"))
        self.blink_start_time = None
        self.next_blink_time = schedule_blink(
            self.blink_min_interval_seconds, self.blink_max_interval_seconds
        )

        # Mediapipe detectors
        self.pose_detector = mp_pose.Pose(
            model_complexity=0,
            enable_segmentation=False,
            smooth_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        # Skip loading the hand model entirely in face-only mode.
        self.hands_detector = (
            None
            if self.face_only
            else mp_hands.Hands(
                max_num_hands=1,
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )
        )

        # Rendering options
        self.hand_in_eyes = bool(p("hand_in_eyes")) and not self.face_only
        self.mirror = bool(p("mirror"))
        self.hand_panel_height = int(p("hand_panel_height"))
        self.hand_panel_fraction = min(max(float(p("hand_panel_fraction")), 0.05), 0.5)
        self.hand_panel_scale = float(p("hand_panel_scale"))
        self.gaze_speed = float(p("gaze_speed"))
        self.gaze_idle_speed = float(p("gaze_idle_speed"))

    # ---------- Mood service ----------
    def _set_mood_cb(self, _req, res: Trigger.Response, mood_id: int, mood_name: str):
        self.eyelash_count = mood_id
        self.get_logger().info(f"Mood set to '{mood_name}' ({mood_id})")
        res.success = True
        res.message = mood_name
        return res

    # ---------- Face identification ----------
    @staticmethod
    def _default_models_dir() -> str:
        try:
            return os.path.join(get_package_share_directory("robot_face"), "models")
        except Exception:
            # Running from source without an install
            return os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "models"))

    def _identify_person(self, frame, hint_xy, now_mono: float):
        """Run face ID on this frame (every ``face_id_interval`` frames) and
        update ``current_person``; publishes ``~/person`` on each result."""
        if self.face_id is None or self.frame_counter % self.face_id_interval != 0:
            return
        try:
            match = self.face_id.identify(frame, near=hint_xy)
        except Exception as e:
            self.get_logger().warning(f"Face ID failed: {e}")
            return
        if match is None:
            return
        self.current_person = match
        if match.known:
            self.last_person_time = now_mono
            label = match.label
            if match.is_new:
                self.get_logger().info(
                    f"New person enrolled as {match.person_id}. Name them with: "
                    f"ros2 topic pub --once {self.get_name()}/set_person_name std_msgs/msg/String "
                    f"\"{{data: 'Their Name'}}\""
                )
            elif label != self.last_person_label:
                self.get_logger().info(f"Recognised {label} (score {match.score:.2f})")
            self.last_person_label = label
        msg = String()
        msg.data = json.dumps({
            "id": match.person_id,
            "name": match.name,
            "score": round(match.score, 3),
            "new": match.is_new,
            "box": list(match.box),
        })
        self.person_pub.publish(msg)

    def _set_person_name_cb(self, msg: String):
        """Name the person currently (or very recently) in view.
        Accepts "Name" or "person_3=Name" to target a specific id."""
        if self.face_id is None:
            self.get_logger().warning("Face ID is disabled; cannot name people.")
            return
        text = msg.data.strip()
        if "=" in text:
            target, name = (s.strip() for s in text.split("=", 1))
        else:
            name = text
            recent = self.current_person is not None and self.current_person.known \
                and (time.monotonic() - self.last_person_time) < 10.0
            if not recent:
                self.get_logger().warning(
                    "No person in view to name. Use 'person_N=Name' to name by id."
                )
                return
            target = self.current_person.person_id
        before = {pp.person_id for pp in self.face_id.people()}
        person = self.face_id.name_person(target, name)
        if person is None:
            self.get_logger().warning(f"Unknown person '{target}'.")
        else:
            merged = before - {pp.person_id for pp in self.face_id.people()}
            if merged:
                self.get_logger().info(
                    f"{', '.join(sorted(merged))} merged into {person.name} ({person.person_id}), "
                    f"now {len(person.embeddings)} views."
                )
            else:
                self.get_logger().info(f"{person.person_id} is now called '{person.name}'.")
            self.last_person_label = None   # re-announce under the new name
            self.current_person = None

    def _forget_person_cb(self, msg: String):
        if self.face_id is None:
            return
        who = msg.data.strip()
        if who.lower() == "all":
            for p in list(self.face_id.people()):
                self.face_id.forget(p.person_id)
            self.get_logger().info("Forgot everyone.")
        elif self.face_id.forget(who):
            self.get_logger().info(f"Forgot '{who}'.")
        else:
            self.get_logger().warning(f"Unknown person '{who}'.")
        self.current_person = None

    def _list_people_cb(self, _req, res: Trigger.Response):
        if self.face_id is None:
            res.success = False
            res.message = "Face ID is disabled."
            return res
        lines = self.face_id.gallery.summary()
        res.success = True
        res.message = "\n".join(lines) if lines else "No people learned yet."
        return res

    def _person_name(self) -> Optional[str]:
        m = self.current_person
        return m.name if (m is not None and m.known and m.name) else None

    def _format_greeting(self, template: str, name: Optional[str]) -> str:
        if name is None:
            return self.greeting_text
        try:
            return template.format(name=name)
        except (KeyError, IndexError):
            return template

    # ---------- Live parameter changes ----------
    def _on_params_changed(self, params):
        """Apply parameters that can change at runtime without a restart."""
        for prm in params:
            name, value = prm.name, prm.value
            try:
                if name == "tts_voice":
                    if self._kokoro is not None:
                        if self._kokoro.voices and value not in self._kokoro.voices:
                            return SetParametersResult(
                                successful=False,
                                reason=f"unknown Kokoro voice '{value}'. Known: {', '.join(self._kokoro.voices)}",
                            )
                        self._kokoro.voice = str(value)
                    self.get_logger().info(f"TTS voice set to '{value}'.")
                elif name == "tts_speed":
                    if self._kokoro is not None:
                        self._kokoro.speed = float(value)
                elif name == "tts_lang":
                    if self._kokoro is not None:
                        self._kokoro.lang = str(value)
                elif name == "greeting_text":
                    self.greeting_text = str(value)
                elif name == "greeting_known_text":
                    self.greeting_known_text = str(value)
                elif name == "greeting_late_text":
                    self.greeting_late_text = str(value)
                elif name == "greeting_enabled":
                    self.greeting_enabled = bool(value)
                elif name == "greeting_wait":
                    self.greeting_wait = max(float(value), 0.0)
                elif name == "speech_lead_silence":
                    self.speech_lead_silence = max(float(value), 0.0)
                elif name == "speech_lead_level":
                    self.speech_lead_level = min(max(float(value), 0.0), 0.2)
                elif name == "mouth_smile":
                    self.mouth_smile = float(value)
                elif name == "mouth_smile_happy":
                    self.mouth_smile_happy = float(value)
                elif name == "mouth_gestures":
                    self.mouth_gestures_enabled = bool(value)
                elif name == "mouth_gain":
                    self.mouth_gain = float(value)
                # Other parameters are read once at startup; accepting them here is
                # harmless but they only take effect after a restart.
            except (TypeError, ValueError) as e:
                return SetParametersResult(successful=False, reason=f"{name}: {e}")
        return SetParametersResult(successful=True)

    # ---------- TTS ----------
    def _init_tts(self):
        """Bring up the preferred engine in the background (model loads take a while)."""
        if self._kokoro is not None:
            if self._kokoro.start():
                self.get_logger().info(
                    f"Kokoro TTS ready in {self._kokoro.load_seconds:.1f}s "
                    f"(voice {self._kokoro.voice}, {len(self._kokoro.voices)} voices available)."
                )
                return
            self.get_logger().warning("Kokoro TTS unavailable; trying the next engine.")
            self._kokoro = None
            if self.tts_engine_pref == "kokoro":
                self.get_logger().warning("Falling back to espeak.")
                return
        if self.tts_engine_pref in ("auto", "coqui"):
            try:
                from TTS.api import TTS
                tts = TTS("tts_models/en/ljspeech/vits", progress_bar=False)
                with self._tts_lock:
                    self._tts = tts
                self.get_logger().info("Coqui TTS model ready.")
                return
            except Exception as e:
                self.get_logger().warning(f"Coqui TTS init failed ({e}).")
        self.get_logger().info("Using espeak for speech.")

    def current_tts_name(self) -> str:
        if self._kokoro is not None:
            return "kokoro"
        if self._tts is not None:
            return "coqui"
        return "espeak"

    def _synthesize(self, text: str, path: str) -> bool:
        """Write ``text`` as a WAV to ``path`` with the best engine available."""
        if self._kokoro is not None and self._kokoro.synthesize(text, path):
            return True
        with self._tts_lock:
            tts = self._tts
        if tts is not None:
            try:
                tts.tts_to_file(text=text, file_path=path)
                return True
            except Exception as e:
                self.get_logger().warning(f"Coqui synthesis failed: {e}")
        return self._espeak.synthesize(text, path)

    def _say_cb(self, msg: String):
        """Speak arbitrary text published on ~/say."""
        text = msg.data.strip()
        if text:
            self._speak(text)

    def _speak(self, text: str):
        """Synthesise ``text`` to a WAV and play it, feeding the samples to the
        mouth waveform in sync."""
        def _run():
            path = None
            try:
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                    path = f.name
                if not self._synthesize(text, path):
                    self.get_logger().warning(f"No TTS engine could say: {text!r}")
                    return
                self._play_wav_with_mouth(path)
            except Exception as e:
                self.get_logger().warning(f"TTS speak failed: {e}")
            finally:
                if path:
                    try:
                        os.remove(path)
                    except OSError:
                        pass
        threading.Thread(target=_run, daemon=True).start()

    def _play_wav_with_mouth(self, path: str):
        """Play a WAV through aplay while streaming its samples to the mouth so
        the waveform tracks the speech actually coming out of the speaker."""
        audio = None
        rate = 0
        try:
            audio, rate = load_wav_mono(path)
        except Exception as e:
            self.get_logger().warning(f"Could not read {path} for mouth sync: {e}")

        if audio is not None and self.speech_lead_silence > 0.0:
            # Rewrite the clip with leading silence; the mouth feed below uses the
            # same padded audio, so it stays still until the speech actually starts.
            audio = prepend_silence(audio, rate, self.speech_lead_silence, level=self.speech_lead_level)
            try:
                write_wav_mono16(path, audio, rate)
            except Exception as e:
                self.get_logger().warning(f"Could not add lead silence: {e}")

        proc = subprocess.Popen(
            ["aplay", "-q", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        if audio is None or not self.mouth_enabled:
            proc.wait()
            return

        # Normalise the clip so the mouth moves the same regardless of TTS volume
        peak = float(np.abs(audio).max()) if audio.size else 0.0
        if peak > 1e-3:
            audio = audio / peak

        t0 = time.monotonic()
        for chunk, end_t in iter_chunks(audio, rate, 0.04):   # 40 ms ≈ one display frame
            self._feed_mouth(chunk)
            delay = t0 + end_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if proc.poll() is not None and end_t < audio.size / rate - 0.1:
                break   # aplay exited early; stop animating
        proc.wait()

    # ---------- Mouth waveform ----------
    def _feed_mouth(self, samples):
        """Common entry point for an audio chunk (ROS topic or local playback)."""
        data, level = chunk_level(samples, self.mouth_gain)
        if data.size == 0:
            return
        self.mouth_target = level
        self.mouth_samples = data
        self.mouth_last_input = time.monotonic()

    def _mouth_level_cb(self, msg: Float32):
        """Scalar loudness in [0, 1] (e.g. RMS of the speech being played)."""
        self.mouth_target = min(max(float(msg.data) * self.mouth_gain, 0.0), 1.0)
        self.mouth_last_input = time.monotonic()

    def _mouth_audio_cb(self, msg: Float32MultiArray):
        """Raw audio chunk, samples in [-1, 1]; drawn as the waveform itself."""
        self._feed_mouth(np.asarray(msg.data, dtype=np.float32))

    _NO_GESTURE = (0.0, 0.0, 0.0, None, 0.0)

    def _idle_gesture(self, now_mono: float, idle: bool):
        """Random mouth gestures (see mouth_gestures.py) while nobody is in view
        and the robot is quiet. Returns (level, smile_px, skew_px, points, blend)."""
        if not self.mouth_gestures_enabled:
            return self._NO_GESTURE
        if not idle:
            if self.gesture_active is not None:         # someone showed up — stop
                self.gesture_active = None
                self.gesture_next_time = now_mono + random.uniform(
                    self.gesture_min_interval, self.gesture_max_interval
                )
            return self._NO_GESTURE
        if self.gesture_active is None:
            if now_mono < self.gesture_next_time:
                return self._NO_GESTURE
            name, side = pick_gesture()
            self.gesture_active = (name, now_mono, gesture_duration(name), side)

        name, start, duration, side = self.gesture_active
        t = (now_mono - start) / duration
        if t >= 1.0:
            self.gesture_active = None
            self.gesture_next_time = now_mono + random.uniform(
                self.gesture_min_interval, self.gesture_max_interval
            )
            return self._NO_GESTURE
        return gesture_frame(name, t, now_mono, side)

    def _update_mouth(self, now_mono: float, seeing_person: bool):
        """Advance mouth state; returns
        (level, samples_or_None, smile_px, skew_px, shape_points_or_None, shape_blend)."""
        if not self.mouth_enabled:
            return 0.0, None, 0.0, 0.0, None, 0.0

        speaking = (now_mono - self.mouth_last_input) <= self.mouth_timeout
        if not speaking:
            self.mouth_target = 0.0
            self.mouth_samples = None

        # Smile wider when someone is in view
        smile_goal = self.mouth_smile_happy if seeing_person else self.mouth_smile
        self.mouth_smile_current += (smile_goal - self.mouth_smile_current) * 0.08

        g_level, g_smile, g_skew, g_points, g_blend = self._idle_gesture(
            now_mono, idle=(not seeing_person and not speaking)
        )

        target = max(self.mouth_target, g_level)
        k = self.mouth_attack if target > self.mouth_level else self.mouth_release
        self.mouth_level += (target - self.mouth_level) * k
        if self.mouth_level < 1e-3:
            self.mouth_level = 0.0
        return (
            self.mouth_level,
            self.mouth_samples,
            self.mouth_smile_current + g_smile,
            g_skew,
            g_points,
            g_blend,
        )

    # ---------- Utility ----------
    def clamp_speed(self, val):
        return min(max(val, 0.01), 1.0)

    # ---------- Image callback ----------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"CvBridge failed: {exc}")
            return

        self.camera_height, self.camera_width = frame.shape[:2]
        if self.raw_video_writer is not None:
            self.raw_video_writer.write(frame)

        current_time = time.time()
        current_mono = time.monotonic()

        # ---- Person detection for eyes ----
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        pose_results = self.pose_detector.process(rgb_frame)
        rgb_frame.flags.writeable = True
        faces = []
        if pose_results and pose_results.pose_landmarks:
            pose_box = pose_box_from_landmarks(
                pose_results.pose_landmarks, self.camera_width, self.camera_height
            )
            if pose_box is not None:
                faces.append(pose_box)

        selected_face = pick_face(faces, self.tracked_face)
        self.frame_counter += 1
        if selected_face is not None:
            if self.tracked_face is None:
                # New person detected — look happy, then greet once we know who it is
                self.happy_until = current_time + self.greeting_happy_seconds
                self.current_person = None
                self.generic_greeted_at = None
                if self.greeting_enabled:
                    self.greeting_due = current_mono + (self.greeting_wait if self.face_id else 0.0)
            self.tracked_face = selected_face
            self.last_seen_time = current_time
            self._identify_person(frame, box_top_center(selected_face), current_mono)
        elif current_time - self.last_seen_time > self.face_stick_seconds:
            if self.tracked_face is not None:       # face just lost — cancel happy
                self.happy_until = 0.0
            self.tracked_face = None
            self.current_person = None
            self.greeting_due = None
            self.generic_greeted_at = None

        # Greeting, in two stages:
        #  1. As soon as the person is recognised by name -> "hello <name>".
        #     If the wait runs out first (face too small / not yet learned) -> generic "hello".
        #  2. If a name arrives within greeting_name_window after the generic hello,
        #     follow up with "oh, hi <name>" so known people still hear their name.
        name = self._person_name()
        if self.greeting_due is not None:
            if name is not None:
                self._speak(self._format_greeting(self.greeting_known_text, name))
                self.greeting_due = None
            elif current_mono >= self.greeting_due:
                self._speak(self.greeting_text)
                self.greeting_due = None
                self.generic_greeted_at = current_mono
                if self.face_id is not None:
                    self.get_logger().info(
                        f"Greeted generically: no face identified within {self.greeting_wait:.1f}s "
                        "(will add the name if recognised shortly)."
                    )
        elif self.generic_greeted_at is not None:
            if name is not None:
                self._speak(self._format_greeting(self.greeting_late_text, name))
                self.generic_greeted_at = None
            elif current_mono - self.generic_greeted_at > self.greeting_name_window:
                self.generic_greeted_at = None

        time_since_face = current_time - self.last_seen_time
        if self.tracked_face is not None:
            center = box_top_center(self.tracked_face)
        elif time_since_face > self.idle_scan_delay_seconds:
            scan_elapsed = time_since_face - self.idle_scan_delay_seconds
            sweep_angle = (2 * math.pi / self.idle_scan_period_seconds) * scan_elapsed
            # Slow figure-of-eight sweep plus a faster, smaller wander so the
            # eyes don't look like they're on rails
            wander_x = math.sin(scan_elapsed * 1.9 + 0.7) * self.camera_width * self.idle_wander_fraction
            wander_y = math.sin(scan_elapsed * 2.6) * self.camera_height * self.idle_wander_fraction
            center = (
                self.camera_width / 2.0
                + math.sin(sweep_angle) * self.camera_width * self.idle_scan_horizontal_fraction
                + wander_x,
                self.camera_height / 2.0
                + math.cos(sweep_angle * 0.7) * self.camera_height * self.idle_scan_vertical_fraction
                + wander_y,
            )
        else:
            center = (self.camera_width / 2.0, self.camera_height / 2.0)

        if self.smoothed_center is None:
            self.smoothed_center = center
        else:
            smoothing_factor_active = self.clamp_speed(self.gaze_speed)
            smoothing_factor_idle = self.clamp_speed(self.gaze_idle_speed)
            current_smoothing = smoothing_factor_active if self.tracked_face is not None else smoothing_factor_idle
            self.smoothed_center = (
                self.smoothed_center[0] + (center[0] - self.smoothed_center[0]) * current_smoothing,
                self.smoothed_center[1] + (center[1] - self.smoothed_center[1]) * current_smoothing,
            )

        if self.blink_start_time is None and current_mono >= self.next_blink_time:
            self.blink_start_time = current_mono

        if self.blink_start_time is not None:
            elapsed = current_mono - self.blink_start_time
            closing = self.blink_duration_seconds / 2.0
            opening = closing
            hold = max(self.blink_closed_hold, 0.0)

            total = closing + hold + opening
            if elapsed >= total:
                self.blink_start_time = None
                self.next_blink_time = schedule_blink(
                    self.blink_min_interval_seconds, self.blink_max_interval_seconds
                )
                blink_amount = 0.0
            else:
                if elapsed <= closing:
                    blink_amount = min(1.0, elapsed / closing)
                elif elapsed <= closing + hold:
                    blink_amount = 1.0
                else:
                    elapsed_open = elapsed - closing - hold
                    blink_amount = max(0.0, 1.0 - (elapsed_open / opening))
        else:
            blink_amount = 0.0

        # Gaze mapping normalized by camera dims
        norm_x = (self.smoothed_center[0] - self.camera_width / 2.0) / (self.camera_width / 2.0)
        norm_y = (self.smoothed_center[1] - self.camera_height / 2.0) / (self.camera_height / 2.0)
        norm_x = max(min(norm_x, 1.0), -1.0)
        norm_y = max(min(norm_y, 1.0), -1.0)

        # Scale by usable pupil travel within the sclera
        max_travel = max(self.eye_white_radius - self.eye_radius, 0)
        gaze_gain_x = 0.7
        gaze_gain_y = 0.6
        offsetX = -norm_x * max_travel * gaze_gain_x  # mirror horizontally
        offsetY = norm_y * max_travel * gaze_gain_y

        # Determine if we need hands processing (for in-eyes overlay or hotkey-triggered streaming)
        if self.hand_stream_enabled and self.global_hotkey_supported:
            # Support multi-key chords like "ctrl+shift+h" while keeping single-key hotkey compatibility
            if "+" in self.hand_stream_hotkey:
                hotkey_pressed = is_hotkey_pressed(self.hand_stream_hotkey)
            else:
                hotkey_pressed = is_global_key_pressed(self.hand_stream_hotkey)
            if hotkey_pressed and not self.hotkey_prev_state:
                self.hand_stream_toggle_active = not self.hand_stream_toggle_active
                state_str = "enabled" if self.hand_stream_toggle_active else "paused"
                self.get_logger().info(f"Hand relay {state_str} via '{self.hand_stream_hotkey}' toggle.")
            self.hotkey_prev_state = hotkey_pressed
        elif self.hand_stream_enabled and not self.global_hotkey_supported:
            self.hand_stream_toggle_active = True

        hand_stream_active = self.hand_stream_enabled and self.hand_stream_toggle_active
        need_hands = self.hand_in_eyes or hand_stream_active
        results = None
        if need_hands:
            rgb_frame.flags.writeable = False
            results = self.hands_detector.process(rgb_frame)
            rgb_frame.flags.writeable = True

        if hand_stream_active:
            hand_payload = None
            if results and results.multi_hand_landmarks:
                joint_offsets = generate_joint_offsets(results.multi_hand_landmarks[0])
                hand_payload = format_joint_command(joint_offsets, self.hand_target_speed)
                self.hand_last_detection_time = current_mono
                self.hand_open_sent = False
            elif (current_mono - self.hand_last_detection_time) > self.hand_idle_timeout and not self.hand_open_sent:
                hand_payload = format_joint_command(JOINT_OPEN_OFFSETS, self.hand_target_speed)
                self.hand_open_sent = True

            if hand_payload is not None and (current_mono - self.last_hand_send) >= self.hand_send_interval:
                if self.hand_socket is None:
                    try:
                        self.hand_socket = socket.create_connection(
                            (self.get_parameter("hand_host").value, int(self.get_parameter("hand_port").value)),
                            timeout=0.5,
                        )
                        self.hand_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    except OSError:
                        self.hand_socket = None
                if self.hand_socket is not None:
                    try:
                        self.hand_socket.sendall(hand_payload.encode("ascii"))
                        self.last_hand_send = current_mono
                    except OSError:
                        self.hand_socket.close()
                        self.hand_socket = None

        # Hand panel overlay
        if self.hand_in_eyes:
            if results is not None and results.multi_hand_landmarks:
                pw, ph = self.gl_renderer._panel_size
                panel_img = np.zeros((ph, pw, 3), dtype=np.uint8)
                mp_drawing.draw_landmarks(
                    panel_img,
                    results.multi_hand_landmarks[0],
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style(),
                )
                if self.mirror:
                    panel_img = cv2.flip(panel_img, 1)
                self.gl_renderer.update_hand_panel(panel_img)
            else:
                self.gl_renderer.update_hand_panel(None)

        # Render eyes + mouth waveform via GL shader
        (mouth_level, mouth_samples, mouth_smile, mouth_skew,
         mouth_path, mouth_shape) = self._update_mouth(
            current_mono, seeing_person=self.tracked_face is not None
        )
        self.gl_renderer.render(
            blink=blink_amount,
            gaze_offset=(offsetX, offsetY),
            mood=3 if current_time < self.happy_until else self.eyelash_count,
            dt=1.0 / self.capture_fps,
            mouth_level=mouth_level,
            mouth_samples=mouth_samples,
            mouth_smile=mouth_smile,
            mouth_skew=mouth_skew,
            mouth_shape=mouth_shape,
            mouth_path=mouth_path,
        )
        if self.gl_renderer.should_close():
            self.get_logger().info("Window closed; shutting down.")
            rclpy.shutdown()

    # ---------- Cleanup ----------
    def destroy_node(self):
        if self._kokoro is not None:
            self._kokoro.stop()
        if self.hand_socket is not None:
            self.hand_socket.close()
        if self.raw_video_writer is not None:
            self.raw_video_writer.release()
        self.gl_renderer.destroy()
        self.pose_detector.close()
        if self.hands_detector is not None:
            self.hands_detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceHandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
