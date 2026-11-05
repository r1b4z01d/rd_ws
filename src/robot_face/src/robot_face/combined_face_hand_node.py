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
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from std_srvs.srv import Trigger

from .audio_sink import SinkWatchdog, player_env
from .audio_utils import chunk_level, iter_chunks, load_wav_mono, prepend_silence, write_wav_mono16
from .face_id import FaceIdentifier
from .people_web import PeopleWebServer
from .gl_eye_renderer import GlEyeRenderer
from .mouth_gestures import gesture_duration, gesture_frame, pick_gesture
from .tts_engines import EspeakTTS, KokoroTTS
from .ur_dashboard import UrDashboard
from .ur_motion import UrMotion, parse_order
from .hand_wiggle import HandWiggle
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
                ("fullscreen", True),              # start fullscreen; a click or F toggles
                ("window_focus_seconds", 10.0),    # keep asking GNOME to focus the window this long after start
                ("render_fps", 30.0),              # face animation rate, independent of the camera
                ("camera_timeout", 3.0),           # seconds without frames before the face stops expecting a person
                ("face_only", False),
                ("hand_in_eyes", False),
                ("mirror", False),
                ("width", 1280),
                ("height", 720),
                ("fps", 30.0),
                ("relay_hand", False),
                ("hand_hotkey", "ctrl+shift+h"),
                ("hand_relay_host", "192.168.11.117"),   # the hand's ESP32, TCP line protocol (:8765): the hand-tracking relay only
                ("hand_relay_port", 8765),                #   (hand_host / hand_port / hand_speed below are the wave's UDP wiggle)
                ("hand_rate", 20.0),
                ("hand_relay_speed", 1000),
                ("hand_panel_height", 400),
                ("hand_panel_fraction", 0.5),
                ("hand_panel_scale", 1.0),
                ("blink_min", 2.5),
                ("blink_max", 15.5),
                ("blink_duration", 0.18),
                ("blink_closed_hold", 0.04),
                ("wink_eye", "right"),             # which eye winks (screen left / right)
                ("wink_duration", 0.7),            # whole wink: close, hold, open (s)
                ("wink_depth", 0.9),               # how far the eye closes: 1.0 shut, 0.9 leaves a sliver open
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
                ("greeting_cooldown", 3600.0),     # seconds before the same recognised person is greeted again (0 = every visit)
                ("ur_wave_enabled", True),         # play a pendant program on the UR arm when someone is recognised
                ("ur_ip", "192.168.11.21"),
                ("ur_wave_program", "wave.urp"),   # program saved on the teach pendant (/programs)
                ("ur_wave_cooldown", 20.0),        # seconds between waves
                ("ur_wave_known_only", True),      # only for people recognised by name (false: every hello)
                ("ur_wave_source", "auto"),        # auto (recording if present, else pendant program) | recording | dashboard
                ("ur_wave_motion_file", "~/.robot_face/motions/wave.json"),   # from scripts/ur_record_motion.py
                ("ur_wave_return", "home"),        # after the wave: home (VLA's home pose) | start (where it was) | none
                ("ur_wave_home_file", "~/.robot_face/motions/home.json,~/rd_ws/VLA/outputs/home_pose.json"),   # first existing wins: the wave's forward home (ur_record_motion.py --capture-home), else the eval's
                ("ur_wave_approach_order", ""),           # joint stages home -> wave pose (scripts/ur_path_eval.py); "" = all at once
                ("ur_wave_return_order", ""),             # joint stages wave pose -> home
                ("ur_wave_retrace", True),                # a take that starts at home but ends elsewhere returns along its own samples, backwards
                ("ur_wave_retrace_speedup", 1.5),         # playback speed of that retrace (1 = as demonstrated)
                ("hand_wiggle_enabled", True),            # AmazingHand: ripple the fingers while the arm waves (thumb still)
                ("hand_host", "192.168.11.117"),          # the hand's ESP32 on the robot WiFi (UDP line protocol)
                ("hand_port", 8766),
                ("hand_speed", 500),                      # firmware J: speed (50..2000)
                ("hand_wiggle_hz", 0.5),                  # finger ripples per second
                ("hand_wiggle_curl", 0.333),              # how far the fingers close (0 open .. 1 closed): 2/3 open
                ("hand_thumb_curl", 0.0),                 # the thumb's fixed curl during the wave
                ("hand_wiggle_phase", 0.25),              # fraction of a period between neighbouring fingers (the wave runs index -> ring)
                ("ur_wave_power_up", True),        # IDLE / protective stop: run the eval's arm startup first (power on, brake release)
                ("audio_sink", "C-Media"),         # speaker: PulseAudio sink name or fragment; kept as the default sink while present ("" = leave the default alone)
                ("audio_sink_check", 5.0),         # seconds between default-sink checks
                ("speech_lead_silence", 1.5),
                ("speech_lead_level", 0.01),
                ("tts_engine", "auto"),            # auto | kokoro | coqui | espeak
                ("tts_voice", "af_heart"),         # Kokoro voice id
                ("tts_speed", 1.0),
                ("tts_lang", "auto"),              # auto = from voice prefix; or en-us, en-gb, es, fr-fr, hi, it, ja, pt-br, cmn
                ("kokoro_python", "~/.robot_face/kokoro-venv/bin/python"),
                ("kokoro_model_dir", "~/.robot_face/kokoro"),
                ("tracker", "auto"),               # auto | face (SCRFD detector on every frame, GPU) | pose (MediaPipe)
                ("face_id_enabled", True),
                ("face_id_backend", "auto"),       # auto | arcface (SCRFD+ArcFace, own venv) | opencv (YuNet+SFace)
                ("face_id_python", "~/.robot_face/faceid-venv/bin/python"),
                ("face_id_arcface_model", "buffalo_l"),
                ("face_id_det_size", 640),         # SCRFD input size (full frames on the GPU; 320 for a CPU head-crop)
                ("face_id_gpu", True),             # CUDAExecutionProvider for the worker when available
                ("face_id_models_dir", ""),        # OpenCV back-end models ("" = share/robot_face/models)
                ("face_id_gallery", "~/.robot_face/people.json"),   # ArcFace uses people.arcface.json
                ("face_thumb_dir", "~/.robot_face/faces"),       # one face picture per person (for the Folks page)
                ("people_web_enabled", True),      # serve the Folks page (known people, editable names)
                ("people_web_port", 8096),
                ("face_id_interval", 3),
                ("face_id_min_period", 0.5),       # seconds between identification (embedding) passes
                ("face_id_threads", 2),            # onnxruntime threads for the ArcFace worker (it spins otherwise)
                ("face_id_match_threshold", 0.0),  # 0 = back-end default (SFace 0.363, ArcFace 0.45)
                ("face_id_auto_enroll", True),
                ("face_id_min_face_px", 40),
                ("face_id_unknown_threshold", 0.0),  # 0 = back-end default (SFace 0.28, ArcFace 0.30)
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
        self._image_topic = str(p("image_topic"))
        img_type = CompressedImage if self._image_topic.rstrip("/").endswith("compressed") else Image
        self.create_subscription(img_type, self._image_topic, self.image_callback, qos)
        self._last_frame_time = 0.0
        self._camera_warned = False
        self._start_mono = time.monotonic()

        # Hand relay state
        self.hand_stream_enabled = bool(p("relay_hand")) and not self.face_only
        self.hand_stream_hotkey = str(p("hand_hotkey")) or "s"
        self.global_hotkey_supported = supports_global_hotkeys()
        self.hand_stream_toggle_active = False
        self.hotkey_prev_state = False
        self.hand_target_speed = int(min(max(int(p("hand_relay_speed")), 50), 2000))
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
            fullscreen=bool(p("fullscreen")),
            grab_focus_seconds=float(p("window_focus_seconds")),
            log=self.get_logger(),
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
        self.greeting_cooldown = max(float(p("greeting_cooldown")), 0.0)
        self.wink_eye_index = 0 if str(p("wink_eye")).strip().lower().startswith("l") else 1
        self.wink_duration = max(float(p("wink_duration")), 0.2)
        self.wink_depth = max(0.0, min(float(p("wink_depth")), 1.0))
        self.wink_start_time: Optional[float] = None
        self._cooldown_logged_for: Optional[str] = None   # person id whose skipped hello was logged this visit
        self.ur_wave_enabled = bool(p("ur_wave_enabled"))
        self.ur_wave_program = str(p("ur_wave_program"))
        self.ur_wave_cooldown = max(float(p("ur_wave_cooldown")), 0.0)
        self.ur_wave_known_only = bool(p("ur_wave_known_only"))
        self.ur_wave_source = str(p("ur_wave_source"))
        self.ur_wave_motion_file = str(p("ur_wave_motion_file"))
        self.ur = UrDashboard(str(p("ur_ip")), log=lambda m: self.get_logger().info(m))
        self.ur_wave_power_up = bool(p("ur_wave_power_up"))
        self.audio_sink = str(p("audio_sink"))
        self.sink_watchdog = SinkWatchdog(self.audio_sink, float(p("audio_sink_check")),
                                          log=lambda m: self.get_logger().info(m))
        self.sink_watchdog.start()
        self.ur_motion = UrMotion(str(p("ur_ip")), log=lambda m: self.get_logger().info(m),
                                  return_to=str(p("ur_wave_return")), home_file=str(p("ur_wave_home_file")),
                                  approach_order=str(p("ur_wave_approach_order")), return_order=str(p("ur_wave_return_order")),
                                  retrace=bool(p("ur_wave_retrace")), retrace_speedup=float(p("ur_wave_retrace_speedup")),
                                  auto_power_up=self.ur_wave_power_up)
        self._last_wave = 0.0
        self.hand_wiggle_enabled = bool(p("hand_wiggle_enabled"))
        self.hand_wiggle = HandWiggle(str(p("hand_host")), int(p("hand_port")), speed=int(p("hand_speed")),
                                      hz=float(p("hand_wiggle_hz")), curl=float(p("hand_wiggle_curl")),
                                      thumb=float(p("hand_thumb_curl")), phase=float(p("hand_wiggle_phase")),
                                      log=lambda m: self.get_logger().info(m))
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
        # True while the robot's own voice is playing (the voice assistant mutes its microphone)
        self.speaking_pub = self.create_publisher(Bool, "~/speaking", 10)

        # Live-tunable parameters (ros2 param set /robot_face tts_voice bm_george)
        self._pending_fullscreen: Optional[bool] = None
        self.add_on_set_parameters_callback(self._on_params_changed)

        # The face animates on its own clock; camera frames only steer it
        self.render_dt = 1.0 / max(float(p("render_fps")), 1.0)
        self.camera_timeout = max(float(p("camera_timeout")), 0.5)
        self.render_timer = self.create_timer(self.render_dt, self._render_tick)

        # Face identification — learns to tell people apart (see face_id.py)
        self.face_id: Optional[FaceIdentifier] = None
        self.face_id_interval = max(int(p("face_id_interval")), 1)
        self.face_id_min_period = max(float(p("face_id_min_period")), 0.0)
        self._face_id_last_submit = 0.0
        self.current_person = None          # latest face_id.Match for the tracked person
        self.last_person_label: Optional[str] = None
        self.last_person_time = 0.0
        self.frame_counter = 0
        # Identification runs in its own thread: the ArcFace back-end costs a few
        # hundred ms per pass on the Orin CPU and must not stall the face animation.
        self._face_id_lock = threading.Lock()
        self._face_id_job: Optional[tuple] = None          # (frame, hint_xy, t_mono) awaiting the worker
        self._face_id_busy = False
        self._face_id_result: Optional[tuple] = None        # (match, t_mono) produced by the worker
        self._face_id_thread: Optional[threading.Thread] = None
        if bool(p("face_id_enabled")):
            models_dir = str(p("face_id_models_dir")) or self._default_models_dir()
            try:
                self.face_id = FaceIdentifier.create(
                    str(p("face_id_backend")),
                    opencv_models_dir=models_dir,
                    gallery_path=str(p("face_id_gallery")) or None,
                    faceid_python=str(p("face_id_python")),
                    arcface_model=str(p("face_id_arcface_model")),
                    det_size=int(p("face_id_det_size")),
                    gpu=bool(p("face_id_gpu")),
                    threads=int(p("face_id_threads")),
                    log=lambda msg: self.get_logger().warning(msg),
                    match_threshold=float(p("face_id_match_threshold")) or None,
                    auto_enroll=bool(p("face_id_auto_enroll")),
                    min_face_px=int(p("face_id_min_face_px")),
                    unknown_threshold=float(p("face_id_unknown_threshold")) or None,
                    enroll_min_face_px=int(p("face_id_enroll_min_face_px")),
                    enroll_frames=int(p("face_id_enroll_frames")),
                    thumb_dir=str(p("face_thumb_dir")) or None,
                )
                g = self.face_id.gallery
                self.get_logger().info(
                    f"Face ID ready: {self.face_id.backend.info}; {len(self.face_id.people())} known people "
                    f"(gallery {g.path}, match ≥ {self.face_id.match_threshold:.2f})"
                )
                if g.dropped_on_load:
                    self.get_logger().warning(
                        f"Ignored {g.dropped_on_load} stored face views from a different embedding model."
                    )
            except Exception as e:
                self.get_logger().warning(f"Face ID disabled: {e}")

        # Person tracking: the face detector on every frame (needs the ArcFace worker,
        # cheap on the GPU, and immune to MediaPipe's phantom people in furniture) or
        # MediaPipe pose as before.
        tracker = str(p("tracker")).lower()
        self.face_tracker = (
            self.face_id is not None and self.face_id.backend.name == "arcface"
            and tracker in ("auto", "face")
        )
        if tracker == "face" and not self.face_tracker:
            self.get_logger().warning("tracker=face needs the ArcFace back-end; using MediaPipe pose.")
        self._vision_result: Optional[tuple] = None     # (faces, match, t_mono) from the vision thread
        self._track_hint: Optional[tuple] = None
        if self.face_id is not None:
            target = self._vision_loop if self.face_tracker else self._face_id_loop
            self._face_id_thread = threading.Thread(target=target, daemon=True)
            self._face_id_thread.start()
        self.get_logger().info(
            "Tracking people with " + ("the face detector (SCRFD) on every frame" if self.face_tracker else "MediaPipe pose")
        )

        # "Folks" web page: the people the robot knows, with editable names (embedded by the VLA manager)
        self.people_web: Optional[PeopleWebServer] = None
        if self.face_id is not None and bool(p("people_web_enabled")):
            self.people_web = PeopleWebServer(
                self.face_id, port=int(p("people_web_port")),
                live=lambda: self.current_person,
                on_change=self._on_people_changed,
                log=lambda m: self.get_logger().warning(m),
                cooldown=lambda: self.greeting_cooldown,
                sink=lambda: self.audio_sink,          # the manager's volume slider acts on the speaker sink
            )
            if self.people_web.start():
                self.get_logger().info(f"Folks page on http://0.0.0.0:{int(p('people_web_port'))}/")
            else:
                self.people_web = None
        self.person_pub = self.create_publisher(String, "~/person", 10)
        self.create_subscription(String, "~/set_person_name", self._set_person_name_cb, 10)
        self.create_service(Trigger, "~/wave", self._wave_srv)    # play the UR wave program now
        self.create_service(Trigger, "~/wink", self._wink_srv)    # wink one eye now
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

        # Mediapipe detectors (pose only when it is the tracker)
        self.pose_detector = None
        if not self.face_tracker:
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
        """Hand this frame to the face-ID thread (every ``face_id_interval``
        frames, and only if it is idle), then apply any finished result:
        update ``current_person`` and publish ``~/person``."""
        if self.face_id is None:
            return
        with self._face_id_lock:
            if (self.frame_counter % self.face_id_interval == 0 and not self._face_id_busy
                    and now_mono - self._face_id_last_submit >= self.face_id_min_period):
                crop, (ox, oy) = self._head_crop(frame, self.tracked_face)
                self._face_id_job = (crop, (hint_xy[0] - ox, hint_xy[1] - oy), (ox, oy), now_mono)
                self._face_id_last_submit = now_mono
            result, self._face_id_result = self._face_id_result, None
        if result is None:
            return
        match, _ = result
        if match is None:
            return
        self._apply_person_match(match, now_mono)

    @staticmethod
    def _head_crop(frame, pose_box):
        """Cut the head region out of the full frame for face ID.

        The pose box spans every visible landmark (its top edge sits at the
        eyes/ears), so the face is near its top. A crop a few hundred pixels
        square keeps full resolution on the face while the detector sees a
        small image — several times cheaper than scanning the whole 720p frame.
        Returns (crop, (x_offset, y_offset)).
        """
        h, w = frame.shape[:2]
        if pose_box is None:
            return frame, (0, 0)
        bx, by, bw, bh = pose_box
        cx = bx + bw / 2.0
        size = int(min(max(bw * 0.9, bh * 0.4, 224), max(w, h)))
        x0 = int(max(0, min(cx - size / 2.0, w - size)))
        y0 = int(max(0, min(by - size * 0.35, h - size)))
        x1, y1 = min(w, x0 + size), min(h, y0 + size)
        if x1 - x0 < 64 or y1 - y0 < 64:
            return frame, (0, 0)
        return np.ascontiguousarray(frame[y0:y1, x0:x1]), (x0, y0)

    def _face_id_loop(self):
        """Worker thread: identify the latest submitted frame."""
        while rclpy.ok():
            with self._face_id_lock:
                job, self._face_id_job = self._face_id_job, None
                if job is not None:
                    self._face_id_busy = True
            if job is None:
                time.sleep(0.01)
                continue
            frame, hint_xy, (ox, oy), t_mono = job
            match = None
            try:
                match = self.face_id.identify(frame, near=hint_xy)
                if match is not None and (ox or oy):
                    bx, by, bw, bh = match.box
                    match.box = (bx + ox, by + oy, bw, bh)     # back to full-frame coordinates
            except Exception as e:
                self.get_logger().warning(f"Face ID failed: {e}")
            with self._face_id_lock:
                self._face_id_result = (match, t_mono)
                self._face_id_busy = False

    def _apply_person_match(self, match, now_mono: float):
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

    def _on_people_changed(self):
        """The Folks page renamed or forgot someone: re-announce under the new name."""
        self.last_person_label = None
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

    def _greeted_recently(self, person_id: str) -> bool:
        """True when this person was greeted less than greeting_cooldown ago (logged once per visit)."""
        if self.greeting_cooldown <= 0.0 or self.face_id is None:
            return False
        person = self.face_id.gallery.people.get(person_id)
        if person is None or not person.last_greeted:
            return False
        since = time.time() - person.last_greeted
        if since >= self.greeting_cooldown:
            return False
        if self._cooldown_logged_for != person_id:
            self._cooldown_logged_for = person_id
            left = self.greeting_cooldown - since
            self.get_logger().info(f"{person.label} is back {since / 60:.0f} min after the last hello; "
                                   f"not greeting again for {left / 60:.0f} min (Folks page: Reset hello)")
        return True

    def _wink(self) -> None:
        """Close one eye for wink_duration (the greeting for someone back within the hour)."""
        if self.wink_start_time is None:
            self.wink_start_time = time.monotonic()

    def _wink_srv(self, request, response):
        self._wink()
        response.success = True
        response.message = f"winking the {'left' if self.wink_eye_index == 0 else 'right'} eye"
        return response

    def _mark_greeted(self, person_id: Optional[str]) -> None:
        if person_id is None or self.face_id is None:
            return
        self._cooldown_logged_for = None
        try:
            self.face_id.gallery.mark_greeted(person_id)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"could not record the greeting for {person_id}: {e}")

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
                elif name == "ur_wave_enabled":
                    self.ur_wave_enabled = bool(value)
                elif name == "ur_wave_program":
                    self.ur_wave_program = str(value)
                elif name == "ur_wave_cooldown":
                    self.ur_wave_cooldown = max(float(value), 0.0)
                elif name == "ur_wave_known_only":
                    self.ur_wave_known_only = bool(value)
                elif name == "ur_ip":
                    self.ur = UrDashboard(str(value), log=lambda m: self.get_logger().info(m))
                    self.ur_motion = UrMotion(str(value), log=lambda m: self.get_logger().info(m),
                                              return_to=self.ur_motion.return_to, home_file=self.ur_motion.home_file,
                                              approach_order=self.ur_motion.approach_order,
                                              return_order=self.ur_motion.return_order,
                                              retrace=self.ur_motion.retrace, retrace_speedup=self.ur_motion.retrace_speedup,
                                              auto_power_up=self.ur_wave_power_up)
                elif name == "ur_wave_power_up":
                    self.ur_wave_power_up = bool(value)
                    self.ur_motion.auto_power_up = bool(value)
                elif name == "audio_sink":
                    self.audio_sink = str(value)
                    self.sink_watchdog.pattern = self.audio_sink
                elif name == "ur_wave_return":
                    mode = str(value).strip().lower()
                    if mode not in ("home", "start", "none"):
                        self.get_logger().warning(f"ur_wave_return={value!r}: expected home | start | none; using home")
                        mode = "home"
                    self.ur_motion.return_to = mode
                elif name == "ur_wave_home_file":
                    self.ur_motion.home_file = str(value)
                elif name == "hand_wiggle_enabled":
                    self.hand_wiggle_enabled = bool(value)
                elif name in ("hand_host", "hand_port"):
                    setattr(self.hand_wiggle, "host" if name == "hand_host" else "port", str(value) if name == "hand_host" else int(value))
                elif name in ("hand_speed", "hand_wiggle_hz", "hand_wiggle_curl", "hand_thumb_curl", "hand_wiggle_phase"):
                    attr = {"hand_speed": "speed", "hand_wiggle_hz": "hz", "hand_wiggle_curl": "curl", "hand_thumb_curl": "thumb",
                            "hand_wiggle_phase": "phase"}[name]
                    setattr(self.hand_wiggle, attr, int(value) if attr == "speed" else float(value))
                elif name == "ur_wave_retrace":
                    self.ur_motion.retrace = bool(value)
                elif name == "ur_wave_retrace_speedup":
                    self.ur_motion.retrace_speedup = max(1.0, float(value))
                elif name in ("ur_wave_approach_order", "ur_wave_return_order"):
                    try:
                        parse_order(str(value))
                    except ValueError as e:
                        self.get_logger().warning(f"{name}: {e}; keeping the previous order")
                    else:
                        setattr(self.ur_motion, "approach_order" if name.endswith("approach_order") else "return_order", str(value))
                elif name == "ur_wave_source":
                    self.ur_wave_source = str(value)
                elif name == "ur_wave_motion_file":
                    self.ur_wave_motion_file = str(value)
                elif name == "greeting_wait":
                    self.greeting_wait = max(float(value), 0.0)
                elif name == "greeting_cooldown":
                    self.greeting_cooldown = max(float(value), 0.0)
                elif name == "wink_eye":
                    self.wink_eye_index = 0 if str(value).strip().lower().startswith("l") else 1
                elif name == "wink_duration":
                    self.wink_duration = max(float(value), 0.2)
                elif name == "wink_depth":
                    self.wink_depth = max(0.0, min(float(value), 1.0))
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
                elif name == "fullscreen":
                    # GLFW calls must come from the render thread; apply on the next frame
                    self._pending_fullscreen = bool(value)
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

    # ---------- UR arm gesture ----------
    def _wave(self, reason: str, force: bool = False) -> str:
        """Play the pendant's wave program on the UR arm in the background (never blocks the
        render loop). Skipped while disabled, within the cooldown, or when the arm is busy."""
        if not force:
            if not self.ur_wave_enabled or not self.ur_wave_program:
                return "wave disabled"
            if time.monotonic() - self._last_wave < self.ur_wave_cooldown:
                return "wave cooldown"
        if self.ur.busy or self.ur_motion.busy:
            return "already waving"
        self._last_wave = time.monotonic()
        program, motion_file, source = self.ur_wave_program, os.path.expanduser(self.ur_wave_motion_file), self.ur_wave_source
        use_recording = source == "recording" or (source == "auto" and os.path.isfile(motion_file))

        def _phase(name):
            # fingers ripple while the arm waves, open again for the way home
            if name == "wave" and self.hand_wiggle_enabled:
                if self.hand_wiggle.start():
                    self.get_logger().info("hand: fingers wiggling (thumb still)")
            elif name in ("retrace", "return", "done"):
                n = self.hand_wiggle.sent
                self.hand_wiggle.stop()
                if n:
                    self.get_logger().info(f"hand: fingers open again ({n} setpoints)")

        def _run():
            if use_recording:
                ok, msg = self.ur_motion.play(motion_file, on_phase=_phase)
                how = f"recording {os.path.basename(motion_file)}"
            else:
                ok, msg = self.ur.play(program, auto_power_up=self.ur_wave_power_up)
                how = f"pendant program {program}"
            # rclpy refuses a different severity from the same call site, so two lines
            if ok:
                self.get_logger().info(f"UR wave ({reason}, {how}): {msg}")
            else:
                self.get_logger().warning(f"UR wave ({reason}, {how}): {msg}")
        threading.Thread(target=_run, daemon=True, name="ur_wave").start()
        return "waving"

    def _wave_srv(self, request, response):
        r = self._wave("service call", force=True)
        response.success = r == "waving"
        response.message = f"{r} on {self.ur.ip}" if response.success else r
        return response

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

        self.speaking_pub.publish(Bool(data=True))
        try:
            self._play_wav_with_mouth_inner(path, audio, rate)
        finally:
            self.speaking_pub.publish(Bool(data=False))

    def _play_wav_with_mouth_inner(self, path: str, audio, rate: int):
        proc = subprocess.Popen(
            ["aplay", "-q", path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            env=player_env(self.audio_sink),       # pinned to the speaker even if the default moved
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
    # ---------- Camera frames: detection + tracking only (rendering runs on its own timer) ----------
    def image_callback(self, msg):
        try:
            if isinstance(msg, CompressedImage):
                frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    return
            else:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"CvBridge failed: {exc}")
            return

        self.camera_height, self.camera_width = frame.shape[:2]
        if self.raw_video_writer is not None:
            self.raw_video_writer.write(frame)
        now_mono = time.monotonic()
        if self._last_frame_time == 0.0 or self._camera_warned:
            self.get_logger().info(
                f"Camera frames arriving on {self._image_topic} ({self.camera_width}x{self.camera_height})."
            )
            self._camera_warned = False
        self._last_frame_time = now_mono
        self._process_frame(frame, time.time(), now_mono)

    def _process_frame(self, frame, current_time: float, current_mono: float):
        self.frame_counter += 1
        if self.face_tracker:
            self._process_frame_faces(frame, current_time, current_mono)
            return

        # ---- Person detection for eyes (MediaPipe pose) ----
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
        if selected_face is not None:
            self._on_person_seen(selected_face, current_time, current_mono)
            self._identify_person(frame, box_top_center(selected_face), current_mono)
        self._process_hands(frame, rgb_frame, current_mono)

    def _on_person_seen(self, box, current_time: float, current_mono: float):
        """A person (pose box or face box) is in view this frame."""
        if self.tracked_face is None:
            # New person detected — look happy, then greet once we know who it is
            self.happy_until = current_time + self.greeting_happy_seconds
            self.current_person = None
            self.generic_greeted_at = None
            if self.greeting_enabled:
                self.greeting_due = current_mono + (self.greeting_wait if self.face_id else 0.0)
        self.tracked_face = box
        self.last_seen_time = current_time

    # ---------- Face-detector tracking (vision thread + per-frame bookkeeping) ----------
    def _process_frame_faces(self, frame, current_time: float, current_mono: float):
        """Hand the newest frame to the vision thread and apply its latest result."""
        with self._face_id_lock:
            self._face_id_job = (np.ascontiguousarray(frame), current_mono)   # newest frame wins
            result, self._vision_result = self._vision_result, None
        if result is None:
            return
        faces, match, _t = result
        boxes = [f.box for f in faces
                 if f.score >= self.face_id.min_detect_score and f.box[2] >= self.face_id.min_face_px]
        selected = pick_face(boxes, self.tracked_face)
        if selected is not None:
            self._on_person_seen(selected, current_time, current_mono)
            self._track_hint = (selected[0] + selected[2] / 2.0, selected[1] + selected[3] / 2.0)
        if match is not None:
            self._apply_person_match(match, current_mono)

    def _vision_loop(self):
        """Vision thread: detect faces on every frame; embed + identify every face_id_min_period."""
        while rclpy.ok():
            with self._face_id_lock:
                job, self._face_id_job = self._face_id_job, None
            if job is None:
                time.sleep(0.005)
                continue
            frame, t_mono = job
            now = time.monotonic()
            do_embed = (now - self._face_id_last_submit) >= self.face_id_min_period
            faces, match = [], None
            try:
                faces = self.face_id.backend.analyze(frame, embed=do_embed)
                if do_embed and faces:
                    self._face_id_last_submit = now
                    match = self.face_id.identify(frame, near=self._track_hint, faces=faces)
            except Exception as e:
                self.get_logger().warning(f"Face tracking failed: {e}")
            with self._face_id_lock:
                self._vision_result = (faces, match, t_mono)

    def _gaze_point(self, box):
        """Where the eyes should look for a tracked box: the face centre for face
        boxes, the top of the body box for MediaPipe pose boxes."""
        if self.face_tracker:
            return (box[0] + box[2] / 2.0, box[1] + box[3] / 2.0)
        return box_top_center(box)

    def _process_hands(self, frame, rgb_frame, current_mono: float):

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
                            (self.get_parameter("hand_relay_host").value, int(self.get_parameter("hand_relay_port").value)),
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


    # ---------- Render tick: runs at render_fps whether or not camera frames arrive ----------
    def _per_tick(self, per_frame_factor: float) -> float:
        """Per-camera-frame smoothing factor -> equivalent per-render-tick factor."""
        f = self.clamp_speed(per_frame_factor)
        return 1.0 - (1.0 - f) ** (self.render_dt * self.capture_fps)

    def _render_tick(self):
        current_time = time.time()
        current_mono = time.monotonic()

        # Camera watchdog: say so once when frames stop; the face keeps animating regardless
        if self._last_frame_time == 0.0 or current_mono - self._last_frame_time > self.camera_timeout:
            if not self._camera_warned and current_mono - self._start_mono > self.camera_timeout:
                self.get_logger().warning(
                    f"No camera frames on {self._image_topic} for {self.camera_timeout:.0f}s; "
                    "the face keeps animating but cannot see anyone."
                )
                self._camera_warned = True

        # Face lost (also when the camera stopped mid-visit): back to idle scanning
        if self.tracked_face is not None and current_time - self.last_seen_time > self.face_stick_seconds:
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
        #  Each recognised person (named or not) is greeted at most once per greeting_cooldown
        #  (an hour by default); the Folks page's "Reset hello" clears that per person.
        name = self._person_name()
        pid = self.current_person.person_id if (self.current_person is not None and self.current_person.known) else None
        if self.greeting_due is not None:
            if pid is not None and self._greeted_recently(pid):
                self.greeting_due = None                   # seen again within the hour: a wink, no hello
                self._wink()
            elif name is not None:
                self._speak(self._format_greeting(self.greeting_known_text, name))
                self.greeting_due = None
                self._mark_greeted(pid)
                self._wave(f"recognised {name}")
            elif current_mono >= self.greeting_due:
                self._speak(self.greeting_text)
                self.greeting_due = None
                self.generic_greeted_at = current_mono
                if pid is not None:                        # recognised but unnamed: that was their hello
                    self._mark_greeted(pid)
                if not self.ur_wave_known_only:
                    self._wave("someone is here")
                if self.face_id is not None:
                    self.get_logger().info(
                        f"Greeted generically: no face identified within {self.greeting_wait:.1f}s "
                        "(will add the name if recognised shortly)."
                    )
        elif self.generic_greeted_at is not None:
            if pid is not None:
                # The generic hello already went out before we knew who it was. Add the name only if
                # this person had not been greeted within the hour anyway; either way it counts.
                recent = self._greeted_recently(pid)
                self._mark_greeted(pid)
                self.generic_greeted_at = None
                if name is not None and not recent:
                    self._speak(self._format_greeting(self.greeting_late_text, name))
                    self._wave(f"recognised {name}")
                elif recent:
                    self._wink()
            elif current_mono - self.generic_greeted_at > self.greeting_name_window:
                self.generic_greeted_at = None

        time_since_face = current_time - self.last_seen_time
        if self.tracked_face is not None:
            center = self._gaze_point(self.tracked_face)
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
            k = self._per_tick(self.gaze_speed if self.tracked_face is not None else self.gaze_idle_speed)
            self.smoothed_center = (
                self.smoothed_center[0] + (center[0] - self.smoothed_center[0]) * k,
                self.smoothed_center[1] + (center[1] - self.smoothed_center[1]) * k,
            )

        if self.blink_start_time is None and current_mono >= self.next_blink_time \
                and self.wink_start_time is None:          # no blink on top of a wink
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

        # Render eyes + mouth waveform via GL shader
        if self._pending_fullscreen is not None:
            self.gl_renderer.set_fullscreen(self._pending_fullscreen)
            self._pending_fullscreen = None

        (mouth_level, mouth_samples, mouth_smile, mouth_skew,
         mouth_path, mouth_shape) = self._update_mouth(
            current_mono, seeing_person=self.tracked_face is not None
        )
        wink_amount = 0.0
        if self.wink_start_time is not None:
            w = current_mono - self.wink_start_time
            close_t, open_t = 0.18 * self.wink_duration, 0.25 * self.wink_duration
            hold_t = self.wink_duration - close_t - open_t
            if w >= self.wink_duration:
                self.wink_start_time = None
                self.next_blink_time = max(self.next_blink_time, current_mono + 1.0)
            elif w <= close_t:
                wink_amount = w / close_t
            elif w <= close_t + hold_t:
                wink_amount = 1.0
            else:
                wink_amount = max(0.0, 1.0 - (w - close_t - hold_t) / open_t)
        self.gl_renderer.render(
            blink=blink_amount,
            wink=wink_amount * self.wink_depth,      # a sliver stays open at the bottom of the wink
            wink_eye=self.wink_eye_index,
            gaze_offset=(offsetX, offsetY),
            mood=3 if current_time < self.happy_until else self.eyelash_count,
            dt=self.render_dt,
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
        if getattr(self, 'people_web', None) is not None:
            self.people_web.stop()
        if self._kokoro is not None:
            self._kokoro.stop()
        if self.face_id is not None:
            self.face_id.stop()
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
