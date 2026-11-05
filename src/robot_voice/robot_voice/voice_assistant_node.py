#!/usr/bin/env python3
"""ROS 2 voice assistant for Robot Disco.

    microphone ─► ears_worker (venv: VAD + faster-whisper) ─► "robot disco, <command>"
             ─► Brain (rules + Claude with tools) ─► reply text ─► face node `say` (Kokoro + mouth)

Skills: weather, news, Home Assistant, local music, time, Nav2 navigation to the
named places in a waypoints YAML and to the landmarks the manager's Landmarks tab
keeps ("move to your charger", "go to the right bookshelf"), and landmark TASKS:
"pick up tigger" drives to the landmark that task belongs to and then runs its
policy (VLA/scripts/vla_action.py) — see tasks.py. The face node's `~/person`
topic tells the brain who is talking.

Topics:
    ~/heard     std_msgs/String  every transcript (debug)
    ~/command   std_msgs/String  command text after the wake phrase
    ~/reply     std_msgs/String  what the robot answered
    ~/state     std_msgs/String  idle | listening | thinking | speaking
    ~/text_in   std_msgs/String  (sub) inject a transcript — test without a microphone
    ~/run_task  std_msgs/String  (sub) JSON {landmark, task} ids — the Landmarks tab's ▶ Run (via the route editor)
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import threading
import time
from typing import List, Optional

import rclpy
import yaml
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .brain import Brain, _match_place
from .audio_sink import player_env
from .skills import ClockSkill, HomeAssistantSkill, MusicSkill, NewsSkill, VolumeSkill, WeatherSkill
from .tasks import LandmarkStore, TaskRunner
from .wake import WakeMatcher

EARS_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ears_worker.py")


# ---------------------------------------------------------------------------
# Nav2 navigator
# ---------------------------------------------------------------------------
class Nav2Navigator:
    """Named poses from waypoints.yaml plus the landmarks (Scripts/landmarks/landmarks.yaml, re-read when
    it changes). `go_to` is fire-and-forget with a spoken arrival; `go_to_landmark` blocks (tasks)."""

    def __init__(self, node: Node, places_file: str, action_name: str, frame_id: str = "map",
                 landmarks: Optional[LandmarkStore] = None):
        self.node = node
        self.frame_id = frame_id
        self._waypoints = self._load(os.path.expanduser(places_file))
        self._landmarks = landmarks
        self._client = None
        self._goal_handle = None
        self._current: Optional[str] = None
        self._announce = True                     # say "I've arrived" when the goal ends (not for task drives)
        self._done = threading.Event()
        self._status: Optional[int] = None
        self._seq = 0                             # goal counter: a cancelled older goal's result must not end the new one
        try:
            from nav2_msgs.action import NavigateToPose
            self._NavigateToPose = NavigateToPose
            self._client = ActionClient(node, NavigateToPose, action_name,
                                        callback_group=ReentrantCallbackGroup())
        except ImportError:
            node.get_logger().warning("nav2_msgs not available; navigation disabled")

    @staticmethod
    def _load(path: str) -> dict:
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return {}
        out = {}
        for wp in data.get("waypoints", []):
            name = str(wp.get("name", "")).strip()
            if name and "position" in wp:
                out[name] = wp
        return out

    @property
    def _places(self) -> dict:
        """waypoints.yaml poses + the landmarks as {name: {position, orientation}} (landmarks win on a clash)."""
        out = dict(self._waypoints)
        if self._landmarks is not None:
            for lm in self._landmarks.landmarks():
                try:
                    yaw = float(lm.get("yaw", 0.0))
                    out[str(lm["name"])] = {"position": {"x": float(lm["x"]), "y": float(lm["y"])},
                                            "orientation": {"z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}, "landmark": lm}
                except (KeyError, TypeError, ValueError):
                    continue
        return out

    def places(self) -> List[str]:
        return [p.replace("_", " ") for p in self._places]

    def _resolve(self, place: str) -> Optional[str]:
        names = {p.replace("_", " "): p for p in self._places}
        hit = _match_place(place, list(names.keys()))
        return names.get(hit) if hit else None

    def _send(self, wp: dict, key: str, announce: bool) -> Optional[str]:
        """Send one NavigateToPose goal; None when sent, else the spoken reason it wasn't."""
        if self._client is None:
            return "Navigation isn't available."
        if not self._client.wait_for_server(timeout_sec=1.5):
            return "The navigation stack isn't running, so I can't drive there."
        goal = self._NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        pos, ori = wp["position"], wp.get("orientation", {"x": 0, "y": 0, "z": 0, "w": 1})
        goal.pose.pose.position.x = float(pos["x"]); goal.pose.pose.position.y = float(pos["y"])
        goal.pose.pose.position.z = float(pos.get("z", 0.0))
        goal.pose.pose.orientation.x = float(ori.get("x", 0.0)); goal.pose.pose.orientation.y = float(ori.get("y", 0.0))
        goal.pose.pose.orientation.z = float(ori.get("z", 0.0)); goal.pose.pose.orientation.w = float(ori.get("w", 1.0))
        self.stop(quiet=True)
        self._done.clear()
        self._status = None
        self._announce = announce
        self._current = key
        self._seq += 1
        seq = self._seq
        future = self._client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, seq))
        return None

    def go_to(self, place: str) -> str:
        key = self._resolve(place)
        if key is None:
            known = ", ".join(self.places()) or "nothing"
            return f"I don't know where {place} is. I know {known}."
        why = self._send(self._places[key], key, announce=True)
        if why:
            return why
        nice = key.replace("_", " ")
        return f"On my way to the {nice}." if nice != "charger" else "Heading back to my charger."

    def go_to_landmark(self, lm: dict, timeout: float = 300.0) -> "tuple[bool, str]":
        """Drive to a landmark's pose and wait for Nav2's verdict (a task's first leg). (ok, why)."""
        try:
            yaw = float(lm.get("yaw", 0.0))
            wp = {"position": {"x": float(lm["x"]), "y": float(lm["y"])},
                  "orientation": {"z": math.sin(yaw / 2), "w": math.cos(yaw / 2)}}
        except (KeyError, TypeError, ValueError) as e:
            return False, f"the landmark has no usable pose ({e})"
        why = self._send(wp, str(lm.get("name", "landmark")), announce=False)
        if why:
            return False, why
        if not self._done.wait(timeout):
            self.stop(quiet=True)
            return False, "navigation took too long"
        st = self._status
        if st == 4:                 # action_msgs GoalStatus.STATUS_SUCCEEDED
            return True, "arrived"
        return False, {5: "the drive was cancelled", 6: "navigation gave up", -1: "the goal was rejected",
                       -2: "the goal failed to send"}.get(st, f"navigation ended with status {st}")

    def _on_goal_response(self, future, seq: int):
        try:
            gh = future.result()
        except Exception as e:
            self.node.get_logger().warning(f"navigation goal failed: {e}")
            if seq == self._seq:
                self._status, self._current = -2, None
                self._done.set()
            return
        if not gh.accepted:
            self.node.get_logger().warning("navigation goal rejected")
            if seq == self._seq:
                self._goal_handle, self._status, self._current = None, -1, None
                self._done.set()
            return
        if seq != self._seq:            # superseded while in flight: cancel it, the newer goal owns the robot
            try:
                gh.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._on_result(f, seq))

    def _on_result(self, future, seq: int):
        if seq != self._seq:            # an older goal (cancelled by a newer one) ending: not ours any more
            return
        self._goal_handle = None
        where = self._current
        self._current = None
        try:
            self._status = int(future.result().status)
        except Exception:  # noqa: BLE001
            self._status = -2
        self.node.get_logger().info(f"navigation to {where} finished (status {self._status})")
        self._done.set()
        if self._announce and self._status == 4:
            self.node.on_navigation_done(where)

    def stop(self, quiet: bool = False) -> str:
        gh, self._goal_handle = self._goal_handle, None
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            self._current = None
            return "" if quiet else "Stopping."
        return "" if quiet else "I'm not going anywhere right now."

    def status(self) -> str:
        return f"Driving to the {self._current.replace('_', ' ')}." if self._current else "I'm parked."


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------
class VoiceAssistantNode(Node):
    def __init__(self):
        super().__init__("robot_voice")
        self.declare_parameters("", [
            # ears
            ("ears_enabled", True),
            ("ears_python", "~/.robot_face/voice-venv/bin/python"),
            ("asr_model", "base.en"),
            ("asr_device", "cpu"),
            ("asr_compute_type", "int8"),
            ("asr_threads", 4),
            ("audio_source", ""),              # pulse source substring; "" = USB mic if any, else default
            ("vad_min_rms", 0.012),
            ("vad_end_ms", 900),
            # wake
            ("wake_phrases", ["robot disco"]),
            ("wake_threshold", 0.74),
            ("follow_up_seconds", 6.0),
            ("ack_sound", True),
            # face integration
            ("say_topic", "/combined_face_hand/say"),
            ("speaking_topic", "/combined_face_hand/speaking"),
            ("person_topic", "/combined_face_hand/person"),
            # brain
            ("llm_enabled", True),
            ("llm_model", "claude-opus-5-5"),
            ("llm_backend", "auto"),           # auto | claude | ollama | none
            ("ollama_url", "http://192.168.11.119:11434,http://127.0.0.1:11434"),   # comma-separated, tried in order
            ("ollama_model", "qwen2.5:7b"),
            ("persona", "Robot Disco"),
            ("location_name", ""),
            # skills
            ("weather_latitude", 0.0),
            ("weather_longitude", 0.0),
            ("weather_place", ""),
            ("units", "fahrenheit"),
            ("news_feeds", ["NPR=https://feeds.npr.org/1001/rss.xml",
                            "BBC=https://feeds.bbci.co.uk/news/world/rss.xml"]),
            ("hass_url", ""),
            ("hass_token", ""),
            ("hass_aliases", [""]),            # "kitchen light=light.kitchen_main"
            ("music_dir", "~/Music"),
            ("player_cmd", ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet"]),
            ("music_continuous", True),         # keep playing the queue when a song ends
            ("music_duck", 0.6),                # music stream volume while listening/thinking/speaking; PulseAudio percent is
                                                # cubic: 0.6 = -13 dB (a normal duck), 0.4 = -24 dB, 0.25 = -36 dB (near silence); 1.0 = off
            ("volume_sink", "C-Media"),          # PulseAudio sink the speaker is on (name or fragment; the face node keeps it default)
            ("volume_step", 10),                 # percent per "louder" / "lower the volume"
            ("volume_max", 100),                 # cap for "full volume" (PulseAudio allows more, it distorts)
            ("places_file", "~/rd_ws/Scripts/waypoints.yaml"),
            ("landmarks_file", "~/rd_ws/Scripts/landmarks/landmarks.yaml"),   # the Landmarks tab's places + their tasks
            ("nav_action", "/navigate_to_pose"),
            ("nav_frame", "map"),
            ("vla_dir", "~/rd_ws/VLA"),                # tasks run <vla_dir>/run.sh scripts/vla_action.py
            ("task_nav_timeout", 300.0),         # s to reach a task's landmark before giving up
            ("task_run_timeout", 600.0),         # s on top of the task's seconds before the run is terminated
        ])
        p = lambda n: self.get_parameter(n).value

        self.pub_heard = self.create_publisher(String, "~/heard", 10)
        self.pub_command = self.create_publisher(String, "~/command", 10)
        self.pub_reply = self.create_publisher(String, "~/reply", 10)
        self.pub_state = self.create_publisher(String, "~/state", 10)
        self.pub_say = self.create_publisher(String, str(p("say_topic")), 10)
        self.create_subscription(String, "~/text_in", self._text_in_cb, 10)
        self.create_subscription(String, "~/run_task", self._run_task_cb, 10)
        self.create_subscription(Bool, str(p("speaking_topic")), self._speaking_cb, 10)
        self.create_subscription(String, str(p("person_topic")), self._person_cb, 10)

        # Skills + brain
        feeds = {}
        for item in p("news_feeds") or []:
            if "=" in item:
                k, v = item.split("=", 1)
                feeds[k.strip()] = v.strip()
        aliases = {}
        for item in p("hass_aliases") or []:
            if "=" in item:
                k, v = item.split("=", 1)
                aliases[k.strip()] = v.strip()
        self.weather = WeatherSkill(float(p("weather_latitude")), float(p("weather_longitude")),
                                    str(p("weather_place")), str(p("units")))
        self.news = NewsSkill(feeds or None)
        self.home = HomeAssistantSkill(str(p("hass_url")), str(p("hass_token")), aliases)
        self.music = MusicSkill(str(p("music_dir")), list(p("player_cmd")), sink=str(p("volume_sink")),
                                continuous=bool(p("music_continuous")), duck_level=float(p("music_duck")))
        self.add_on_set_parameters_callback(self._on_params_changed)   # ros2 param set /robot_voice music_duck 0.7
        self.clock = ClockSkill()
        self.volume = VolumeSkill(str(p("volume_sink")), int(p("volume_step")), int(p("volume_max")),
                                  defer=self._after_speech)
        self.landmarks = LandmarkStore(str(p("landmarks_file")))
        self.nav = Nav2Navigator(self, str(p("places_file")), str(p("nav_action")), str(p("nav_frame")),
                                 landmarks=self.landmarks)
        nav_timeout = float(p("task_nav_timeout"))
        self.tasks = TaskRunner(self.landmarks, navigate=lambda lm: self.nav.go_to_landmark(lm, nav_timeout),
                                say=self.say, log=lambda m: self.get_logger().info(m), vla_dir=str(p("vla_dir")),
                                run_timeout=float(p("task_run_timeout")))
        self.brain = Brain(self.weather, self.news, self.home, self.music, self.clock, self.nav,
                           volume=self.volume, tasks=self.tasks,
                           persona=str(p("persona")), location_name=str(p("location_name")),
                           model=str(p("llm_model")), llm_enabled=bool(p("llm_enabled")),
                           llm_backend=str(p("llm_backend")), ollama_url=str(p("ollama_url")),
                           ollama_model=str(p("ollama_model")),
                           log=lambda m: self.get_logger().info(m))
        self.get_logger().info(
            f"Brain: {self.brain.active_backend}{'' if self.brain.llm_available else ' (' + self.brain.llm_reason + ')'}; "
            f"places: {', '.join(self.nav.places()) or 'none'}; tasks: {', '.join(self.tasks.names()) or 'none'}; Home Assistant: "
            f"{'connected' if self.home.available() else 'not configured'}; weather: "
            f"{'configured' if self.weather.available() else 'not configured'}"
        )

        # Wake + dialogue state
        self.wake = WakeMatcher(list(p("wake_phrases")), float(p("wake_threshold")))
        self.follow_up_seconds = float(p("follow_up_seconds"))
        self.ack_sound = bool(p("ack_sound"))
        self._follow_up_until = 0.0
        self._person: Optional[str] = None
        self._person_time = 0.0
        self._speaking = False
        self._after_speech_fns: List = []      # run once the current reply has been spoken (e.g. mute)
        self._after_lock = threading.Lock()
        self._queue: List[str] = []
        self._queue_lock = threading.Lock()
        self._worker: Optional[threading.Thread] = None

        # Ears worker
        self._ears: Optional[subprocess.Popen] = None
        self._ears_lock = threading.Lock()
        if bool(p("ears_enabled")):
            threading.Thread(target=self._ears_loop, daemon=True).start()
        self._set_state("idle")

    # ------------------------------------------------------------------
    # Ears worker management
    # ------------------------------------------------------------------
    def _ears_args(self) -> List[str]:
        p = lambda n: self.get_parameter(n).value
        py = os.path.expanduser(str(p("ears_python")))
        if not os.path.exists(py):
            self.get_logger().warning(f"ears venv python not found at {py}; using system python3")
            py = "python3"
        return [py, EARS_SCRIPT, "--model", str(p("asr_model")), "--device", str(p("asr_device")),
                "--compute-type", str(p("asr_compute_type")), "--threads", str(int(p("asr_threads"))),
                "--source", str(p("audio_source")), "--min-rms", str(float(p("vad_min_rms"))),
                "--end-ms", str(int(p("vad_end_ms")))]

    def _ears_loop(self):
        backoff = 2.0
        while rclpy.ok():
            try:
                with self._ears_lock:
                    self._ears = subprocess.Popen(self._ears_args(), stdin=subprocess.PIPE,
                                                  stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                                  text=True, bufsize=1)
                if self._speaking:
                    self._ears_cmd({"cmd": "mute"})
                for line in self._ears.stdout:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        ev = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    self._on_ears_event(ev)
                backoff = 2.0
            except Exception as e:
                self.get_logger().warning(f"ears worker error: {e}")
            if not rclpy.ok():
                break
            self.get_logger().warning(f"ears worker exited; restarting in {backoff:.0f}s")
            time.sleep(backoff)
            backoff = min(backoff * 2, 30.0)

    def _ears_cmd(self, cmd: dict) -> None:
        with self._ears_lock:
            p = self._ears
            if p is None or p.stdin is None or p.poll() is not None:
                return
            try:
                p.stdin.write(json.dumps(cmd) + "\n")
                p.stdin.flush()
            except (BrokenPipeError, OSError):
                pass

    def _on_ears_event(self, ev: dict) -> None:
        kind = ev.get("event")
        if kind == "ready":
            self.get_logger().info(f"Ears ready: whisper {ev.get('model')} on {ev.get('device')} "
                                   f"({ev.get('compute')}), source {ev.get('source')}, loaded in {ev.get('load_s')}s")
        elif kind == "utterance":
            text = ev.get("text", "")
            self.get_logger().info(f"heard ({ev.get('seconds')}s, asr {ev.get('asr_ms')} ms, rms {ev.get('rms')}): {text!r}")
            self.pub_heard.publish(String(data=text))
            self._handle_transcript(text)
        elif kind == "listening":
            if ev.get("on") and not self._speaking:
                self._set_state("listening")
                self.get_logger().debug("speech started")
            elif not ev.get("on"):
                self._set_state("idle")
        elif kind == "discarded":
            # the VAD triggered but Whisper found no speech in it (noise, a cough, the robot's own tail)
            self.get_logger().info(f"heard noise ({ev.get('seconds')}s) but no words")
        elif kind == "info":
            self.get_logger().info(f"ears: {ev.get('info')}")
        elif kind == "error":
            self.get_logger().warning(f"ears: {ev.get('error')}")

    # ------------------------------------------------------------------
    # Dialogue
    # ------------------------------------------------------------------
    def _text_in_cb(self, msg: String):
        """Inject a transcript (testing without a microphone)."""
        text = msg.data.strip()
        if text:
            self.get_logger().info(f"text_in: {text!r}")
            self.pub_heard.publish(String(data=text))
            self._handle_transcript(text)

    def _run_task_cb(self, msg: String):
        """The Landmarks tab's ▶ Run: the route editor publishes {landmark, task} ids here."""
        try:
            d = json.loads(msg.data)
            lid, tid = str(d["landmark"]), str(d["task"])
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning(f"run_task: bad message {msg.data!r}")
            return
        self.get_logger().info(f"run_task: {lid}/{tid} (from {d.get('from', '?')})")
        reply = self.tasks.run_ids(lid, tid)
        self.pub_reply.publish(String(data=reply))
        self.say(reply)

    def _handle_transcript(self, text: str) -> None:
        now = time.monotonic()
        res = self.wake.match(text)
        if res.woke:
            command = res.command
            self.get_logger().info(f"wake word matched ({res.matched!r}, {res.score:.2f}); command: {command!r}")
        elif now < self._follow_up_until:
            command = text
            self._follow_up_until = 0.0
            self.get_logger().info(f"follow-up command: {command!r}")
        else:
            return   # not for us
        if not command.strip():
            # Wake word alone: a short chime (not speech, which would mute the mic
            # right when the rest of the sentence arrives), then wait for the command
            self._follow_up_until = now + self.follow_up_seconds
            self._play_ack()
            self._set_state("listening")
            return
        self.pub_command.publish(String(data=command))
        with self._queue_lock:
            if len(self._queue) >= 2:
                self.get_logger().info(f"dropping command (queue full): {command!r}")
                return
            self._queue.append(command)
            if self._worker is None or not self._worker.is_alive():
                self._worker = threading.Thread(target=self._respond_loop, daemon=True)
                self._worker.start()

    def _respond_loop(self) -> None:
        """Answer queued commands one after another (a slow first LLM answer must not
        make the next command vanish)."""
        while True:
            with self._queue_lock:
                if not self._queue:
                    return
                command = self._queue.pop(0)
            self._respond(command)

    def _respond(self, command: str) -> None:
        try:
            self._set_state("thinking")
            if self.ack_sound:
                self._play_ack()
            person = self._person if (time.monotonic() - self._person_time) < 60.0 else None
            reply = self.brain.handle(command, person)
            self.pub_reply.publish(String(data=reply))
            self.say(reply)
        except Exception as e:
            self.get_logger().error(f"responding failed: {e}")
            self.say("Something went wrong.")
        finally:
            self._set_state("idle")

    def on_navigation_done(self, where: Optional[str]) -> None:
        if where:
            self.say("I'm at the charger." if where == "charger" else f"I've arrived at the {where.replace('_', ' ')}.")

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    def say(self, text: str) -> None:
        text = (text or "").strip()
        if not text:
            return
        self.get_logger().info(f"say: {text!r}")
        # Mute immediately; the face node confirms through the speaking topic
        self._ears_cmd({"cmd": "mute"})
        self.pub_say.publish(String(data=text))
        if not self._speaking:
            # if the face node isn't running nothing will unmute us: do it after a guess at the duration
            threading.Timer(1.5 + 0.08 * len(text), self._unmute_if_silent).start()

    def _unmute_if_silent(self):
        if not self._speaking:
            self._ears_cmd({"cmd": "unmute"})

    def _speaking_cb(self, msg: Bool):
        was = self._speaking
        self._speaking = bool(msg.data)
        if self._speaking:
            self._set_state("speaking")
            self._ears_cmd({"cmd": "mute"})
        else:
            self._ears_cmd({"cmd": "unmute"})
            self._set_state("idle")
            if was:
                self._run_after_speech()

    def _after_speech(self, fn) -> None:
        """Run fn once the reply being produced has been spoken, or after 10 s if nothing is ever
        spoken (face node down). The volume skill uses this so "Muting" is heard before the mute."""
        with self._after_lock:
            self._after_speech_fns.append(fn)
        threading.Timer(10.0, self._run_after_speech).start()

    def _run_after_speech(self) -> None:
        with self._after_lock:
            fns, self._after_speech_fns = self._after_speech_fns, []
        for fn in fns:
            try:
                fn()
            except Exception as e:
                self.get_logger().warning(f"after-speech action failed: {e}")

    def _person_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if d.get("name"):
            self._person = d["name"]
            self._person_time = time.monotonic()

    def _on_params_changed(self, params):
        """Live tuning without a restart: music_duck, music_continuous, volume_step."""
        for prm in params:
            try:
                if prm.name == "music_duck":
                    self.music.duck_level = max(0.0, min(float(prm.value), 1.0))
                    if self.music._ducked:
                        self.music.duck()
                    self.get_logger().info(f"music_duck -> {self.music.duck_level:.2f}")
                elif prm.name == "music_continuous":
                    self.music.continuous = bool(prm.value)
                elif prm.name == "volume_step":
                    self.volume.step = max(1, int(prm.value))
            except (TypeError, ValueError) as e:
                return SetParametersResult(successful=False, reason=f"{prm.name}: {e}")
        return SetParametersResult(successful=True)

    def _set_state(self, state: str) -> None:
        self.pub_state.publish(String(data=state))
        # music ducks while the robot listens, thinks or talks, and comes back when it is idle
        try:
            if state == "idle":
                self.music.unduck()
            else:
                self.music.duck()
        except Exception as e:  # noqa: BLE001
            self.get_logger().debug(f"duck: {e}")

    def _play_ack(self) -> None:
        """Short rising two-tone chime through aplay (no extra deps)."""
        if not self.ack_sound:
            return
        try:
            import numpy as np
            import wave
            import tempfile
            rate = 22050
            t = np.arange(int(rate * 0.09)) / rate
            env = np.sin(np.pi * np.arange(t.size) / t.size)
            tone = np.concatenate([np.sin(2 * np.pi * 880 * t) * env, np.sin(2 * np.pi * 1320 * t) * env]) * 0.25
            pcm = (tone * 32767).astype("<i2")
            path = os.path.join(tempfile.gettempdir(), "robot_voice_ack.wav")
            if not os.path.exists(path):
                with wave.open(path, "wb") as wf:
                    wf.setnchannels(1); wf.setsampwidth(2); wf.setframerate(rate); wf.writeframes(pcm.tobytes())
            subprocess.Popen(["aplay", "-q", path], env=player_env(str(self.get_parameter("volume_sink").value)), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

    def destroy_node(self):
        self._ears_cmd({"cmd": "quit"})
        self.tasks.stop()
        self.music.stop(quiet=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAssistantNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
