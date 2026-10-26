"""Text-to-speech back-ends for the robot face (no ROS dependency).

* :class:`KokoroTTS`  — Kokoro-82M via ``kokoro_worker.py`` running in its own
  virtualenv (kokoro-onnx needs numpy 2, which the ROS/MediaPipe stack can't
  have). The worker keeps the model loaded; requests go over a pipe.
* :class:`EspeakTTS`  — ``espeak -w`` fallback, always available on Ubuntu.

Every engine implements ``synthesize(text, out_path) -> bool`` and writes a
PCM WAV file the node then plays through ``aplay`` while feeding the mouth.
"""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from typing import List, Optional

WORKER_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kokoro_worker.py")

# Kokoro voice prefix (first letter) -> espeak-ng language code used for phonemisation
VOICE_LANGS = {
    "a": "en-us", "b": "en-gb", "e": "es", "f": "fr-fr", "h": "hi",
    "i": "it", "j": "ja", "p": "pt-br", "z": "cmn",
}


def lang_for_voice(voice: str, override: str = "auto") -> str:
    """Language to phonemise with: an explicit override, else derived from the voice."""
    if override and override.lower() != "auto":
        return override
    return VOICE_LANGS.get((voice or "a")[0].lower(), "en-us")


class EspeakTTS:
    name = "espeak"

    def synthesize(self, text: str, out_path: str) -> bool:
        r = subprocess.run(
            ["espeak", "-w", out_path, text],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        return r.returncode == 0 and os.path.exists(out_path)

    def stop(self) -> None:
        pass


class KokoroTTS:
    """Client for ``kokoro_worker.py``. Thread-safe; one request at a time."""

    name = "kokoro"

    def __init__(
        self,
        python: str = "~/.robot_face/kokoro-venv/bin/python",
        model_dir: str = "~/.robot_face/kokoro",
        voice: str = "af_heart",
        speed: float = 1.0,
        lang: str = "auto",
        start_timeout: float = 60.0,
        request_timeout: float = 20.0,
        log=None,
    ):
        self.python = os.path.expanduser(python)
        self.model_dir = os.path.expanduser(model_dir)
        self.voice = voice
        self.speed = float(speed)
        self.lang = lang
        self.start_timeout = float(start_timeout)
        self.request_timeout = float(request_timeout)
        self._log = log or (lambda msg: None)
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._restarts = 0
        self.voices: List[str] = []
        self.load_seconds = 0.0

    # -- lifecycle ----------------------------------------------------------
    def available(self) -> bool:
        return (
            os.path.exists(self.python)
            and os.path.exists(os.path.join(self.model_dir, "kokoro-v1.0.onnx"))
            and os.path.exists(os.path.join(self.model_dir, "voices-v1.0.bin"))
        )

    def start(self) -> bool:
        """Launch the worker and wait for its ready line. Returns True on success."""
        if not self.available():
            self._log(f"Kokoro not available (need {self.python} and models in {self.model_dir})")
            return False
        with self._lock:
            return self._start_locked()

    def _start_locked(self) -> bool:
        self._stop_locked()
        t0 = time.monotonic()
        try:
            self._proc = subprocess.Popen(
                [self.python, WORKER_SCRIPT, "--model-dir", self.model_dir, "--voice", self.voice],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                text=True, bufsize=1,
            )
        except OSError as e:
            self._log(f"Kokoro worker failed to launch: {e}")
            self._proc = None
            return False
        ready = self._read_reply(self.start_timeout)
        if not ready or not ready.get("ready"):
            self._log(f"Kokoro worker did not become ready: {ready}")
            self._stop_locked()
            return False
        self.voices = list(ready.get("voices", []))
        self.load_seconds = time.monotonic() - t0
        if self.voice not in self.voices and self.voices:
            self._log(f"Kokoro voice '{self.voice}' unknown; using af_heart. Known: {', '.join(self.voices)}")
            self.voice = "af_heart" if "af_heart" in self.voices else self.voices[0]
        return True

    def _read_reply(self, timeout: float) -> Optional[dict]:
        """Read one JSON line from the worker with a timeout (thread-based)."""
        if self._proc is None or self._proc.stdout is None:
            return None
        result: List[Optional[str]] = [None]

        def _reader():
            try:
                result[0] = self._proc.stdout.readline()
            except Exception:
                result[0] = None

        th = threading.Thread(target=_reader, daemon=True)
        th.start()
        th.join(timeout)
        line = result[0]
        if th.is_alive() or not line:
            return None
        try:
            return json.loads(line)
        except json.JSONDecodeError:
            return None

    def _stop_locked(self) -> None:
        p = self._proc
        self._proc = None
        if p is None:
            return
        try:
            if p.stdin:
                p.stdin.write(json.dumps({"cmd": "quit"}) + "\n")
                p.stdin.flush()
            p.wait(timeout=2.0)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass

    def stop(self) -> None:
        with self._lock:
            self._stop_locked()

    # -- synthesis ----------------------------------------------------------
    def synthesize(self, text: str, out_path: str) -> bool:
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                if self._restarts >= 3 or not self._start_locked():
                    return False
                self._restarts += 1
            req = {
                "text": text, "voice": self.voice, "speed": self.speed,
                "lang": lang_for_voice(self.voice, self.lang), "out": out_path,
            }
            try:
                self._proc.stdin.write(json.dumps(req) + "\n")
                self._proc.stdin.flush()
            except (BrokenPipeError, OSError, AttributeError) as e:
                self._log(f"Kokoro worker pipe broken: {e}")
                self._stop_locked()
                return False
            reply = self._read_reply(self.request_timeout)
            if reply is None:
                self._log("Kokoro worker timed out; restarting it")
                self._stop_locked()
                return False
            if not reply.get("ok"):
                self._log(f"Kokoro synthesis failed: {reply.get('error')}")
                return False
            return os.path.exists(out_path)


__all__ = ["KokoroTTS", "EspeakTTS", "WORKER_SCRIPT", "VOICE_LANGS", "lang_for_voice"]
