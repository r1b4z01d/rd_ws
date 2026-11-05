#!/usr/bin/env python3
"""Ears worker: microphone → voice activity detection → Whisper transcription.

Runs in its own virtualenv (faster-whisper / ctranslate2 / onnxruntime pins
don't mix with the ROS python). The voice node spawns it and reads one JSON
event per line from stdout; it accepts one JSON command per line on stdin.

Events (stdout):
    {"event": "ready", "model": "base.en", "device": "cpu", "source": "..."}
    {"event": "utterance", "text": "robot disco what's the weather", "seconds": 2.4, "asr_ms": 610, "rms": 0.03}
    {"event": "listening", "on": true}          # speech start/end (for a UI cue)
    {"event": "error", "error": "..."}
Commands (stdin):
    {"cmd": "mute"} / {"cmd": "unmute"}          # drop audio while the robot itself talks
    {"cmd": "transcribe", "path": "/tmp/x.wav"}  # offline test; replies with an utterance event
    {"cmd": "quit"}

Audio capture is a `parecord` (PulseAudio/PipeWire) subprocess producing raw
16 kHz mono s16le, so no Python audio bindings are needed. Segmentation is an
adaptive energy VAD (fast, streaming); each segment is then run through
faster-whisper with its Silero VAD filter, which drops non-speech segments.

Test by hand:
    voice-venv/bin/python ears_worker.py --file hello.wav
    voice-venv/bin/python ears_worker.py --list-sources
    voice-venv/bin/python ears_worker.py --source <pulse source name> --print   # live, prints utterances
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import wave
from collections import deque
from typing import Optional

import numpy as np

RATE = 16000
FRAME_MS = 30
FRAME = RATE * FRAME_MS // 1000          # samples per frame


# ---------------------------------------------------------------------------
# Output helpers (stdout is the event channel; everything else goes to stderr)
# ---------------------------------------------------------------------------
_out_lock = threading.Lock()


def emit(**event) -> None:
    with _out_lock:
        sys.stdout.write(json.dumps(event) + "\n")
        sys.stdout.flush()


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# Audio sources
# ---------------------------------------------------------------------------
def list_sources() -> list:
    try:
        out = subprocess.run(["pactl", "list", "sources", "short"], capture_output=True, text=True, timeout=5).stdout
    except Exception as e:
        log(f"pactl failed: {e}")
        return []
    names = []
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) >= 2 and ".monitor" not in parts[1]:
            names.append(parts[1])
    return names


def pick_source(preferred: str = "") -> str:
    """A pulse source name: the preferred one if present, else a USB mic, else default."""
    names = list_sources()
    if preferred:
        for n in names:
            if preferred in n:
                return n
        log(f"source '{preferred}' not found; available: {names}")
    for n in names:
        if "usb" in n.lower():
            return n
    return names[0] if names else ""


def read_exact(stream, n: int) -> Optional[bytes]:
    """Read exactly ``n`` bytes from a pipe, or None at EOF.

    A raw pipe read returns whatever is available (parecord writes in chunks
    that never line up with our frames), so a single ``read(n)`` is usually
    short — the first live version of this worker dropped every such read and
    never saw a full frame.
    """
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def open_capture(source: str) -> subprocess.Popen:
    cmd = ["parecord", "--raw", f"--format=s16le", f"--rate={RATE}", "--channels=1", "--latency-msec=50"]
    if source:
        cmd.append(f"--device={source}")
    try:
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
    except FileNotFoundError:
        cmd = ["arecord", "-q", "-f", "S16_LE", "-r", str(RATE), "-c", "1", "-t", "raw"]
        return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)


# ---------------------------------------------------------------------------
# Streaming energy VAD
# ---------------------------------------------------------------------------
class Segmenter:
    """Adaptive-threshold energy VAD that yields complete utterances.

    * noise floor = slow EMA of frame RMS while not speaking
    * speech starts after ``start_frames`` consecutive loud frames (> floor*ratio and > min_rms)
    * speech ends after ``end_ms`` of quiet; utterances are capped at ``max_s``
    * ``pre_ms`` of audio before the trigger is kept so the first syllable survives
    """

    def __init__(self, min_rms=0.012, ratio=3.0, start_frames=3, end_ms=700, pre_ms=300,
                 max_s=15.0, min_s=0.4):
        self.min_rms = min_rms
        self.ratio = ratio
        self.start_frames = start_frames
        self.end_frames = max(1, end_ms // FRAME_MS)
        self.pre = deque(maxlen=max(1, pre_ms // FRAME_MS))
        self.max_frames = int(max_s * 1000 / FRAME_MS)
        self.min_frames = int(min_s * 1000 / FRAME_MS)
        self.floor = 0.01
        self.speaking = False
        self.loud_run = 0
        self.quiet_run = 0
        self.buf: list = []
        self.peak_rms = 0.0

    def feed(self, frame: np.ndarray):
        """Return a finished utterance (float32 array) or None. Also sets
        ``self.just_started`` when speech begins (for the listening cue)."""
        self.just_started = False
        rms = float(np.sqrt(np.mean(frame * frame))) if frame.size else 0.0
        thresh = max(self.min_rms, self.floor * self.ratio)
        loud = rms > thresh
        if not self.speaking:
            # track the noise floor only while quiet
            self.floor = 0.97 * self.floor + 0.03 * rms if rms < thresh * 1.5 else self.floor
            self.pre.append(frame)
            self.loud_run = self.loud_run + 1 if loud else 0
            if self.loud_run >= self.start_frames:
                self.speaking = True
                self.just_started = True
                self.buf = list(self.pre)
                self.quiet_run = 0
                self.peak_rms = rms
            return None
        self.buf.append(frame)
        self.peak_rms = max(self.peak_rms, rms)
        self.quiet_run = 0 if loud else self.quiet_run + 1
        if self.quiet_run >= self.end_frames or len(self.buf) >= self.max_frames:
            audio = np.concatenate(self.buf) if self.buf else np.zeros(0, np.float32)
            self.speaking = False
            self.loud_run = 0
            self.buf = []
            if len(audio) < self.min_frames * FRAME:
                return None
            return audio
        return None


# ---------------------------------------------------------------------------
# Whisper
# ---------------------------------------------------------------------------
class Transcriber:
    def __init__(self, model: str, device: str, compute_type: str, threads: int, language: str,
                 initial_prompt: str, beam_size: int):
        from faster_whisper import WhisperModel   # imported late so --help works anywhere
        self.model = WhisperModel(model, device=device, compute_type=compute_type, cpu_threads=threads)
        self.language = language or None
        self.initial_prompt = initial_prompt
        self.beam_size = beam_size

    def transcribe(self, audio: np.ndarray) -> str:
        segments, _info = self.model.transcribe(
            audio,
            language=self.language,
            beam_size=self.beam_size,
            vad_filter=True,
            vad_parameters={"min_silence_duration_ms": 300, "speech_pad_ms": 200},
            condition_on_previous_text=False,
            initial_prompt=self.initial_prompt,
            without_timestamps=True,
        )
        return " ".join(s.text.strip() for s in segments).strip()


def load_wav(path: str) -> np.ndarray:
    with wave.open(path, "rb") as wf:
        rate, ch, width, n = wf.getframerate(), wf.getnchannels(), wf.getsampwidth(), wf.getnframes()
        raw = wf.readframes(n)
    if width != 2:
        raise ValueError("expected 16-bit PCM")
    a = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
    if ch > 1:
        a = a[: (a.size // ch) * ch].reshape(-1, ch).mean(axis=1)
    if rate != RATE:
        idx = np.linspace(0, a.size - 1, int(a.size * RATE / rate)).astype(np.int64)
        a = a[idx]
    return a.astype(np.float32)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
class Ears:
    def __init__(self, args):
        self.args = args
        self.muted = False
        self.transcriber: Optional[Transcriber] = None
        self.seg = Segmenter(min_rms=args.min_rms, ratio=args.ratio, end_ms=args.end_ms, max_s=args.max_seconds)
        self._stop = False

    def load(self):
        t0 = time.perf_counter()
        self.transcriber = Transcriber(self.args.model, self.args.device, self.args.compute_type,
                                       self.args.threads, self.args.language, self.args.initial_prompt,
                                       self.args.beam_size)
        return time.perf_counter() - t0

    def handle_utterance(self, audio: np.ndarray, rms: float):
        t0 = time.perf_counter()
        try:
            text = self.transcriber.transcribe(audio)
        except Exception as e:
            emit(event="error", error=f"asr: {type(e).__name__}: {e}")
            return
        ms = (time.perf_counter() - t0) * 1000.0
        if text:
            emit(event="utterance", text=text, seconds=round(len(audio) / RATE, 2), asr_ms=round(ms), rms=round(rms, 4))
        else:
            emit(event="discarded", seconds=round(len(audio) / RATE, 2), asr_ms=round(ms))

    def command_thread(self):
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                req = json.loads(line)
            except json.JSONDecodeError:
                continue
            cmd = req.get("cmd")
            if cmd == "quit":
                self._stop = True
                break
            elif cmd == "mute":
                self.muted = True
                self.seg.speaking = False
                self.seg.buf = []
            elif cmd == "unmute":
                self.muted = False
            elif cmd == "transcribe":
                try:
                    audio = load_wav(req["path"])
                    self.handle_utterance(audio, float(np.sqrt(np.mean(audio * audio))))
                except Exception as e:
                    emit(event="error", error=f"transcribe: {e}")

    def run_live(self):
        source = pick_source(self.args.source)
        cap = open_capture(source)
        load_s = self.load()
        emit(event="ready", model=self.args.model, device=self.args.device, compute=self.args.compute_type,
             source=source or "default", load_s=round(load_s, 1))
        if self.args.source and self.args.source not in (source or ""):
            emit(event="error", error=f"preferred microphone '{self.args.source}' is not present; "
                                      f"listening on {source or 'default'} until it appears")
        threading.Thread(target=self.command_thread, daemon=True).start()
        nbytes = FRAME * 2
        unmute_guard_until = 0.0
        last_check = time.monotonic()
        lost = False
        while not self._stop:
            # Microphone watchdog: PulseAudio silently moves our stream to another source when
            # the USB mic disappears, so check every few seconds that it is still there and
            # re-open the capture on it when it comes back.
            now = time.monotonic()
            if now - last_check > 5.0:
                last_check = now
                names = list_sources()
                wanted = pick_source(self.args.source) if self.args.source else source
                present = (source in names) if source else True
                if wanted and wanted != source and self.args.source and self.args.source in wanted:
                    # the preferred microphone (re)appeared while we were on a fallback source
                    emit(event="info", info=f"microphone {wanted} is back; capture restarted on it")
                    try:
                        cap.terminate()
                    except Exception:
                        pass
                    source = wanted
                    cap = open_capture(source)
                    lost = False
                elif not present and not lost:
                    lost = True
                    emit(event="error", error=f"microphone {source} disappeared (unplugged or powered off); listening is paused")
                elif present and lost:
                    lost = False
                    emit(event="info", info=f"microphone {source} is back; capture restarted")
                    try:
                        cap.terminate()
                    except Exception:
                        pass
                    cap = open_capture(source)
            data = read_exact(cap.stdout, nbytes)
            if data is None:
                # EOF: the capture process died (device unplugged?) — restart it
                emit(event="error", error="capture process exited; restarting")
                time.sleep(1.0)
                cap = open_capture(source)
                continue
            if self.muted or lost:
                unmute_guard_until = time.monotonic() + self.args.unmute_guard
                continue
            if time.monotonic() < unmute_guard_until:
                continue   # speaker tail / echo right after the robot stopped talking
            frame = np.frombuffer(data, dtype="<i2").astype(np.float32) / 32768.0
            audio = self.seg.feed(frame)
            if self.seg.just_started:
                emit(event="listening", on=True)
            if audio is not None:
                emit(event="listening", on=False)
                rms = self.seg.peak_rms
                # transcribe in a thread so capture never stalls (the pipe would overflow)
                threading.Thread(target=self.handle_utterance, args=(audio, rms), daemon=True).start()
        try:
            cap.terminate()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="base.en", help="faster-whisper model (tiny.en, base.en, small.en, ...)")
    ap.add_argument("--device", default="cpu", help="cpu | cuda")
    ap.add_argument("--compute-type", default="int8", help="int8 (cpu) | float16 (cuda) | int8_float16")
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--language", default="en")
    ap.add_argument("--beam-size", type=int, default=2)
    ap.add_argument("--initial-prompt", default="Robot Disco, turn on the kitchen light. Robot Disco, what's the weather?",
                    help="biases spelling toward the wake phrase and typical commands")
    ap.add_argument("--source", default="", help="pulse source name (substring); default: a USB mic, else default source")
    ap.add_argument("--min-rms", type=float, default=0.012)
    ap.add_argument("--ratio", type=float, default=3.0)
    ap.add_argument("--end-ms", type=int, default=900)
    ap.add_argument("--max-seconds", type=float, default=15.0)
    ap.add_argument("--unmute-guard", type=float, default=0.4, help="seconds to keep ignoring audio after unmute")
    ap.add_argument("--file", help="transcribe this WAV and exit")
    ap.add_argument("--list-sources", action="store_true")
    ap.add_argument("--print", action="store_true", help="live mode but print plain utterances (manual test)")
    args = ap.parse_args()

    if args.list_sources:
        for n in list_sources():
            print(n)
        return 0

    ears = Ears(args)
    if args.file:
        load_s = ears.load()
        audio = load_wav(args.file)
        t0 = time.perf_counter()
        text = ears.transcriber.transcribe(audio)
        print(f"loaded in {load_s:.1f}s; {len(audio)/RATE:.1f}s of audio transcribed in "
              f"{(time.perf_counter()-t0)*1000:.0f} ms:\n  {text!r}")
        return 0

    if args.print:
        # wrap emit so a human can read it
        global emit
        _emit = emit

        def emit(**event):
            if event.get("event") == "utterance":
                print(f"[{event['seconds']}s, {event['asr_ms']} ms] {event['text']}", flush=True)
            else:
                _emit(**event)
    ears.run_live()
    return 0


if __name__ == "__main__":
    sys.exit(main())
