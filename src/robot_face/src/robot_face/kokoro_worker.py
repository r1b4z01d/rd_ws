#!/usr/bin/env python3
"""Kokoro-82M text-to-speech worker.

Runs in its own virtualenv (kokoro-onnx needs numpy 2, which the ROS/MediaPipe
stack can't have), so the face node talks to it over a pipe instead of
importing it. The model (~325 MB) is loaded once at startup.

Protocol: one JSON object per line on stdin, one JSON reply per line on stdout.

    request : {"text": "hello Bryan", "voice": "af_heart", "speed": 1.0, "out": "/tmp/x.wav"}
    reply   : {"ok": true, "path": "/tmp/x.wav", "seconds": 0.9, "sample_rate": 24000}
              {"ok": false, "error": "..."}
    request : {"cmd": "voices"}      -> {"ok": true, "voices": [...]}
    request : {"cmd": "quit"}        -> exits

Run directly for a quick test:
    kokoro-venv/bin/python kokoro_worker.py --model-dir ~/.robot_face/kokoro --say "hello there" --out /tmp/k.wav
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
import wave

import numpy as np

MODEL_FILE = "kokoro-v1.0.onnx"
VOICES_FILE = "voices-v1.0.bin"
DEFAULT_VOICE = "af_heart"


def write_wav(path: str, samples: np.ndarray, rate: int) -> None:
    pcm = (np.clip(np.asarray(samples, dtype=np.float32), -1.0, 1.0) * 32767.0).astype("<i2")
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(int(rate))
        wf.writeframes(pcm.tobytes())


def load(model_dir: str):
    from kokoro_onnx import Kokoro   # imported here so --help works without the venv
    model = os.path.join(model_dir, MODEL_FILE)
    voices = os.path.join(model_dir, VOICES_FILE)
    for p in (model, voices):
        if not os.path.exists(p):
            raise FileNotFoundError(p)
    return Kokoro(model, voices)


def synthesize(kokoro, text: str, voice: str, speed: float, out: str, lang: str = "en-us") -> dict:
    samples, rate = kokoro.create(text, voice=voice, speed=float(speed), lang=lang)
    write_wav(out, samples, rate)
    return {"ok": True, "path": out, "seconds": round(len(samples) / float(rate), 3), "sample_rate": int(rate)}


def serve(kokoro, default_voice: str) -> None:
    out = sys.stdout
    print(json.dumps({"ok": True, "ready": True, "voices": sorted(kokoro.get_voices())}), file=out, flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            cmd = req.get("cmd")
            if cmd == "quit":
                break
            if cmd == "voices":
                reply = {"ok": True, "voices": sorted(kokoro.get_voices())}
            else:
                reply = synthesize(
                    kokoro,
                    text=str(req["text"]),
                    voice=str(req.get("voice") or default_voice),
                    speed=float(req.get("speed", 1.0)),
                    out=str(req["out"]),
                    lang=str(req.get("lang", "en-us")),
                )
        except Exception as e:   # never die on a bad request; the node falls back to espeak
            reply = {"ok": False, "error": f"{type(e).__name__}: {e}"}
        print(json.dumps(reply), file=out, flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model-dir", default=os.path.expanduser("~/.robot_face/kokoro"))
    ap.add_argument("--voice", default=DEFAULT_VOICE)
    ap.add_argument("--say", help="synthesise this text to --out and exit (test mode)")
    ap.add_argument("--out", default="/tmp/kokoro_test.wav")
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()

    t0 = time.perf_counter()
    kokoro = load(args.model_dir)
    load_s = time.perf_counter() - t0

    if args.say:
        t0 = time.perf_counter()
        r = synthesize(kokoro, args.say, args.voice, args.speed, args.out)
        print(f"loaded in {load_s:.1f}s; {r['seconds']}s of audio in {time.perf_counter() - t0:.2f}s -> {r['path']}")
        return 0

    serve(kokoro, args.voice)
    return 0


if __name__ == "__main__":
    sys.exit(main())
