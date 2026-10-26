#!/usr/bin/env bash
# Install Kokoro-82M text-to-speech for robot_face in its own virtualenv.
#
# kokoro-onnx needs numpy 2, which the ROS 2 / MediaPipe stack on the robot can't
# have, so it lives in ~/.robot_face/kokoro-venv and the face node drives it as a
# worker process (robot_face/kokoro_worker.py).
#
# Usage (on the robot):
#   ./setup_kokoro.sh                      # online: pip + model download from GitHub
#   ./setup_kokoro.sh /path/to/wheels      # offline: wheels dir prepared with
#                                          #   pip download kokoro-onnx -d wheels \
#                                          #     --platform manylinux2014_x86_64 --platform manylinux_2_17_x86_64 \
#                                          #     --platform manylinux_2_28_x86_64 --platform any \
#                                          #     --python-version 3.10 --implementation cp --abi cp310 --abi none \
#                                          #     --only-binary=:all:
#                                          # and the two model files copied into ~/.robot_face/kokoro/
set -euo pipefail

VENV="$HOME/.robot_face/kokoro-venv"
MODELS="$HOME/.robot_face/kokoro"
WHEELS="${1:-}"
RELEASE="https://github.com/thewh1teagle/kokoro-onnx/releases/download/model-files-v1.0"

mkdir -p "$MODELS"
if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv "$VENV"
fi

if [ -n "$WHEELS" ]; then
  "$VENV/bin/pip" install --no-index --find-links "$WHEELS" kokoro-onnx
else
  "$VENV/bin/pip" install --upgrade pip
  "$VENV/bin/pip" install kokoro-onnx
  if [ "$(uname -m)" = "aarch64" ]; then
    # onnxruntime >= 1.20 aborts on Jetson Orin: 1.21+ at import, 1.20.1 when a session runs
    # (cpuinfo vector assertion; the Orin's CPU topology with offline cores trips it). 1.18.1 is the
    # newest that synthesises there (verified 2026-10-10, JetPack 6.2 / L4T r36.5, MODE_30W). It is
    # below kokoro-onnx's declared minimum, which only pip complains about.
    "$VENV/bin/pip" install "onnxruntime==1.18.1"
  fi
  for f in kokoro-v1.0.onnx voices-v1.0.bin; do
    [ -s "$MODELS/$f" ] || curl -L -o "$MODELS/$f" "$RELEASE/$f"
  done
fi

for f in kokoro-v1.0.onnx voices-v1.0.bin; do
  [ -s "$MODELS/$f" ] || { echo "missing $MODELS/$f (copy it there, or run online)"; exit 1; }
done

"$VENV/bin/python" -c "import kokoro_onnx, numpy, onnxruntime; print('kokoro-onnx OK; numpy', numpy.__version__, 'onnxruntime', onnxruntime.__version__)"
echo "Kokoro installed. The face node picks it up automatically (tts_engine: auto)."
echo "Quick test: $VENV/bin/python ~/rd_ws/install/robot_face/lib/python3.10/site-packages/robot_face/kokoro_worker.py --say 'hello there' && aplay /tmp/kokoro_test.wav"
