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
    # Jetson: prefer NVIDIA's onnxruntime-gpu (CUDA) from the Jetson AI Lab index — Kokoro then runs on the
    # GPU (the face node pins ONNX_PROVIDER=CUDAExecutionProvider for the worker). Fallback: CPU 1.18.1,
    # the newest PyPI CPU build that does not abort on the Orin (cpuinfo vector assertion in >= 1.20).
    "$VENV/bin/pip" uninstall -y onnxruntime >/dev/null 2>&1 || true
    "$VENV/bin/pip" install "onnxruntime-gpu==1.24.0" --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 --extra-index-url https://pypi.org/simple \
      || "$VENV/bin/pip" install "onnxruntime==1.18.1"
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
