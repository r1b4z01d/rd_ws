#!/usr/bin/env bash
# Install the SCRFD + ArcFace face-recognition back-end (InsightFace) for robot_face in its own venv.
#
# The face node prefers this back-end automatically (face_id_backend: auto) when
# ~/.robot_face/faceid-venv exists; otherwise it falls back to OpenCV's YuNet + SFace.
#
# Usage (on the robot, online):   ./setup_faceid.sh
# The buffalo_l model pack (~280 MB) is downloaded by insightface on first use into ~/.insightface/models.
set -euo pipefail

VENV="$HOME/.robot_face/faceid-venv"
WORKER="$HOME/rd_ws/src/robot_face/src/robot_face/faceid_worker.py"

mkdir -p "$HOME/.robot_face"
[ -x "$VENV/bin/python" ] || python3 -m venv "$VENV"
"$VENV/bin/pip" install --upgrade pip wheel >/dev/null
if [ "$(uname -m)" = "aarch64" ]; then
  # Jetson: NVIDIA's own onnxruntime-gpu build (CUDA + TensorRT providers) from the Jetson AI Lab index.
  # Verified on the AGX Orin / JetPack 6.2 (1.24.0): SCRFD 6 ms + ArcFace 8.5 ms on the GPU.
  # (The PyPI CPU builds >= 1.20 abort on the Orin with a cpuinfo assertion; 1.18.1 is the CPU fallback.)
  "$VENV/bin/pip" install "numpy<2" opencv-python-headless insightface
  "$VENV/bin/pip" uninstall -y onnxruntime >/dev/null 2>&1 || true
  "$VENV/bin/pip" install "onnxruntime-gpu==1.24.0" --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 --extra-index-url https://pypi.org/simple \
    || "$VENV/bin/pip" install "onnxruntime==1.18.1"
else
  "$VENV/bin/pip" install "numpy<2" onnxruntime opencv-python-headless insightface
fi
"$VENV/bin/python" -c "import insightface, onnxruntime, numpy; print('insightface', insightface.__version__, 'onnxruntime', onnxruntime.__version__, 'numpy', numpy.__version__)"

# First run downloads the model pack and proves the worker starts
if [ -f "$WORKER" ]; then
  "$VENV/bin/python" - <<PY
import json, subprocess, sys, time
t0 = time.time()
p = subprocess.Popen(["$VENV/bin/python", "$WORKER", "--det-size", "640", "--gpu"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
line = p.stdout.readline().decode().strip()
p.stdin.write(b'{"cmd": "quit"}\n'); p.stdin.flush(); p.wait(timeout=10)
print("worker ready in %.1fs: %s" % (time.time() - t0, line[:100]))
PY
fi
echo "Face ID (ArcFace) installed. The face node will use it on its next start (log line 'Face ID ready: SCRFD + ArcFace')."
echo "Note: it has its own gallery (~/.robot_face/people.arcface.json), so people need to be taught again once."
