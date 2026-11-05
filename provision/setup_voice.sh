#!/usr/bin/env bash
# Install the voice assistant's speech stack on the robot:
#   ~/.robot_face/voice-venv   faster-whisper (ctranslate2) + onnxruntime for its Silero VAD
#   system python              anthropic SDK (the ROS node imports it)
#   ~/.robot_face/voice.env    secrets template (ANTHROPIC_API_KEY, HASS_URL, HASS_TOKEN)
#
# Usage (on the robot, online):   ./setup_voice.sh [whisper_model]     # default base.en
set -euo pipefail

VENV="$HOME/.robot_face/voice-venv"
MODEL="${1:-base.en}"
ENVF="$HOME/.robot_face/voice.env"

mkdir -p "$HOME/.robot_face"
[ -x "$VENV/bin/python" ] || python3 -m venv "$VENV"
"$VENV/bin/pip" install --upgrade pip wheel >/dev/null
"$VENV/bin/pip" install faster-whisper numpy
if [ "$(uname -m)" = "aarch64" ]; then
  # Jetson: NVIDIA's builds from the Jetson AI Lab index — onnxruntime-gpu (Silero VAD) and a CUDA
  # ctranslate2 so Whisper runs on the Orin GPU (then set asr_device: cuda, asr_compute_type: float16).
  # Fallbacks: CPU onnxruntime 1.18.1 (>= 1.20 PyPI CPU builds abort on the Orin) and the PyPI CPU ctranslate2.
  "$VENV/bin/pip" uninstall -y onnxruntime >/dev/null 2>&1 || true
  "$VENV/bin/pip" install "numpy<2" "onnxruntime-gpu==1.24.0" --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 --extra-index-url https://pypi.org/simple \
    || "$VENV/bin/pip" install "onnxruntime==1.18.1" "numpy<2"
  "$VENV/bin/pip" install --force-reinstall --no-deps ctranslate2 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 || true
  "$VENV/bin/python" -c "import ctranslate2 as c; print('ctranslate2', c.__version__, 'CUDA devices:', c.get_cuda_device_count())"
fi
# pre-download the Whisper model so the first start is fast and works offline afterwards
"$VENV/bin/python" - <<PY
from faster_whisper import WhisperModel
import time; t0 = time.time()
m = WhisperModel("$MODEL", device="cpu", compute_type="int8")
print("whisper $MODEL ready in %.1fs" % (time.time() - t0))
PY

# The ROS node (system python) talks to Claude through the official SDK
python3 -m pip install --user --upgrade anthropic >/dev/null && python3 -c "import anthropic; print('anthropic', anthropic.__version__)"

# Tools the skills shell out to
for t in parecord aplay pactl ffplay; do command -v "$t" >/dev/null || echo "WARNING: $t not found (apt: pulseaudio-utils / alsa-utils / ffmpeg)"; done

if [ ! -f "$ENVF" ]; then
  cat > "$ENVF" <<EOF
# Secrets for the robot_voice supervisor unit (sourced by services/robot_voice.conf). chmod 600.
ANTHROPIC_API_KEY=
HASS_URL=
HASS_TOKEN=
EOF
  chmod 600 "$ENVF"
  echo "Wrote $ENVF — add your ANTHROPIC_API_KEY (and Home Assistant URL/token) there."
fi
# Local language model (optional): Ollama answers general questions without an API key. The GPU box's is a
# supervisor unit (provision/gpu/provision_gpu.sh llm) and is tried first; this block is for a copy on RD's own Orin.
if command -v ollama >/dev/null 2>&1; then
  ollama list 2>/dev/null | grep -q "^${OLLAMA_MODEL:-qwen2.5:7b}" || ollama pull "${OLLAMA_MODEL:-qwen2.5:7b}" || true
else
  echo "Optional: install Ollama for a local LLM (GPU):  curl -fsSL https://ollama.com/install.sh | sh && ollama pull qwen2.5:7b"
fi
echo "Voice stack installed. Test the ears:  $VENV/bin/python ~/rd_ws/src/robot_voice/robot_voice/ears_worker.py --list-sources"
