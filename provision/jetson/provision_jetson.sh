#!/usr/bin/env bash
# Provision an NVIDIA Jetson Orin dev kit (JetPack 6.x = Ubuntu 22.04, aarch64) as the Robot Disco robot
# PC, replacing the Intel box. Same role, same user/paths/IP as ../provision.sh, plus a local GPU so the
# VLA inference server (VLA/scripts/infer_server.py) runs on the robot instead of the GPU box.
#
# Flash the dev kit with SDK Manager (JetPack 6.x, Jetson runtime components, user "rd"), then:
#
#   git clone https://github.com/r1b4z01d/rd_ws.git ~/rd_ws
#   ~/rd_ws/provision/jetson/provision_jetson.sh all
#   sudo reboot
#   # copy ~/VLA over (see ../README.md §3), then:
#   ~/rd_ws/provision/jetson/provision_jetson.sh vla infer
#   ~/rd_ws/provision/jetson/provision_jetson.sh network
#
# This file sources ../provision.sh and overrides the phases that differ on Jetson:
#   preflight  aarch64 + L4T/JetPack + CUDA checks
#   base       ../apt-packages.txt + apt-packages-jetson.txt
#   jetson     power mode MAXN, jetson_clocks at boot, nvidia docker runtime, jtop (new phase)
#   vla        venv with NVIDIA's CUDA torch wheels, ur_rtde/moderngl from source, lerobot[smolvla]
#   infer      serve_ckpt symlink + vla_infer_server supervisor unit check (new phase)
#   foxglove   never loads an x86 tarball; builds an arm64 image
# All other phases (ros, groups, udev, sysctl, workspace, face, services, display, network) are inherited.
#
# Extra knobs (see ../provision.sh for the shared ones):
#   JETSON_TORCH=2.8.0 JETSON_TORCHVISION=0.23.0   wheel versions from the Jetson AI Lab index
#   JETSON_PIP_INDEX=https://pypi.jetson-ai-lab.io/jp6/cu126
#   SERVE_CKPT=/path/to/pretrained_model          what outputs/serve_ckpt should point at (phase infer)
#   NVPMODEL=0                                     nvpmodel id (0 = MAXN on Orin dev kits)
set -euo pipefail

JETSON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$JETSON_DIR/../provision.sh"

JETSON_PIP_INDEX="${JETSON_PIP_INDEX:-https://pypi.jetson-ai-lab.io/jp6/cu126}"
JETSON_TORCH="${JETSON_TORCH:-2.8.0}"
JETSON_TORCHVISION="${JETSON_TORCHVISION:-0.23.0}"
NVPMODEL="${NVPMODEL:-0}"
NETPLAN_RENDERER="${NETPLAN_RENDERER:-NetworkManager}"   # JetPack desktops run NetworkManager
PHASES="$PHASES jetson infer"

phase_preflight() {
  log "Preflight (Jetson)"
  [ "$(id -un)" = "$RD_USER" ] || die "run this as user '$RD_USER' (RD_USER=... to override), not $(id -un)"
  sudo -n true 2>/dev/null || sudo -v || die "$RD_USER needs sudo"
  [ "$(uname -m)" = "aarch64" ] || die "this is the Jetson script; $(uname -m) is not aarch64 (use ../provision.sh)"
  [ -f /etc/nv_tegra_release ] || die "/etc/nv_tegra_release missing: not a JetPack image?"
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${VERSION_ID:-}" = "22.04" ] || die "JetPack 6.x (Ubuntu 22.04) required for ROS 2 Humble; this is ${PRETTY_NAME:-unknown}"
  log "L4T: $(head -1 /etc/nv_tegra_release)"
  if [ -x /usr/local/cuda/bin/nvcc ]; then
    log "CUDA: $(/usr/local/cuda/bin/nvcc --version | grep -o 'release [0-9.]*')"
  else
    warn "nvcc not found; phase base installs nvidia-jetpack"
  fi
  setup_proxy
}

# Base packages: the shared list plus the Jetson extras. hostname/ssh/hosts handling is shared.
_shared_phase_base="$(declare -f phase_base)"
eval "${_shared_phase_base/phase_base/shared_phase_base}"
phase_base() {
  shared_phase_base
  log "Jetson extra packages"
  apt_install "$(pkglist "$JETSON_DIR/apt-packages-jetson.txt")"
}

phase_jetson() {
  log "Jetson power mode and clocks"
  # Switching to MAXN on an AGX Orin asks for a reboot interactively, so set the boot default in
  # /etc/nvpmodel.conf instead; it takes effect at the reboot that ends provisioning anyway.
  if [ -f /etc/nvpmodel.conf ]; then
    sudo sed -i -E "s/^< PM_CONFIG DEFAULT=[0-9]+ >/< PM_CONFIG DEFAULT=$NVPMODEL >/" /etc/nvpmodel.conf
    # nvpmodel restores the LAST mode from /var/lib/nvpmodel/status at boot and only falls back to the
    # conf default when that file is absent (verified: DEFAULT=0 alone still booted into MODE_30W).
    printf 'pmode:%04d\n' "$NVPMODEL" | sudo tee /var/lib/nvpmodel/status >/dev/null
    log "nvpmodel boot default -> $(grep -E '^< PM_CONFIG DEFAULT' /etc/nvpmodel.conf) ($(grep -E "POWER_MODEL ID=$NVPMODEL " /etc/nvpmodel.conf | grep -oE 'NAME=[A-Z0-9_]+')); current: $(sudo nvpmodel -q 2>/dev/null | head -1)"
  fi
  if command -v jetson_clocks >/dev/null && [ ! -f /etc/systemd/system/jetson-clocks.service ]; then
    sudo tee /etc/systemd/system/jetson-clocks.service >/dev/null <<'UNIT'
[Unit]
Description=Pin Jetson clocks to max (robot PC runs inference at a fixed control rate)
After=nvpmodel.service

[Service]
Type=oneshot
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
UNIT
    sudo systemctl daemon-reload && sudo systemctl enable --now jetson-clocks.service
  fi

  log "docker: nvidia runtime as default"
  if command -v nvidia-ctk >/dev/null; then
    sudo nvidia-ctk runtime configure --runtime=docker --set-as-default
    sudo systemctl restart docker
  else
    warn "nvidia-ctk missing (nvidia-container-toolkit is part of JetPack runtime components)"
  fi

  # Xbox-protocol gamepads (8BitDo in X-input mode, Xbox pads) need xpad; the Tegra kernel ships it
  # as a module that is not auto-loaded for every ID, and joy_node needs /dev/input/js0 from it.
  log "xpad gamepad driver at boot"
  echo xpad | sudo tee /etc/modules-load.d/xpad.conf >/dev/null
  sudo modprobe xpad || warn "xpad module not available on this kernel"

  log "jtop (jetson-stats) for monitoring"
  sudo python3 -m pip install -U jetson-stats || warn "jetson-stats install failed"
}

# cuSPARSELt is dlopen'ed by NVIDIA's torch wheels and is not part of JetPack. Try apt from NVIDIA's CUDA
# repo first, else the redist tarball into /usr/local/cuda (what pytorch's own CI script does).
install_cusparselt() {
  if ldconfig -p | grep -q libcusparseLt.so || ls /usr/local/cuda/lib64/libcusparseLt.so* >/dev/null 2>&1; then
    log "cuSPARSELt already present"; return 0
  fi
  log "cuSPARSELt"
  if apt-cache show libcusparselt0 >/dev/null 2>&1; then
    apt_install libcusparselt0 libcusparselt-dev && return 0
  fi
  local ver="${CUSPARSELT_VERSION:-0.7.1.0}"
  local base="https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64"
  local name="libcusparse_lt-linux-aarch64-${ver}-archive"
  local tmp; tmp="$(mktemp -d)"
  curl -fL "$base/$name.tar.xz" -o "$tmp/$name.tar.xz" || die "could not download cuSPARSELt $ver; see https://developer.nvidia.com/cusparselt-downloads (Linux / aarch64-jetson)"
  tar xf "$tmp/$name.tar.xz" -C "$tmp"
  sudo cp -a "$tmp/$name/include/." /usr/local/cuda/include/
  sudo cp -a "$tmp/$name/lib/." /usr/local/cuda/lib64/
  echo "/usr/local/cuda/lib64" | sudo tee /etc/ld.so.conf.d/cuda-lib64.conf >/dev/null
  sudo ldconfig
  rm -rf "$tmp"
}

# Override: CUDA torch from NVIDIA's index first, then the rest from PyPI. An x86 snapshot freeze is
# reused minus torch/CUDA/x86-only lines.
vla_pip_install() {
  local pip="$1"
  install_cusparselt
  log "torch $JETSON_TORCH / torchvision $JETSON_TORCHVISION from $JETSON_PIP_INDEX"
  "$pip" install --index-url "$JETSON_PIP_INDEX" "torch==$JETSON_TORCH" "torchvision==$JETSON_TORCHVISION"
  if [ -n "$SNAPSHOT" ] && [ -f "$SNAPSHOT/vla-venv-freeze.txt" ]; then
    log "Reusing the old robot PC's freeze (without torch/CUDA/x86-only pins)"
    local req; req="$(mktemp)"
    grep -vE '^(torch|torchvision|torchaudio|torchcodec|triton|nvidia-|intel-|mkl|ur_rtde|moderngl|lerobot)' \
      "$SNAPSHOT/vla-venv-freeze.txt" > "$req" || true
    "$pip" install -r "$req" || warn "some pins from the snapshot failed on aarch64; continuing with pip-vla-jetson.txt"
    rm -f "$req"
  fi
  log "VLA stack (ur_rtde and moderngl compile from source here: several minutes)"
  "$pip" install -r "$JETSON_DIR/pip-vla-jetson.txt"
  # NVIDIA's torch wheel is compiled against NumPy 1.x: with numpy 2 torch prints "Failed to initialize
  # NumPy" and from_numpy()/.numpy() break, which lerobot uses on every observation. lerobot's rerun-sdk
  # and opencv-python 4.12 declare numpy>=2, which is why this cannot go in the requirements file (pip
  # calls it unresolvable), but extensions built against numpy 2 run fine on 1.26 at runtime, so the
  # downgrade goes in as a post-step; `pip check` will grumble about those two, nothing else.
  log "numpy<2 for NVIDIA's torch ABI (pip warns about rerun-sdk/opencv metadata; harmless)"
  "$pip" install "numpy<2"
  "$(dirname "$pip")/python" - <<'PY' || die "torch has no CUDA; check cuSPARSELt/ldconfig and that the wheel is not a +cpu build"
import warnings; warnings.simplefilter("error")   # "Failed to initialize NumPy" is only a warning
import numpy as np, torch, lerobot
print("torch", torch.__version__, "cuda", torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else "-")
print("lerobot", lerobot.__version__, "numpy", np.__version__)
assert torch.cuda.is_available()
assert torch.from_numpy(np.ones(2, dtype=np.float32)).sum().item() == 2.0
PY
}

phase_infer() {
  local vla="$RD_HOME/VLA"
  log "Local VLA inference server"
  [ -x "$vla/.venv/bin/python" ] || die "run 'provision_jetson.sh vla' first"
  [ -f "$vla/services/rd/vla_infer_server.conf" ] \
    || die "$vla/services/rd/vla_infer_server.conf missing: copy a VLA tree that has it (it is in the VLA repo)"
  # services/rd/ also holds the MANUAL-start apps as units: vla_teleop (recording panel) and vla_eval
  # (policy eval), each reading its arguments from ~/VLA/config/{teleop,eval}.env — install_services.sh rd
  # creates those from the .example files on first install and never overwrites them. Start them with
  # `sudo supervisorctl start vla_teleop|vla_eval` or the ▶ buttons on the manager's RobotDisco Teleop
  # tab (which needs the GPU box's ssh key authorized for rd@<this box>). They need the X display of
  # rd's autologin session (DISPLAY=:0, XAUTHORITY=/run/user/1000/gdm/Xauthority), as the units declare.
  mkdir -p "$vla/outputs" "$vla/log"
  if [ -n "${SERVE_CKPT:-}" ]; then
    [ -d "$SERVE_CKPT" ] || die "SERVE_CKPT=$SERVE_CKPT is not a directory"
    ln -sfn "$SERVE_CKPT" "$vla/outputs/serve_ckpt"
  fi
  if [ ! -e "$vla/outputs/serve_ckpt" ]; then
    warn "outputs/serve_ckpt is not set. Copy a checkpoint from the GPU box and point the symlink at it:
  rsync -a gpu:VLA/outputs/train/<run>/checkpoints/last/ ~/VLA/outputs/train/<run>/checkpoints/last/
  ln -sfn ~/VLA/outputs/train/<run>/checkpoints/last/pretrained_model ~/VLA/outputs/serve_ckpt"
  fi
  # SmolVLA checkpoints only hold the action expert; the base VLM (config.json: vlm_model_name) is
  # loaded from the HuggingFace cache, and the unit runs with HF_HUB_OFFLINE=1 because the robot
  # subnet has no internet. Fill the cache now while this box is online (or copy
  # ~/.cache/huggingface/hub/models--HuggingFaceTB--* from the GPU box).
  if [ -f "$vla/outputs/serve_ckpt/config.json" ]; then
    local vlm
    vlm="$("$vla/.venv/bin/python" -c "import json,sys; print(json.load(open(sys.argv[1])).get('vlm_model_name',''))" "$vla/outputs/serve_ckpt/config.json")"
    if [ -n "$vlm" ]; then
      log "HuggingFace cache: $vlm"
      (cd "$vla" && HF_HUB_OFFLINE=0 PYTHONNOUSERSITE=1 .venv/bin/python -c "from huggingface_hub import snapshot_download; print(snapshot_download('$vlm', ignore_patterns=['onnx/*', '*.onnx', '*.gguf']))") \
        || warn "could not fetch $vlm (offline?); copy it from the GPU box's ~/.cache/huggingface/hub before starting vla_infer_server"
    fi
  fi
  (cd "$vla" && sudo ./scripts/install_services.sh rd)
  sudo supervisorctl status vla_infer_server || true
  log "Start it with: sudo supervisorctl start vla_infer_server
Then on this same box:  ~/VLA/run.sh scripts/eval_hw.py --remote 127.0.0.1:8777 --cameras udp --fps 12"
}

phase_foxglove() {
  log "Foxglove Studio docker image (arm64 build; an x86 tarball from the old PC will not run here)"
  if [ -n "${FOXGLOVE_IMAGE_TAR:-}" ]; then
    sudo docker load -i "$FOXGLOVE_IMAGE_TAR"
  elif ! sudo docker image inspect "$FOXGLOVE_TAG" >/dev/null 2>&1; then
    log "Building from foxglove_studio/Dockerfile (needs internet; node:16 + yarn on an Orin: 30+ min)"
    sudo docker build -t "$FOXGLOVE_TAG" "$WS/foxglove_studio"
  fi
  local arch
  arch="$(sudo docker image inspect --format '{{.Architecture}}' "$FOXGLOVE_TAG" 2>/dev/null || true)"
  [ "$arch" = "arm64" ] || warn "foxglove image architecture is '${arch:-none}', expected arm64"
  local conf_image
  conf_image="$(grep -oE '[A-Za-z0-9_./-]+/foxglove' "$WS/services/robot_disco_foxglove_studio.conf" | head -1)"
  if [ -n "$conf_image" ] && [ "$conf_image" != "$FOXGLOVE_TAG" ]; then
    sudo docker tag "$FOXGLOVE_TAG" "$conf_image" 2>/dev/null \
      || warn "services/robot_disco_foxglove_studio.conf references '$conf_image' (docker needs lowercase); use '$FOXGLOVE_TAG'"
  fi
}

phase_all() {
  for p in preflight base jetson ros groups udev sysctl workspace face services display foxglove; do "phase_$p"; done
  log "Done. Next: reboot, copy ~/VLA, then 'provision_jetson.sh vla infer' and finally 'provision_jetson.sh network'."
}

dispatch "$@"
