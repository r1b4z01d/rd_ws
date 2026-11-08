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
#   # push the VLA tree (workstation: rd_ws/provision/deploy_to_rd.sh --vla, see ../README.md and the wiki page Provisioning-and-Deploy), then:
#   ~/rd_ws/provision/jetson/provision_jetson.sh vla infer
#   ~/rd_ws/provision/jetson/provision_jetson.sh network
#
# This file sources ../provision.sh and overrides the phases that differ on Jetson:
#   preflight  aarch64 + L4T/JetPack + CUDA checks
#   base       ../apt-packages.txt + apt-packages-jetson.txt
#   jetson     power mode MAXN, jetson_clocks at boot, nvidia docker runtime, jtop (new phase)
#   data       optional second drive (NVMe labelled 'data'): mount it, put ~/rd_ws (VLA inside it) and the
#              big caches on it behind symlinks, order supervisor after the mount (new phase)
#   vla        venv with NVIDIA's CUDA torch wheels, ur_rtde/moderngl from source, lerobot[smolvla]
#   infer      serve_ckpt symlink + vla_infer_server supervisor unit check (new phase)
#   vlamove    one-time: a robot set up with ~/VLA beside ~/rd_ws gets the submodule layout, ~/rd_ws/VLA (new phase)
#   vlacheck   audit: anything on the box still naming the old ~/VLA (new phase)
#   foxglove   never loads an x86 tarball; builds an arm64 image
# All other phases (ros, groups, udev, sysctl, workspace, face, audio, services, llm, display, network) are
# inherited unchanged; "all" = preflight base jetson data ros groups udev sysctl workspace face audio services llm display foxglove.
#
# Extra knobs (see ../provision.sh for the shared ones):
#   JETSON_TORCH=2.8.0 JETSON_TORCHVISION=0.23.0   wheel versions from the Jetson AI Lab index
#   JETSON_PIP_INDEX=https://pypi.jetson-ai-lab.io/jp6/cu126
#   CUSPARSELT_VERSION=0.7.1.0                     cuSPARSELt redist tarball installed before torch (phase vla)
#   NETPLAN_RENDERER=NetworkManager                (phase network; the Jetson desktop runs NetworkManager, so the
#   NET_GATEWAY=192.168.11.1 NET_DNS=8.8.8.8         address is set with nmcli and these two apply)
#   SERVE_CKPT=/path/to/pretrained_model          what outputs/serve_ckpt should point at (phase infer)
#   NVPMODEL=0                                     nvpmodel id (0 = MAXN on Orin dev kits)
#   DATA_LABEL=data DATA_MOUNT=/mnt/data           filesystem label + mount point of the data drive (phase data)
set -euo pipefail

JETSON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$JETSON_DIR/../provision.sh"

JETSON_PIP_INDEX="${JETSON_PIP_INDEX:-https://pypi.jetson-ai-lab.io/jp6/cu126}"
JETSON_TORCH="${JETSON_TORCH:-2.8.0}"
JETSON_TORCHVISION="${JETSON_TORCHVISION:-0.23.0}"
NVPMODEL="${NVPMODEL:-0}"
DATA_LABEL="${DATA_LABEL:-data}"
DATA_MOUNT="${DATA_MOUNT:-/mnt/data}"
NETPLAN_RENDERER="${NETPLAN_RENDERER:-NetworkManager}"   # JetPack desktops run NetworkManager
PHASES="$PHASES jetson data infer vlamove vlacheck"

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

  # The RoboClaw motor controller talks packet serial on the 40-pin header UART (pins 8 TX / 10 RX /
  # 6 GND = ttyTHS1 = serial@3100000). On JetPack 6 that UART's receive DMA hands back zeroed bytes
  # (NVIDIA forum "DMA on /dev/ttyTHS1 corrupts receiving data"); ttyTHS2 ships without DMA bindings
  # and is fine, so boot a copy of the stock device tree with the two DMA properties removed, via an
  # FDT line in extlinux.conf (the mechanism jetson-io uses). Takes effect at the next reboot.
  log "ttyTHS1 (header UART, RoboClaw link) in PIO mode"
  uart1_pio_dtb
}

# Idempotent: rebuilds the PIO copy when the stock DTB is newer (kernel/dtb package update), leaves an
# extlinux.conf that already has an FDT line (jetson-io) alone, keeps the stock entry as a boot-menu fallback.
uart1_pio_dtb() {
  local base pio conf=/boot/extlinux/extlinux.conf
  base=$(ls /boot/dtb/kernel_tegra234-*.dtb 2>/dev/null | grep -v -- '-uart1-pio' | head -1)
  [ -n "$base" ] || { warn "no /boot/dtb/kernel_tegra234-*.dtb; skipping the ttyTHS1 PIO device tree"; return; }
  pio="${base%.dtb}-uart1-pio.dtb"
  if [ ! -f "$pio" ] || [ "$base" -nt "$pio" ]; then
    sudo cp "$base" "$pio"
    sudo fdtput -d "$pio" /bus@0/serial@3100000 dma-names
    sudo fdtput -d "$pio" /bus@0/serial@3100000 dmas
    log "built $pio (stock tree minus serial@3100000 dmas/dma-names)"
  fi
  if grep -q "FDT $pio" "$conf"; then
    log "extlinux.conf already boots $pio"
  elif grep -qE '^\s*FDT ' "$conf"; then
    warn "extlinux.conf already has an FDT line (jetson-io?): not touched; drop dmas/dma-names from serial@3100000 in that DTB instead"
  else
    sudo cp -n "$conf" "$conf.pre-uart1-pio"
    sudo python3 - "$conf" "$pio" <<'PY'
import re, sys
conf, pio = sys.argv[1], sys.argv[2]
s = open(conf).read()
m = re.search(r"^(\s*)LINUX /boot/Image\n", s, re.M)
s = s[:m.end()] + m.group(1) + "FDT " + pio + "\n" + s[m.end():]
append = re.search(r"^\s*APPEND .*$", s, re.M).group(0)
s = s.rstrip("\n") + "\n\nLABEL stock\n      MENU LABEL stock device tree (ttyTHS1 with DMA)\n      LINUX /boot/Image\n      INITRD /boot/initrd\n" + append + "\n"
open(conf, "w").write(s)
PY
    log "extlinux.conf: primary entry boots $pio ('stock' entry kept, backup $conf.pre-uart1-pio); reboot to apply"
  fi
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

# ---- phase data: an optional second drive holds everything big ------------------------------------
# The AGX Orin's 64 GB eMMC filled up with datasets, checkpoints and the HF cache (2026-10-26), so a
# 512 GB NVMe (ext4, label 'data') was fitted. ~/rd_ws (VLA inside it, as rd_ws/VLA) and the two large caches live on it
# behind SYMLINKS, so every path in this kit, the supervisor confs, the venv shebangs
# (#!/home/rd/rd_ws/VLA/.venv/bin/python3), the colcon install prefix and the workstation's rsync targets
# stay /home/rd/... . The fstab entry is nofail (the robot must still boot without the drive), which
# also means systemd does not order anything after it: a drop-in makes supervisor wait for the mount,
# otherwise its autostart units would start into empty symlink targets. Idempotent: re-running on a
# migrated box reports "(already)" and changes nothing. Without such a drive the phase is a no-op.
data_mount() {
  local dev uuid
  if findmnt -rn "$DATA_MOUNT" >/dev/null 2>&1; then
    log "$DATA_MOUNT is mounted ($(findmnt -rno SOURCE,FSTYPE "$DATA_MOUNT"))"
  else
    dev="$(sudo blkid -L "$DATA_LABEL" 2>/dev/null || true)"
    [ -n "$dev" ] || return 1
    uuid="$(sudo blkid -s UUID -o value "$dev")"
    sudo mkdir -p "$DATA_MOUNT"
    if ! findmnt --fstab -rn "$DATA_MOUNT" >/dev/null 2>&1; then
      echo "UUID=$uuid $DATA_MOUNT ext4 defaults,nofail,x-systemd.device-timeout=10 0 2" | sudo tee -a /etc/fstab >/dev/null
      log "fstab: UUID=$uuid -> $DATA_MOUNT (nofail)"
    fi
    sudo systemctl daemon-reload
    sudo mount "$DATA_MOUNT"
    log "mounted $dev at $DATA_MOUNT"
  fi
  sudo chmod 755 "$DATA_MOUNT"   # GNOME Disks formats the root 700
}

# pids whose cwd, exe or an open file is inside $1 — a REAL path, /proc links are resolved (supervisor units,
# the foxglove docker bind); this script and its subshell are excluded (bash keeps its script file open).
tree_users() {
  sudo find /proc -maxdepth 3 -regex '/proc/[0-9]+/\(cwd\|exe\|fd/[0-9]+\)' \( -lname "$1" -o -lname "$1/*" \) 2>/dev/null \
    | cut -d/ -f3 | grep -vxE "$$|$PPID|$BASHPID" | sort -un | xargs || true
}

# Put one directory on the data drive behind a symlink. $1 = path in the home, $2 = target on the drive.
# Already a symlink: verify. Absent: create the target and the link (a fresh box, or a re-provision of
# a box whose drive already holds the tree). A real directory: copy (rsync -aHAX), checksum-verify,
# swap, delete the original — refused while any process runs from it or if the target is non-empty.
data_relocate() {
  local src="$1" dst="$2" n users
  if [ -L "$src" ]; then
    [ "$(readlink -f "$src")" = "$dst" ] || die "$src is a symlink to $(readlink "$src"), expected $dst: fix by hand"
    log "$src -> $dst (already)"
    return 0
  fi
  if [ ! -e "$src" ]; then
    sudo mkdir -p "$dst" && sudo chown "$RD_USER:$RD_USER" "$dst"
    mkdir -p "$(dirname "$src")" && ln -s "$dst" "$src"
    log "$src -> $dst (new)"
    return 0
  fi
  [ -d "$src" ] || die "$src exists and is not a directory"
  if [ -e "$dst" ] && [ -n "$(sudo ls -A "$dst")" ]; then
    die "$src is a real directory and $dst already has content: decide which copy is current and move the other aside, then re-run"
  fi
  users="$(tree_users "$src")"
  [ -z "$users" ] || die "$src is in use by pid(s) $users ($(ps -o comm= -p "${users// /,}" 2>/dev/null | sort -u | xargs)): stop those units (sudo supervisorctl stop <unit>) and re-run"
  log "moving $src -> $dst ($(du -sh "$src" | cut -f1))"
  sudo mkdir -p "$dst"
  sudo rsync -aHAX --numeric-ids --delete "$src/" "$dst/"
  n="$(sudo rsync -aHAXc --numeric-ids --delete --dry-run --itemize-changes "$src/" "$dst/" | grep -c '^[<>ch*]' || true)"
  [ "$n" = 0 ] || die "$dst differs from $src after the copy ($n entries): nothing was swapped"
  mv "$src" "$src.pre-$DATA_LABEL" && ln -s "$dst" "$src"
  sudo rm -rf "$src.pre-$DATA_LABEL"
  log "$src -> $dst (moved, verified, original removed)"
}

data_dropin() {
  local d=/etc/systemd/system/supervisor.service.d f want
  f="$d/nvme-data.conf"
  want="[Unit]
# ~/rd_ws (VLA inside it) lives on $DATA_MOUNT (rd_ws/provision/jetson/provision_jetson.sh data); the fstab entry is
# nofail, so order supervisor after the mount explicitly or its units start into empty symlink targets
RequiresMountsFor=$DATA_MOUNT
"
  if [ "$(sudo cat "$f" 2>/dev/null)" != "$(printf '%s' "$want")" ]; then
    sudo mkdir -p "$d"
    printf '%s' "$want" | sudo tee "$f" >/dev/null
    sudo systemctl daemon-reload
    log "supervisor drop-in $f"
  fi
}

phase_data() {
  log "Data drive (label '$DATA_LABEL' -> $DATA_MOUNT)"
  cd "$RD_HOME"   # not inside a tree about to move
  if ! data_mount; then
    warn "no filesystem labelled '$DATA_LABEL' and nothing mounted at $DATA_MOUNT: everything stays on the system disk
  (fit a drive, 'sudo mkfs.ext4 -L $DATA_LABEL /dev/nvme0n1p1', then 'provision_jetson.sh data')"
    return 0
  fi
  data_relocate "$WS" "$DATA_MOUNT/rd_ws"                                                      # VLA rides inside
  data_relocate "$RD_HOME/.cache/huggingface" "$DATA_MOUNT/cache/huggingface"               # base VLM + Arrow copies
  data_relocate "$RD_HOME/.cache/robot_descriptions" "$DATA_MOUNT/cache/robot_descriptions" # robot meshes
  data_dropin
  df -h / "$DATA_MOUNT" | sed 's/^/  /'
}

phase_infer() {
  local vla="$WS/VLA"
  log "Local VLA inference server"
  [ -x "$vla/.venv/bin/python" ] || die "run 'provision_jetson.sh vla' first"
  [ -f "$vla/services/rd/vla_infer_server.conf" ] \
    || die "$vla/services/rd/vla_infer_server.conf missing: copy a VLA tree that has it (it is in the VLA repo)"
  # services/rd/ also holds the MANUAL-start apps as units: vla_teleop (recording panel) and vla_eval
  # (policy eval), each reading its arguments from ~/rd_ws/VLA/config/{teleop,eval}.env — install_services.sh rd
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
    warn "outputs/serve_ckpt is not set. Ship a checkpoint from the GPU box (this box cannot reach it: relay
  through the workstation, pretrained_model only) and point the symlink at it:
  # workstation:
  ssh GPU 'cd VLA/outputs/train && tar cf - <run>/capture.json <run>/checkpoints/<step>/pretrained_model' \\
    | ssh RD 'tar xf - -C rd_ws/VLA/outputs/train'
  # here:
  ln -sfn ~/rd_ws/VLA/outputs/train/<run>/checkpoints/<step>/pretrained_model ~/rd_ws/VLA/outputs/serve_ckpt"
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
Then on this same box:  ~/rd_ws/VLA/run.sh scripts/eval_hw.py --remote 127.0.0.1:8777 --cameras udp --fps 12"
}

# ---- phase vlamove: the VLA tree becomes rd_ws/VLA (one-time, VLA is a submodule of rd_ws since 2026-11-02) --
# Until then the robot had two trees, ~/VLA and ~/rd_ws (both symlinks onto the data drive). VLA is now the
# rd_ws/VLA submodule, so the robot's copy moves to ~/rd_ws/VLA and ~/VLA goes away — no compatibility link.
# One rename on the NVMe (same filesystem) keeps the 8 GB of outputs in place. The venv cannot be rebuilt
# here (no internet), so its absolute shebangs are patched in place. The unit files in the tree must already
# say /home/rd/rd_ws/VLA: push the rewritten tree first (for this one-time move, into the OLD ~/VLA:
# `rsync -a --exclude .git --exclude '.venv*' --exclude outputs --exclude log --exclude config VLA/ RD:VLA/`
# from rd_ws on the workstation). Idempotent: a second run reports "(already)" and only re-audits.
VLA_UNITS="vla_cameras vla_camera_ros_bridge vla_infer_server vla_webui vla_teleop vla_eval"

phase_vlamove() {
  local old="$RD_HOME/VLA" new="$WS/VLA" src="" new_real venv u users t stale
  log "VLA tree -> $new (rd_ws submodule layout, no compatibility link)"
  cd "$RD_HOME"                                   # never inside a tree about to move
  [ -d "$WS" ] || die "$WS is missing"
  new_real="$(readlink -f "$WS")/VLA"
  # 1. where it is: the live layout, a run that died after unlinking ~/VLA, or already done
  if [ -L "$old" ] || [ -d "$old" ]; then src="$(readlink -f "$old")"
  elif [ -d "$DATA_MOUNT/VLA" ]; then src="$DATA_MOUNT/VLA"; fi
  if [ -z "$src" ]; then
    [ -d "$new/ur5e_lerobot" ] || die "no VLA tree at $old, $DATA_MOUNT/VLA or $new"
    log "$new (already)"
  else
    # 2. guards
    [ -d "$src/ur5e_lerobot" ] || die "$src does not look like the VLA tree"
    [ "$src" != "$new_real" ] || die "$old already points at $new_real: just remove the link"
    if [ -e "$new_real" ] && [ -n "$(ls -A "$new_real")" ]; then
      die "$new_real exists and is not empty: decide which copy is current, move the other aside, re-run"
    fi
    [ "$(stat -c %d "$src")" = "$(stat -c %d "$(dirname "$new_real")")" ] \
      || die "$src and $(dirname "$new_real") are on different filesystems (mv would copy $(du -sh "$src" | cut -f1)): rsync by hand"
    stale="$(grep -L "/home/$RD_USER/rd_ws/VLA" "$src"/services/rd/*.conf 2>/dev/null | xargs)"
    [ -z "$stale" ] || die "unit files still carry the old path (push the rewritten VLA tree first): $stale"
    # 3. nothing may run from the tree
    for u in $VLA_UNITS; do sudo supervisorctl stop "$u" >/dev/null 2>&1 || true; done
    users="$(tree_users "$src")"
    [ -z "$users" ] || die "$src is in use by pid(s) $users ($(ps -o comm= -p "${users// /,}" 2>/dev/null | sort -u | xargs)): stop them and re-run"
    # 4. the installed confs, for a rollback
    if [ ! -d /etc/supervisor/conf.d.pre-vlamove ]; then
      sudo mkdir -p /etc/supervisor/conf.d.pre-vlamove
      sudo cp /etc/supervisor/conf.d/vla_*.conf /etc/supervisor/conf.d.pre-vlamove/ 2>/dev/null || true
    fi
    # 5. the move: one rename (-T: never INTO an existing directory)
    [ -d "$new_real" ] && sudo rmdir "$new_real"   # an empty placeholder only
    sudo mv -T "$src" "$new_real"
    [ -L "$old" ] && rm "$old"
    log "moved $src -> $new_real ($old removed)"
  fi
  # 6. the venv: the console scripts' shebang lines, the activate scripts, pyvenv.cfg
  venv="$new/.venv"
  [ -x "$venv/bin/python" ] || die "$venv missing: run 'provision_jetson.sh vla'"
  find "$venv/bin" -maxdepth 1 -type f -print0 \
    | xargs -0 grep -lIZ -- "^#!$old/\.venv/bin/" 2>/dev/null \
    | xargs -0 -r sed -i "1s|^#!$old/\.venv/bin/|#!$new/.venv/bin/|"
  for t in bin/activate bin/activate.csh bin/activate.fish bin/Activate.ps1 pyvenv.cfg; do
    [ -f "$venv/$t" ] && sed -i "s|$old/\.venv|$new/.venv|g" "$venv/$t"
  done
  "$venv/bin/pip" --version >/dev/null || die "console-script shebang still broken: $(head -1 "$venv/bin/pip")"
  grep -rlI --include='*.pth' --include='*.egg-link' --include='direct_url.json' --include='__editable__*' -- "$old/" "$venv/lib" 2>/dev/null \
    | sed 's/^/  still holds the old path (an editable install?): /' || true
  # 7. absolute links inside the tree: serve_ckpt becomes relative so it never breaks again
  t="$(readlink "$new/outputs/serve_ckpt" 2>/dev/null || true)"
  case "$t" in
    "$old"/outputs/*|"$src"/outputs/*)   # the link may hold the resolved (/mnt/data) path as well as ~/VLA
      t="${t#"$old"/outputs/}"; t="${t#"$src"/outputs/}"
      ln -sfn "$t" "$new/outputs/serve_ckpt"; log "serve_ckpt -> $t (relative)" ;;
    "$old"/*)         ln -sfn "$new/${t#"$old"/}" "$new/outputs/serve_ckpt" ;;
  esac
  [ -z "$t" ] || [ -e "$new/outputs/serve_ckpt" ] || warn "outputs/serve_ckpt -> $(readlink "$new/outputs/serve_ckpt") dangles"
  find "$new" -path "$venv" -prune -o -type l \( -lname "$old/*" -o -lname "$src/*" \) -print 2>/dev/null | sed 's/^/  stale link: /' || true
  # 8. the units from the rewritten confs: update autostarts vla_cameras + vla_camera_ros_bridge from the new path
  (cd "$new" && sudo ./scripts/install_services.sh rd)
  sudo supervisorctl status | grep -E '^vla_' || true
  log "manual units stay stopped: sudo supervisorctl start vla_infer_server | vla_teleop | vla_eval"
  # 9. the rd_ws units that hold the path (robot_voice's vla_dir, the face's home-pose file, path.py's VLA_DIR)
  #    read it from their rewritten sources on restart
  for u in robot_voice robot_face robot_disco_route_editor; do
    sudo supervisorctl status "$u" 2>/dev/null | grep -q RUNNING && { sudo supervisorctl restart "$u" | tail -1 || true; }
  done
  phase_vlacheck
}

# Audit: everything on this box that still names the old tree. Prints the offenders; nothing = clean.
phase_vlacheck() {
  local pat="(/home/$RD_USER|~|\\\$HOME)/VLA([^[:alnum:]_.-]|\$)" new="$WS/VLA"
  log "old-path audit: $pat"
  {
    sudo grep -nE "$pat" /etc/supervisor/conf.d/*.conf /etc/systemd/system/supervisor.service.d/*.conf 2>/dev/null
    grep -rIlE "$pat" "$new/.venv/bin" 2>/dev/null
    grep -rIlE --include='*.pth' --include='*.egg-link' --include='*.json' --include='*.cfg' --include='*.txt' "$pat" "$new/.venv/lib" 2>/dev/null
    grep -rInE --exclude-dir='.venv*' --exclude-dir=outputs --exclude-dir=log --exclude-dir=__pycache__ --exclude-dir=.git "$pat" "$new" 2>/dev/null
    grep -rInE --include='*.json' --include='*.env' "$pat" "$new/config" "$new/outputs/evals" 2>/dev/null
    grep -nE "$pat" "$RD_HOME/.bashrc" "$RD_HOME/.profile" "$RD_HOME/start_face.sh" 2>/dev/null
    crontab -l 2>/dev/null | grep -nE "$pat"
    grep -rInE --exclude-dir=__pycache__ "$pat" "$WS/Scripts" "$WS/src/robot_voice" "$WS/src/robot_face" "$WS/src/robot_disco/scripts" "$WS/services" 2>/dev/null
  } | sed 's/^/  /' || true
  [ -e "$RD_HOME/VLA" ] && warn "$RD_HOME/VLA still exists"
  [ -e "$DATA_MOUNT/VLA" ] && warn "$DATA_MOUNT/VLA still exists"
  return 0
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
  for p in preflight base jetson data ros groups udev sysctl workspace face audio services llm display foxglove; do "phase_$p"; done
  log "Done. Next: reboot, push the VLA tree (workstation: provision/deploy_to_rd.sh --vla), then 'provision_jetson.sh vla infer' and finally 'provision_jetson.sh network'."
}

dispatch "$@"
