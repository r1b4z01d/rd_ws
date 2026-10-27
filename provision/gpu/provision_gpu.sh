#!/usr/bin/env bash
# Provision the GPU box (training + the VLA web manager): Ubuntu 24.04, RTX 4090, user `bryan`.
# Idempotent and phased, like ../provision.sh:
#
#   ./provision_gpu.sh base venv services          # on the box, after ~/VLA has been pushed (README step 3)
#   ./provision_gpu.sh data                        # pull datasets + checkpoints from RD (needs `ssh RD`)
#   ./provision_gpu.sh wifi PASSWORD               # join the robot Wi-Fi (not in `all`; password never stored here)
#   ./provision_gpu.sh extras                      # optional desktop apps (not in `all`)
#   ./provision_gpu.sh all                         # = base venv services
#
# Knobs: SSID (default RobotDisco), WIFI_METRIC (600; Ethernet keeps 100 so the default route stays wired),
#        VLA_DIR (~/VLA), RD_HOST (RD, an ssh alias), SERVE_RUN/SERVE_STEP (the checkpoint outputs/serve_ckpt points at).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VLA_DIR="${VLA_DIR:-$HOME/VLA}"
RD_HOST="${RD_HOST:-RD}"
SSID="${SSID:-RobotDisco}"
WIFI_METRIC="${WIFI_METRIC:-600}"
SERVE_RUN="${SERVE_RUN:-tigger_bed_1_smolvla}"
SERVE_STEP="${SERVE_STEP:-020000}"
export PATH="$HOME/.local/bin:$PATH"

say() { printf '\n>>> %s\n' "$*"; }
manifest() { grep -v -E '^\s*(#|$)' "$1"; }

phase_preflight() {
  . /etc/os-release
  [ "$VERSION_ID" = "24.04" ] || echo "warning: tested on Ubuntu 24.04, this is $VERSION_ID"
  nvidia-smi --query-gpu=name,driver_version --format=csv,noheader \
    || { echo "no NVIDIA driver: install it from 'Additional Drivers' (or ubuntu-drivers install) and reboot" >&2; exit 1; }
  sudo -n true 2>/dev/null || echo "note: sudo needs a password; for unattended runs: echo '$USER ALL=(ALL) NOPASSWD: ALL' | sudo tee /etc/sudoers.d/$USER"
}

phase_base() {
  say "base: apt packages, supervisor, uv"
  sudo apt-get update -qq
  # shellcheck disable=SC2046
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y $(manifest "$HERE/apt-packages-gpu.txt")
  sudo systemctl enable --now supervisor
  command -v uv >/dev/null || curl -LsSf https://astral.sh/uv/install.sh | sh
  uv --version
}

phase_venv() {
  say "venv: $VLA_DIR/.venv (python 3.12, torch 2.8.0+cu128, lerobot 0.4.4)"
  [ -d "$VLA_DIR" ] || { echo "$VLA_DIR missing — push the VLA tree first (README step 3)" >&2; exit 1; }
  cd "$VLA_DIR"
  [ -x .venv/bin/python ] || uv venv --python 3.12 .venv
  # The freeze is the known-good set; fall back to the hand list (pins explained in it).
  if [ -f "$HERE/vla-venv-freeze.txt" ]; then
    # --no-deps: the freeze is already the complete set, and resolving it fails on the declared numpy>=2
    # of opencv/rerun-sdk against the numpy==1.26.4 it deliberately pins
    uv pip install --python .venv/bin/python --no-deps -r "$HERE/vla-venv-freeze.txt"
  else
    uv pip install --python .venv/bin/python -r "$HERE/pip-vla-gpu.txt"
  fi
  .venv/bin/python - <<'PY'
import torch, lerobot, transformers, numpy, pandas
import lerobot.policies.smolvla.modeling_smolvla  # noqa: F401
assert torch.cuda.is_available(), "torch cannot see the GPU"
print("torch", torch.__version__, torch.cuda.get_device_name(0), "| lerobot", lerobot.__version__,
      "| transformers", transformers.__version__, "| numpy", numpy.__version__, "| pandas", pandas.__version__)
PY
}

phase_services() {
  say "services: supervisor web UI (:9999), vla_webui (:8081, autostart) + vla_infer_server (fallback, manual)"
  # Same drop-in as RD (provision.sh): a stock install has only the unix socket, and [inet_http_server] is
  # read at supervisord start, so it needs a restart — not just reread/update.
  if ! grep -rqs "inet_http_server" /etc/supervisor/; then
    printf '; supervisord web UI / XML-RPC (no auth: trusted robot subnet)\n[inet_http_server]\nport=0.0.0.0:9999\n' \
      | sudo tee /etc/supervisor/conf.d/00-inet-http.conf >/dev/null
    sudo systemctl restart supervisor
  fi
  cd "$VLA_DIR"
  chmod +x scripts/*.sh
  ./scripts/install_services.sh
  sleep 6
  sudo supervisorctl status | grep vla_ || true
  curl -fsS -o /dev/null -w "supervisor UI http=%{http_code}  (:9999)\n" localhost:9999/
  curl -fsS -o /dev/null -w "web manager http=%{http_code}  (http://$(hostname -I | awk '{print $1}'):8081)\n" localhost:8081/
}

phase_data() {
  say "data: outputs/ from $RD_HOST (datasets, train runs, evals; ~6 GB)"
  cd "$VLA_DIR"; mkdir -p outputs
  # serve_ckpt is excluded: on RD it is an absolute symlink under /mnt/data/VLA that means nothing here
  rsync -a --info=stats1 "$RD_HOST:VLA/outputs/" outputs/ --exclude=serve_ckpt
  ln -sfn "$VLA_DIR/outputs/train/$SERVE_RUN/checkpoints/$SERVE_STEP/pretrained_model" outputs/serve_ckpt
  ls outputs/serve_ckpt/ && du -sh outputs
}

phase_wifi() {
  local pw="${1:-}"
  [ -n "$pw" ] || { echo "usage: provision_gpu.sh wifi <password>   (or: sudo nmcli device wifi connect $SSID --ask)" >&2; exit 1; }
  say "wifi: $SSID"
  sudo nmcli device wifi connect "$SSID" password "$pw"
  # Two NICs on two subnets: keep the wired default route (metric 100) and let Wi-Fi (higher metric) carry
  # only the robot subnet — the robot subnet 192.168.11.x has no internet.
  sudo nmcli connection modify "$SSID" ipv4.route-metric "$WIFI_METRIC" connection.autoconnect yes
  sudo nmcli connection up "$SSID" >/dev/null
  ip -4 -br addr; ip route | head -4
}

phase_extras() {
  say "extras: Steam, Brave, VLC, Remmina, HandBrake, GIMP, Angry IP Scanner, Bambu Studio"
  sudo dpkg --add-architecture i386
  sudo add-apt-repository -y multiverse >/dev/null
  sudo curl -fsSLo /usr/share/keyrings/brave-browser-archive-keyring.gpg \
    https://brave-browser-apt-release.s3.brave.com/brave-browser-archive-keyring.gpg
  echo "deb [signed-by=/usr/share/keyrings/brave-browser-archive-keyring.gpg] https://brave-browser-apt-release.s3.brave.com/ stable main" \
    | sudo tee /etc/apt/sources.list.d/brave-browser-release.list >/dev/null
  sudo apt-get update -qq
  echo steam steam/question select "I AGREE" | sudo debconf-set-selections
  echo steam steam/license note "" | sudo debconf-set-selections
  # shellcheck disable=SC2046
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y $(manifest "$HERE/apt-packages-extras.txt")
  # Angry IP Scanner is not in apt: the release .deb (needs a JRE, in the manifest)
  local url
  url=$(curl -fsSL https://api.github.com/repos/angryip/ipscan/releases/latest | grep browser_download_url | grep -E 'ipscan_[0-9.]+_amd64.deb' | head -1 | cut -d'"' -f4)
  curl -fsSLo /tmp/ipscan.deb "$url" && sudo apt-get install -y /tmp/ipscan.deb
  # Bambu Studio: AppImage. NB the asset is named "ubuntu24.04" (no hyphen) — a grep for ubuntu-24.04 matches nothing
  url=$(curl -fsSL https://api.github.com/repos/bambulab/BambuStudio/releases/latest | grep browser_download_url | grep -E 'ubuntu24\.04.*AppImage' | head -1 | cut -d'"' -f4)
  [ -n "$url" ] || { echo "no Bambu Studio ubuntu24.04 AppImage found" >&2; return 1; }
  mkdir -p ~/Applications ~/.local/share/applications
  curl -fsSLo ~/Applications/BambuStudio.AppImage "$url" && chmod +x ~/Applications/BambuStudio.AppImage
  printf '[Desktop Entry]\nType=Application\nName=Bambu Studio\nExec=env GDK_BACKEND=x11 %s/Applications/BambuStudio.AppImage\nCategories=Graphics;\nTerminal=false\n' "$HOME" \
    > ~/.local/share/applications/bambu-studio.desktop
  echo "Steam: sign in on the desktop, Settings > Compatibility > Steam Play for all titles (Proton) before installing Windows games."
  echo "Fusion 360 is NOT installed here (Wine installer, see README 'Not automated')."
}

[ $# -gt 0 ] || { sed -n '2,12p' "$0"; exit 1; }
phases=("$@")
[ "${1:-}" = all ] && phases=(base venv services)
phase_preflight
i=0
while [ $i -lt ${#phases[@]} ]; do
  p="${phases[$i]}"
  if [ "$p" = wifi ]; then phase_wifi "${phases[$((i+1))]:-}"; i=$((i+2)); continue; fi
  "phase_$p"; i=$((i+1))
done
say "done: ${phases[*]}"
