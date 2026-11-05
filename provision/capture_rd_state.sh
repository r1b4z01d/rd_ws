#!/usr/bin/env bash
# Snapshot a LIVE robot PC so a replacement can be provisioned to match it, and so its unversioned
# state survives. Run from the workstation (needs SSH access to the robot; host "RD" by default):
#
#   provision/capture_rd_state.sh            # -> provision/snapshot-<date>/
#   provision/capture_rd_state.sh RD ~/rd-backup
#
# What it collects (everything provision.sh can restore when pointed at the dir with SNAPSHOT=...):
#   inventory/    apt-mark showmanual, ros packages, pip freezes (system + ~/rd_ws/VLA/.venv), groups,
#                 supervisor confs, udev rules, netplan, sysctl.d, gdm custom.conf, crontab,
#                 docker images, lsusb, ip addr, ~/.bashrc, start_face.sh
#   rd_ws.local.diff / rd_ws.untracked.txt   the robot's uncommitted edits to ~/rd_ws (RD is NOT
#                 git-synced; e.g. the black eye palette in gl_eye_renderer.py)
#   maps/         src/robot_disco/maps (pgm/yaml/posegraph/data)
#   people.json / people.arcface.json   learned faces (one gallery per back-end), faces/ (Folks thumbnails)
#   motions/      arm gesture recordings (~/.robot_face/motions, the UR wave)
#   Music/        the voice assistant's music library (~/Music, ~0.5 GB; skip with NO_MUSIC=1)
#   voice.env     voice assistant secrets; inventory/pulse-default.pa the PulseAudio output policy
#   tts-cache.tar.gz   Coqui model cache (~/.local/share/tts), if present
#   kokoro-models.tar.gz   Kokoro-82M ONNX model + voices (~/.robot_face/kokoro), if present
#   vla-venv-freeze.txt   exact VLA venv pins
#   foxglove-image.tar    the foxglove docker image (big, ~100 MB; skip with NO_DOCKER=1)
# Then diff inventory/ against provision/*.txt to catch anything the manifests miss.
set -euo pipefail
HOST="${1:-RD}"
OUT="${2:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/snapshot-$(date +%Y%m%d)}"
mkdir -p "$OUT/inventory"
r() { ssh -o BatchMode=yes "$HOST" "$@"; }

echo "==> inventory from $HOST into $OUT"
r 'lsb_release -ds; uname -r; hostname; id' > "$OUT/inventory/system.txt"
r 'apt-mark showmanual | sort' > "$OUT/inventory/apt-manual.txt"
r 'dpkg-query -W -f="\${Package}\n" "ros-*" 2>/dev/null | sort' > "$OUT/inventory/ros-packages.txt"
r 'python3 -m pip list --format=freeze 2>/dev/null' > "$OUT/inventory/pip-system-freeze.txt"
r 'python3 -m pip list --user --format=freeze 2>/dev/null' > "$OUT/inventory/pip-user-freeze.txt" || true
r 'test -x ~/rd_ws/VLA/.venv/bin/pip && ~/rd_ws/VLA/.venv/bin/pip freeze' > "$OUT/vla-venv-freeze.txt" || rm -f "$OUT/vla-venv-freeze.txt"
r 'groups' > "$OUT/inventory/groups.txt"
r 'sudo -n supervisorctl status 2>&1; echo; sudo -n cat /etc/supervisor/conf.d/*.conf 2>&1' > "$OUT/inventory/supervisor.txt" || true
r 'cat /etc/udev/rules.d/* 2>/dev/null' > "$OUT/inventory/udev-rules.txt" || true
r 'sudo -n cat /etc/netplan/*.yaml 2>&1; ip -br addr; ip route' > "$OUT/inventory/network.txt" || true
r 'cat /etc/sysctl.d/*.conf 2>/dev/null' > "$OUT/inventory/sysctl.txt" || true
r 'cat /etc/gdm3/custom.conf 2>/dev/null' > "$OUT/inventory/gdm-custom.conf" || true
r 'crontab -l 2>/dev/null; systemctl list-unit-files --state=enabled --no-pager' > "$OUT/inventory/units-cron.txt" || true
r 'sudo -n docker images 2>&1' > "$OUT/inventory/docker-images.txt" || true
r 'lsusb; ls -l /dev/ttyACM* /dev/ttyUSB* /dev/video* /dev/hidraw* 2>/dev/null' > "$OUT/inventory/devices.txt" || true
r 'cat ~/.bashrc' > "$OUT/inventory/bashrc" || true
r 'cat ~/start_face.sh 2>/dev/null' > "$OUT/inventory/start_face.sh" || true
r 'cat ~/rd_ws/VLA/run.sh 2>/dev/null' > "$OUT/inventory/vla-run.sh" || true

echo "==> rd_ws local edits"
r 'cd ~/rd_ws && git rev-parse HEAD && git status --short' > "$OUT/inventory/rd_ws-git-status.txt" || true
r 'cd ~/rd_ws && git diff' > "$OUT/rd_ws.local.diff" || true
r 'cd ~/rd_ws && git ls-files --others --exclude-standard' > "$OUT/rd_ws.untracked.txt" || true

echo "==> maps, people, TTS cache"
mkdir -p "$OUT/maps"
rsync -a "$HOST:rd_ws/src/robot_disco/maps/" "$OUT/maps/" || true
rsync -a "$HOST:.robot_face/people.json" "$OUT/people.json" 2>/dev/null || true
r 'test -d ~/.local/share/tts && tar czf - -C ~/.local/share tts' > "$OUT/tts-cache.tar.gz" 2>/dev/null \
  || rm -f "$OUT/tts-cache.tar.gz"
# voice assistant secrets (ANTHROPIC_API_KEY, HASS_*) and the ArcFace people gallery
rsync -a "$HOST:.robot_face/voice.env" "$OUT/voice.env" 2>/dev/null || true
rsync -a "$HOST:.robot_face/people.arcface.json" "$OUT/people.arcface.json" 2>/dev/null || true
rsync -a "$HOST:.robot_face/faces/" "$OUT/faces/" 2>/dev/null || true
rsync -a "$HOST:.robot_face/motions/" "$OUT/motions/" 2>/dev/null || true
rsync -a "$HOST:.config/pulse/default.pa" "$OUT/inventory/pulse-default.pa" 2>/dev/null || true
r 'ollama list 2>/dev/null' > "$OUT/inventory/ollama-models.txt" || true
if [ "${NO_MUSIC:-0}" != "1" ]; then
  echo "==> ~/Music (NO_MUSIC=1 to skip)"
  rsync -a "$HOST:Music/" "$OUT/Music/" 2>/dev/null || true
fi
# Kokoro-82M models (~350 MB; the venv itself is rebuilt by setup_kokoro.sh)
r 'test -d ~/.robot_face/kokoro && tar czf - -C ~/.robot_face kokoro' > "$OUT/kokoro-models.tar.gz" 2>/dev/null \
  || rm -f "$OUT/kokoro-models.tar.gz"

if [ "${NO_DOCKER:-0}" != "1" ]; then
  echo "==> foxglove docker image"
  img="$(r 'sudo -n docker images --format "{{.Repository}}:{{.Tag}}" 2>/dev/null | grep -i foxglove | head -1' || true)"
  if [ -n "$img" ]; then
    r "sudo -n docker save '$img'" > "$OUT/foxglove-image.tar" || rm -f "$OUT/foxglove-image.tar"
    echo "$img" > "$OUT/inventory/foxglove-image-name.txt"
  else
    echo "no foxglove image found on $HOST"
  fi
fi

echo "==> done: $OUT"
du -sh "$OUT"
echo "Compare against the manifests, e.g.:"
echo "  diff <(sed -e 's/#.*//' -e '/^\s*$/d' provision/ros-packages.txt | sort) $OUT/inventory/ros-packages.txt"
