#!/usr/bin/env bash
# Provision a fresh Ubuntu 22.04 machine as the Robot Disco robot PC ("RD", rd@192.168.11.80).
#
# Run ON the new machine, as the robot user (default "rd", must have sudo), from any directory:
#
#   git clone https://github.com/r1b4z01d/rd_ws.git ~/rd_ws     # or copy this provision/ dir over
#   ~/rd_ws/provision/provision.sh all
#
# or one phase at a time (they are idempotent and safe to re-run):
#
#   provision.sh preflight base ros groups udev sysctl workspace face audio services llm display foxglove vla network
#
# "all" runs everything except "network" and "vla" (network last and on purpose, because it moves
# the box onto the internet-less robot subnet; vla needs the rd_ws/VLA submodule tree in place first).
#
# Environment knobs (all optional):
#   RD_USER=rd                     robot user (must be the user running this script)
#   HOSTNAME_NEW=robotdisco        hostname to set (empty = leave alone)
#   RD_WS_REPO=https://github.com/r1b4z01d/rd_ws.git
#   RD_WS_REF=main                 branch/tag/commit to check out
#   SNAPSHOT=/path/to/snapshot     output dir of capture_rd_state.sh from the old robot PC; when set,
#                                  pip freezes, TTS cache, people.json, foxglove image and maps are
#                                  restored from it (see README.md "Restoring from the old machine")
#   WITH_TTS=1                     also install Coqui TTS (pip-robot-face-tts.txt; pulls torch)
#   WITH_KOKORO=0                  skip Kokoro-82M TTS (default 1: installs via setup_kokoro.sh)
#   WITH_FACEID=0                  skip the ArcFace face-recognition venv (setup_faceid.sh)
#   WITH_VOICE=0                   skip the voice assistant venv (setup_voice.sh); WHISPER_MODEL=base.en
#   WITH_LLM=0                     skip the local language model (phase llm: Ollama under supervisor); OLLAMA_MODEL=qwen2.5:7b
#   AUDIO_SINK=alsa_output...      PulseAudio sink the robot speaks through (phase audio; default: the dock's C-Media adapter)
#   KOKORO_WHEELS=/path/to/wheels  offline Kokoro install from a pre-downloaded wheel folder
#   FOXGLOVE_IMAGE_TAR=file.tar    docker image tarball to `docker load` instead of building
#   PROXY=socks5h://127.0.0.1:1080 apt/pip/git proxy, for provisioning over an SSH reverse tunnel
#                                  when the box is already on the internet-less subnet (README.md)
#   AUTHORIZED_KEY="ssh-rsa ..."   public key to append to ~/.ssh/authorized_keys
#   NET_IFACE=enp3s0               wired interface for phase "network" (auto-detected if empty)
#   NET_GATEWAY=192.168.11.1       phase "network" on a NetworkManager-managed interface (the Jetson desktop):
#   NET_DNS=8.8.8.8                  gateway + DNS set with nmcli (the netplan path sets no gateway)
#   NETPLAN_RENDERER=networkd      renderer written into netplan/01-robot-disco.yaml (the Jetson script says NetworkManager)
#   SKIP_SUBMODULES=1              phase workspace leaves src/ submodules alone (when src/ was rsynced
#                                  from a workstation whose tree is ahead of GitHub)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RD_USER="${RD_USER:-rd}"
RD_HOME="$( { command -v getent >/dev/null && getent passwd "$RD_USER" | cut -d: -f6; } || true)"
RD_HOME="${RD_HOME:-/home/$RD_USER}"
WS="$RD_HOME/rd_ws"
RD_WS_REPO="${RD_WS_REPO:-https://github.com/r1b4z01d/rd_ws.git}"
RD_WS_REF="${RD_WS_REF:-main}"
HOSTNAME_NEW="${HOSTNAME_NEW:-robotdisco}"
SNAPSHOT="${SNAPSHOT:-}"
ROS_DISTRO=humble
FOXGLOVE_TAG=robotdisco/foxglove
OLLAMA_MODEL="${OLLAMA_MODEL:-qwen2.5:7b}"
AUDIO_SINK="${AUDIO_SINK:-alsa_output.usb-C-Media_Electronics_Inc._USB_Audio_Device-00.analog-stereo}"

log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mWARN: %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

# Strip comments/blank lines from a package list file.
pkglist() { sed -e 's/#.*//' -e '/^[[:space:]]*$/d' "$1" | tr '\n' ' '; }

apt_install() {
  # shellcheck disable=SC2086
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends $*
}

setup_proxy() {
  [ -n "${PROXY:-}" ] || return 0
  log "Routing apt/pip/git through $PROXY"
  export ALL_PROXY="$PROXY" all_proxy="$PROXY" https_proxy="$PROXY" http_proxy="$PROXY"
  printf 'Acquire::http::Proxy "%s";\nAcquire::https::Proxy "%s";\n' "$PROXY" "$PROXY" \
    | sudo tee /etc/apt/apt.conf.d/99-provision-proxy >/dev/null
  export PIP_PROXY="$PROXY"
}

teardown_proxy() {
  sudo rm -f /etc/apt/apt.conf.d/99-provision-proxy
}

# ---------------------------------------------------------------------------------------------
phase_preflight() {
  log "Preflight"
  [ "$(id -un)" = "$RD_USER" ] || die "run this as user '$RD_USER' (RD_USER=... to override), not $(id -un)"
  sudo -n true 2>/dev/null || sudo -v || die "$RD_USER needs sudo"
  # shellcheck disable=SC1091
  . /etc/os-release
  [ "${VERSION_ID:-}" = "22.04" ] || warn "expected Ubuntu 22.04 (ROS 2 Humble); this is ${PRETTY_NAME:-unknown}"
  [ "$(dpkg --print-architecture)" = "amd64" ] || warn "expected amd64 (RD is an Intel box)"
  setup_proxy
}

phase_base() {
  log "Base system packages"
  sudo apt-get update
  apt_install "$(pkglist "$HERE/apt-packages.txt")"
  if ! command -v docker >/dev/null; then
    log "docker (none present): docker.io"
    apt_install docker.io
  fi
  sudo systemctl enable --now ssh supervisor docker
  sudo timedatectl set-ntp true || true     # README: a skewed clock makes colcon/make skip work
  if [ -n "$HOSTNAME_NEW" ] && [ "$(hostname)" != "$HOSTNAME_NEW" ]; then
    log "Hostname -> $HOSTNAME_NEW"
    sudo hostnamectl set-hostname "$HOSTNAME_NEW"
    grep -q "$HOSTNAME_NEW" /etc/hosts || echo "127.0.1.1 $HOSTNAME_NEW" | sudo tee -a /etc/hosts >/dev/null
  fi
  if [ -n "${AUTHORIZED_KEY:-}" ]; then
    mkdir -p "$RD_HOME/.ssh" && chmod 700 "$RD_HOME/.ssh"
    grep -qF "$AUTHORIZED_KEY" "$RD_HOME/.ssh/authorized_keys" 2>/dev/null \
      || echo "$AUTHORIZED_KEY" >> "$RD_HOME/.ssh/authorized_keys"
    chmod 600 "$RD_HOME/.ssh/authorized_keys"
  fi
  # Robot subnet peers by name (README / VLA wiki). Harmless if already present.
  for entry in "192.168.11.21 ur5e" "192.168.11.117 amazinghand" "192.168.11.119 gpu"; do   # gpu = the 4090 box's Wi-Fi address (provision/gpu/README.md)
    grep -q "^${entry%% *} " /etc/hosts || echo "$entry" | sudo tee -a /etc/hosts >/dev/null
  done
}

phase_ros() {
  log "ROS 2 $ROS_DISTRO apt repository"
  sudo add-apt-repository -y universe
  if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    # shellcheck disable=SC1091
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi
  sudo apt-get update
  log "ROS 2 packages (ros-packages.txt)"
  apt_install "$(pkglist "$HERE/ros-packages.txt")"
  [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
  rosdep update || warn "rosdep update failed (no internet?); phase workspace will still try"
  if ! grep -q "/opt/ros/$ROS_DISTRO/setup.bash" "$RD_HOME/.bashrc"; then
    cat >> "$RD_HOME/.bashrc" <<'BRC'

# --- Robot Disco (added by rd_ws/provision/provision.sh)
source /opt/ros/humble/setup.bash
[ -f "$HOME/rd_ws/install/setup.bash" ] && source "$HOME/rd_ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/rd_ws/services/cyclonedds.xml"   # localhost-only DDS, see services/cyclonedds.xml
export ROS_DOMAIN_ID=0
BRC
  fi
  # librealsense's full udev rule set ships with the ROS wrapper's dependency; install it if present.
  local rs_rules
  rs_rules="$(find /opt/ros/$ROS_DISTRO /usr/lib/udev /lib/udev -name '99-realsense-libusb.rules' 2>/dev/null | head -1 || true)"
  if [ -n "$rs_rules" ]; then
    sudo cp "$rs_rules" /etc/udev/rules.d/99-realsense-libusb.rules
  else
    warn "99-realsense-libusb.rules not found; relying on the subset in udev/99-robot-disco.rules"
  fi
}

phase_groups() {
  log "Groups for $RD_USER: dialout video input plugdev audio docker render"
  for g in dialout video input plugdev audio docker render; do
    getent group "$g" >/dev/null && sudo usermod -aG "$g" "$RD_USER"
  done
  warn "group changes apply to new logins (log out/in or reboot before using serial/USB/docker)"
}

phase_udev() {
  log "udev rules"
  sudo cp "$HERE/udev/99-robot-disco.rules" /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
}

phase_sysctl() {
  # Same file and values VLA/scripts/install_services.sh writes, so the two never fight.
  log "UDP receive buffers for the multicast camera stream"
  printf 'net.core.rmem_max = 16777216\nnet.core.rmem_default = 1048576\n' \
    | sudo tee /etc/sysctl.d/99-vla-camera-udp.conf >/dev/null
  sudo sysctl -q -p /etc/sysctl.d/99-vla-camera-udp.conf
}

phase_workspace() {
  log "rd_ws checkout at $WS"
  # Submodules are pinned with a git@github.com URL (AmazingHand); a fresh box has no GitHub key.
  git config --global url."https://github.com/".insteadOf "git@github.com:"
  if [ ! -d "$WS/.git" ]; then
    git clone "$RD_WS_REPO" "$WS"
  fi
  git -C "$WS" fetch --all --tags
  git -C "$WS" checkout "$RD_WS_REF"
  git -C "$WS" pull --ff-only 2>/dev/null || true
  # Only the ROS packages under src/ are needed to run the robot; AmazingHand-main and
  # foxglove_studio's node tree are not built here.
  if [ "${SKIP_SUBMODULES:-0}" = "1" ]; then
    warn "SKIP_SUBMODULES=1: not touching submodules (src/ was synced from a workstation)"
  else
    git -C "$WS" submodule sync --recursive
    # One at a time so a dead upstream (AprilTag_Detection's repo is gone) does not abort the rest;
    # an uninitialised submodule is an empty dir, which colcon ignores.
    local sm
    for sm in $(git -C "$WS" config -f .gitmodules --get-regexp '^submodule\..*\.path$' | awk '$2 ~ /^src\// {print $2}'); do
      git -C "$WS" submodule update --init --recursive -- "$sm" \
        || { warn "submodule $sm could not be fetched; skipping it"; continue; }
      # A partial earlier run can leave the checkout empty while git reports it done; force it.
      if [ -z "$(ls -A "$WS/$sm" 2>/dev/null)" ]; then
        git -C "$WS" submodule update --init --recursive --force -- "$sm" || warn "submodule $sm is still empty"
      fi
    done
  fi
  # Source patches carried for submodules: provision/patches/<submodule dir>-<topic>.patch, applied
  # from that submodule's root. Idempotent (a patch already in the tree is skipped), and it runs with
  # SKIP_SUBMODULES=1 too, since a workstation rsync may or may not carry it. Today:
  # apriltag_ros-single-opencv.patch — on JetPack the apt cv_bridge/image_geometry link Ubuntu's
  # OpenCV 4.5 while find_package(OpenCV) gives NVIDIA's 4.8; two OpenCVs in the tag container
  # abort it ("Assertion failed s >= 0 in setSize"), so the node converts the image itself.
  local patch sm_dir
  for patch in "$HERE"/patches/*.patch; do
    [ -e "$patch" ] || continue
    sm_dir="$WS/src/$(basename "$patch" .patch | cut -d- -f1)"
    if [ ! -d "$sm_dir" ]; then
      warn "patch $(basename "$patch"): $sm_dir is missing; skipping"
    elif git -C "$sm_dir" apply --reverse --check "$patch" 2>/dev/null; then
      log "patch $(basename "$patch") already applied"
    elif git -C "$sm_dir" apply "$patch"; then
      log "applied $(basename "$patch") in $sm_dir"
    else
      warn "patch $(basename "$patch") did not apply in $sm_dir; the build (or the tag detector) may fail"
    fi
  done
  mkdir -p "$WS/log"

  log "rosdep (system deps of the source packages)"
  # ROS setup files reference unset variables; relax nounset around the source.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/$ROS_DISTRO/setup.bash
  set -u
  sudo apt-get update
  rosdep install --from-paths "$WS/src" --ignore-src -r -y \
    --skip-keys "gazebo_ros2_control gazebo_ros" || warn "rosdep reported missing keys; check the build"

  if [ -n "$SNAPSHOT" ] && [ -d "$SNAPSHOT/maps" ]; then
    log "Restoring maps from snapshot"
    cp -a "$SNAPSHOT/maps/." "$WS/src/robot_disco/maps/"
  fi

  log "colcon build --symlink-install (this takes a while: velodyne, apriltag, nav helpers)"
  (cd "$WS" && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release) \
    || die "colcon build failed; fix and re-run 'provision.sh workspace' (or build with --packages-skip)"
}

phase_face() {
  log "robot_face python deps (system python, --user)"
  python3 -m pip install --user --upgrade pip
  python3 -m pip install --user -r "$HERE/pip-robot-face.txt"
  if [ "${WITH_TTS:-0}" = "1" ]; then
    log "Coqui TTS (large)"
    python3 -m pip install --user -r "$HERE/pip-robot-face-tts.txt"
  fi
  if [ -n "$SNAPSHOT" ] && [ -f "$SNAPSHOT/tts-cache.tar.gz" ]; then
    log "Restoring Coqui TTS model cache"
    mkdir -p "$RD_HOME/.local/share" && tar xzf "$SNAPSHOT/tts-cache.tar.gz" -C "$RD_HOME/.local/share"
  fi
  mkdir -p "$RD_HOME/.robot_face" "$RD_HOME/.robot_face/faces" "$RD_HOME/.robot_face/motions"
  # Arm gestures: joint-space recordings the face node replays on the UR5e (UrMotion, servoJ). The repo
  # ships the robot's wave (src/robot_face/motions/wave.json, recorded with scripts/ur_record_motion.py);
  # a recording already on the robot wins (cp -n).
  if ls "$WS"/src/robot_face/motions/*.json >/dev/null 2>&1; then
    cp -n "$WS"/src/robot_face/motions/*.json "$RD_HOME/.robot_face/motions/"
  fi
  if [ -n "$SNAPSHOT" ] && [ -f "$SNAPSHOT/kokoro-models.tar.gz" ]; then
    log "Restoring Kokoro TTS models"
    tar xzf "$SNAPSHOT/kokoro-models.tar.gz" -C "$RD_HOME/.robot_face"
  fi
  if [ "${WITH_FACEID:-1}" = "1" ]; then
    # SCRFD + ArcFace face recognition (InsightFace) in its own venv; the node falls back to OpenCV without it
    log "Face ID (ArcFace, own venv at ~/.robot_face/faceid-venv)"
    "$HERE/setup_faceid.sh" || warn "Face ID (ArcFace) install failed; the face will use the OpenCV back-end"
  fi
  if [ "${WITH_VOICE:-1}" = "1" ]; then
    # Voice assistant: faster-whisper venv + anthropic SDK; secrets template in ~/.robot_face/voice.env
    log "Voice assistant (whisper venv at ~/.robot_face/voice-venv)"
    "$HERE/setup_voice.sh" "${WHISPER_MODEL:-base.en}" || warn "voice stack install failed"
    if [ -n "$SNAPSHOT" ] && [ -f "$SNAPSHOT/voice.env" ]; then
      cp "$SNAPSHOT/voice.env" "$RD_HOME/.robot_face/voice.env" && chmod 600 "$RD_HOME/.robot_face/voice.env"
    fi
  fi
  if [ "${WITH_KOKORO:-1}" = "1" ]; then
    # Kokoro-82M voice in its own venv (kokoro-onnx needs numpy 2). ~50 MB of wheels + ~350 MB of models.
    log "Kokoro TTS (own venv at ~/.robot_face/kokoro-venv)"
    "$HERE/setup_kokoro.sh" "${KOKORO_WHEELS:-}" || warn "Kokoro install failed; the face will fall back to Coqui/espeak"
  fi
  if [ -n "$SNAPSHOT" ]; then
    # Who the robot knows (both back-ends keep their own gallery), the Folks page thumbnails, the arm
    # gestures recorded on the robot, and the voice assistant's music library. Existing files win.
    if [ -f "$SNAPSHOT/people.json" ]; then
      log "Restoring learned people (OpenCV gallery)"; cp -n "$SNAPSHOT/people.json" "$RD_HOME/.robot_face/"
    fi
    if [ -f "$SNAPSHOT/people.arcface.json" ]; then
      log "Restoring learned people (ArcFace gallery)"; cp -n "$SNAPSHOT/people.arcface.json" "$RD_HOME/.robot_face/"
    fi
    [ -d "$SNAPSHOT/faces" ]   && cp -an "$SNAPSHOT/faces/."   "$RD_HOME/.robot_face/faces/"
    [ -d "$SNAPSHOT/motions" ] && cp -an "$SNAPSHOT/motions/." "$RD_HOME/.robot_face/motions/"
    if [ -d "$SNAPSHOT/Music" ]; then
      log "Restoring ~/Music"; mkdir -p "$RD_HOME/Music"; cp -an "$SNAPSHOT/Music/." "$RD_HOME/Music/"
    fi
  fi
  python3 - <<'PY' || warn "a robot_face import failed; see above"
import cv2, mediapipe as mp, moderngl, glfw, numpy
assert tuple(int(x) for x in cv2.__version__.split('.')[:2]) >= (4, 8), cv2.__version__
assert hasattr(mp, "solutions"), "mediapipe %s has no legacy solutions API (need <1)" % mp.__version__
# 0.10.x fetches the Pose/Hands .tflite models from the internet on first use; do that now so an
# offline robot has them cached in site-packages.
mp.solutions.pose.Pose(model_complexity=0).close(); mp.solutions.hands.Hands(max_num_hands=1).close()
print("robot_face deps OK: opencv", cv2.__version__, "mediapipe", mp.__version__, "(pose/hands models cached)")
PY
}

phase_audio() {
  # Speaker = the dock's C-Media USB adapter, microphone = the TONOR (robot_voice picks it by name).
  # Three things silenced the live robot (2026-10-31 / 11-01): PulseAudio's module-switch-on-connect made
  # a newly plugged USB device the default output; a boot-time race left the onboard jack as the default
  # sink (the adapter enumerates after pulse starts); and udev's `alsactl restore` re-applied the adapter's
  # hardware mute at every boot once it had been muted. The user-level default.pa settles the first two
  # for the rd session (the face and voice nodes additionally pin PULSE_SINK and watch the default sink);
  # unmuting + `alsactl store` settles the third.
  log "Audio: PulseAudio policy (~/.config/pulse/default.pa -> $AUDIO_SINK), unmute, alsactl store"
  mkdir -p "$RD_HOME/.config/pulse"
  sed "s|^set-default-sink .*|set-default-sink $AUDIO_SINK|" "$HERE/pulse/default.pa" > "$RD_HOME/.config/pulse/default.pa"
  export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
  if ! pactl info >/dev/null 2>&1; then
    warn "no PulseAudio session for $RD_USER yet (desktop not logged in); the policy applies at the next login"
    return 0
  fi
  if ! pactl list short sinks | awk '{print $2}' | grep -qx "$AUDIO_SINK"; then
    warn "sink $AUDIO_SINK is not present (adapter unplugged?); the policy applies when it is"
    return 0
  fi
  pactl set-default-sink "$AUDIO_SINK" || true
  pactl set-sink-mute "$AUDIO_SINK" 0 || true
  local vol
  vol="$(pactl get-sink-volume "$AUDIO_SINK" | grep -oE '[0-9]+%' | head -1 | tr -d %)"
  [ "${vol:-0}" -ge 10 ] || pactl set-sink-volume "$AUDIO_SINK" 60%
  sudo alsactl store 2>/dev/null || warn "alsactl store failed (the adapter may come back muted after a reboot)"
  echo "default sink: $(pactl get-default-sink); $(pactl get-sink-mute "$AUDIO_SINK"); $(pactl get-sink-volume "$AUDIO_SINK" | head -1 | tr -s ' ')"
}

phase_llm() {
  # The fallback brain for robot_voice when the GPU box is off: voice_assistant.yaml tries the box
  # (192.168.11.119:11434) first, then this one on loopback. Same shape as provision/gpu/ollama.conf:
  # Ollama's installer brings its own systemd unit, which would race supervisor for :11434, so it is
  # disabled and supervisor owns the process like every other service on the robot.
  if [ "${WITH_LLM:-1}" != "1" ]; then log "WITH_LLM=0: skipping the local language model"; return 0; fi
  log "Local LLM: Ollama under supervisor (unit 'ollama', loopback :11434) with $OLLAMA_MODEL"
  if ! command -v ollama >/dev/null; then
    curl -fsSL https://ollama.com/install.sh | sh     # on the Orin this picks the JetPack 6 build (cuda_jetpack6 libs)
  fi
  if systemctl list-unit-files ollama.service 2>/dev/null | grep -q '^ollama.service'; then
    sudo systemctl disable --now ollama.service
  fi
  sudo cp "$WS/services/ollama.conf" /etc/supervisor/conf.d/ollama.conf
  sudo supervisorctl reread >/dev/null && sudo supervisorctl update >/dev/null
  sudo supervisorctl status ollama | grep -qE 'RUNNING|STARTING' || sudo supervisorctl start ollama
  local i
  for i in $(seq 1 30); do curl -fsS 127.0.0.1:11434/api/tags >/dev/null 2>&1 && break; sleep 1; done
  if ! curl -fsS 127.0.0.1:11434/api/version >/dev/null; then
    warn "ollama is not answering on :11434 — sudo supervisorctl tail ollama"; return 0
  fi
  if ollama list 2>/dev/null | grep -q "^${OLLAMA_MODEL}"; then
    echo "model $OLLAMA_MODEL present"
  elif curl -fsI -m 8 https://ollama.com >/dev/null 2>&1; then
    log "Pulling $OLLAMA_MODEL (~4.7 GB)"
    ollama pull "$OLLAMA_MODEL" || warn "pull failed; later: ollama pull $OLLAMA_MODEL"
  else
    warn "no internet: pull the model once the uplink is back (ollama pull $OLLAMA_MODEL); until then the voice assistant uses the GPU box's"
  fi
  sudo supervisorctl status ollama
}

phase_services() {
  log "Supervisor units from services/*.conf"
  [ -d "$WS/install" ] || warn "$WS/install missing; units will fail until 'provision.sh workspace' succeeds"
  # Web UI / XML-RPC on :9999 (http://192.168.11.80:9999/), as on the old robot PC. A stock install only
  # has the unix socket, and this section needs a supervisord restart, not just reread/update.
  if ! grep -rqs "inet_http_server" /etc/supervisor/; then
    printf '; supervisord web UI / XML-RPC (no auth: trusted robot subnet)\n[inet_http_server]\nport=0.0.0.0:9999\n' \
      | sudo tee /etc/supervisor/conf.d/00-inet-http.conf >/dev/null
    sudo systemctl restart supervisor
  fi
  sudo "$WS/Scripts/install_services.sh"
  install -m 755 "$HERE/start_face.sh" "$RD_HOME/start_face.sh"
  sudo supervisorctl status || true
}

phase_display() {
  # The face window needs the robot PC's own GNOME session: log rd in automatically, never blank/lock.
  log "GNOME autologin for $RD_USER and no screen blanking"
  if [ -f /etc/gdm3/custom.conf ]; then
    sudo sed -i -E \
      -e "s/^#?\s*AutomaticLoginEnable\s*=.*/AutomaticLoginEnable = true/" \
      -e "s/^#?\s*AutomaticLogin\s*=.*/AutomaticLogin = $RD_USER/" /etc/gdm3/custom.conf
    grep -q "^AutomaticLoginEnable" /etc/gdm3/custom.conf \
      || sudo sed -i "/^\[daemon\]/a AutomaticLoginEnable = true\nAutomaticLogin = $RD_USER" /etc/gdm3/custom.conf
  else
    warn "/etc/gdm3/custom.conf not found (no desktop installed?). robot_face needs a display session."
  fi
  if command -v gsettings >/dev/null; then
    dbus-launch gsettings set org.gnome.desktop.session idle-delay 0 || true
    dbus-launch gsettings set org.gnome.desktop.screensaver lock-enabled false || true
    dbus-launch gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type nothing || true
  fi
}

phase_foxglove() {
  log "Foxglove Studio docker image"
  local conf_image
  conf_image="$(grep -oE '[A-Za-z0-9_./-]+/foxglove' "$WS/services/robot_disco_foxglove_studio.conf" | head -1)"
  if [ -n "${FOXGLOVE_IMAGE_TAR:-}" ]; then
    sudo docker load -i "$FOXGLOVE_IMAGE_TAR"
  elif [ -n "$SNAPSHOT" ] && [ -f "$SNAPSHOT/foxglove-image.tar" ]; then
    sudo docker load -i "$SNAPSHOT/foxglove-image.tar"
  elif ! sudo docker image inspect "$FOXGLOVE_TAG" >/dev/null 2>&1; then
    log "Building from foxglove_studio/Dockerfile (needs internet and ~20 min; yarn install is heavy)"
    sudo docker build -t "$FOXGLOVE_TAG" "$WS/foxglove_studio"
  fi
  if [ -n "$conf_image" ] && [ "$conf_image" != "$FOXGLOVE_TAG" ]; then
    sudo docker tag "$FOXGLOVE_TAG" "$conf_image" 2>/dev/null \
      || warn "services/robot_disco_foxglove_studio.conf references '$conf_image', which docker rejects (image names must be lowercase). Use '$FOXGLOVE_TAG' in the conf."
  fi
  sudo docker images | grep -i foxglove || warn "no foxglove image present"
}

# pip-installs the VLA stack into the venv whose pip is $1. CPU torch here; the Jetson variant overrides this.
vla_pip_install() {
  local pip="$1"
  if [ -n "$SNAPSHOT" ] && [ -f "$SNAPSHOT/vla-venv-freeze.txt" ]; then
    log "Installing the exact freeze from the old robot PC"
    if ! "$pip" install -r "$SNAPSHOT/vla-venv-freeze.txt"; then
      warn "exact freeze failed; falling back to pip-vla.txt"
      "$pip" install --extra-index-url https://download.pytorch.org/whl/cpu -r "$HERE/pip-vla.txt"
    fi
  else
    "$pip" install --extra-index-url https://download.pytorch.org/whl/cpu -r "$HERE/pip-vla.txt"
  fi
}

phase_vla() {
  local vla="$WS/VLA"
  log "VLA project venv + camera services at $vla"
  # VLA is the rd_ws/VLA submodule (not under src/, so phase_workspace leaves it alone; its COLCON_IGNORE
  # keeps colcon out of it). A box with internet can fetch it here; the robot has none, so the tree is
  # pushed from the workstation into the same place (rd_ws/provision/deploy_to_rd.sh --vla, README §3).
  if [ ! -d "$vla/ur5e_lerobot" ]; then
    git -C "$WS" submodule update --init -- VLA 2>/dev/null \
      || die "VLA tree missing and the submodule could not be fetched (no internet?); push it from the workstation:
  cd rd_ws && provision/deploy_to_rd.sh --vla        # or, from rd_ws/VLA, without the script:
  tar czf - --exclude=__pycache__ --exclude=.venv --exclude=outputs ur5e_lerobot scripts services webui assets | ssh $RD_USER@<rd> 'mkdir -p ~/rd_ws/VLA && tar xzf - -C ~/rd_ws/VLA'"
  fi
  if [ ! -x "$vla/.venv/bin/python" ]; then
    python3 -m venv "$vla/.venv"
  fi
  "$vla/.venv/bin/pip" install --upgrade pip wheel
  vla_pip_install "$vla/.venv/bin/pip"
  if [ ! -x "$vla/run.sh" ]; then
    cat > "$vla/run.sh" <<'RUN'
#!/usr/bin/env bash
# Every robot-PC VLA command goes through this (see VLA/wiki/Machines-and-Network.md):
# project venv, PYTHONPATH=., no user site-packages, no HuggingFace network calls (isolated subnet).
cd "$(dirname "$(readlink -f "$0")")" || exit 1
export PYTHONPATH=.
export PYTHONNOUSERSITE=1
export HF_HUB_OFFLINE=1
exec .venv/bin/python "$@"
RUN
    chmod +x "$vla/run.sh"
  fi
  mkdir -p "$vla/log"
  if [ -x "$vla/scripts/install_services.sh" ]; then
    (cd "$vla" && sudo ./scripts/install_services.sh rd)
  else
    warn "$vla/scripts/install_services.sh missing; VLA supervisor units not installed"
  fi
}

phase_network() {
  log "Static address 192.168.11.80 on the robot subnet"
  local iface="${NET_IFACE:-}"
  if [ -z "$iface" ]; then
    iface="$(ip -o link show | awk -F': ' '$2 ~ /^(en|eth)/ {print $2; exit}')"
  fi
  [ -n "$iface" ] || die "no wired interface found; set NET_IFACE=..."
  # NetworkManager boxes (JetPack desktop): change the existing NM connection in place rather than
  # layering a netplan file on top of it (two profiles for one interface race at boot).
  if command -v nmcli >/dev/null && nmcli -t -f DEVICE,STATE dev status 2>/dev/null | grep -q "^$iface:connected"; then
    local con
    con="$(nmcli -t -f DEVICE,CONNECTION dev status | awk -F: -v d="$iface" '$1==d {print $2}')"
    warn "NetworkManager manages $iface via '$con': setting 192.168.11.80/24, gw ${NET_GATEWAY:-192.168.11.1}"
    nohup bash -c "sleep 2; sudo -n nmcli con mod '$con' ipv4.method manual ipv4.addresses 192.168.11.80/24 \
      ipv4.gateway '${NET_GATEWAY:-192.168.11.1}' ipv4.dns '${NET_DNS:-8.8.8.8}' connection.autoconnect yes \
      && sudo -n nmcli con up '$con'" >/tmp/netswitch.log 2>&1 </dev/null &
    teardown_proxy
    log "address change scheduled in 2 s (this SSH session will drop); reconnect to 192.168.11.80"
    return 0
  fi
  warn "this replaces DHCP on $iface with 192.168.11.80/24 and NO gateway (the robot subnet has no internet)"
  sed -e "s/__IFACE__/$iface/" -e "s/renderer: networkd/renderer: ${NETPLAN_RENDERER:-networkd}/" \
    "$HERE/netplan/01-robot-disco.yaml" | sudo tee /etc/netplan/01-robot-disco.yaml >/dev/null
  sudo chmod 600 /etc/netplan/01-robot-disco.yaml
  # Ubuntu's installer leaves a DHCP config for the same interface; it would keep a second address.
  for f in /etc/netplan/*.yaml; do
    [ "$f" = /etc/netplan/01-robot-disco.yaml ] && continue
    if grep -q "$iface" "$f"; then
      sudo mv "$f" "$f.disabled-by-provision"
      warn "moved $f aside (it configured $iface too)"
    fi
  done
  sudo netplan generate && sudo netplan apply
  teardown_proxy
  ip -br addr show "$iface"
}

phase_all() {
  for p in preflight base ros groups udev sysctl workspace face audio services llm display foxglove; do "phase_$p"; done
  log "Done. Next: reboot (groups + autologin), then optionally 'provision.sh vla' and 'provision.sh network'."
}

# ---------------------------------------------------------------------------------------------
# PHASES lists every phase a caller may name; jetson/provision_jetson.sh sources this file and extends it.
PHASES="preflight base ros groups udev sysctl workspace face audio services llm display foxglove vla network"

dispatch() {
  [ $# -gt 0 ] || { sed -n '2,32p' "$0"; exit 1; }
  local phase
  for phase in "$@"; do
    if [ "$phase" = all ]; then
      phase_all
    elif [[ " $PHASES " == *" $phase "* ]]; then
      [ "$phase" = preflight ] || phase_preflight
      "phase_$phase"
    else
      die "unknown phase '$phase'"
    fi
  done
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  dispatch "$@"
fi
