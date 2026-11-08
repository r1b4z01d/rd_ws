#!/usr/bin/env bash
# Push this working tree's robot-side code to RD and bring it up.
#
#   provision/deploy_to_rd.sh            # rsync src/robot_face src/robot_voice + units + provision scripts, build, restart face, (re)start voice
#   provision/deploy_to_rd.sh --setup    # additionally run setup_faceid.sh / setup_voice.sh / setup_kokoro.sh on RD (first time)
#   provision/deploy_to_rd.sh --vla      # additionally push the VLA submodule (rd_ws/VLA -> RD:~/rd_ws/VLA) and reinstall its units
#
# RD is not git-synced: its rd_ws — VLA included, since VLA is the rd_ws/VLA submodule — is an rsync of this
# tree, and the robot has no internet, so `git submodule update` can never run there. This is THE way code
# gets there. The VLA push never uses --delete and skips the robot's own .venv, outputs (datasets,
# checkpoints, 8 GB), log and config/*.env; install_services.sh then re-reads services/rd/*.conf, which
# restarts a running vla_cameras / vla_camera_ros_bridge only if its unit file changed.
# Needs the ssh host "RD" (192.168.11.80) — i.e. this machine on the robot network.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$HERE/.." && pwd)"
SETUP=0; VLA=0
for a in "$@"; do
  case "$a" in --setup) SETUP=1 ;; --vla) VLA=1 ;; *) echo "usage: $0 [--setup] [--vla]" >&2; exit 2 ;; esac
done
SSH="ssh -o ConnectTimeout=8 -o BatchMode=yes"

cd "$WS"
$SSH RD 'echo "RD reachable: $(hostname), up $(uptime -p)"'
rsync -a -e "$SSH" --exclude __pycache__ src/robot_face/  RD:/home/rd/rd_ws/src/robot_face/
rsync -a -e "$SSH" --exclude __pycache__ src/robot_voice/ RD:/home/rd/rd_ws/src/robot_voice/
rsync -a -e "$SSH" Scripts/path.py RD:/home/rd/rd_ws/Scripts/                      # the route runner's VLA_DIR (rd_ws/VLA)
rsync -a -e "$SSH" src/robot_disco/scripts/ur_joint_state_publisher.py RD:/home/rd/rd_ws/src/robot_disco/scripts/   # symlink-installed
rsync -a -e "$SSH" services/robot_face.conf services/robot_voice.conf RD:/home/rd/rd_ws/services/
rsync -a -e "$SSH" provision/setup_faceid.sh provision/setup_voice.sh provision/setup_kokoro.sh \
                   provision/provision.sh provision/capture_rd_state.sh RD:/home/rd/rd_ws/provision/
rsync -a -e "$SSH" provision/jetson/provision_jetson.sh RD:/home/rd/rd_ws/provision/jetson/
rsync -a -e "$SSH" provision/tools/ RD:/home/rd/rd_ws/provision/tools/                  # RoboClaw / UART probes
rsync -a -e "$SSH" README.md RD:/home/rd/rd_ws/
if [ $VLA = 1 ]; then
  # a robot still on the old layout (~/VLA beside ~/rd_ws) must be moved first, or this would plant a second,
  # venv-less tree at ~/rd_ws/VLA that provision_jetson.sh vlamove then refuses to overwrite
  $SSH RD 'test ! -e ~/VLA' || { echo "RD still has the old ~/VLA layout: run 'provision_jetson.sh vlamove' on RD first (provision/README.md §3)" >&2; exit 1; }
  rsync -a -e "$SSH" --exclude /.git --exclude '/.venv*' --exclude /outputs --exclude /log --exclude /config \
        --exclude __pycache__ --exclude /ros2_ws/build --exclude /ros2_ws/install --exclude /ros2_ws/log \
        VLA/ RD:/home/rd/rd_ws/VLA/
fi
echo "pushed"

$SSH RD "set -e; export XDG_RUNTIME_DIR=/run/user/1000; chmod +x ~/rd_ws/provision/*.sh ~/rd_ws/provision/jetson/*.sh   # no set -u: ROS setup.bash trips it
if [ $SETUP = 1 ]; then
  echo '=== setup: face ID (ArcFace)';   ~/rd_ws/provision/setup_faceid.sh 2>&1 | tail -3
  echo '=== setup: voice (whisper)';     ~/rd_ws/provision/setup_voice.sh base.en 2>&1 | tail -4
  [ -x ~/.robot_face/kokoro-venv/bin/python ] || { echo '=== setup: kokoro'; ~/rd_ws/provision/setup_kokoro.sh 2>&1 | tail -2; }
fi
echo '=== build'; cd ~/rd_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select robot_face robot_voice --symlink-install 2>&1 | tail -2
echo '=== units'; sudo -n cp ~/rd_ws/services/robot_face.conf ~/rd_ws/services/robot_voice.conf /etc/supervisor/conf.d/
sudo -n supervisorctl reread >/dev/null; sudo -n supervisorctl update >/dev/null || true
if [ $VLA = 1 ]; then echo '=== vla units'; (cd ~/rd_ws/VLA && sudo -n ./scripts/install_services.sh rd 2>&1 | tail -4); fi
sudo -n supervisorctl restart robot_face | tail -1
if sudo -n supervisorctl status robot_voice | grep -q RUNNING; then sudo -n supervisorctl restart robot_voice | tail -1; else sudo -n supervisorctl start robot_voice | tail -1; fi
sleep 45; sudo -n supervisorctl status robot_face robot_voice | awk '{print \$1, \$2}'
echo '=== face log'; grep -E 'Face ID|Kokoro|Traceback' ~/rd_ws/log/robot_face-launch.err.log | tail -3
echo '=== voice log'; grep -E 'Brain|Ears|Traceback|WARN' ~/rd_ws/log/robot_voice.err.log | tail -4
grep -q 'ANTHROPIC_API_KEY=.\+' ~/.robot_face/voice.env 2>/dev/null || echo 'NOTE: ANTHROPIC_API_KEY is empty in ~/.robot_face/voice.env — Claude answers are off until it is set (then: sudo supervisorctl restart robot_voice)'"
