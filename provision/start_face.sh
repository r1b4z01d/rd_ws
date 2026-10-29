#!/usr/bin/env bash
# Start the robot face on the robot PC's own display, detached from the calling shell.
# The supervisor unit `robot_face` is the normal way to run it (its environment carries the display
# variables since 2026-10-28); this script is the by-hand fallback: it exports the GNOME session
# environment and runs the node detached. Installed to
# /home/rd/start_face.sh by provision.sh (phase "services"). Log: ~/rd_ws/log/robot_face-manual.log
set -euo pipefail
WS="$HOME/rd_ws"
LOG="$WS/log/robot_face-manual.log"
mkdir -p "$WS/log"

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
# glfw picks its platform from these: a Wayland session (the old Intel PC) needs WAYLAND_DISPLAY,
# while under the Jetson's X11 GDM session the Wayland variables make glfw.init() fail, and the
# X server wants GDM's Xauthority file (there is no ~/.Xauthority).
if [ -S "$XDG_RUNTIME_DIR/${WAYLAND_DISPLAY:-wayland-0}" ]; then
  export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
  export XDG_SESSION_TYPE="${XDG_SESSION_TYPE:-wayland}"
else
  unset WAYLAND_DISPLAY
  export XDG_SESSION_TYPE=x11
  [ -r "$XDG_RUNTIME_DIR/gdm/Xauthority" ] && export XAUTHORITY="${XAUTHORITY:-$XDG_RUNTIME_DIR/gdm/Xauthority}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Stop a previous instance. The bracket keeps this script's own command line from matching.
pkill -f "lib/robot_face/[c]ombined_face_hand_node" 2>/dev/null || true
sleep 0.5

set +u   # ROS setup files reference unset variables
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

nohup ros2 run robot_face combined_face_hand_node "$@" >>"$LOG" 2>&1 </dev/null &
disown
echo "robot_face started (pid $!), logging to $LOG"
