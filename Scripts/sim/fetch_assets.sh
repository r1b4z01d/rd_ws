#!/usr/bin/env bash
# Pull the robot's real URDF and the UR5e visual meshes over HTTP from the route editor on RD, so the
# simulator (route_editor.py --sim) shows the real model instead of the primitive robot_sim.urdf.
# Needs the robot reachable; nothing else. Re-run after a URDF change on the robot.
set -euo pipefail
RD="${RD_URL:-http://192.168.11.80:8092}"
HERE="$(cd "$(dirname "$0")" && pwd)"
curl -sf -m 10 "$RD/api/urdf" -o "$HERE/robot_disco.urdf.tmp"
grep -q "<robot" "$HERE/robot_disco.urdf.tmp" && mv "$HERE/robot_disco.urdf.tmp" "$HERE/robot_disco.urdf"
echo "URDF: $(wc -c < "$HERE/robot_disco.urdf") bytes"
D="$HERE/meshes/ur_description/meshes/ur5e/visual"; mkdir -p "$D"
for f in base forearm shoulder upperarm wrist1 wrist2 wrist3; do
  curl -sf -m 60 "$RD/mesh/ur_description/meshes/ur5e/visual/$f.dae" -o "$D/$f.dae" && echo "  $f.dae $(wc -c < "$D/$f.dae") bytes"
done
echo "done — restart route_editor.py --sim to use them"
