#!/usr/bin/env bash
# The whole robot GUI on a laptop, no robot needed:
#   route editor + 2D map + 3D view with a simulated robot   (route_editor.py --sim, :8094 / cams :8095)
#   teleop panel with the MuJoCo arm and on-screen sliders    (manual_panel --engine mujoco, :8090)
#   VLA manager with every tab pointed at the two above       (webui.server, :8081)
#
#   Scripts/dev_stack.sh start|stop|status|logs
#
# The manager's Map tab also starts the simulator by itself when its switch goes to sim (same port).
# Needs: python3 with numpy/Pillow/PyYAML for the editor; VLA/.venv with mujoco, lerobot, fastapi,
# uvicorn, python-multipart for the panel and the manager. Logs go to /tmp/dev_stack/.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
VLA="${VLA_DIR:-$HERE/../../VLA}"
LOG=/tmp/dev_stack; mkdir -p "$LOG"
EDITOR_PORT="${EDITOR_PORT:-8094}"; PANEL_PORT="${PANEL_PORT:-8090}"; MANAGER_PORT="${MANAGER_PORT:-8081}"   # 8094: the sim never shares the live editor's 8092
ROUTE="${SIM_ROUTE:-}"

running() { pgrep -f "$1" >/dev/null 2>&1; }
start_one() {   # name, pattern, log, command...
  local name=$1 pat=$2 log=$3; shift 3
  if running "$pat"; then echo "  $name: already running"; return; fi
  nohup "$@" >"$LOG/$log" 2>&1 &
  echo "  $name: started (pid $!, log $LOG/$log)"
}
case "${1:-status}" in
  start)
    echo "starting the local stack"
    ( cd "$HERE" && start_one "route editor (sim)" "route_editor.py --sim" editor.log python3 route_editor.py --sim --port "$EDITOR_PORT" ${ROUTE:+--sim-route "$ROUTE"} )
    ( cd "$VLA" && start_one "teleop panel (mujoco)" "ur5e_lerobot.teleop.manual_panel" panel.log .venv/bin/python -m ur5e_lerobot.teleop.manual_panel --engine mujoco --input sliders --ui web --web-port "$PANEL_PORT" )
    # the Map / 3D tabs start on the simulation (the Map tab's switch flips to the robot's map); the
    # Teleop tab shows the local MuJoCo panel since there is no robot here
    ( cd "$VLA" && start_one "manager" "webui.server" manager.log .venv/bin/python -m webui.server --port "$MANAGER_PORT" --map-source sim --sim-route-url "http://127.0.0.1:$EDITOR_PORT/" --robot-url "http://127.0.0.1:$PANEL_PORT/" )
    sleep 4
    echo "open http://127.0.0.1:$MANAGER_PORT/   (Robot / Routes / Map / 3D tabs; the panel alone: :$PANEL_PORT, the 3D view alone: :$EDITOR_PORT/3d)"
    ;;
  stop)
    for pat in "route_editor.py --sim" "ur5e_lerobot.teleop.manual_panel" "webui.server"; do pkill -f "$pat" && echo "  stopped: $pat" || echo "  not running: $pat"; done
    ;;
  status)
    for pat in "route_editor.py --sim" "ur5e_lerobot.teleop.manual_panel" "webui.server"; do if running "$pat"; then echo "  running: $pat"; else echo "  stopped: $pat"; fi; done
    for p in $EDITOR_PORT $PANEL_PORT $MANAGER_PORT; do printf "  :%s -> " "$p"; curl -s -m 2 -o /dev/null -w "%{http_code}\n" "http://127.0.0.1:$p/" || echo "no answer"; done
    ;;
  logs) tail -n 20 "$LOG"/*.log ;;
  *) echo "usage: $0 start|stop|status|logs"; exit 2 ;;
esac
