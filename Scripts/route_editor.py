#!/usr/bin/env python3
"""Route editor: a web page on the robot that shows the map and lets you make a route for path.py.

    cd ~/rd_ws/Scripts && python3 route_editor.py           # then open http://192.168.11.80:8092/
    python3 route_editor.py --port 8092 --map ../src/robot_disco/maps/zone_one.yaml --no-ros

Map view (`?view=map`, the manager's Map tab): the same map with the robot and both RPLiDAR scans
(S1 yellow, S2E green) transformed into the map frame by this node — the first piece of the
Foxglove-free robot console. /api/scans serves them; the page polls it a few times a second.

What it does: draws the nav2 map (the same zone_one.yaml AMCL uses) with a 1 m grid, loads any route
.yaml from this directory (and the matching .raw.yaml trace, greyed), shows the robot's live map pose,
and edits the route: click on the map to add a waypoint after the selected one, drag to move, shift-drag
to turn its heading, type a name and post_actions in the side panel, reorder / delete, "add at robot".
Save writes <name>.yaml in path.py's format (via record_path.write_route) and runs `path.py --check`
on it; the centre / densify buttons run record_path.py's tools on the saved file and reload it.

Plain stdlib HTTP, LAN only, no auth — like the other robot pages. The live pose comes from TF
(map -> base_footprint) through rclpy when ROS is sourced; without it the page just has no robot dot.
"""
from __future__ import annotations

import argparse
import io
import json
import math
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import record_path as rp  # noqa: E402  (write_route, quat_of)

STATE: dict = {"map": None, "png": None, "pose": None, "pose_t": 0.0, "ros": "off", "scans": {}}
# Drawn yellow / green on the map view. The FILTERED scans by default: the laser_filters polygon chain
# has removed the robot's own body, and these are what the costmaps and AMCL consume (--raw-scans for
# the driver output, --scan-topics name=topic,... for anything else).
SCAN_TOPICS = {"s1": "/rp_s1_lidar/scan_filtered", "s2e": "/rp_s2e_lidar/scan_filtered"}
RAW_SCAN_TOPICS = {"s1": "/rp_s1_lidar/scan", "s2e": "/rp_s2e_lidar/scan"}
SCAN_MAX_POINTS = 720


# --------------------------------------------------------------------------- map
def load_map(yaml_path: Path) -> None:
    meta = yaml.safe_load(open(yaml_path))
    img_path = yaml_path.parent / meta["image"]
    from PIL import Image

    im = Image.open(img_path).convert("L")
    buf = io.BytesIO()
    im.save(buf, "PNG")
    STATE["png"] = buf.getvalue()
    STATE["map"] = {"resolution": float(meta["resolution"]), "origin": [float(meta["origin"][0]), float(meta["origin"][1])],
                    "width": im.size[0], "height": im.size[1], "name": yaml_path.name, "image": "/map.png"}


# --------------------------------------------------------------------------- live pose (optional)
def ros_pose_thread() -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener
    except Exception as e:  # noqa: BLE001
        STATE["ros"] = f"no rclpy/tf2 ({type(e).__name__}): run with ROS sourced for the live pose"
        return
    try:
        import numpy as np
        from sensor_msgs.msg import LaserScan

        rclpy.init()
        node = Node("route_editor")
        buf = Buffer()
        TransformListener(buf, node)
        latest: dict = {}

        def _keep(name):
            return lambda m: latest.__setitem__(name, m)

        for name, topic in SCAN_TOPICS.items():
            node.create_subscription(LaserScan, topic, _keep(name), 5)

        def _scan_to_map(m) -> "list | None":
            """One scan -> [[x, y], ...] in the map frame (latest TF), downsampled to SCAN_MAX_POINTS."""
            try:
                tf = buf.lookup_transform("map", m.header.frame_id, rclpy.time.Time())
            except Exception:  # noqa: BLE001 — no TF for that lidar yet
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            r = np.asarray(m.ranges, dtype=float)
            ang = m.angle_min + np.arange(len(r)) * m.angle_increment
            ok = np.isfinite(r) & (r > max(0.05, m.range_min)) & (r < min(30.0, m.range_max or 30.0))
            r, ang = r[ok], ang[ok]
            if len(r) > SCAN_MAX_POINTS:
                step = int(math.ceil(len(r) / SCAN_MAX_POINTS))
                r, ang = r[::step], ang[::step]
            px, py = r * np.cos(ang), r * np.sin(ang)
            mx = t.x + px * math.cos(yaw) - py * math.sin(yaw)
            my = t.y + px * math.sin(yaw) + py * math.cos(yaw)
            return np.round(np.stack([mx, my], 1), 3).tolist()

        last_scan_pub = 0.0
        STATE["ros"] = "waiting for TF map->base_footprint"
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            now_t = time.time()
            if now_t - last_scan_pub > 0.2:      # ~5 Hz is plenty for a browser
                last_scan_pub = now_t
                scans = {}
                for name, m in list(latest.items()):
                    pts = _scan_to_map(m)
                    if pts is not None:
                        scans[name] = {"points": pts, "frame": m.header.frame_id, "topic": SCAN_TOPICS[name], "t": now_t}
                STATE["scans"] = scans
            try:
                tf = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                t, q = tf.transform.translation, tf.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                STATE["pose"] = {"x": round(t.x, 4), "y": round(t.y, 4), "yaw": round(yaw, 4)}
                STATE["pose_t"] = time.time()
                STATE["ros"] = "ok"
            except Exception:  # noqa: BLE001
                if time.time() - STATE["pose_t"] > 3.0:
                    STATE["ros"] = "no map->base_footprint (localization not running or not initialised)"
            time.sleep(0.1)
    except Exception as e:  # noqa: BLE001
        STATE["ros"] = f"ros thread died: {e}"


# --------------------------------------------------------------------------- routes
def route_files() -> list[str]:
    return sorted(p.name[:-5] for p in HERE.glob("*.yaml") if not p.name.endswith(".raw.yaml"))


def yaw_of_q(o: dict) -> float:
    z, w = float(o.get("z", 0.0)), float(o.get("w", 1.0))
    return math.atan2(2 * w * z, 1 - 2 * z * z)


def load_route(name: str) -> dict:
    p = HERE / f"{name}.yaml"
    d = yaml.safe_load(open(p)) or {}
    wps = []
    for w in d.get("waypoints") or []:
        pos, o = w.get("position", {}) or {}, w.get("orientation", {}) or {}
        wps.append({"name": str(w.get("name", "")), "x": float(pos.get("x", 0)), "y": float(pos.get("y", 0)),
                    "yaw": yaw_of_q(o), "post_actions": w.get("post_actions") or [], "pause": float(w.get("pause") or 0)})
    out = {"name": name, "waypoints": wps, "initial_pose": d.get("initial_pose")}
    raw = HERE / f"{name}.raw.yaml"
    if raw.exists():
        r = yaml.safe_load(open(raw)) or {}
        out["raw_trace"] = [[s[1], s[2]] for s in r.get("samples") or []]
    return out


def save_route(name: str, data: dict) -> str:
    wps = []
    for w in data.get("waypoints") or []:
        q = rp.quat_of(float(w.get("yaw", 0.0)))
        entry = {"name": str(w.get("name") or "wp"), "position": {"x": round(float(w["x"]), 4), "y": round(float(w["y"]), 4), "z": 0.0},
                 "orientation": {"x": 0.0, "y": 0.0, "z": q["z"], "w": q["w"]}}
        pa = w.get("post_actions")
        if isinstance(pa, dict) and "_yaml" in pa:           # typed in the page's textarea
            pa = yaml.safe_load(pa["_yaml"]) or []
            if not isinstance(pa, list):
                raise ValueError(f"post_actions of {entry['name']!r} must be a YAML list")
        if pa:
            entry["post_actions"] = pa
        if float(w.get("pause") or 0) > 0:
            entry["pause"] = float(w["pause"])
        wps.append(entry)
    route = {"waypoints": wps}
    if data.get("initial_pose"):
        route["initial_pose"] = data["initial_pose"]
    p = HERE / f"{name}.yaml"
    rp.write_route(route, p, [f"# {name}.yaml — edited with route_editor.py on {time.strftime('%Y-%m-%d %H:%M')} ({len(wps)} waypoints)."])
    return check_route(name)


def check_route(name: str) -> str:
    r = subprocess.run([sys.executable, str(HERE / "path.py"), "--check", str(HERE / f"{name}.yaml")],
                       capture_output=True, text=True, timeout=30)
    return (r.stdout + r.stderr).strip()


def run_tool(tool: str, name: str, args: dict) -> str:
    cmd = [sys.executable, str(HERE / "record_path.py")]
    if tool == "center":
        cmd += ["center", str(HERE / f"{name}.yaml"), "--map", str(ARGS.map), "--max-shift", str(args.get("max_shift", 0.6))]
    elif tool == "densify":
        cmd += ["densify", str(HERE / f"{name}.yaml"), "--spacing", str(args.get("spacing", 1.0))]
    else:
        return f"unknown tool {tool}"
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return (r.stdout + r.stderr).strip()


# --------------------------------------------------------------------------- http
class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *a):  # quiet
        pass

    def _send(self, code: int, body: bytes, ctype: str = "application/json") -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code: int = 200) -> None:
        self._send(code, json.dumps(obj).encode())

    def do_GET(self) -> None:  # noqa: N802
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        try:
            if u.path == "/":
                self._send(200, (HERE / "route_editor.html").read_bytes(), "text/html; charset=utf-8")
            elif u.path == "/map.png":
                self._send(200, STATE["png"], "image/png")
            elif u.path == "/api/map":
                self._json(STATE["map"])
            elif u.path == "/api/routes":
                self._json({"routes": route_files()})
            elif u.path == "/api/route":
                self._json(load_route(q["name"]))
            elif u.path == "/api/pose":
                fresh = STATE["pose"] if time.time() - STATE["pose_t"] < 3.0 else None
                self._json({"pose": fresh, "ros": STATE["ros"]})
            elif u.path == "/api/scans":
                now = time.time()
                self._json({"scans": {k: v for k, v in STATE["scans"].items() if now - v["t"] < 3.0},
                            "topics": SCAN_TOPICS, "ros": STATE["ros"]})
            else:
                self._send(404, b"not found", "text/plain")
        except Exception as e:  # noqa: BLE001
            self._json({"error": f"{type(e).__name__}: {e}"}, 500)

    def do_POST(self) -> None:  # noqa: N802
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        n = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(n) or b"{}") if n else {}
        try:
            name = q.get("name") or body.get("name") or ""
            if not name or "/" in name or name.startswith("."):
                self._json({"error": "bad route name"}, 400)
                return
            if u.path == "/api/route":
                self._json({"ok": True, "check": save_route(name, body)})
            elif u.path == "/api/tool":
                out = run_tool(q.get("tool", ""), name, body)
                self._json({"ok": True, "output": out, "route": load_route(name)})
            else:
                self._send(404, b"not found", "text/plain")
        except Exception as e:  # noqa: BLE001
            self._json({"error": f"{type(e).__name__}: {e}"}, 500)


def main() -> int:
    global ARGS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--map", type=Path, default=HERE.parent / "src" / "robot_disco" / "maps" / "zone_one.yaml")
    ap.add_argument("--no-ros", action="store_true", help="do not try to read the robot's pose")
    ap.add_argument("--raw-scans", action="store_true", help="draw the lidar drivers' raw scans instead of the filtered ones")
    ap.add_argument("--scan-topics", default=None, metavar="name=topic,...",
                    help="override the scan topics drawn on the map (default: the filtered RPLiDAR scans)")
    ARGS = ap.parse_args()
    if ARGS.scan_topics:
        SCAN_TOPICS.clear()
        SCAN_TOPICS.update(dict(kv.split("=", 1) for kv in ARGS.scan_topics.split(",")))
    elif ARGS.raw_scans:
        SCAN_TOPICS.clear()
        SCAN_TOPICS.update(RAW_SCAN_TOPICS)
    load_map(ARGS.map)
    if not ARGS.no_ros:
        threading.Thread(target=ros_pose_thread, daemon=True).start()
    else:
        STATE["ros"] = "disabled (--no-ros)"
    srv = ThreadingHTTPServer((ARGS.host, ARGS.port), Handler)
    print(f"[route_editor] map {ARGS.map.name} {STATE['map']['width']}x{STATE['map']['height']} @ {STATE['map']['resolution']} m; "
          f"routes: {route_files()}; scans: {SCAN_TOPICS}\n[route_editor] open http://<this host>:{ARGS.port}/", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
