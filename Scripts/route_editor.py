#!/usr/bin/env python3
"""Route editor: a web page on the robot that shows the map and lets you make a route for path.py.

    cd ~/rd_ws/Scripts && python3 route_editor.py           # then open http://192.168.11.80:8092/
    python3 route_editor.py --port 8092 --map ../src/robot_disco/maps/zone_one.yaml --no-ros

Map view (`?view=map`, the manager's Map tab): the same map with the robot and both RPLiDAR scans
(S1 yellow, S2E green) transformed into the map frame by this node — the first piece of the
Foxglove-free robot console. /api/scans serves them; the page polls it a few times a second.

3D view (`/3d`, `map3d.html`, the manager's 3D tab): a three.js scene replicating Foxglove's 3D panel —
the map plane, costmap overlays (the local one placed through map->odom), the Velodyne and RPLiDAR
clouds (binary `/api/cloud.bin`, sensor frame + the frame's pose), the URDF robot model with live
joints (`/api/urdf`, meshes from the package share dirs under `/mesh/<pkg>/…`), every TF frame
(`/api/tf`), nav2's plans / goal / footprint / collision polygons / AMCL particles (`/api/nav`), the
route, a follow-robot camera, and click tools that publish /goal_pose and /initialpose
(`POST /api/publish`). three.js + urdf-loader are vendored under static/vendor (no internet on RD).

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

STATE: dict = {"map": None, "png": None, "pose": None, "pose_t": 0.0, "ros": "off", "scans": {}, "costmaps": {},
               "frame_tf": {},   # frame -> (x, y, yaw) of that frame in the map frame (for grids not published in "map")
               # 3D view: every TF frame in the map frame, joint positions, the URDF, nav2 state, raw sensor msgs
               "frames": {}, "joints": {}, "urdf": None, "nav": {}, "particles": None, "latest": {},
               "publish_q": [], "published": None, "want3d": {}}
COSTMAP_TOPICS = {"global": "/global_costmap/costmap", "local": "/local_costmap/costmap"}
# Drawn yellow / green on the map view. The FILTERED scans by default: the laser_filters polygon chain
# has removed the robot's own body, and these are what the costmaps and AMCL consume (--raw-scans for
# the driver output, --scan-topics name=topic,... for anything else).
SCAN_TOPICS = {"s1": "/rp_s1_lidar/scan_filtered", "s2e": "/rp_s2e_lidar/scan_filtered"}
RAW_SCAN_TOPICS = {"s1": "/rp_s1_lidar/scan", "s2e": "/rp_s2e_lidar/scan"}
# The Velodyne as a top-down projection of its point cloud, keeping the height band the costmaps mark
# (0.1-2.0 m above the floor): this is the layer that painted the phantom wall on 2026-10-27.
CLOUD_TOPICS = {"velodyne": "/velodyne_points"}
CLOUD_Z = (0.1, 2.0)
CLOUD_MAX_POINTS = 2500
SCAN_MAX_POINTS = 720
# 3D view (/3d): nav2's plans, the footprint / collision-monitor polygons, AMCL's particles
NAV_TOPICS = {"plan": "/plan", "local_plan": "/local_plan"}
POLY_TOPICS = {"footprint": "/local_costmap/published_footprint", "stop": "/polygon_stop", "slowdown": "/polygon_slowdown"}
CLOUD_BIN_MAX = 20000          # points per /api/cloud.bin answer (the Velodyne has ~29k per revolution)
MIME = {".js": "text/javascript", ".mjs": "text/javascript", ".css": "text/css", ".html": "text/html; charset=utf-8",
        ".png": "image/png", ".jpg": "image/jpeg", ".stl": "application/octet-stream", ".dae": "model/vnd.collada+xml",
        ".json": "application/json", ".md": "text/plain", ".txt": "text/plain"}


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
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2
        from sensor_msgs_py import point_cloud2 as pc2

        rclpy.init()
        node = Node("route_editor")
        buf = Buffer()
        TransformListener(buf, node)
        latest: dict = {}

        def _keep(name):
            return lambda m: latest.__setitem__(name, m)

        # sensor QoS (best effort) matches both the reliable driver output and the best-effort filter
        # chain output; a reliable subscriber silently never matched /rp_*_lidar/scan_filtered
        for name, topic in SCAN_TOPICS.items():
            node.create_subscription(LaserScan, topic, _keep(name), qos_profile_sensor_data)
        for name, topic in CLOUD_TOPICS.items():
            node.create_subscription(PointCloud2, topic, _keep(name), qos_profile_sensor_data)
        # costmaps: nav2 publishes them latched (transient local); keep the latest message, render on demand
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

        cm_qos = QoSProfile(depth=1)
        cm_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        cm_qos.reliability = ReliabilityPolicy.RELIABLE

        def _keep_costmap(name):
            def cb(m):
                STATE["costmaps"][name] = {"msg": m, "t": time.time()}
            return cb

        for name, topic in COSTMAP_TOPICS.items():
            node.create_subscription(OccupancyGrid, topic, _keep_costmap(name), cm_qos)
        STATE["latest"] = latest                       # raw scans / clouds for /api/cloud.bin
        # ---- 3D view inputs
        from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped
        from nav_msgs.msg import Path
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String

        def _yaw(q) -> float:
            return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        node.create_subscription(String, "/robot_description", lambda m: STATE.__setitem__("urdf", m.data), cm_qos)

        def _joints(m):
            for n_, p_ in zip(m.name, m.position):
                STATE["joints"][n_] = round(float(p_), 5)
        node.create_subscription(JointState, "/joint_states", _joints, 10)

        def _path(name):
            def cb(m):
                step = max(1, len(m.poses) // 400)
                STATE["nav"][name] = {"frame": m.header.frame_id, "t": time.time(),
                                      "pts": [[round(p.pose.position.x, 3), round(p.pose.position.y, 3), round(p.pose.position.z, 3)]
                                              for p in m.poses[::step]]}
            return cb
        for name, topic in NAV_TOPICS.items():
            node.create_subscription(Path, topic, _path(name), 10)

        def _goal(m):
            STATE["nav"]["goal"] = {"frame": m.header.frame_id, "x": round(m.pose.position.x, 3), "y": round(m.pose.position.y, 3),
                                    "yaw": round(_yaw(m.pose.orientation), 4), "t": time.time()}
        node.create_subscription(PoseStamped, "/goal_pose", _goal, 10)
        polys: dict = {}
        for name, topic in POLY_TOPICS.items():
            node.create_subscription(PolygonStamped, topic, (lambda n_: (lambda m: polys.__setitem__(n_, m)))(name), 10)
        try:
            from nav2_msgs.msg import ParticleCloud
            node.create_subscription(ParticleCloud, "/particle_cloud", _keep("particles"), qos_profile_sensor_data)
        except Exception:  # noqa: BLE001 — nav2_msgs not installed: no particles layer
            pass
        # click-to-publish from the 3D page (queued by the HTTP thread, sent from here)
        # transient-local so a late or latched-expecting subscriber (one asked for DURABILITY) still matches
        goal_pub = node.create_publisher(PoseStamped, "/goal_pose", cm_qos)
        init_pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        def _frame2d(frame):
            if frame in ("", "map"):
                return 0.0, 0.0, 0.0
            f = STATE["frames"].get(frame)
            if not f:
                return None
            return f[0], f[1], math.atan2(2 * (f[6] * f[5] + f[3] * f[4]), 1 - 2 * (f[4] * f[4] + f[5] * f[5]))

        def _tf2d(frame):
            tf = buf.lookup_transform("map", frame, rclpy.time.Time())
            t, q = tf.transform.translation, tf.transform.rotation
            return t.x, t.y, t.z, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        def _cloud_to_map(m) -> "list | None":
            try:
                tx, ty, tz, yaw = _tf2d(m.header.frame_id)
            except Exception:  # noqa: BLE001
                return None
            # read_points (not read_points_numpy): the Velodyne cloud mixes float32 x/y/z with other
            # field types, which the numpy reader refuses ("All fields need to have the same datatype")
            arr = pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)
            if len(arr) == 0:
                return []
            pts = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(float)
            z = pts[:, 2] + tz
            keep = (z > CLOUD_Z[0]) & (z < CLOUD_Z[1])
            pts = pts[keep]
            if len(pts) > CLOUD_MAX_POINTS:
                pts = pts[:: int(math.ceil(len(pts) / CLOUD_MAX_POINTS))]
            c, s_ = math.cos(yaw), math.sin(yaw)
            mx = tx + pts[:, 0] * c - pts[:, 1] * s_
            my = ty + pts[:, 0] * s_ + pts[:, 1] * c
            return np.round(np.stack([mx, my], 1), 3).tolist()

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
        frames_listed_t, frame_names = 0.0, []
        STATE["ros"] = "waiting for TF map->base_footprint"
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
            now_t = time.time()
            if now_t - last_scan_pub > 0.2:      # ~5 Hz is plenty for a browser
                last_scan_pub = now_t
                want = STATE.get("want") or {}
                wanted = set(want.get("names") or []) if now_t - want.get("t", 0) < 3.0 else set()
                scans = dict(STATE["scans"])
                for name, m in list(latest.items()):
                    try:
                        if name in CLOUD_TOPICS:
                            if name not in wanted:        # the cloud is the expensive one: only when shown
                                continue
                            pts = _cloud_to_map(m)
                            topic = CLOUD_TOPICS[name]
                        else:
                            pts = _scan_to_map(m)
                            topic = SCAN_TOPICS[name]
                    except Exception as e:  # noqa: BLE001 — one bad message must not kill the node
                        STATE["ros"] = f"{name}: {type(e).__name__}: {str(e)[:80]}"
                        continue
                    if pts is not None:
                        scans[name] = {"points": pts, "frame": m.header.frame_id, "topic": topic, "t": now_t}
                STATE["scans"] = scans
                # the rolling local costmap is published in odom: keep map->odom (AMCL's correction,
                # translation AND rotation) so the page can place it on the map
                for entry in list(STATE["costmaps"].values()):
                    fr = entry["msg"].header.frame_id
                    if fr and fr != "map":
                        try:
                            tx_, ty_, _, yaw_ = _tf2d(fr)
                            STATE["frame_tf"][fr] = (tx_, ty_, yaw_)
                        except Exception:  # noqa: BLE001 — not localized yet
                            pass
                # ---- 3D view: every frame in the map frame (frame list refreshed every 2 s)
                if now_t - frames_listed_t > 2.0:
                    frames_listed_t = now_t
                    try:
                        frame_names = sorted((yaml.safe_load(buf.all_frames_as_yaml()) or {}).keys())
                    except Exception:  # noqa: BLE001
                        frame_names = []
                fr = {"map": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]}
                for f in frame_names:
                    try:
                        tf = buf.lookup_transform("map", f, rclpy.time.Time())
                        t, q = tf.transform.translation, tf.transform.rotation
                        fr[f] = [round(t.x, 4), round(t.y, 4), round(t.z, 4), round(q.x, 5), round(q.y, 5), round(q.z, 5), round(q.w, 5)]
                    except Exception:  # noqa: BLE001 — frame not connected to map (yet)
                        pass
                STATE["frames"] = fr
                polys_out = {}
                for pname, pm in list(polys.items()):
                    f2 = _frame2d(pm.header.frame_id)
                    if f2 is None:
                        continue
                    tx_, ty_, yaw_ = f2
                    c_, s_ = math.cos(yaw_), math.sin(yaw_)
                    polys_out[pname] = {"frame": pm.header.frame_id,
                                        "pts": [[round(tx_ + p.x * c_ - p.y * s_, 3), round(ty_ + p.x * s_ + p.y * c_, 3)] for p in pm.polygon.points]}
                STATE["nav"]["polygons"] = polys_out
                pc_ = latest.get("particles")
                if pc_ is not None and now_t - STATE["want3d"].get("particles", 0) < 3.0:
                    ps = pc_.particles
                    step = max(1, len(ps) // 1500)
                    STATE["particles"] = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3), round(_yaw(p.pose.orientation), 3)]
                                          for p in ps[::step]]
                while STATE["publish_q"]:
                    req = STATE["publish_q"].pop(0)
                    stamp = node.get_clock().now().to_msg()
                    qz, qw = math.sin(req["yaw"] / 2), math.cos(req["yaw"] / 2)
                    if req["type"] == "goal":
                        g = PoseStamped()
                        g.header.frame_id, g.header.stamp = "map", stamp
                        g.pose.position.x, g.pose.position.y = req["x"], req["y"]
                        g.pose.orientation.z, g.pose.orientation.w = qz, qw
                        goal_pub.publish(g)
                    else:
                        ip = PoseWithCovarianceStamped()
                        ip.header.frame_id, ip.header.stamp = "map", stamp
                        ip.pose.pose.position.x, ip.pose.pose.position.y = req["x"], req["y"]
                        ip.pose.pose.orientation.z, ip.pose.pose.orientation.w = qz, qw
                        cov = [0.0] * 36
                        cov[0] = cov[7] = 0.25
                        cov[35] = 0.0685
                        ip.pose.covariance = cov
                        init_pub.publish(ip)
                    STATE["published"] = {**req, "t": time.time()}
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


# --------------------------------------------------------------------------- costmap rendering
_cm_cache: dict = {}   # name -> {"stamp": (sec, nsec), "png": bytes, "meta": dict}


def costmap_png(name: str) -> "tuple[bytes, dict] | None":
    """The latest costmap as a transparent PNG (free / unknown clear; inflation yellow->orange, inscribed
    orange, lethal red) plus its georeference. Rendered once per new message."""
    import numpy as np
    from PIL import Image

    entry = STATE["costmaps"].get(name)
    if not entry:
        return None
    m = entry["msg"]
    st = (m.header.stamp.sec, m.header.stamp.nanosec, m.info.width, m.info.height)
    c = _cm_cache.get(name)
    if c and c["stamp"] == st:
        return c["png"], _georef(c["meta"], entry["t"])
    w, h = m.info.width, m.info.height
    d = np.array(m.data, dtype=np.int16).reshape(h, w)
    rgba = np.zeros((h, w, 4), np.uint8)
    g = (d > 0) & (d < 99)
    rgba[g, 0] = 255
    rgba[g, 1] = (255 - d[g] * 2).clip(60, 255)
    rgba[g, 2] = 0
    rgba[g, 3] = 255
    rgba[d == 99] = (255, 120, 0, 255)
    rgba[d == 100] = (200, 0, 0, 255)
    buf = io.BytesIO()
    Image.fromarray(rgba[::-1]).save(buf, "PNG")      # row 0 of the grid is the bottom
    q = m.info.origin.orientation
    base = {"name": name, "topic": COSTMAP_TOPICS[name], "resolution": float(m.info.resolution),
            "frame": m.header.frame_id or "map",
            "origin_frame": [float(m.info.origin.position.x), float(m.info.origin.position.y)],
            "origin_yaw": math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
            "width": w, "height": h, "stamp": f"{st[0]}.{st[1]:09d}",
            "image": f"/costmap/{name}.png?v={st[0]}.{st[1]}"}
    _cm_cache[name] = {"stamp": st, "png": buf.getvalue(), "meta": base}
    return _cm_cache[name]["png"], _georef(base, entry["t"])


def _georef(base: dict, t_rx: float) -> dict:
    """The grid's bottom-left corner and x-axis heading IN THE MAP FRAME (the global costmap is published
    in map, the local one in odom; map->odom is AMCL's drifting correction, re-read on every request)."""
    meta = dict(base)
    meta["age_s"] = round(time.time() - t_rx, 1)
    ox, oy = base["origin_frame"]
    if base["frame"] == "map":
        meta["origin"], meta["yaw"] = [ox, oy], base["origin_yaw"]
        return meta
    tf = STATE["frame_tf"].get(base["frame"])
    if tf is None:
        meta["error"] = f"no TF map->{base['frame']} yet (localization not initialised?)"
        return meta
    tx, ty, tyaw = tf
    c, s_ = math.cos(tyaw), math.sin(tyaw)
    meta["origin"] = [round(tx + ox * c - oy * s_, 4), round(ty + ox * s_ + oy * c, 4)]
    meta["yaw"] = base["origin_yaw"] + tyaw
    return meta


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
# --------------------------------------------------------------------------- 3D view helpers
def cloud_bin(name: str, max_points: int) -> "bytes | None":
    """One scan / cloud as a binary blob: u32 little-endian header length, a JSON header (frame, n, the
    frame's pose in the map frame, stamp), then n x [x, y, z, intensity] float32 in the SENSOR frame."""
    import numpy as np

    m = STATE["latest"].get(name)
    if m is None:
        return None
    if name in CLOUD_TOPICS:
        from sensor_msgs_py import point_cloud2 as pc2

        names = [f.name for f in m.fields]
        want = ["x", "y", "z"] + (["intensity"] if "intensity" in names else [])
        arr = pc2.read_points(m, field_names=want, skip_nans=True)
        step = max(1, int(math.ceil(len(arr) / max(1, max_points))))
        arr = arr[::step]
        out = np.empty((len(arr), 4), np.float32)
        out[:, 0], out[:, 1], out[:, 2] = arr["x"], arr["y"], arr["z"]
        out[:, 3] = arr["intensity"] if "intensity" in want else 0.0
    else:
        r = np.asarray(m.ranges, dtype=np.float32)
        ang = (m.angle_min + np.arange(len(r)) * m.angle_increment).astype(np.float32)
        ok = np.isfinite(r) & (r > max(0.02, m.range_min)) & (r < min(60.0, m.range_max or 60.0))
        inten = np.asarray(m.intensities, dtype=np.float32) if len(m.intensities) == len(r) else np.zeros(len(r), np.float32)
        r, ang, inten = r[ok], ang[ok], inten[ok]
        out = np.stack([r * np.cos(ang), r * np.sin(ang), np.zeros_like(r), inten], 1).astype(np.float32)
    hdr = json.dumps({"name": name, "frame": m.header.frame_id, "n": int(len(out)), "fields": ["x", "y", "z", "i"],
                      "stamp": f"{m.header.stamp.sec}.{m.header.stamp.nanosec:09d}",
                      "tf": STATE["frames"].get(m.header.frame_id)}).encode()
    hdr += b" " * ((-(4 + len(hdr))) % 4)          # keep the float32 block 4-byte aligned
    return len(hdr).to_bytes(4, "little") + hdr + out.tobytes()


_share_cache: dict = {}


def mesh_file(pkg: str, rel: str) -> "Path | None":
    """package://<pkg>/<rel> -> the file in that package's share directory (ament index), or None."""
    if pkg not in _share_cache:
        try:
            from ament_index_python.packages import get_package_share_directory

            _share_cache[pkg] = Path(get_package_share_directory(pkg)).resolve()
        except Exception:  # noqa: BLE001
            _share_cache[pkg] = None
    base = _share_cache[pkg]
    if base is None:
        return None
    # normalise WITHOUT following symlinks: a --symlink-install share dir points into src/, so a resolved
    # path would never sit under the share dir; the ".." check happens on the lexical path instead
    p = Path(os.path.normpath(str(base / rel)))
    return p if str(p).startswith(str(base) + os.sep) and p.is_file() else None


def static_file(rel: str) -> "Path | None":
    base = (HERE / "static").resolve()
    p = (base / rel).resolve()
    return p if base in p.parents and p.is_file() else None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *a):  # quiet
        pass

    def _send(self, code: int, body: bytes, ctype: str = "application/json", cache: int = 0) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", f"max-age={cache}" if cache else "no-cache")
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
            elif u.path == "/api/costmap":
                r = costmap_png(q.get("name", "global"))
                self._json(r[1] if r else {"error": "no costmap received yet (is navigation running?)"})
            elif u.path.startswith("/costmap/") and u.path.endswith(".png"):
                r = costmap_png(u.path[len("/costmap/"):-4])
                if r:
                    self._send(200, r[0], "image/png")
                else:
                    self._send(404, b"no costmap", "text/plain")
            elif u.path == "/api/scans":
                now = time.time()
                names = [n for n in (q.get("layers") or "").split(",") if n]
                STATE["want"] = {"names": names or list(SCAN_TOPICS) + list(CLOUD_TOPICS), "t": now}
                self._json({"scans": {k: v for k, v in STATE["scans"].items()
                                      if now - v["t"] < 3.0 and (not names or k in names)},
                            "topics": {**SCAN_TOPICS, **CLOUD_TOPICS}, "ros": STATE["ros"]})
            # ---- 3D view
            elif u.path == "/3d":
                self._send(200, (HERE / "map3d.html").read_bytes(), "text/html; charset=utf-8")
            elif u.path.startswith("/static/"):
                p = static_file(u.path[len("/static/"):])
                if p:
                    self._send(200, p.read_bytes(), MIME.get(p.suffix.lower(), "application/octet-stream"), cache=3600)
                else:
                    self._send(404, b"not found", "text/plain")
            elif u.path.startswith("/mesh/"):
                parts = u.path[len("/mesh/"):].split("/", 1)
                p = mesh_file(parts[0], parts[1]) if len(parts) == 2 else None
                if p:
                    self._send(200, p.read_bytes(), MIME.get(p.suffix.lower(), "application/octet-stream"), cache=86400)
                else:
                    self._send(404, b"no such mesh", "text/plain")
            elif u.path == "/api/urdf":
                if STATE["urdf"]:
                    self._send(200, STATE["urdf"].encode(), "application/xml", cache=60)
                else:
                    self._json({"error": "no /robot_description received yet (is the base launch running?)"}, 503)
            elif u.path == "/api/tf":
                self._json({"frames": STATE["frames"], "joints": STATE["joints"], "t": time.time(), "ros": STATE["ros"],
                            "published": STATE["published"]})
            elif u.path == "/api/cloud.bin":
                name = q.get("name", "velodyne")
                b = cloud_bin(name, int(q.get("max", CLOUD_BIN_MAX)))
                if b:
                    self._send(200, b, "application/octet-stream")
                else:
                    self._json({"error": f"no {name} data yet"}, 404)
            elif u.path == "/api/nav":
                nav = dict(STATE["nav"])
                if q.get("particles"):
                    STATE["want3d"]["particles"] = time.time()
                    nav["particles"] = STATE["particles"]
                self._json(nav)
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
            if u.path == "/api/publish":          # the 3D page's "nav goal" / "initial pose" click tools
                kind = body.get("type")
                if kind not in ("goal", "initialpose") or STATE["ros"] != "ok":
                    self._json({"error": "type must be goal|initialpose and ROS must be up"}, 400)
                    return
                req = {"type": kind, "x": float(body["x"]), "y": float(body["y"]), "yaw": float(body.get("yaw", 0.0))}
                STATE["publish_q"].append(req)
                self._json({"ok": True, "queued": req})
                return
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
