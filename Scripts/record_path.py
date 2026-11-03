#!/usr/bin/env python3
"""Record a route by driving the robot, then reduce it to a few waypoints for path.py.

    cd ~/rd_ws/Scripts
    python3 record_path.py record tigger --init-from waypoints.yaml   # robot at the dock: seeds AMCL first
    python3 record_path.py record tigger            # drive with the gamepad; type a name + Enter at the
                                                    # places that matter (bookshelf, banana_bed, …), q to finish
    python3 record_path.py simplify tigger.raw.yaml --tolerance 0.3   # re-reduce a recording without re-driving
    python3 record_path.py center tigger.yaml --map ../src/robot_disco/maps/zone_one.yaml   # unmarked waypoints
                                                    # to the middle of the free corridor they sit in (-o, --max-shift)
    python3 record_path.py densify tigger.yaml --spacing 1.0   # intermediate waypoints on legs longer than that
    python3 path.py --check tigger.yaml             # what the route runner will do with it

`record` samples the robot's map pose (TF map -> base_footprint by default, /amcl_pose with --source amcl)
whenever it has moved, writes every sample to <name>.raw.yaml, and on `q` / Ctrl-C reduces the trace to
<name>.yaml in path.py's route format plus <name>.svg (raw trace in grey, the route in blue, marks in red).

Reduction: Ramer-Douglas-Peucker on the xy trace with --tolerance metres of allowed deviation, cut at
every MARK (a named pose you typed while driving is always kept, with the heading you parked at), then
points closer than --min-spacing to the previous one are dropped. Unmarked waypoints face the next
waypoint, so Nav2 drives through them without spinning to match a noisy recorded heading; marked ones
and the last one keep their recorded heading. Marked names become the waypoint names, which is where
post_actions go: --actions-from <route.yaml> copies the post_actions of same-named waypoints from an
existing route (e.g. tigger_route.yaml), so a recorded route gets its vla: actions in one step.
"""
from __future__ import annotations

import argparse
import math
import os
import queue
import sys
import threading
import time
from pathlib import Path

import yaml

MOVE_EPS_M = 0.02      # a new sample needs this much motion …
TURN_EPS_RAD = 0.035   # … or this much turn (2°) since the last one


# --------------------------------------------------------------------------- geometry
def yaw_of(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_of(yaw: float) -> dict:
    return {"x": 0.0, "y": 0.0, "z": round(math.sin(yaw / 2.0), 6), "w": round(math.cos(yaw / 2.0), 6)}


def _perp_dist(p, a, b) -> float:
    (px, py), (ax, ay), (bx, by) = p, a, b
    dx, dy = bx - ax, by - ay
    if dx == dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def rdp(points: list, tol: float) -> list:
    """Indices (into `points`) kept by Ramer-Douglas-Peucker; always the first and the last."""
    n = len(points)
    if n < 3:
        return list(range(n))
    keep = [False] * n
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        worst, wd = -1, tol
        for i in range(a + 1, b):
            d = _perp_dist(points[i], points[a], points[b])
            if d > wd:
                worst, wd = i, d
        if worst >= 0:
            keep[worst] = True
            stack.append((a, worst))
            stack.append((worst, b))
    return [i for i in range(n) if keep[i]]


def simplify(raw: dict, tolerance: float, min_spacing: float) -> list[dict]:
    """raw recording -> ordered waypoints [{name, x, y, yaw, marked}]."""
    samples = raw.get("samples") or []
    if len(samples) < 2:
        raise ValueError("fewer than two samples — nothing to reduce")
    pts = [(float(s[1]), float(s[2])) for s in samples]
    marks = {int(m["i"]): m for m in raw.get("marks") or []}
    # RDP per stretch between marks (marks are hard cuts), then merge
    cuts = sorted({0, len(pts) - 1, *marks})
    keep: list[int] = []
    for a, b in zip(cuts, cuts[1:]):
        seg = rdp(pts[a:b + 1], tolerance)
        keep += [a + i for i in seg if a + i not in keep]
    keep = sorted(set(keep))
    # spacing: drop an unmarked point too close to the previous kept one (never the last)
    out: list[int] = []
    for i in keep:
        if out and i not in marks and i != keep[-1] and math.hypot(pts[i][0] - pts[out[-1]][0],
                                                                    pts[i][1] - pts[out[-1]][1]) < min_spacing:
            continue
        out.append(i)
    wps = []
    for k, i in enumerate(out):
        x, y = pts[i]
        yaw = float(samples[i][3])
        marked = i in marks
        if not marked and k < len(out) - 1:            # face the next waypoint
            nx, ny = pts[out[k + 1]]
            yaw = math.atan2(ny - y, nx - x)
        name = marks[i]["name"] if marked else ("start" if k == 0 else f"wp_{k}")
        wps.append({"name": name, "x": round(x, 4), "y": round(y, 4), "yaw": yaw, "marked": marked})
    return wps


# --------------------------------------------------------------------------- output
def route_yaml(wps: list[dict], raw: dict, tolerance: float, min_spacing: float, actions: dict) -> str:
    lines = [f"# Recorded with record_path.py on {raw.get('started', '?')} ({len(raw.get('samples') or [])} samples, "
             f"{raw.get('distance_m', '?')} m), reduced with tolerance {tolerance} m / min spacing {min_spacing} m.",
             "# Marked poses keep their recorded heading; the others face the next waypoint. Re-run",
             f"# `python3 record_path.py simplify {raw.get('name', 'NAME')}.raw.yaml --tolerance …` to change the reduction.",
             "waypoints:"]
    for w in wps:
        q = quat_of(w["yaw"])
        tag = "   # marked" if w["marked"] else ""
        lines.append(f"  - name: {w['name']}{tag}")
        lines.append(f"    position: {{x: {w['x']}, y: {w['y']}, z: 0.0}}")
        lines.append(f"    orientation: {{x: 0.0, y: 0.0, z: {q['z']}, w: {q['w']}}}")
        acts = actions.get(w["name"].lower())
        if acts:
            lines.append("    post_actions:")
            lines += ["      " + l for l in yaml.safe_dump(acts, sort_keys=False, allow_unicode=True,
                                                             default_flow_style=False).rstrip().splitlines()]
    return "\n".join(lines) + "\n"


def svg_preview(raw: dict, wps: list[dict], size: int = 800) -> str:
    pts = [(float(s[1]), float(s[2])) for s in raw.get("samples") or []]
    if not pts:
        return ""
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    span = max(maxx - minx, maxy - miny, 0.5)
    pad = 40
    sc = (size - 2 * pad) / span

    def P(x, y):
        return (pad + (x - minx) * sc, size - pad - (y - miny) * sc)   # y up

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}" style="background:#fff">',
           f'<text x="{pad}" y="24" font-family="sans-serif" font-size="14">{raw.get("name", "")}: '
           f'{len(pts)} samples -> {len(wps)} waypoints; grid 1 m</text>']
    for g in range(int(math.floor(minx)), int(math.ceil(maxx)) + 1):          # 1 m grid
        x = P(g, 0)[0]
        out.append(f'<line x1="{x:.1f}" y1="{pad}" x2="{x:.1f}" y2="{size - pad}" stroke="#eee"/>')
    for g in range(int(math.floor(miny)), int(math.ceil(maxy)) + 1):
        y = P(0, g)[1]
        out.append(f'<line x1="{pad}" y1="{y:.1f}" x2="{size - pad}" y2="{y:.1f}" stroke="#eee"/>')
    out.append('<polyline fill="none" stroke="#aaa" stroke-width="2" points="'
               + " ".join(f"{P(x, y)[0]:.1f},{P(x, y)[1]:.1f}" for x, y in pts) + '"/>')
    out.append('<polyline fill="none" stroke="#2a6fdb" stroke-width="2.5" points="'
               + " ".join(f"{P(w['x'], w['y'])[0]:.1f},{P(w['x'], w['y'])[1]:.1f}" for w in wps) + '"/>')
    for w in wps:
        cx, cy = P(w["x"], w["y"])
        ex, ey = cx + 18 * math.cos(w["yaw"]), cy - 18 * math.sin(w["yaw"])
        col = "#d33" if w["marked"] else "#2a6fdb"
        out.append(f'<line x1="{cx:.1f}" y1="{cy:.1f}" x2="{ex:.1f}" y2="{ey:.1f}" stroke="{col}" stroke-width="2"/>')
        out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{6 if w["marked"] else 4}" fill="{col}"/>')
        out.append(f'<text x="{cx + 8:.1f}" y="{cy - 8:.1f}" font-family="sans-serif" font-size="12" fill="{col}">{w["name"]}</text>')
    out.append("</svg>")
    return "\n".join(out)


def load_actions(path: str | None) -> dict:
    """{lowercased waypoint name: post_actions} from an existing route file."""
    if not path:
        return {}
    d = yaml.safe_load(open(path)) or {}
    return {str(w.get("name", "")).lower(): w["post_actions"] for w in d.get("waypoints") or []
            if w.get("post_actions")}


def write_outputs(raw: dict, out_base: Path, tolerance: float, min_spacing: float, actions_from: str | None) -> list[dict]:
    wps = simplify(raw, tolerance, min_spacing)
    actions = load_actions(actions_from)
    route = out_base.with_suffix(".yaml")
    route.write_text(route_yaml(wps, raw, tolerance, min_spacing, actions))
    svg = out_base.with_suffix(".svg")
    svg.write_text(svg_preview(raw, wps))
    print(f"{route}: {len(raw.get('samples') or [])} samples -> {len(wps)} waypoints "
          f"(tolerance {tolerance} m, min spacing {min_spacing} m); preview {svg}")
    for w in wps:
        acts = actions.get(w["name"].lower())
        print(f"  {'*' if w['marked'] else ' '} {w['name']:<16} ({w['x']:7.3f}, {w['y']:7.3f}) yaw {math.degrees(w['yaw']):6.1f}°"
              + (f"  + {len(acts)} post_action(s)" if acts else ""))
    missing = [n for n in actions if n not in {w["name"].lower() for w in wps}]
    if missing:
        print(f"  note: --actions-from waypoints with no matching name here (not copied): {missing}")
    return wps


# --------------------------------------------------------------------------- center on the map
class GridMap:
    """A nav2 map (yaml + pgm): world (x, y) -> free / not free."""

    def __init__(self, yaml_path: str):
        import numpy as np

        meta = yaml.safe_load(open(yaml_path))
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), meta["image"])
        try:
            from PIL import Image
            img = np.array(Image.open(img_path).convert("L"))
        except ImportError:
            import cv2
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        negate = int(meta.get("negate", 0))
        free_thresh = float(meta.get("free_thresh", 0.25))
        occ = img.astype(float) / 255.0
        occ = occ if negate else 1.0 - occ
        self.free = occ < free_thresh                      # occupancy probability below the free threshold
        self.free &= np.abs(img.astype(int) - 205) > 3     # unknown (205) counts as blocked for centring
        self.h, self.w = img.shape

    def is_free(self, x: float, y: float) -> bool:
        c = int((x - self.ox) / self.res)
        r = self.h - 1 - int((y - self.oy) / self.res)
        return 0 <= r < self.h and 0 <= c < self.w and bool(self.free[r, c])


def free_span(gm: GridMap, x: float, y: float, nx: float, ny: float, window: float, step: float = 0.02):
    """Free distance from (x, y) along +n and along -n (unit vector n), each capped at `window`."""
    def run(sign: float) -> float:
        d = 0.0
        while d < window and gm.is_free(x + sign * nx * d, y + sign * ny * d):
            d += step
        return d
    return run(1.0), run(-1.0)


def center_route(route: dict, gm: GridMap, max_shift: float, window: float, max_width: float, log=print) -> int:
    """Move each UNMARKED waypoint to the middle of the free corridor across the route direction.
    Marked ones (named by the operator) stay. Skips open areas wider than `max_width`."""
    wps = route["waypoints"]
    moved = 0
    for i, w in enumerate(wps):
        if w.get("post_actions") or w.get("pause") or w.get("marked") or not w["name"].startswith(("wp_", "start")):
            continue
        prev_ = wps[i - 1] if i > 0 else None
        next_ = wps[i + 1] if i + 1 < len(wps) else None
        a, b = (prev_ or w), (next_ or w)
        dx, dy = b["position"]["x"] - a["position"]["x"], b["position"]["y"] - a["position"]["y"]
        n = math.hypot(dx, dy)
        if n < 1e-6:
            continue
        nx, ny = -dy / n, dx / n                     # left-hand normal of the route direction
        x, y = w["position"]["x"], w["position"]["y"]
        if not gm.is_free(x, y):
            log(f"  {w['name']:<10} sits on a blocked cell — left alone")
            continue
        left, right = free_span(gm, x, y, nx, ny, window)
        width = left + right
        if width >= 2 * window - 0.05 or width > max_width:
            log(f"  {w['name']:<10} open area ({width:.1f} m free across) — left alone")
            continue
        shift = (left - right) / 2.0                 # + = move left, toward the wider side
        shift = max(-max_shift, min(max_shift, shift))
        if abs(shift) < 0.03:
            log(f"  {w['name']:<10} already centred (corridor {width:.2f} m)")
            continue
        w["position"]["x"], w["position"]["y"] = round(x + nx * shift, 4), round(y + ny * shift, 4)
        moved += 1
        log(f"  {w['name']:<10} corridor {width:.2f} m (free {left:.2f} left / {right:.2f} right) -> moved "
            f"{abs(shift):.2f} m {'left' if shift > 0 else 'right'} of travel to ({w['position']['x']}, {w['position']['y']})")
    # unmarked waypoints face the next one; re-aim after the moves
    for i, w in enumerate(wps):
        if w.get("post_actions") or not w["name"].startswith(("wp_", "start")) or i + 1 >= len(wps):
            continue
        nx_, ny_ = wps[i + 1]["position"]["x"], wps[i + 1]["position"]["y"]
        q = quat_of(math.atan2(ny_ - w["position"]["y"], nx_ - w["position"]["x"]))
        w["orientation"] = {"x": 0.0, "y": 0.0, "z": q["z"], "w": q["w"]}
    return moved


def densify_route(route: dict, spacing: float) -> int:
    """Insert intermediate (unmarked) waypoints on every leg longer than `spacing`, evenly spaced, so
    Nav2 gets short hops along the recorded line instead of one long goal. Names: <prev>_<k>."""
    wps = route["waypoints"]
    out, added = [], 0
    for i, w in enumerate(wps):
        out.append(w)
        if i + 1 >= len(wps):
            break
        nx, ny = wps[i + 1]["position"]["x"], wps[i + 1]["position"]["y"]
        x, y = w["position"]["x"], w["position"]["y"]
        length = math.hypot(nx - x, ny - y)
        n = int(math.ceil(length / spacing)) - 1
        base = w["name"].replace(" ", "_")
        for k in range(1, n + 1):
            f = k / (n + 1)
            px, py = round(x + (nx - x) * f, 4), round(y + (ny - y) * f, 4)
            q = quat_of(math.atan2(ny - y, nx - x))
            out.append({"name": f"{base}_{k}", "position": {"x": px, "y": py, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": q["z"], "w": q["w"]}})
            added += 1
    route["waypoints"] = out
    return added


def write_route(route: dict, path: Path, header: list[str]) -> None:
    lines = list(header) + ["waypoints:"]
    for w in route["waypoints"]:
        p, o = w["position"], w["orientation"]
        lines.append(f"  - name: {w['name']}")
        lines.append(f"    position: {{x: {p['x']}, y: {p['y']}, z: {p.get('z', 0.0)}}}")
        lines.append(f"    orientation: {{x: {o.get('x', 0.0)}, y: {o.get('y', 0.0)}, z: {o['z']}, w: {o['w']}}}")
        if w.get("pause"):
            lines.append(f"    pause: {float(w['pause']):g}")
        if w.get("post_actions"):
            lines.append("    post_actions:")
            lines += ["      " + l for l in yaml.safe_dump(w["post_actions"], sort_keys=False, allow_unicode=True,
                                                             default_flow_style=False).rstrip().splitlines()]
    if route.get("map"):
        lines.append(f"map: {route['map']}")          # the nav map the waypoints belong to (route_editor.py tags it)
    if route.get("initial_pose"):
        lines += ["initial_pose:"] + ["  " + l for l in yaml.safe_dump(route["initial_pose"], sort_keys=False,
                                                                        default_flow_style=False).rstrip().splitlines()]
    path.write_text("\n".join(lines) + "\n")


# --------------------------------------------------------------------------- record
def init_pose_from(path: str) -> tuple:
    """(x, y, yaw) from a route file's initial_pose (or its first waypoint)."""
    d = yaml.safe_load(open(path)) or {}
    e = d.get("initial_pose") or (d.get("waypoints") or [{}])[0]
    if not e or "position" not in e:
        raise ValueError(f"{path} has no initial_pose / waypoints to take a pose from")
    pos, o = e["position"], e.get("orientation", {}) or {}
    yaw = math.atan2(2.0 * float(o.get("w", 1.0)) * float(o.get("z", 0.0)), 1.0 - 2.0 * float(o.get("z", 0.0)) ** 2)
    return float(pos.get("x", 0.0)), float(pos.get("y", 0.0)), yaw


def record(name: str, out_dir: Path, rate: float, source: str, base_frame: str, tolerance: float,
           min_spacing: float, actions_from: str | None, init_pose: "tuple | None" = None) -> int:
    import rclpy
    from rclpy.node import Node

    out_dir.mkdir(parents=True, exist_ok=True)
    raw = {"name": name, "frame": "map", "base_frame": base_frame, "source": source,
           "started": time.strftime("%Y-%m-%dT%H:%M:%S"), "samples": [], "marks": [], "distance_m": 0.0}
    rclpy.init()
    node = Node("record_path")
    latest: dict = {"pose": None}   # (x, y, yaw)

    if source == "amcl":
        from geometry_msgs.msg import PoseWithCovarianceStamped

        def _cb(msg):
            p = msg.pose.pose
            latest["pose"] = (p.position.x, p.position.y, yaw_of(p.orientation))

        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", _cb, 10)
        pose_of = lambda: latest["pose"]  # noqa: E731
    else:
        from tf2_ros import Buffer, TransformListener

        buf = Buffer()
        TransformListener(buf, node)

        def pose_of():
            try:
                tf = buf.lookup_transform("map", base_frame, rclpy.time.Time())
            except Exception:  # noqa: BLE001 — no TF yet
                return None
            t, q = tf.transform.translation, tf.transform.rotation
            return (t.x, t.y, yaw_of(q))

    if init_pose is not None:
        # AMCL publishes nothing (no map->odom) until it is given an initial pose; seed it the way
        # path.py does, from the route file's initial_pose or an explicit x y yaw.
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        ix, iy, iyaw = init_pose
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(ix), float(iy)
        q = quat_of(float(iyaw))
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q["z"], q["w"]
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        # Wait until AMCL's subscription is matched: a fresh publisher's first messages are dropped
        # until DDS discovery completes (seen 2026-10-27: three quick publishes, AMCL never got them).
        t_wait = time.time()
        while pub.get_subscription_count() < 1 and time.time() - t_wait < 15.0:
            rclpy.spin_once(node, timeout_sec=0.2)
        if pub.get_subscription_count() < 1:
            print("[record] WARNING: nothing subscribes to /initialpose (is robot_disco_localization running?)", flush=True)
        for _ in range(6):
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.5)
        print(f"[record] initial pose published: ({ix:.2f}, {iy:.2f}) {math.degrees(iyaw):.0f}° — the robot must be there",
              flush=True)

    lines: "queue.Queue[str]" = queue.Queue()

    def _stdin():
        for line in sys.stdin:
            lines.put(line.rstrip("\n"))
        lines.put("q")

    threading.Thread(target=_stdin, daemon=True).start()
    print(f"[record] {name}: sampling {source} map->{base_frame} at {rate:g} Hz while the robot moves.\n"
          f"         Drive with the gamepad. Type a name + Enter to MARK the current pose (e.g. bookshelf),\n"
          f"         Enter alone marks wp_N, q + Enter (or Ctrl-C) finishes and writes {out_dir / name}.yaml",
          flush=True)
    period = 1.0 / rate
    last = None
    t0 = time.time()
    no_pose_since = time.time()
    n_marks = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=period)
            p = pose_of()
            now = time.time()
            if p is None:
                if now - no_pose_since > 5.0:
                    print(f"[record] no map pose yet ({'TF map->' + base_frame if source == 'tf' else '/amcl_pose'}) — "
                          "is robot_disco_localization running and the map loaded?", flush=True)
                    no_pose_since = now
                continue
            x, y, yaw = p
            if last is None or math.hypot(x - last[0], y - last[1]) >= MOVE_EPS_M \
                    or abs((yaw - last[2] + math.pi) % (2 * math.pi) - math.pi) >= TURN_EPS_RAD:
                if last is not None:
                    raw["distance_m"] = round(raw["distance_m"] + math.hypot(x - last[0], y - last[1]), 3)
                raw["samples"].append([round(now - t0, 2), round(x, 4), round(y, 4), round(yaw, 4)])
                last = (x, y, yaw)
                if len(raw["samples"]) % 25 == 0:
                    print(f"[record] {len(raw['samples'])} samples, {raw['distance_m']:.1f} m, "
                          f"at ({x:.2f}, {y:.2f}) {math.degrees(yaw):.0f}°", flush=True)
            try:
                while True:
                    cmd = lines.get_nowait().strip()
                    if cmd.lower() == "q":
                        raise KeyboardInterrupt
                    if not raw["samples"]:
                        raw["samples"].append([round(now - t0, 2), round(x, 4), round(y, 4), round(yaw, 4)])
                        last = (x, y, yaw)
                    n_marks += 1
                    mark_name = cmd or f"wp_{n_marks}"
                    raw["marks"].append({"name": mark_name, "i": len(raw["samples"]) - 1,
                                         "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4),
                                         "t": round(now - t0, 2)})
                    print(f"[mark] {mark_name} at ({x:.2f}, {y:.2f}) {math.degrees(yaw):.0f}°", flush=True)
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raw_path = out_dir / f"{name}.raw.yaml"
    raw_path.write_text(yaml.safe_dump(raw, sort_keys=False))
    print(f"[record] {len(raw['samples'])} samples, {raw['distance_m']:.1f} m, {len(raw['marks'])} marks -> {raw_path}")
    if len(raw["samples"]) < 2:
        print("[record] too short to reduce; drive a route next time")
        return 1
    write_outputs(raw, out_dir / name, tolerance, min_spacing, actions_from)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--tolerance", type=float, default=0.25, help="RDP deviation allowed, metres")
    common.add_argument("--min-spacing", type=float, default=0.4, help="drop unmarked waypoints closer than this, metres")
    common.add_argument("--actions-from", default=None, metavar="ROUTE.yaml",
                        help="copy post_actions of same-named waypoints from this route file")
    p = sub.add_parser("record", parents=[common], help="drive and record; writes NAME.raw.yaml, NAME.yaml, NAME.svg")
    p.add_argument("name")
    p.add_argument("--out", default=str(Path(__file__).parent), help="directory for the files (default: Scripts/)")
    p.add_argument("--rate", type=float, default=5.0, help="sampling rate, Hz")
    p.add_argument("--source", choices=["tf", "amcl"], default="tf")
    p.add_argument("--base-frame", default="base_footprint")
    p.add_argument("--init-from", default=None, metavar="ROUTE.yaml",
                   help="publish that file's initial_pose (or first waypoint) to /initialpose first: AMCL needs "
                        "an initial pose before it publishes any map pose. The robot must be standing there")
    p.add_argument("--init", nargs=3, type=float, default=None, metavar=("X", "Y", "YAW_DEG"),
                   help="publish this initial pose instead (map frame, yaw in degrees)")
    p = sub.add_parser("simplify", parents=[common], help="re-reduce an existing NAME.raw.yaml")
    p.add_argument("raw")
    p.add_argument("-o", "--out", default=None, help="route file to write (default: next to the raw file, .yaml)")
    p = sub.add_parser("center", help="move unmarked waypoints to the middle of the free corridor on the map")
    p.add_argument("route", help="a route .yaml (post_actions are kept)")
    p.add_argument("--map", required=True, help="the nav2 map .yaml AMCL uses (its .pgm sits next to it)")
    p.add_argument("--max-shift", type=float, default=0.6, help="largest move per waypoint, metres")
    p.add_argument("--window", type=float, default=2.0, help="how far across the route to look for walls, metres")
    p.add_argument("--max-width", type=float, default=3.0, help="wider free spans are open areas: left alone")
    p.add_argument("-o", "--out", default=None, help="write here (default: overwrite the route file)")
    p = sub.add_parser("densify", help="insert intermediate waypoints on long legs")
    p.add_argument("route")
    p.add_argument("--spacing", type=float, default=1.0, help="maximum distance between consecutive waypoints, metres")
    p.add_argument("-o", "--out", default=None)
    a = ap.parse_args()
    if a.cmd == "densify":
        route = yaml.safe_load(open(a.route)) or {}
        added = densify_route(route, a.spacing)
        out = Path(a.out) if a.out else Path(a.route)
        write_route(route, out, [f"# {os.path.basename(a.route)}: {added} intermediate waypoint(s) added by record_path.py densify "
                                 f"(spacing {a.spacing} m); marked poses untouched."])
        print(f"{out}: {added} intermediate waypoint(s) added -> {len(route['waypoints'])} waypoints")
        return 0
    if a.cmd == "center":
        route = yaml.safe_load(open(a.route)) or {}
        gm = GridMap(a.map)
        moved = center_route(route, gm, a.max_shift, a.window, a.max_width)
        out = Path(a.out) if a.out else Path(a.route)
        header = [f"# {os.path.basename(a.route)}: unmarked waypoints centred in their corridor on "
                  f"{os.path.basename(a.map)} by record_path.py center ({moved} moved, max shift {a.max_shift} m)."]
        write_route(route, out, header)
        print(f"{out}: {moved} waypoint(s) moved")
        return 0
    if a.cmd == "record":
        init = None
        if a.init is not None:
            init = (a.init[0], a.init[1], math.radians(a.init[2]))
        elif a.init_from:
            init = init_pose_from(a.init_from)
        return record(a.name, Path(a.out), a.rate, a.source, a.base_frame, a.tolerance, a.min_spacing,
                      a.actions_from, init)
    raw = yaml.safe_load(open(a.raw))
    base = Path(a.out).with_suffix("") if a.out else Path(a.raw.replace(".raw.yaml", ""))
    raw.setdefault("name", base.name)
    write_outputs(raw, base, a.tolerance, a.min_spacing, a.actions_from)
    return 0


if __name__ == "__main__":
    sys.exit(main())
