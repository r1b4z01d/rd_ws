#!/usr/bin/env python3
"""Simulated RobotDisco for `route_editor.py --sim`: develop the map / 3D pages without the robot.

Fills the same STATE the ROS thread fills, from the nav2 map alone:
  - the robot drives the selected route back and forth (pure-pursuit-ish, 0.5 m/s), or to a goal
    published from the page ("nav goal" tool); "initial pose" teleports it;
  - TF: map->odom (a fixed offset, so the local costmap placement is exercised), base_footprint,
    base_link and every sensor frame from the URDF's fixed joints;
  - RPLiDAR S1 / S2E scans and a 16-ring Velodyne cloud raycast against the map (walls, floor,
    a 2.6 m ceiling), with noise; raw messages for /api/cloud.bin and map-frame layers for /api/scans;
  - global costmap = the map inflated like nav2 (lethal / inscribed / exponential band), local costmap
    = a 6 m rolling window in odom sampled from it;
  - nav2 state: plan to the remaining waypoints, local plan, goal, footprint + collision polygons,
    AMCL-like particles; joint states (wheels integrate, the UR waves slowly);
  - the URDF: Scripts/sim/robot_disco.urdf (the real one, see sim/fetch_assets.sh) else
    Scripts/sim/robot_sim.urdf (primitives).
No ROS, numpy + PIL only.
"""
from __future__ import annotations

import heapq
import math
import threading
import time
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SIM_DIR = HERE / "sim"

# sensor frames relative to base_link (x, y, z, yaw) — overridden by the URDF's fixed joints when present
DEFAULT_SENSORS = {"velodyne_frame": (0.14, 0.0, 0.8, 0.0), "rp_s1_lidar": (0.21, 0.0, 0.14, math.pi),
                   "rp_s2e_lidar": (-0.52, 0.0, 0.0, 0.0), "camera_link": (0.24, 0.0, 0.18, 0.0)}
BASE_LINK_Z = 0.1592                       # base_footprint -> base_link
MAP_TO_ODOM = (0.5, -0.3, 0.15)            # a deliberate offset: the local costmap lives in odom
FOOTPRINT = [[0.32, 0.32], [0.45, 0.25], [0.45, -0.25], [0.32, -0.32], [-0.45, -0.32], [-0.45, -0.1],
             [-0.6, -0.1], [-0.6, 0.1], [-0.45, 0.1], [-0.45, 0.32]]
UR_HOME = [0.0, -1.9, 1.7, -1.4, -1.571, 0.0]
UR_JOINTS = ["ur_shoulder_pan_joint", "ur_shoulder_lift_joint", "ur_elbow_joint", "ur_wrist_1_joint", "ur_wrist_2_joint", "ur_wrist_3_joint"]
CEILING = 2.6
V_MAX, W_MAX = 0.5, 1.0
CONE_R, CONE_H = 0.22, 0.7            # a traffic cone: base radius, height (sensed as a cone, radius shrinks with height)
# simulated camera feeds: first-person raycast renders (name -> mount offset x/y on the base, yaw, fov,
# height above the floor); the wrist camera rides on the arm, so it sways a little
SIM_CAMS = {"scene": {"xy": (0.22, 0.0), "yaw": 0.0, "fov": 90.0, "h": 0.95},
            "side": {"xy": (0.0, -0.22), "yaw": -math.pi / 2, "fov": 90.0, "h": 0.9},
            "wrist": {"xy": (0.45, 0.12), "yaw": 0.0, "fov": 60.0, "h": 0.6},
            "realsense": {"xy": (0.24, 0.0), "yaw": 0.0, "fov": 69.0, "h": 0.34}}


def _ns(**kw):
    return types.SimpleNamespace(**kw)


def _stamp(t=None):
    t = time.time() if t is None else t
    return _ns(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def _hdr(frame):
    return _ns(frame_id=frame, stamp=_stamp())


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _tf7(x, y, z, yaw):
    return [round(x, 4), round(y, 4), round(z, 4), 0.0, 0.0, round(math.sin(yaw / 2), 5), round(math.cos(yaw / 2), 5)]


def urdf_text() -> str | None:
    for n in ("robot_disco.urdf", "robot_sim.urdf"):
        p = SIM_DIR / n
        if p.is_file():
            return p.read_text()
    return None


def sensor_offsets(urdf: str | None) -> dict:
    """frame -> (x, y, z, yaw) relative to base_link, from the URDF's fixed joints off base_link."""
    out = dict(DEFAULT_SENSORS)
    if not urdf:
        return out
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError:
        return out
    for j in root.findall("joint"):
        if j.find("parent").get("link") != "base_link":
            continue
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        child = j.find("child").get("link")
        if child in DEFAULT_SENSORS:
            out[child] = (xyz[0], xyz[1], xyz[2], rpy[2])
    return out


class GridMap:
    """The nav2 map as boolean grids in OccupancyGrid order (row 0 = bottom) + a vectorised raycaster."""

    def __init__(self, yaml_path: Path, meta: dict):
        import yaml
        from PIL import Image

        m = yaml.safe_load(open(yaml_path))
        im = np.asarray(Image.open(yaml_path.parent / m["image"]).convert("L"))
        g = im[::-1]                                    # PNG row 0 is the top; the grid's row 0 is the bottom
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.H, self.W = g.shape
        self.occ = g < 100
        self.unknown = (g >= 100) & (g < 235)
        self.free = g >= 235
        self._d = {}

    def cell(self, x, y):
        return int((x - self.ox) / self.res), int((y - self.oy) / self.res)

    def is_free(self, x, y):
        c, r = self.cell(x, y)
        return 0 <= c < self.W and 0 <= r < self.H and bool(self.free[r, c])

    def free_center(self):
        rows, cols = np.nonzero(self.free)
        return self.ox + float(np.median(cols)) * self.res, self.oy + float(np.median(rows)) * self.res

    def raycast(self, x, y, angles, max_range):
        """Range to the first occupied cell along each world angle; inf where the ray leaves the map or
        enters unknown space first (no return, like a window)."""
        key = round(max_range, 2)
        if key not in self._d:
            self._d[key] = np.arange(1, int(max_range / self.res) + 1, dtype=np.float32) * self.res
        d = self._d[key]
        cx = x + np.cos(angles)[:, None] * d[None, :]
        cy = y + np.sin(angles)[:, None] * d[None, :]
        col = np.floor((cx - self.ox) / self.res).astype(np.int32)
        row = np.floor((cy - self.oy) / self.res).astype(np.int32)
        inside = (col >= 0) & (col < self.W) & (row >= 0) & (row < self.H)
        colc = np.clip(col, 0, self.W - 1)
        rowc = np.clip(row, 0, self.H - 1)
        hit = self.occ[rowc, colc] & inside
        blocked = hit | ~inside | (self.unknown[rowc, colc] & inside)
        first = np.argmax(blocked, axis=1)
        idx = np.arange(len(angles))
        ok = blocked[idx, first] & hit[idx, first]
        return np.where(ok, d[first], np.inf).astype(np.float32)

    def inflated(self, inflation_m=0.45, inscribed_m=0.25, scaling=3.0) -> np.ndarray:
        """nav2-like costs: 100 lethal, 99 within the inscribed radius, exponential band to the inflation
        radius, -1 unknown, 0 free (int8, grid order)."""
        n_inf = max(1, int(round(inflation_m / self.res)))
        n_ins = int(round(inscribed_m / self.res))
        dist = np.full(self.occ.shape, n_inf + 1, np.int16)
        dist[self.occ] = 0
        cur = self.occ.copy()
        for i in range(1, n_inf + 1):
            p = np.pad(cur, 1)
            dil = (p[:-2, :-2] | p[:-2, 1:-1] | p[:-2, 2:] | p[1:-1, :-2] | p[1:-1, 2:] | p[2:, :-2] | p[2:, 1:-1] | p[2:, 2:] | cur)
            new = dil & (dist > n_inf)
            dist[new] = i
            cur = dil
        cost = np.zeros(self.occ.shape, np.int16)
        band = (dist > n_ins) & (dist <= n_inf)
        cost[band] = np.clip(98 * np.exp(-scaling * (dist[band] - n_ins) * self.res), 1, 98).astype(np.int16)
        cost[(dist > 0) & (dist <= n_ins)] = 99
        cost[self.occ] = 100
        cost[self.unknown & (cost == 0)] = -1
        return cost.astype(np.int8)


def grid_msg(frame, ox, oy, res, w, h, data, yaw=0.0):
    return _ns(header=_hdr(frame), data=data,
               info=_ns(width=w, height=h, resolution=res,
                        origin=_ns(position=_ns(x=ox, y=oy, z=0.0),
                                   orientation=_ns(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))


class Sim:
    def __init__(self, state: dict, map_yaml: Path, opts: dict):
        self.S = state
        self.opts = opts
        self.map = GridMap(map_yaml, state["map"])
        self.urdf = urdf_text()
        self.sensors = sensor_offsets(self.urdf)
        self.route_name = opts.get("route")
        self.targets: list = []                       # [(x, y, yaw, name)]
        self.ti = 0
        self.direction = 1
        self.goal_override = None
        self.halted = False
        self.frozen_joints = None
        self.scan_full: dict = {}                     # name -> full-resolution map-frame scan points
        self.obstacles: list = []                     # movable cones: {id, x, y, r, h}
        self._obs_seq = 0
        self.global_stamp = 0
        self.poly_state: dict = {}                    # name -> triggered (the sim stops like the collision monitor)
        # the planner (nav2's global planner stand-in): A* on the inflated costmap + cones at 0.1 m, in its
        # own thread, replanned every second and whenever a cone or the target changes
        self.plan_lock = threading.Lock()
        self.plan_path: list = []                     # [(x, y), ...] from the robot to the current target
        self.plan_status = "idle"
        self.plan_dirty = True
        self.plan_grid = None
        self.plan_idx = 0
        self.nopath_since = None                      # a waypoint that stays unreachable is skipped (nav2's
                                                      # waypoint follower goes on to the next one on failure)
        self.t0 = time.time()
        self.wheel = [0.0, 0.0]
        self.rng = np.random.default_rng(1)
        wps = opts.get("waypoints") or []
        self.targets = [(w["x"], w["y"], w.get("yaw", 0.0), w.get("name", "")) for w in wps]
        if self.targets:
            self.x, self.y, self.yaw = self.targets[0][0], self.targets[0][1], self.targets[0][2]
            self.ti = 1 if len(self.targets) > 1 else 0
        else:
            self.x, self.y = self.map.free_center()
            self.yaw = 0.0
        self.global_cost = self.map.inflated()
        self._rebuild_plan_grid()
        threading.Thread(target=self._planner_loop, daemon=True, name="sim-planner").start()
        self.S["costmaps"]["global"] = {"msg": grid_msg("map", self.map.ox, self.map.oy, self.map.res, self.map.W, self.map.H, self.global_cost.ravel()), "t": time.time()}
        if self.urdf:
            self.S["urdf"] = self.urdf
        self.S["frame_tf"]["odom"] = MAP_TO_ODOM
        self.S["sim"] = True

    # ------------------------------------------------------------------ obstacles (sim only)
    def obstacle_ranges(self, x, y, angles, z_at):
        """Range along each world angle to the first cone, inf where none. `z_at(d)` gives the beam's
        height above the floor at horizontal distance d (a lidar: constant; a Velodyne ring: rising)."""
        out = np.full(len(angles), np.inf, np.float32)
        if not self.obstacles:
            return out
        ca, sa = np.cos(angles), np.sin(angles)
        for o in self.obstacles:
            dx, dy = o["x"] - x, o["y"] - y
            along = dx * ca + dy * sa
            perp = -dx * sa + dy * ca
            r0 = o["r"]
            with np.errstate(invalid="ignore"):
                d = along - np.sqrt(np.maximum(r0 * r0 - perp * perp, 0.0))
            z = z_at(np.maximum(d, 0.0))
            r_at = r0 * np.clip(1.0 - z / o["h"], 0.0, 1.0)              # a cone narrows towards the tip
            hit = (along > 0) & (np.abs(perp) < r_at) & (z >= 0.0) & (z <= o["h"]) & (d > 0.05)
            out = np.where(hit & (d < out), d, out).astype(np.float32)
        return out

    def apply_obstacle(self, req):
        a = req.get("action")
        if a == "clear":
            self.obstacles = []
        elif a == "add":
            self._obs_seq += 1
            self.obstacles.append({"id": self._obs_seq, "x": float(req["x"]), "y": float(req["y"]),
                                   "r": float(req.get("r", CONE_R)), "h": float(req.get("h", CONE_H)), "kind": "cone"})
        elif a in ("move", "remove"):
            for o in list(self.obstacles):
                if o["id"] == int(req.get("id", -1)):
                    if a == "remove":
                        self.obstacles.remove(o)
                    else:
                        o["x"], o["y"] = float(req["x"]), float(req["y"])
        self.global_stamp += 1
        self.S["costmaps"]["global"] = {"msg": grid_msg("map", self.map.ox, self.map.oy, self.map.res, self.map.W, self.map.H,
                                                        self.cost_with_obstacles(self.global_cost, None).ravel()), "t": time.time()}
        self._rebuild_plan_grid()
        self.plan_dirty = True

    def cost_with_obstacles(self, cost, window):
        """Lethal + inscribed + inflation marks for every cone (like the obstacle layer would mark the
        scan hits); `window` = (wx, wy) cell-centre arrays for a local grid, None = the global grid."""
        if not self.obstacles:
            return cost
        if window is None:
            cols = self.map.ox + (np.arange(self.map.W) + 0.5) * self.map.res
            rows = self.map.oy + (np.arange(self.map.H) + 0.5) * self.map.res
            wx, wy = np.meshgrid(cols, rows)
        else:
            wx, wy = window
        c = cost.astype(np.int16)
        for o in self.obstacles:
            d = np.hypot(wx - o["x"], wy - o["y"])
            r0 = o["r"]
            mark = np.where(d <= r0, 100, np.where(d <= r0 + 0.25, 99, np.where(d <= r0 + 0.45, (98 * np.exp(-3.0 * (d - r0 - 0.25))).astype(np.int16), 0)))
            c = np.maximum(c, mark)
        return c.astype(np.int8)

    # ------------------------------------------------------------------ planner
    def _rebuild_plan_grid(self):
        """Planning grid at 2x the map cell (0.1 m): max-pooled costmap with the cones; -1 = unknown."""
        c = self.cost_with_obstacles(self.global_cost, None).astype(np.int16)
        H, W = (c.shape[0] // 2) * 2, (c.shape[1] // 2) * 2
        self.plan_grid = c[:H, :W].reshape(H // 2, 2, W // 2, 2).max(axis=(1, 3))
        self.plan_res = self.map.res * 2

    def _plan_cell(self, x, y):
        return int((x - self.map.ox) / self.plan_res), int((y - self.map.oy) / self.plan_res)

    def _free_line(self, a, b):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (self.plan_res / 2)) + 1)
        xs = np.linspace(a[0], b[0], n)
        ys = np.linspace(a[1], b[1], n)
        col = ((xs - self.map.ox) / self.plan_res).astype(int)
        row = ((ys - self.map.oy) / self.plan_res).astype(int)
        g = self.plan_grid
        ok = (col >= 0) & (col < g.shape[1]) & (row >= 0) & (row < g.shape[0])
        if not ok.all():
            return False
        v = g[row, col]
        return bool(((v >= 0) & (v < 99)).all())

    def plan(self, start, goal, max_expand=80000):
        """Weighted A* (8-connected) on the planning grid: lethal / unknown cells are walls, the inscribed
        ring is passable at a steep price and inflation is a penalty — like nav2's NavFn, which squeezes
        past a cone in a corridor rather than giving up. Returns [(x, y), ...] or []."""
        g = self.plan_grid
        H, W = g.shape
        sc, sr = self._plan_cell(*start)
        gc, gr = self._plan_cell(*goal)
        if not (0 <= sc < W and 0 <= sr < H and 0 <= gc < W and 0 <= gr < H):
            return []
        if g[gr, gc] >= 100 or g[gr, gc] < 0:
            return []                                   # the goal sits inside an obstacle
        start_n, goal_n = (sr, sc), (gr, gc)
        came = {start_n: None}
        gcost = {start_n: 0.0}
        h = lambda n: 1.2 * math.hypot(n[0] - gr, n[1] - gc)
        pq = [(h(start_n), 0.0, start_n)]
        nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        expanded = 0
        while pq:
            _, gc_, n = heapq.heappop(pq)
            if n == goal_n:
                break
            if gc_ > gcost.get(n, 1e18):
                continue
            expanded += 1
            if expanded > max_expand:
                return []
            r, c = n
            for dr, dc, step in nbrs:
                rr, cc = r + dr, c + dc
                if not (0 <= rr < H and 0 <= cc < W):
                    continue
                v = g[rr, cc]
                if v < 0 or v >= 100:
                    continue
                ng = gc_ + step * (25.0 if v >= 99 else 1.0 + 4.0 * v / 98.0)   # inscribed: last resort; inflation: keep to the middle
                m = (rr, cc)
                if ng < gcost.get(m, 1e18):
                    gcost[m] = ng
                    came[m] = n
                    heapq.heappush(pq, (ng + h(m), ng, m))
        if goal_n not in came:
            return []
        cells = []
        n = goal_n
        while n is not None:
            cells.append(n)
            n = came[n]
        cells.reverse()
        pts = [(self.map.ox + (c + 0.5) * self.plan_res, self.map.oy + (r + 0.5) * self.plan_res) for r, c in cells]
        pts[0], pts[-1] = tuple(start), tuple(goal)
        # string-pull: skip ahead while the straight segment stays in free cells
        out = [pts[0]]
        i = 0
        while i < len(pts) - 1:
            j = i + 1
            while j + 1 < len(pts) and self._free_line(pts[i], pts[j + 1]):
                j += 1
            out.append(pts[j])
            i = j
        return out

    def _planner_loop(self):
        last = 0.0
        while True:
            now = time.time()
            tgt = None if self.halted else self.current_target()
            if tgt is None:
                with self.plan_lock:
                    self.plan_path, self.plan_status = [], "idle"
                time.sleep(0.2)
                continue
            if self.plan_dirty or now - last >= 1.0:
                self.plan_dirty = False
                last = now
                path = self.plan((self.x, self.y), (tgt[0], tgt[1]))
                with self.plan_lock:
                    self.plan_path, self.plan_status, self.plan_idx = path, ("following" if path else "no path"), 0
                if path:
                    self.nopath_since = None
                else:
                    self.nopath_since = self.nopath_since or now
                    if now - self.nopath_since > 3.0 and not self.goal_override and self.targets:
                        self.nopath_since = None
                        self.plan_status = "waypoint unreachable, skipped"
                        if self.ti + self.direction >= len(self.targets) or self.ti + self.direction < 0:
                            self.direction *= -1
                        self.ti += self.direction
                        self.plan_dirty = True
            time.sleep(0.05)

    def lookahead(self, dist=0.5):
        """The pure-pursuit carrot: the first path point at least `dist` ahead of the closest one."""
        with self.plan_lock:
            path = list(self.plan_path)
        if len(path) < 2:
            return None
        d = [math.hypot(p[0] - self.x, p[1] - self.y) for p in path]
        k = min(range(len(path)), key=lambda i: d[i])
        acc = 0.0
        for i in range(k, len(path) - 1):
            acc += math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            if acc >= dist:
                return path[i + 1]
        return path[-1]

    # ------------------------------------------------------------------ geometry
    def sensor_pose(self, frame):
        ox, oy, oz, oyaw = self.sensors.get(frame, (0, 0, 0, 0))
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + ox * c - oy * s, self.y + ox * s + oy * c, BASE_LINK_Z + oz, _wrap(self.yaw + oyaw)

    def frames(self):
        f = {"map": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], "odom": _tf7(*MAP_TO_ODOM[:2], 0.0, MAP_TO_ODOM[2]),
             "base_footprint": _tf7(self.x, self.y, 0.0, self.yaw), "base_link": _tf7(self.x, self.y, BASE_LINK_Z, self.yaw)}
        for name in self.sensors:
            f[name] = _tf7(*self.sensor_pose(name))
        sx, sy, sz, syaw = self.sensor_pose("rp_s2e_lidar")
        f["ur_base_link"] = _tf7(sx, sy, sz + 0.03, _wrap(syaw + math.pi / 2))
        return f

    def odom_pose(self):
        mx, my, myaw = MAP_TO_ODOM
        dx, dy = self.x - mx, self.y - my
        c, s = math.cos(-myaw), math.sin(-myaw)
        return dx * c - dy * s, dx * s + dy * c, _wrap(self.yaw - myaw)

    # ------------------------------------------------------------------ motion
    def current_target(self):
        if self.goal_override:
            return self.goal_override
        return self.targets[self.ti] if self.targets else None

    def step(self, dt):
        while self.S["publish_q"]:
            req = self.S["publish_q"].pop(0)
            if req["type"] == "obstacle":
                self.apply_obstacle(req)
                self.S["published"] = {**req, "t": time.time()}
                continue
            if req["type"] == "halt":
                self.halted, self.frozen_joints = True, dict(self.joints())
            elif req["type"] == "resume":
                self.halted = False
            elif req["type"] == "initialpose":
                self.x, self.y, self.yaw = req["x"], req["y"], req["yaw"]
                self.halted = False
            else:
                self.goal_override = (req["x"], req["y"], req["yaw"], "goal")
                self.halted = False
            self.S["published"] = {**req, "t": time.time()}
        self.S["halted"] = self.halted
        tgt = None if self.halted else self.current_target()
        if tgt is not getattr(self, "_last_tgt", None):
            self._last_tgt = tgt
            self.plan_dirty = True
        v = w = 0.0
        if tgt:
            dx, dy = tgt[0] - self.x, tgt[1] - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.2:
                err = _wrap(tgt[2] - self.yaw)
                if self.goal_override:
                    if abs(err) < 0.1:
                        self.goal_override = None
                        self.ti = min(range(len(self.targets)), key=lambda i: math.hypot(self.targets[i][0] - self.x, self.targets[i][1] - self.y)) if self.targets else 0
                    else:
                        w = max(-W_MAX, min(W_MAX, 2.5 * err))
                else:
                    if self.ti + self.direction >= len(self.targets) or self.ti + self.direction < 0:
                        self.direction *= -1
                    self.ti += self.direction
            else:
                carrot = self.lookahead()
                if carrot is None:
                    v = w = 0.0                               # no path (blocked goal): wait, like nav2
                else:
                    err = _wrap(math.atan2(carrot[1] - self.y, carrot[0] - self.x) - self.yaw)
                    w = max(-W_MAX, min(W_MAX, 2.5 * err))
                    v = V_MAX * max(0.0, math.cos(err)) if abs(err) < 1.2 else 0.0
                    v = min(v, 0.6 * dist + 0.1)
        if self.poly_state.get("stop"):                 # what the collision monitor does with a tripped zone
            v = w = 0.0
        elif self.poly_state.get("slowdown"):
            v, w = v * 0.3, w * 0.3
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = _wrap(self.yaw + w * dt)
        rw, L = 0.1592, 0.544
        self.wheel[0] = _wrap(self.wheel[0] + (v - w * L / 2) / rw * dt)
        self.wheel[1] = _wrap(self.wheel[1] + (v + w * L / 2) / rw * dt)
        return v, w

    # ------------------------------------------------------------------ sensors
    def laser(self, frame, n_beams, max_range, min_range):
        sx, sy, _, syaw = self.sensor_pose(frame)
        ang = np.linspace(-math.pi, math.pi, n_beams, endpoint=False).astype(np.float32)
        r = self.map.raycast(sx, sy, syaw + ang, max_range)
        sz = self.sensor_pose(frame)[2]
        r = np.minimum(r, self.obstacle_ranges(sx, sy, syaw + ang, lambda d: np.full_like(d, sz)))
        fin = np.isfinite(r)
        r = r + (self.rng.normal(0, 0.008, len(r)).astype(np.float32) * fin)
        r[r < min_range] = np.inf
        msg = _ns(header=_hdr(frame), angle_min=-math.pi, angle_max=math.pi, angle_increment=2 * math.pi / n_beams,
                  range_min=min_range, range_max=max_range, ranges=r, intensities=np.full(n_beams, 47.0, np.float32))
        wx = sx + r * np.cos(syaw + ang)
        wy = sy + r * np.sin(syaw + ang)
        pts = np.stack([wx[fin], wy[fin]], 1)
        return msg, pts

    def velodyne(self):
        frame = "velodyne_frame"
        sx, sy, sz, syaw = self.sensor_pose(frame)
        az = np.linspace(-math.pi, math.pi, 900, endpoint=False).astype(np.float32)
        r_wall = self.map.raycast(sx, sy, syaw + az, 25.0)
        chunks = []
        for e_deg in range(-15, 16, 2):
            te = math.tan(math.radians(e_deg))
            if te < 0:
                d_floor = np.float32(sz / -te)
                floor = d_floor < r_wall
                d = np.where(floor, d_floor, r_wall)
                z = np.where(floor, -sz, r_wall * te)
            elif te > 0:
                d_ceil = np.float32((CEILING - sz) / te)
                ceil = (d_ceil < r_wall) & np.isfinite(r_wall)      # no ceiling outdoors / over unknown space
                d = np.where(ceil, d_ceil, r_wall)
                z = np.where(ceil, CEILING - sz, r_wall * te)
            else:
                d, z = r_wall, np.zeros_like(r_wall)
            r_obs = self.obstacle_ranges(sx, sy, syaw + az, lambda dd, te=te: sz + dd * te)
            obs = r_obs < d
            d = np.where(obs, r_obs, d)
            z = np.where(obs, r_obs * te, z)
            ok = np.isfinite(d) & (d > 0.5) & (d < 25.0)
            d, z, a = d[ok], z[ok], az[ok]
            if not len(d):
                continue
            inten = np.where(z > -sz + 0.05, 80.0, 25.0) + self.rng.normal(0, 6, len(d))
            chunks.append(np.stack([d * np.cos(a), d * np.sin(a), z, inten], 1))
        pts = np.concatenate(chunks).astype(np.float32) if chunks else np.zeros((0, 4), np.float32)
        pts[:, :3] += self.rng.normal(0, 0.01, (len(pts), 3)).astype(np.float32)
        msg = _ns(header=_hdr(frame), sim_points=pts)
        # the 2D page's top-down layer: points between CLOUD_Z above the floor, in the map frame
        z0, z1 = self.opts.get("cloud_z", (0.1, 2.0))
        wz = pts[:, 2] + sz
        keep = (wz > z0) & (wz < z1)
        c, s = math.cos(syaw), math.sin(syaw)
        top = np.stack([sx + pts[keep, 0] * c - pts[keep, 1] * s, sy + pts[keep, 0] * s + pts[keep, 1] * c], 1)
        return msg, top

    def local_costmap(self):
        res, n = self.map.res, 120
        bx, by, _ = self.odom_pose()
        ox = math.floor((bx - n * res / 2) / res) * res
        oy = math.floor((by - n * res / 2) / res) * res
        cx = ox + (np.arange(n) + 0.5) * res
        cy = oy + (np.arange(n) + 0.5) * res
        gx, gy = np.meshgrid(cx, cy)                            # odom coords, row-major (row = y)
        mx, my, myaw = MAP_TO_ODOM
        c, s = math.cos(myaw), math.sin(myaw)
        wx = mx + gx * c - gy * s
        wy = my + gx * s + gy * c
        col = np.floor((wx - self.map.ox) / res).astype(int)
        row = np.floor((wy - self.map.oy) / res).astype(int)
        inside = (col >= 0) & (col < self.map.W) & (row >= 0) & (row < self.map.H)
        data = np.full((n, n), -1, np.int8)
        data[inside] = self.global_cost[row[inside], col[inside]]
        data = self.cost_with_obstacles(data, (wx, wy))
        return grid_msg("odom", ox, oy, res, n, n, data.ravel())

    # ------------------------------------------------------------------ nav2 state
    def nav(self):
        tgt = self.current_target()
        out = {"polygons": {}}
        with self.plan_lock:
            path, status = list(self.plan_path), self.plan_status
        out["status"] = status
        if tgt:
            if len(path) >= 2:
                pts = [path[0]]
                for a, b in zip(path, path[1:]):              # resample the string-pulled path every 0.1 m
                    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.1))
                    pts += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
                pts = pts[:400]
                out["plan"] = {"frame": "map", "t": time.time(), "pts": [[round(p[0], 3), round(p[1], 3), 0.0] for p in pts]}
                out["local_plan"] = {"frame": "odom", "t": time.time(), "pts": out["plan"]["pts"][:25]}
            out["goal"] = {"frame": "map", "x": round(tgt[0], 3), "y": round(tgt[1], 3), "yaw": round(tgt[2], 4), "t": time.time()}
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        rot = lambda pts: [[round(self.x + px * c - py * s, 3), round(self.y + px * s + py * c, 3)] for px, py in pts]
        out["polygons"]["footprint"] = {"frame": "odom", "pts": rot(FOOTPRINT)}
        # the robot's collision_monitor_params.yaml shapes (PolygonStop / PolygonSlow)
        out["polygons"]["stop"] = {"frame": "base_footprint", "pts": rot([[0.3, 0.3], [0.6, 0.0], [0.3, -0.3], [-0.35, -0.2], [-0.35, 0.2]])}
        out["polygons"]["slowdown"] = {"frame": "base_footprint", "pts": rot([[0.8, 0.4], [0.8, -0.4], [-0.8, -0.4], [-0.8, 0.4]])}
        mark = self.opts.get("mark_polygons")
        if mark:
            mark(out["polygons"], list(self.scan_full.values()))
            self.poly_state = {n: bool(pg.get("triggered")) for n, pg in out["polygons"].items()}
        out["obstacles"] = [dict(o) for o in self.obstacles]
        return out

    def particles(self, n=300):
        xs = self.x + self.rng.normal(0, 0.08, n)
        ys = self.y + self.rng.normal(0, 0.08, n)
        ya = self.yaw + self.rng.normal(0, 0.06, n)
        return [[round(float(a), 3), round(float(b), 3), round(float(_wrap(c)), 3)] for a, b, c in zip(xs, ys, ya)]

    def camera_frame(self, name, w=320, h=240) -> bytes:
        """A first-person view: walls from the map raycast (height ~ 1/distance, shaded), floor and
        ceiling gradients, no return where the map is unknown — labelled, JPEG-encoded."""
        import io

        from PIL import Image, ImageDraw

        cfg = SIM_CAMS[name]
        c, s_ = math.cos(self.yaw), math.sin(self.yaw)
        ox, oy = cfg["xy"]
        cx, cy = self.x + ox * c - oy * s_, self.y + ox * s_ + oy * c
        sway = 0.08 * math.sin(time.time() * 0.7) if name == "wrist" else 0.0
        yaw = self.yaw + cfg["yaw"] + sway
        fov = math.radians(cfg["fov"])
        cols = np.linspace(fov / 2, -fov / 2, w).astype(np.float32)        # image left = +angle
        r = self.map.raycast(cx, cy, yaw + cols, 15.0)
        r_obs = self.obstacle_ranges(cx, cy, yaw + cols, lambda dd: np.full_like(dd, 0.25))
        is_obs = r_obs < r
        r = np.where(is_obs, r_obs, r)
        d = r * np.cos(cols)                                             # perpendicular distance
        horizon = h * 0.5
        img = np.empty((h, w, 3), np.uint8)
        rows = np.arange(h)[:, None].astype(np.float32)
        ceil = np.clip(60 - rows * 0.25, 20, 60)
        floor = np.clip(40 + (rows - horizon) * 0.9, 40, 170)
        img[..., 0] = np.where(rows < horizon, ceil, floor * 0.75)
        img[..., 1] = np.where(rows < horizon, ceil + 5, floor * 0.7)
        img[..., 2] = np.where(rows < horizon, ceil + 15, floor * 0.6)
        wall_h = np.where(np.isfinite(d), np.clip(h * 1.4 / np.maximum(d, 0.2), 0, h), 0.0)
        top = horizon - wall_h * (cfg["h"] / 1.4)                        # a lower camera sees more wall above
        bot = horizon + wall_h * ((2.6 - cfg["h"]) / 1.4) * 0.45
        mask = (rows >= top[None, :]) & (rows < bot[None, :]) & np.isfinite(d)[None, :]
        shade = np.clip(1.0 - np.nan_to_num(d, nan=15.0, posinf=15.0) / 14.0, 0.12, 1.0)
        base = np.where(is_obs[:, None], np.array([[255, 110, 0]], np.float32), np.array([[205, 190, 165]], np.float32))
        colc = (base * shade[:, None]).astype(np.uint8)                   # per-column wall / cone colour
        wall_h = np.where(is_obs, np.minimum(wall_h, h * 0.7 / np.maximum(d, 0.2)), wall_h)   # a cone is shorter than a wall
        top = horizon - wall_h * (cfg["h"] / 1.4)
        bot = horizon + wall_h * ((2.6 - cfg["h"]) / 1.4) * 0.45
        mask = (rows >= top[None, :]) & (rows < bot[None, :]) & np.isfinite(d)[None, :]
        img = np.where(mask[..., None], colc[None, :, :], img)
        open_ = ~np.isfinite(d)                                          # unknown / outside: bright haze
        img[:, open_, :] = np.where(rows < horizon, 150, img[:, open_, :].mean() if open_.any() else 0).astype(np.uint8)[:, :, None] if open_.any() else img[:, open_, :]
        im = Image.fromarray(img)
        dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, w, 14], fill=(0, 0, 0))
        dr.text((4, 1), f"SIM {name}  ({self.x:.1f}, {self.y:.1f}) {math.degrees(self.yaw):.0f}°  {time.strftime('%H:%M:%S')}", fill=(230, 230, 230))
        buf = io.BytesIO()
        im.save(buf, "JPEG", quality=70)
        return buf.getvalue()

    def joints(self):
        if self.halted and self.frozen_joints:
            return self.frozen_joints
        t = time.time() - self.t0
        j = {"left_wheel_joint": round(self.wheel[0], 4), "right_wheel_joint": round(self.wheel[1], 4)}
        for i, name in enumerate(UR_JOINTS):
            j[name] = round(UR_HOME[i] + 0.25 * math.sin(0.35 * t + i * 1.3), 4)
        return j

    # ------------------------------------------------------------------ main loop
    def run(self):
        S = self.S
        S["ros"] = "ok"
        last_sense = last_slow = last_cam = 0.0
        put_jpeg, cam_want = self.opts.get("put_jpeg"), self.opts.get("cam_want", {})
        t_prev = time.time()
        while True:
            now = time.time()
            dt = min(0.2, now - t_prev)
            t_prev = now
            self.step(dt)
            S["pose"] = {"x": round(self.x, 4), "y": round(self.y, 4), "yaw": round(self.yaw, 4)}
            S["pose_t"] = now
            S["frames"] = self.frames()
            S["joints"] = self.joints()
            if now - last_sense >= 0.1:
                last_sense = now
                scans = {}
                for name, frame, n, rmax, rmin in (("s1", "rp_s1_lidar", 720, 10.0, 0.15), ("s2e", "rp_s2e_lidar", 720, 12.0, 0.1)):
                    msg, pts = self.laser(frame, n, rmax, rmin)
                    S["latest"][name] = msg
                    self.scan_full[name] = pts
                    step = max(1, int(math.ceil(len(pts) / self.opts.get("scan_max", 720))))
                    scans[name] = {"points": np.round(pts[::step], 3).tolist(), "frame": frame, "topic": f"sim/{name}", "t": now}
                msg, top = self.velodyne()
                S["latest"]["velodyne"] = msg
                step = max(1, int(math.ceil(len(top) / self.opts.get("cloud_max", 3000))))
                scans["velodyne"] = {"points": np.round(top[::step], 3).tolist(), "frame": "velodyne_frame", "topic": "sim/velodyne", "t": now}
                S["scans"] = scans
            if put_jpeg is not None:
                watched = any(now - t < 3.0 for t in cam_want.values())
                if now - last_cam >= (0.125 if watched else 1.0):      # 8 fps while someone looks, else a heartbeat
                    last_cam = now
                    for name in SIM_CAMS:
                        put_jpeg(name, self.camera_frame(name))
            if now - last_slow >= 0.5:
                last_slow = now
                S["costmaps"]["local"] = {"msg": self.local_costmap(), "t": now}
                S["nav"] = self.nav()
                S["particles"] = self.particles()
            time.sleep(0.05)


def run(state: dict, map_yaml: Path, opts: dict) -> None:
    try:
        Sim(state, map_yaml, opts).run()
    except Exception as e:  # noqa: BLE001
        state["ros"] = f"sim crashed: {type(e).__name__}: {e}"
        raise


def start(state: dict, map_yaml: Path, opts: dict) -> threading.Thread:
    t = threading.Thread(target=run, args=(state, map_yaml, opts), daemon=True, name="sim")
    t.start()
    return t
