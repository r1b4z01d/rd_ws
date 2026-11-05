#!/usr/bin/env python3
"""Choose the joint stage orders for the wave's ingress (home -> wave pose) and egress (wave pose
-> home) so the hand stays within the robot's width.

    python3 ur_path_eval.py [--home ~/rd_ws/VLA/outputs/home_pose.json] [--wave ~/.robot_face/motions/wave.json]
                            [--half-width 0.27] [--chassis-top 0.77] [--clearance 0.15] [--tip 0.17] [--track]

Nominal UR5e DH forward kinematics (per-robot calibration differs by mm). Frames: the UR's RTDE base
frame; on Robot Disco the arm sits on the back lidar (base_link x = -0.52 m) yawed so the ROBOT's
lateral axis is the arm frame's x and the body lies at +y (VLA ur5e_lerobot/robot/workspace.py). The
chassis (0.8 m tall box, 0.4 wide, 0.41 long) is 0.3-0.7 m in front of the arm base; its top is at
z = 0.77 in the arm frame. The hand is modelled as the flange plus `--tip` metres along the tool z.

Every ordering of the six joints into 1-4 stages is simulated with the same per-joint clipped-step
ramp UrMotion uses (0.5 rad/s), and ranked by (1) the fingertips' peak distance from the centreline
during the transit (the endpoint poses excluded: the wave pose itself is 43 cm out), (2) time spent
outside the half-width, (3) duration. Orderings that bring any hand/wrist/elbow point closer than
`--clearance` to the chassis top while over its footprint are dropped. Paste the winners into
combined_face_hand_defaults.yaml (`ur_wave_approach_order`, `ur_wave_return_order`); 2026-11-01's
answer was 0245>3>1 in (29 cm instead of 50) and 1345>0>2 out (23 cm instead of 45).
"""
import argparse
import itertools
import json
import math
import os
from math import cos, pi, sin

DH = ((0.0, 0.1625, pi / 2), (-0.425, 0.0, 0.0), (-0.3922, 0.0, 0.0),
      (0.0, 0.1333, pi / 2), (0.0, 0.0997, -pi / 2), (0.0, 0.0996, 0.0))
V, DT = 0.5, 0.02                     # UrMotion approach_speed, motion rate
BODY = dict(xmin=-0.27, xmax=0.27, ymin=0.30, ymax=0.90)   # chassis footprint in the arm frame
HAND = ("wrist2", "flange", "tip")


def mat(a, d, al, th):
    ct, st, ca, sa = cos(th), sin(th), cos(al), sin(al)
    return [[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]]


def mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def fk_points(q, tip):
    T = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    pts = {}
    for name, (a, d, al), th in zip(("base", "shoulder", "elbow", "wrist1", "wrist2", "flange"), DH, q):
        T = mul(T, mat(a, d, al, th))
        pts[name] = (T[0][3], T[1][3], T[2][3])
    pts["tip"] = tuple(pts["flange"][k] + tip * T[k][2] for k in range(3))
    return pts


def staged(q0, q1, stages):
    """The ramp UrMotion drives: per-joint clipped steps, one stage of joints after another."""
    step = V * DT
    q, path = list(q0), [list(q0)]
    for st in stages:
        while True:
            d = [(q1[j] - q[j]) if j in st else 0.0 for j in range(6)]
            if max(abs(x) for x in d) <= step:
                for j in st:
                    q[j] = q1[j]
                path.append(list(q))
                break
            q = [a + max(-step, min(step, x)) for a, x in zip(q, d)]
            path.append(list(q))
    return path


def metrics(path, skip_start, skip_end, half_width, ztop, tip):
    peak = outside = 0.0
    clear, where = 9.0, None
    n = len(path)
    for i, q in enumerate(path):
        pts = fk_points(q, tip)
        for name in HAND + ("wrist1", "elbow"):
            x, y, z = pts[name]
            over = (BODY["xmin"] - 0.05 <= x <= BODY["xmax"] + 0.05) and (BODY["ymin"] - 0.05 <= y <= BODY["ymax"])
            if over and z - ztop < clear:
                clear, where = z - ztop, f"{name} at {i * DT:.1f}s"
        lat = max(abs(pts[name][0]) for name in HAND)
        if skip_start <= i < n - skip_end:
            peak = max(peak, lat)
        outside += max(0.0, lat - half_width) * DT
    return peak, outside, clear, where


def orderings():
    yield "", [tuple(range(6))]
    for k in (2, 3, 4):
        for assign in itertools.product(range(k), repeat=6):
            g = [tuple(j for j in range(6) if assign[j] == s) for s in range(k)]
            if all(g):
                yield ">".join("".join(map(str, s)) for s in g), g


def track(path, tip):
    for i in range(0, len(path), 25):
        p = fk_points(path[i], tip)
        t = p["tip"]
        print(f"   t={i * DT:4.1f}s q={[round(math.degrees(v)) for v in path[i]]}  tip: robot x={t[1] - 0.52:+.2f} "
              f"y={-t[0]:+.2f} z={t[2]:.2f}   |lateral| tip {abs(t[0]) * 100:3.0f} wrist {abs(p['wrist2'][0]) * 100:3.0f} cm")


def rank(q0, q1, label, skip_start, skip_end, a):
    rows = []
    for lbl, g in orderings():
        p = staged(q0, q1, g)
        rows.append((lbl or "(all at once)", *metrics(p, skip_start, skip_end, a.half_width, a.chassis_top, a.tip), len(p) * DT, g))
    ok = sorted((r for r in rows if r[3] >= a.clearance), key=lambda r: (round(r[1], 2), round(r[2], 2), r[5]))
    cur = rows[0]
    print(f"\n== {label}: {len(rows)} orderings, {len(ok)} keep >= {a.clearance * 100:.0f} cm above the chassis top ==")
    print(f"  current {cur[0]:22s} transit peak {cur[1] * 100:3.0f} cm   outside {cur[2]:4.1f} cm·s   clearance {cur[3] * 100:+4.0f} cm ({cur[4]})   {cur[5]:.1f}s")
    for r in ok[:8]:
        print(f"  {r[0]:30s} transit peak {r[1] * 100:3.0f} cm   outside {r[2]:4.1f} cm·s   clearance {r[3] * 100:+4.0f} cm ({r[4]})   {r[5]:.1f}s")
    if a.track and ok:
        print(f"  -- track for {ok[0][0]} --")
        track(staged(q0, q1, ok[0][6]), a.tip)
    return ok[0][0] if ok else ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--home", default="~/rd_ws/VLA/outputs/home_pose.json")
    ap.add_argument("--wave", default="~/.robot_face/motions/wave.json")
    ap.add_argument("--half-width", type=float, default=0.27, help="robot half-width, m (VLA NO_GO_XY)")
    ap.add_argument("--chassis-top", type=float, default=0.77, help="chassis top above the arm base, m")
    ap.add_argument("--clearance", type=float, default=0.15, help="required height above the chassis top, m")
    ap.add_argument("--tip", type=float, default=0.17, help="fingertips beyond the flange along tool z, m")
    ap.add_argument("--track", action="store_true", help="print the fingertip track of the best orders")
    a = ap.parse_args()
    home = json.load(open(os.path.expanduser(a.home)))["q"]
    wave = json.load(open(os.path.expanduser(a.wave)))["q"]
    for name, q in (("home", home), ("wave pose", wave[0])):
        t = fk_points(q, a.tip)["tip"]
        print(f"{name:10s} {[round(math.degrees(v)) for v in q]}  fingertips {abs(t[0]) * 100:.0f} cm from the centreline, "
              f"{t[2]:.2f} m above the arm base")
    s_in = rank(home, wave[0], "INGRESS home -> wave pose", 0, 25, a)
    s_out = rank(wave[-1], home, "EGRESS wave pose -> home", 25, 0, a)
    print(f"\nur_wave_approach_order: \"{'' if s_in == '(all at once)' else s_in}\"\nur_wave_return_order: \"{'' if s_out == '(all at once)' else s_out}\"")


if __name__ == "__main__":
    main()
