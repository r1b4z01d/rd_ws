#!/usr/bin/env python3
"""Record a joint trajectory for the UR arm so the face node can replay it (robot_face/ur_motion.py).

Two ways to get one:

* **Freedrive demonstration** (``--freedrive``): the arm is released (teach mode), the face says so,
  and the recording starts the moment you begin moving the arm by hand. Move it through the gesture —
  start at the home pose and finish back at the home pose, keeping the hand within the robot's width —
  then hold it still for 4 s (--still-end); the arm locks again, the trace is smoothed (100 ms moving average) and
  saved. The previous file is kept as <name>_<timestamp>.json.

      python3 ur_record_motion.py --freedrive --say        # on the robot PC; --say announces through the face

* **Capture a home pose** (``--capture-home``): freedrive, move the arm to the pose you want, hold still
  4 s, and it is saved to ~/.robot_face/motions/home.json ({"q", "tcp", ...}) — the wave's "forward home"
  (``ur_wave_home_file``), separate from the eval/training home in ~/rd_ws/VLA/outputs/home_pose.json.
  With ``--freedrive`` as well, the wave recording follows right after, from the new home.

      python3 ur_record_motion.py --capture-home --freedrive --say

* **Keyframes** (``--keyframes``): build the gesture point by point. Where the arm is when the script
  starts is point one; freedrive comes on and every pose you hold still for 4 s is the next point (the
  face counts them); hold 10 s longer at the last point to finish. The motion eases between the points
  at ``--speed`` rad/s (max joint speed), with the home pose added before the first and after the last
  point, so the replay starts and ends at home; ``--repeat N`` runs through the points N times (a wave
  that swings back and forth). The points are kept in <name>_keyframes.json, and ``--regen`` rebuilds
  the motion from them with a new ``--repeat`` / ``--speed`` without touching the arm.

      python3 ur_record_motion.py --keyframes --say
      python3 ur_record_motion.py --regen --repeat 3

* **Pendant program** (default): waits until the named program is PLAYING on the pendant (press Play
  there, in Local Control), samples the joints while it runs and keeps one cycle of a looping program.

      python3 ur_record_motion.py                          # waits up to 2 h for wave.urp
      python3 ur_record_motion.py --program hello.urp --wait 600 --out ~/.robot_face/motions/hello.json

Output: ~/.robot_face/motions/<name>.json  {"name", "rate", "q": [[6 joints] ...], "recorded", "source", "duration_s"}.
RTDE receive is read-only and coexists with the joint-state publisher's connection; freedrive needs the
control interface (nothing else may be driving the arm) and PolyScope in Remote Control.
"""
import argparse
import json
import math
import os
import socket
import sys
import time
from datetime import datetime


def dashboard(ip, cmds):
    s = socket.create_connection((ip, 29999), timeout=5)
    f = s.makefile("r", newline="\n")
    f.readline()
    out = []
    for c in cmds:
        s.sendall((c + "\n").encode())
        out.append(f.readline().strip())
    s.sendall(b"quit\n")
    s.close()
    return out


_SAY = None


def make_say(enabled):
    """Speak through the face node (/combined_face_hand/say) when --say and rclpy are available.
    One ROS context per process: a second call returns the same speaker (rclpy.init() twice raises)."""
    global _SAY
    if _SAY is not None:
        return _SAY
    if not enabled:
        _SAY = lambda text: print(f"[say] {text}", flush=True)  # noqa: E731
        return _SAY
    try:
        import rclpy
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node("ur_record_motion_say")
        pub = node.create_publisher(String, "/combined_face_hand/say", 10)
        time.sleep(1.0)                                   # discovery

        def say(text):
            print(f"[say] {text}", flush=True)
            pub.publish(String(data=text))
            rclpy.spin_once(node, timeout_sec=0.1)
        _SAY = say
    except Exception as e:  # noqa: BLE001
        print(f"[say] face unavailable ({type(e).__name__}); printing only", flush=True)
        _SAY = lambda text: print(f"[say] {text}", flush=True)  # noqa: E731
    return _SAY


def smooth(samples, n):
    """Centred moving average over n samples per joint; endpoints kept."""
    if n < 2 or len(samples) < n:
        return samples
    h = n // 2
    out = []
    for i in range(len(samples)):
        lo, hi = max(0, i - h), min(len(samples), i + h + 1)
        out.append([round(sum(s[j] for s in samples[lo:hi]) / (hi - lo), 5) for j in range(6)])
    out[0], out[-1] = samples[0], samples[-1]
    return out


def trim_still(samples, rate, still, lead_s=0.2):
    def moving(i):
        return max(abs(p - q) for p, q in zip(samples[i], samples[i - 1])) > still
    first = next((i for i in range(1, len(samples)) if moving(i)), 1)
    last = next((i for i in range(len(samples) - 1, 0, -1) if moving(i)), len(samples) - 1)
    lead = int(lead_s * rate)
    return samples[max(0, first - lead): min(len(samples), last + lead + 1)]


def save(out, name, rate, kept, source):
    if os.path.exists(out):
        stamp = datetime.fromtimestamp(os.path.getmtime(out)).strftime("%Y%m%d-%H%M")
        backup = out.replace(".json", f"_{stamp}.json")
        os.replace(out, backup)
        print(f"[record] previous recording kept as {backup}", flush=True)
    span = [math.degrees(max(s[j] for s in kept) - min(s[j] for s in kept)) for j in range(6)]
    m = {"name": name, "rate": rate, "q": kept, "recorded": datetime.now().isoformat(timespec="seconds"),
         "source": source, "duration_s": round(len(kept) / rate, 2)}
    with open(out, "w") as f:
        json.dump(m, f)
    print(f"[record] saved {out}: {len(kept)} samples, {len(kept) / rate:.1f}s, joint travel (deg) "
          + ", ".join(f"{x:.0f}" for x in span), flush=True)
    home_file = next((os.path.expanduser(h) for h in HOME_FILES if os.path.exists(os.path.expanduser(h))), "")
    if home_file:
        try:
            h = json.load(open(home_file))["q"]
            d0 = max(abs(a - b) for a, b in zip(kept[0], h))
            d1 = max(abs(a - b) for a, b in zip(kept[-1], h))
            print(f"[record] starts {math.degrees(d0):.0f} deg and ends {math.degrees(d1):.0f} deg from the home pose "
                  f"{home_file} (the replay ramps from/to home over whatever is left)", flush=True)
        except (OSError, ValueError, KeyError):
            pass


HOME_FILES = ("~/.robot_face/motions/home.json", "~/rd_ws/VLA/outputs/home_pose.json")   # the wave's home, then the eval's


def arm_free(ip):
    """Preconditions for taking the arm: Remote Control, RUNNING/NORMAL, no program running. A leftover
    ur_rtde control script ("<unnamed>", e.g. from an earlier recorder) is stopped here; anything else
    (teleop, eval, a pendant program) is left alone."""
    st = dashboard(ip, ["is in remote control", "robotmode", "safetystatus", "running", "programState"])
    print(f"[record] UR: {st}", flush=True)
    if st[0] != "true":
        print("[record] PolyScope is in Local Control; switch the pendant to Remote Control", flush=True)
        return False
    if "RUNNING" not in st[1] or "NORMAL" not in st[2]:
        print("[record] the arm is not RUNNING/NORMAL (power it on and release the brakes, or clear the stop)", flush=True)
        return False
    if st[3].endswith("true"):
        if st[4].endswith("<unnamed>"):
            print("[record] stopping a leftover ur_rtde control script", flush=True)
            dashboard(ip, ["stop"])
            time.sleep(1.0)
            return arm_free(ip) if dashboard(ip, ["running"])[0].endswith("false") else False
        print(f"[record] a program is running on the arm ({st[4]}: teleop, eval, a wave?); stop it first", flush=True)
        return False
    return True


def wait_still(r, rate, still, still_end):
    """Block until the arm has not moved (> still over 0.2 s) for still_end seconds; returns the joints."""
    back = int(0.2 * rate)
    hist = []
    last_motion = time.monotonic()
    while True:
        q = list(r.getActualQ())
        hist.append(q)
        if len(hist) > back and max(abs(p - v) for p, v in zip(q, hist[-1 - back])) > still:
            last_motion = time.monotonic()
        if time.monotonic() - last_motion > still_end:
            return q
        time.sleep(1.0 / rate)


def capture_home(a, out_home, say, save_pose=True):
    """Freedrive; the pose the arm is held still in becomes the wave's home (save_pose=False: just
    release the arm so it can be placed by hand, lock it again when it is held still, save nothing)."""
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
    if not arm_free(a.ip):
        return 2
    r = RTDEReceiveInterface(a.ip, a.rate)
    c = RTDEControlInterface(a.ip)
    try:
        if not c.teachMode():
            print("[record] teachMode() refused", flush=True)
            return 2
        time.sleep(1.0)
        q_rest = list(r.getActualQ())
        what = "the new home pose" if save_pose else "where you want it"
        print(f"[record] FREEDRIVE ON — move the arm to {what} and hold it still {a.still_end:.0f}s "
              f"(waiting up to {a.wait / 60:.0f} min)", flush=True)
        say(f"Freedrive is on. Move my arm to {what} and hold it still.")
        t_wait = time.monotonic()
        while max(abs(p - q) for p, q in zip(r.getActualQ(), q_rest)) < a.start_motion:
            if time.monotonic() - t_wait > a.wait:
                print("[record] nobody moved the arm; giving up", flush=True)
                say("Nobody moved my arm. Freedrive off.")
                return 2
            time.sleep(0.02)
        print("[record] motion detected; waiting for the arm to be held still", flush=True)
        q = wait_still(r, a.rate, a.still, a.still_end)
        tcp = list(r.getActualTCPPose())
    finally:
        try:
            c.endTeachMode()
        except Exception:  # noqa: BLE001
            pass
        for iface in (c,):
            try:
                iface.stopScript()
            except Exception:  # noqa: BLE001
                pass
        for iface in (c, r):
            try:
                iface.disconnect()
            except Exception:  # noqa: BLE001
                pass
    if not save_pose:
        home_file = next((os.path.expanduser(h) for h in HOME_FILES if os.path.exists(os.path.expanduser(h))), "")
        note = ""
        if home_file:
            h = json.load(open(home_file))["q"]
            note = f"; {math.degrees(max(abs(x - y) for x, y in zip(q, h))):.0f} deg from the home pose in {home_file}"
        print(f"[record] arm locked at {[round(math.degrees(v)) for v in q]} deg{note}", flush=True)
        say("Locked. Freedrive is off.")
        return 0
    out_home = os.path.expanduser(out_home)
    os.makedirs(os.path.dirname(out_home), exist_ok=True)
    if os.path.exists(out_home):
        stamp = datetime.fromtimestamp(os.path.getmtime(out_home)).strftime("%Y%m%d-%H%M")
        os.replace(out_home, out_home.replace(".json", f"_{stamp}.json"))
    with open(out_home, "w") as f:
        json.dump({"q": [round(v, 5) for v in q], "tcp": [round(v, 4) for v in tcp],
                   "recorded": datetime.now().isoformat(timespec="seconds"),
                   "source": f"freedrive capture on {a.ip} (ur_record_motion.py --capture-home)"}, f, indent=1)
    print(f"[record] home pose saved to {out_home}: {[round(math.degrees(v)) for v in q]} deg", flush=True)
    say("Home pose saved.")
    return 0


def load_home_q():
    for h in HOME_FILES:
        path = os.path.expanduser(h)
        if os.path.exists(path):
            try:
                return [float(v) for v in json.load(open(path))["q"]], path
            except (OSError, ValueError, KeyError):
                pass
    return None, ""


def keyframes_motion(frames, rate, speed):
    """Joint-space path through the frames: each leg takes max|dq| / speed (>= 0.5 s) with a cosine
    ease in/out, sampled at `rate`."""
    samples = []
    for a, b in zip(frames, frames[1:]):
        d = max(abs(x - y) for x, y in zip(a, b))
        n = max(2, int(round(max(0.5, d / speed) * rate)))
        for k in range(n):
            u = k / (n - 1)
            u = 0.5 - 0.5 * math.cos(math.pi * u)
            samples.append([round(x + (y - x) * u, 5) for x, y in zip(a, b)])
    return samples


def record_keyframes(a, name, out):
    """Point one = where the arm is; then freedrive, a point per 4 s hold, 10 s more = done."""
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
    say = make_say(a.say)
    if not arm_free(a.ip):
        return 2
    r = RTDEReceiveInterface(a.ip, a.rate)
    c = RTDEControlInterface(a.ip)
    frames = [[round(v, 5) for v in r.getActualQ()]]
    print(f"[record] point 1 = where the arm is: {[round(math.degrees(v)) for v in frames[0]]} deg", flush=True)
    try:
        if not c.teachMode():
            print("[record] teachMode() refused", flush=True)
            return 2
        time.sleep(1.0)
        q_rest = list(r.getActualQ())
        print(f"[record] FREEDRIVE ON — move to the next point and hold still {a.still_end:.0f}s per point; hold "
              f"{a.end_after:.0f}s longer at the last point to finish (waiting up to {a.wait / 60:.0f} min per point)", flush=True)
        say("Point one is where my arm is now. Freedrive is on: move my arm to the next point and hold still. "
            "At the last point keep holding and I will finish.")
        while True:
            t_wait = time.monotonic()
            while max(abs(p - q) for p, q in zip(r.getActualQ(), q_rest)) < a.start_motion:
                if time.monotonic() - t_wait > a.wait:
                    print("[record] no more motion; finishing", flush=True)
                    break
                time.sleep(0.02)
            else:
                q = [round(v, 5) for v in wait_still(r, a.rate, a.still, a.still_end)]
                frames.append(q)
                print(f"[record] point {len(frames)}: {[round(math.degrees(v)) for v in q]} deg", flush=True)
                say(f"Point {len(frames)}.")
                t_hold = time.monotonic()
                moved = False
                while time.monotonic() - t_hold < a.end_after:
                    if max(abs(p - v) for p, v in zip(r.getActualQ(), q)) > a.start_motion:
                        moved = True
                        break
                    time.sleep(0.05)
                if moved:
                    q_rest = q
                    continue
            break
    finally:
        for fn in (c.endTeachMode, c.stopScript):
            try:
                fn()
            except Exception:  # noqa: BLE001
                pass
        for iface in (c, r):
            try:
                iface.disconnect()
            except Exception:  # noqa: BLE001
                pass
        print("[record] freedrive off (arm holds)", flush=True)
    say(f"Done, {len(frames)} points. Freedrive is off.")
    kf_out = out.replace(".json", "_keyframes.json")
    with open(kf_out, "w") as f:
        json.dump({"frames": frames, "speed": a.speed, "repeat": a.repeat,
                   "recorded": datetime.now().isoformat(timespec="seconds")}, f, indent=1)
    print(f"[record] points kept in {kf_out}", flush=True)
    return build_from_keyframes(a, name, out, frames)


def build_from_keyframes(a, name, out, frames):
    """wave.json = home, the points (--repeat times), home — eased at --speed."""
    home, home_file = load_home_q()
    full = ([home] if home else []) + frames * max(1, a.repeat) + ([home] if home else [])
    samples = keyframes_motion(full, a.rate, a.speed)
    print(f"[record] {len(frames)} points x{a.repeat}{' + home before and after' if home else ''} -> {len(samples)} samples "
          f"at {a.speed:g} rad/s max joint speed", flush=True)
    save(out, name, a.rate, samples, f"{len(frames)} freedrive keyframes x{a.repeat} eased at {a.speed:g} rad/s"
                                     f"{', home from ' + home_file if home else ''}")
    return 0


def regen(a, name, out):
    kf = out.replace(".json", "_keyframes.json")
    d = json.load(open(kf))
    frames = d["frames"]
    print(f"[record] rebuilding {out} from {len(frames)} points in {kf}", flush=True)
    return build_from_keyframes(a, name, out, frames)


def record_freedrive(a, name, out):
    """Release the arm, record the hand-guided motion, lock it again."""
    from rtde_control import RTDEControlInterface
    from rtde_receive import RTDEReceiveInterface
    say = make_say(a.say)
    if not arm_free(a.ip):
        return 2
    r = RTDEReceiveInterface(a.ip, a.rate)
    c = RTDEControlInterface(a.ip)
    dt = 1.0 / a.rate
    rc = 0
    try:
        if not c.teachMode():
            print("[record] teachMode() refused", flush=True)
            return 2
        time.sleep(1.0)                      # the arm sags a few degrees when teach mode releases it; settle first
        q_rest = list(r.getActualQ())
        print(f"[record] FREEDRIVE ON — move the arm through the gesture (start and finish at home), then hold still "
              f"{a.still_end:.0f}s. Waiting up to {a.wait / 60:.0f} min for you to start.", flush=True)
        say("Freedrive is on. Show me the wave: start from home, finish at home, then hold still.")
        t_wait = time.monotonic()
        while True:
            # wait for a deliberate start: a joint moved > start_motion from rest
            while max(abs(p - q) for p, q in zip(r.getActualQ(), q_rest)) < a.start_motion:
                if time.monotonic() - t_wait > a.wait:
                    print("[record] nobody moved the arm; giving up", flush=True)
                    say("Nobody moved my arm. Freedrive off.")
                    return 2
                time.sleep(0.02)
            print("[record] motion detected; recording", flush=True)
            say("Recording.")
            samples = [q_rest]
            t0 = time.monotonic()
            next_t = t0
            last_motion = t0
            travel = 0.0
            while True:
                now = time.monotonic()
                if now >= next_t:
                    q = [round(v, 5) for v in r.getActualQ()]
                    samples.append(q)
                    next_t += dt
                    back = int(0.2 * a.rate)
                    if len(samples) > back and max(abs(p - v) for p, v in zip(q, samples[-1 - back])) > a.still:
                        last_motion = now
                    travel = max(travel, max(abs(p - v) for p, v in zip(q, q_rest)))
                if now - last_motion > a.still_end:
                    break
                if now - t0 > a.max_seconds:
                    print(f"[record] stopping at the {a.max_seconds:.0f}s cap", flush=True)
                    break
                time.sleep(0.002)
            if travel >= a.min_travel and now - t0 >= a.still_end + a.min_seconds:
                break
            # a nudge or a pause before the real gesture: start over from where the arm is now
            print(f"[record] only {math.degrees(travel):.0f} deg of travel in {now - t0:.1f}s — that was not the gesture; "
                  f"waiting again", flush=True)
            say("That was short. Show me the whole wave, then hold still.")
            q_rest = list(r.getActualQ())
    finally:
        try:
            c.endTeachMode()
        except Exception:  # noqa: BLE001
            pass
        try:
            c.stopScript()               # otherwise the control script stays "running" on the UR
        except Exception:  # noqa: BLE001
            pass
        print("[record] freedrive off (arm holds)", flush=True)
    kept = trim_still(samples, a.rate, a.still)
    kept = smooth(kept, a.smooth)
    say(f"Got it, {len(kept) / a.rate:.0f} seconds. Freedrive off.")
    save(out, name, a.rate, kept, f"freedrive demonstration on {a.ip}, smoothed over {a.smooth} samples")
    for iface in (c, r):
        try:
            iface.disconnect()
        except Exception:  # noqa: BLE001
            pass
    return rc


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.11.21")
    ap.add_argument("--program", default="wave.urp", help="pendant program to wait for (names the output too)")
    ap.add_argument("--out", default="", help="default ~/.robot_face/motions/<program>.json")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--wait", type=float, default=7200.0, help="seconds to wait for Play / for the hand-guided motion to start")
    ap.add_argument("--max-seconds", type=float, default=30.0)
    ap.add_argument("--still", type=float, default=0.004, help="rad: joint change counted as motion when trimming")
    ap.add_argument("--cycle", default="auto", choices=["auto", "one", "all"],
                    help="looping pendant programs: keep one cycle (auto = when the cap was hit)")
    ap.add_argument("--freedrive", action="store_true", help="record a hand-guided demonstration instead of a pendant program")
    ap.add_argument("--keyframes", action="store_true", help="build the motion point by point in freedrive (see the module doc)")
    ap.add_argument("--end-after", type=float, default=10.0, help="keyframes: seconds of extra stillness at a point that end the session")
    ap.add_argument("--speed", type=float, default=0.6, help="keyframes: max joint speed of the generated motion, rad/s")
    ap.add_argument("--repeat", type=int, default=1, help="keyframes: run through the points this many times (a wave that swings back and forth)")
    ap.add_argument("--regen", action="store_true", help="rebuild <name>.json from <name>_keyframes.json with the given --repeat/--speed; the arm is not touched")
    ap.add_argument("--release", action="store_true",
                    help="freedrive only: release the arm so it can be placed by hand, lock it when held still, save nothing")
    ap.add_argument("--capture-home", nargs="?", const="~/.robot_face/motions/home.json", default="",
                    help="freedrive: save the pose the arm is held still in as the wave's home (default ~/.robot_face/motions/home.json); with --freedrive the wave recording follows")
    ap.add_argument("--say", action="store_true", help="announce freedrive on/off through the face (/combined_face_hand/say)")
    ap.add_argument("--still-end", type=float, default=4.0, help="freedrive: seconds of stillness that end the recording")
    ap.add_argument("--start-motion", type=float, default=0.1, help="freedrive: rad on any joint that starts the recording (6 deg)")
    ap.add_argument("--min-travel", type=float, default=0.26, help="freedrive: a take with less joint travel than this (15 deg) is discarded and the wait resumes")
    ap.add_argument("--min-seconds", type=float, default=2.0, help="freedrive: minimum motion duration of a take")
    ap.add_argument("--smooth", type=int, default=5, help="freedrive: moving-average window in samples (5 = 100 ms)")
    a = ap.parse_args()
    name = os.path.splitext(os.path.basename(a.program))[0]
    out = os.path.expanduser(a.out or f"~/.robot_face/motions/{name}.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    if a.regen:
        return regen(a, name, out)
    if a.release:
        return capture_home(a, "", make_say(a.say), save_pose=False)
    if a.keyframes:
        return record_keyframes(a, name, out)
    if a.capture_home:
        rc = capture_home(a, a.capture_home, make_say(a.say))
        if rc != 0 or not a.freedrive:
            return rc
        time.sleep(2.0)
    if a.freedrive:
        if a.max_seconds == 30.0:
            a.max_seconds = 90.0
        return record_freedrive(a, name, out)

    from rtde_receive import RTDEReceiveInterface
    r = RTDEReceiveInterface(a.ip, a.rate)
    print(f"[record] waiting up to {a.wait / 60:.0f} min for '{a.program}' to start PLAYING on the pendant "
          f"(RTDE at {a.rate:.0f} Hz, output {out})", flush=True)
    t_start = time.monotonic()
    last_dash = 0.0
    playing_named = False
    while True:
        if time.monotonic() - t_start > a.wait:
            print("[record] gave up: the program never started", flush=True)
            return 2
        running = bool(r.getRobotStatus() & 2)
        if running:
            st = dashboard(a.ip, ["programState"])[0]
            if st.startswith("PLAYING") and st.endswith(a.program):
                playing_named = True
                break
            # something else is running (teleop script, another program): wait for it to end
            if time.monotonic() - last_dash > 10.0:
                print(f"[record] ignoring: {st} (not {a.program})", flush=True)
                last_dash = time.monotonic()
            time.sleep(0.5)
            continue
        time.sleep(0.05)

    print(f"[record] {a.program} is playing; recording", flush=True)
    dt = 1.0 / a.rate
    samples = []
    hit_cap = False
    t0 = time.monotonic()
    next_t = t0
    stopped_since = None
    while True:
        now = time.monotonic()
        if now >= next_t:
            samples.append([round(v, 5) for v in r.getActualQ()])
            next_t += dt
        if int((now - t0) * a.rate) % int(a.rate / 4) == 0:   # 4 checks a second
            if not (r.getRobotStatus() & 2):
                stopped_since = stopped_since or now
                if now - stopped_since > 0.3:
                    break
            else:
                stopped_since = None
        if now - t0 > a.max_seconds:
            print(f"[record] stopping at the {a.max_seconds:.0f}s cap (the program is still running: it loops)", flush=True)
            hit_cap = True
            break
        time.sleep(0.002)

    # a looping program ("Program loops forever", PolyScope's default): keep exactly one cycle, the
    # samples up to the first return to the start pose after at least a second of motion
    if a.cycle == "one" or (a.cycle == "auto" and hit_cap):
        start = samples[0]
        best, best_err = None, 1e9
        for i in range(int(1.0 * a.rate), len(samples)):
            err = max(abs(p - q) for p, q in zip(samples[i], start))
            if err < best_err:
                best, best_err = i, err
            if best_err < a.still and err > 10 * a.still:   # closed the loop and left again
                break
        if best is not None and best_err < 0.02:
            print(f"[record] looping program: keeping one cycle of {best} samples ({best / a.rate:.2f}s), "
                  f"loop closure {math.degrees(best_err):.2f} deg", flush=True)
            samples = samples[:best]
        else:
            print("[record] could not find the cycle (no return to the start pose); keeping everything", flush=True)

    # trim still frames at both ends (the program's own settle time), keep 0.2 s of lead-in
    kept = trim_still(samples, a.rate, a.still)
    save(out, name, a.rate, kept, f"pendant program {a.program} on {a.ip}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
