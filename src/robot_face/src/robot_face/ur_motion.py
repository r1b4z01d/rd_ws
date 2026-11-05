"""Replay a recorded joint-space motion on the UR arm the way the teleop drives it: ur_rtde
servoJ streamed at a fixed rate (VLA ur5e_lerobot/robot/rtde_arm.py's streamer: 50 Hz, 20 ms
servo time, 0.1 s lookahead, gain 300, a hard per-tick joint step cap, receive-before-control
connect, servoStop + stopScript on the way out).

The motion comes from ``scripts/ur_record_motion.py`` (one cycle of the pendant's wave program,
sampled over RTDE) and is stored as JSON on the robot PC::

    {"name": "wave", "rate": 50, "q": [[j0..j5], ...], "recorded": "...", "source": "..."}

play() = preconditions (Remote Control, RUNNING, NORMAL, nothing else driving the arm) -> connect
-> servoJ ramp from the current joints to the first sample at <= approach_speed -> stream the
samples -> ramp to the HOME pose (``return_to="home"``: the joints in VLA's
``outputs/home_pose.json``, the pose the teleop panel's *Set Home* captured and the eval returns to;
``"start"`` = back to where the arm was; ``"none"``) -> servoStop/stopScript/disconnect.
A protective or emergency stop mid-motion aborts it. ur_rtde is imported lazily so the module
loads anywhere.

The two ramps between home and the wave pose are STAGED (``approach_order`` / ``return_order``,
e.g. "0245>3>1" = joints 0,2,4,5 together, then 3, then 1) so the hand stays within the robot's
width instead of sweeping out sideways while the base turns and wrist 2 swings 200 deg. The orders
come from ``scripts/ur_path_eval.py`` (forward kinematics over every 1-4 stage ordering); re-run it
after re-recording the wave or capturing a new home pose.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Callable, List, Optional

from .ur_dashboard import DEFAULT_UR_IP, UrDashboard

# Where the arm parks after a gesture: the wave's own "forward home" captured with
# `scripts/ur_record_motion.py --capture-home`, else the eval/training home (the teleop panel's "Set Home").
# A comma-separated list; the first file that exists wins.
DEFAULT_HOME_FILE = "~/.robot_face/motions/home.json,~/rd_ws/VLA/outputs/home_pose.json"
TRACE_FILE = "~/.robot_face/motions/last_wave_trace.csv"   # per-tick record of the last play(), for post-mortems
# Joint stage orders for the ramps between home and the wave pose (scripts/ur_path_eval.py, 2026-11-01).
# Hard rule: no part of the arm ever passes above the chassis footprint (whatever is mounted on top — the
# first orders chosen on lateral reach alone, 0245>3>1 / 1345>0>2, swung the hand over the robot and were
# stopped there twice). Within that: ingress = elbow + wrists first, then the base, the shoulder dips the hand
# out into the wave pose last (fingertips within 36 cm of the centreline instead of 50); egress = base +
# shoulder + wrist 3, then wrists 1/2, the elbow last (35 cm instead of 45). All joints at once would be "".
# A wave demonstrated by freedrive from home back to home needs no staging (the ramps are a few degrees), so
# the defaults are "" (all joints at once); the orders above apply to the 2026-11-01 pendant wave + eval home.
DEFAULT_APPROACH_ORDER = ""
DEFAULT_RETURN_ORDER = ""


def parse_order(order: str) -> List[tuple]:
    """'0245>3>1' -> [(0,2,4,5), (3,), (1,)]; '' -> one stage with every joint. Every joint 0-5
    exactly once, else ValueError."""
    order = (order or "").replace(" ", "")
    if not order:
        return [tuple(range(6))]
    stages = [tuple(int(c) for c in part) for part in order.split(">") if part]
    flat = [j for st in stages for j in st]
    if sorted(flat) != list(range(6)):
        raise ValueError(f"stage order {order!r} must name each joint 0-5 exactly once")
    return stages


class UrMotion:
    def __init__(self, ip: str = DEFAULT_UR_IP, log: Optional[Callable[[str], None]] = None,
                 approach_speed: float = 0.5, max_start_distance: float = 2.5, max_dq: float = 0.05,
                 max_lag: float = 0.35, max_lead: float = 0.2,
                 lookahead: float = 0.1, gain: float = 300.0, speed: float = 0.25, accel: float = 1.2,
                 return_to: str = "home", home_file: str = DEFAULT_HOME_FILE, home_tolerance: float = 0.05,
                 approach_order: str = DEFAULT_APPROACH_ORDER, return_order: str = DEFAULT_RETURN_ORDER,
                 retrace: bool = True, retrace_speedup: float = 1.5, retrace_join: float = 0.35,
                 auto_power_up: bool = True):
        self.ip = ip
        self._log = log or (lambda m: None)
        self.approach_speed = float(approach_speed)        # rad/s for the ramps to/from the motion
        self.max_start_distance = float(max_start_distance)  # rad on any joint; farther = refuse
        self.max_dq = float(max_dq)                        # rad per tick, hard cap (2.5 rad/s at 50 Hz)
        self.max_lag = float(max_lag)                      # rad: actual joints this far behind the command = abort
        self.max_lead = float(max_lead)                    # rad: a ramp never commands further ahead of the arm than this
        self.lookahead, self.gain = float(lookahead), float(gain)
        self.speed, self.accel = float(speed), float(accel)  # servoJ signature parity (unused by servoj)
        self.return_to = str(return_to)                    # home | start | none (where the arm goes after the motion)
        self.home_file = str(home_file)                    # {"q": [6 joints]} — VLA's outputs/home_pose.json
        self.home_tolerance = float(home_tolerance)        # rad: this close to home = "parked at home"
        self.approach_order = str(approach_order)          # staged joint order home -> motion start ("" = all at once)
        self.return_order = str(return_order)              # staged joint order motion end -> home
        self.retrace = bool(retrace)                       # a recording that started at home but ends away from it
        self.retrace_speedup = max(1.0, float(retrace_speedup))   # ...returns home along ITS OWN samples, backwards
        self.retrace_join = float(retrace_join)            # rad: join the recording at the earliest sample this close to the end
        self.auto_power_up = bool(auto_power_up)           # IDLE / protective stop -> eval's power-up first
        self.dash = UrDashboard(ip, log=log)
        self._lock = threading.Lock()
        self.busy = False
        self.abort = threading.Event()

    # -- motion files --------------------------------------------------------------
    @staticmethod
    def load(path: str) -> dict:
        path = os.path.expanduser(path)
        with open(path) as f:
            m = json.load(f)
        q = m.get("q") or []
        if len(q) < 2 or any(len(s) != 6 for s in q):
            raise ValueError(f"{path}: needs at least two 6-joint samples")
        m["rate"] = float(m.get("rate") or 50.0)
        m.setdefault("name", os.path.splitext(os.path.basename(path))[0])
        return m

    @staticmethod
    def duration(m: dict) -> float:
        return len(m["q"]) / m["rate"]

    def home_path(self) -> Optional[str]:
        """The first existing file of the comma-separated home_file list."""
        for part in str(self.home_file).split(","):
            path = os.path.expanduser(part.strip())
            if path and os.path.isfile(path):
                return path
        return None

    def load_home(self) -> Optional[List[float]]:
        """The home joints from the first existing home file, or None (the caller falls back)."""
        path = self.home_path()
        if path is None:
            self._log(f"home pose unavailable (none of {self.home_file} exists); the arm will return to where it started")
            return None
        try:
            with open(path) as f:
                q = json.load(f).get("q")
            if not (isinstance(q, list) and len(q) == 6):
                raise ValueError("needs a 6-joint 'q'")
            return [float(v) for v in q]
        except (OSError, ValueError, AttributeError) as e:
            self._log(f"home pose unavailable ({path}: {e}); the arm will return to where it started")
            return None

    # -- play ----------------------------------------------------------------------
    def play(self, path: str, timeout: Optional[float] = None,
             on_phase: Optional[Callable[[str], None]] = None) -> tuple[bool, str]:
        """Drive the motion. Returns (ok, message); the message is meant for the log. on_phase is
        called with 'approach', 'wave', 'retrace', 'return' as each begins and 'done' at the end
        (also after an abort) — the face node wiggles the hand's fingers during 'wave'."""
        if not self._lock.acquire(blocking=False):
            return False, "motion busy"
        self.busy = True
        self.abort.clear()
        t0 = time.monotonic()
        c = r = None

        def notify(name):
            if on_phase is not None:
                try:
                    on_phase(name)
                except Exception as e:  # noqa: BLE001
                    self._log(f"on_phase({name}) failed: {type(e).__name__}: {e}")
        try:
            try:
                m = self.load(path)
            except (OSError, ValueError) as e:
                return False, f"motion file: {e}"
            try:
                st = self.dash.status()
            except OSError as e:
                return False, f"cannot reach the UR dashboard at {self.ip} ({e})"
            if not st["remote"]:
                return False, "PolyScope is in Local Control; switch the pendant to Remote Control"
            if st["running"]:
                return False, f"a program is already running ({st['state']}); arm is busy"
            note = ""
            if st["robotmode"] != "RUNNING" or st["safety"] != "NORMAL":
                if not self.auto_power_up:
                    if st["robotmode"] != "RUNNING":
                        return False, f"robot mode is {st['robotmode']} (needs RUNNING: powered on, brakes released)"
                    return False, f"safety status is {st['safety']}"
                self._log(f"UR is {st['robotmode']}/{st['safety']}: running the eval/teleop arm startup first")
                try:
                    note = self.dash.power_up() + "; "
                except RuntimeError as e:
                    return False, f"arm startup failed: {e}"
                st = self.dash.status()
                if st["robotmode"] != "RUNNING" or st["safety"] != "NORMAL":
                    return False, f"arm still {st['robotmode']}/{st['safety']} after the startup"
            try:
                from rtde_control import RTDEControlInterface
                from rtde_receive import RTDEReceiveInterface
            except ImportError as e:
                return False, f"ur_rtde is not installed for this interpreter ({e})"
            try:
                r = RTDEReceiveInterface(self.ip)          # receive first: read-only, zero risk
                c = RTDEControlInterface(self.ip)
            except Exception as e:  # noqa: BLE001
                return False, f"ur_rtde connect failed ({e}); is the teleop or eval holding the arm?"
            q_before = [float(v) for v in r.getActualQ()]
            # Where the arm goes afterwards: home (the eval/teleop home pose) by default, else where it was.
            q_home = self.load_home() if self.return_to == "home" else None
            if self.return_to == "none":
                q_after, where = None, ""
            elif q_home is not None:
                q_after, where = q_home, "home"
            else:
                q_after, where = q_before, "start"
            at_home = q_home is not None and max(abs(a - b) for a, b in zip(q_before, q_home)) < self.home_tolerance
            far = max(abs(a - b) for a, b in zip(q_before, m["q"][0]))
            # The distance guard protects against a long blind joint-space sweep from an unknown pose; the
            # home pose is where the arm parks itself after every wave, so the approach from there is allowed.
            if far > self.max_start_distance and not at_home:
                return False, (f"arm is {math.degrees(far):.0f} deg away from the motion's start pose on a "
                               f"joint (limit {math.degrees(self.max_start_distance):.0f}); not moving")
            # Staged orders apply on the two designed routes (home -> wave pose, wave pose -> home); from
            # anywhere else every joint moves at once as before.
            def stages_for(order, use):
                if not use:
                    return [tuple(range(6))]
                try:
                    return parse_order(order)
                except ValueError as e:
                    self._log(f"ignoring the stage order: {e}")
                    return [tuple(range(6))]
            st_in = stages_for(self.approach_order, at_home)
            st_out = stages_for(self.return_order, where == "home")
            # Return along the demonstration: a freedrive take usually starts at home and ends wherever the hand
            # stopped; a straight joint ramp from there is a path nobody chose (it swung the hand over the robot,
            # 2026-11-01), so play the take backwards instead, from the earliest sample within retrace_join of the end.
            retrace_from = None
            if (self.retrace and q_home is not None and where == "home"
                    and max(abs(a - b) for a, b in zip(m["q"][0], q_home)) < 2 * self.home_tolerance
                    and max(abs(a - b) for a, b in zip(m["q"][-1], q_home)) > 0.17):
                q_end = m["q"][-1]
                retrace_from = next(i for i in range(len(m["q"]))
                                    if max(abs(a - b) for a, b in zip(m["q"][i], q_end)) < self.retrace_join)

            def ramp_time(q0, q1, stages):
                return sum(max(abs(q1[j] - q0[j]) for j in st) for st in stages) / self.approach_speed

            dt = 1.0 / m["rate"]
            stats = {"ticks": 0, "late": 0, "max_ms": 0.0, "capped": 0, "paced": 0, "lag": {}}
            phase = ["approach"]
            script_bad = [0]            # consecutive ticks with the control script not running
            reuploaded = [False]
            progress = {"q": None, "t": 0.0}   # stall detector: the arm must keep moving while the command is ahead
            try:
                trace = open(os.path.expanduser(TRACE_FILE), "w")
                trace.write("t,phase,runtime,servoj_ok,speed_scaling,safety,rtde_ts," + ",".join(f"cmd{j}" for j in range(6))
                            + "," + ",".join(f"act{j}" for j in range(6)) + "," + ",".join(f"tgt{j}" for j in range(6)) + "\n")
            except OSError:
                trace = None
            t_ramps = ramp_time(q_before, m["q"][0], st_in) + (ramp_time(m["q"][-1], q_after, st_out) if q_after else 0.0)
            if retrace_from is not None:
                t_ramps += retrace_from / m["rate"] / self.retrace_speedup
            limit = timeout if timeout is not None else 3.0 * self.duration(m) + t_ramps + 25.0   # x3: pacing by the arm

            clock = {"next": time.monotonic()}

            def tick(q_target, dt_tick):
                """One servoJ, paced at dt_tick on the monotonic clock like the teleop streamer (NOT
                ur_rtde's waitPeriod, which is the interface's 2 ms cycle); aborts on a safety stop,
                a tracking error or a timeout."""
                if self.abort.is_set():
                    raise _Abort("aborted")
                if r.isProtectiveStopped() or r.isEmergencyStopped():
                    raise _Abort("protective or emergency stop")
                if time.monotonic() - t0 > limit:
                    raise _Abort(f"over the {limit:.0f}s limit")
                actual = r.getActualQ()
                lags = [abs(a - b) for a, b in zip(actual, q_target)]
                lag = max(lags)
                stats["lag"][phase[0]] = max(stats["lag"].get(phase[0], 0.0), lag)
                if lag > self.max_lag:
                    j = lags.index(lag)
                    raise _Abort(f"arm is {math.degrees(lag):.0f} deg behind the command on joint {j} "
                                 f"(actual {math.degrees(actual[j]):.0f}, commanded {math.degrees(q_target[j]):.0f}) "
                                 f"during the {phase[0]}")
                ok = c.servoJ(q_target, self.speed, self.accel, dt_tick, self.lookahead, self.gain)
                rs = r.getRuntimeState()        # 2 playing, 5 resuming = the ur_rtde control script is executing
                if trace is not None:
                    tq = r.getTargetQ()
                    trace.write(f"{time.monotonic() - t0:.3f},{phase[0]},{rs},{int(bool(ok))},{r.getSpeedScaling():.2f},{r.getSafetyMode()},{r.getTimestamp():.3f},"
                                + ",".join(f"{v:.4f}" for v in q_target) + "," + ",".join(f"{v:.4f}" for v in actual)
                                + "," + ",".join(f"{v:.4f}" for v in tq) + "\n")
                # Blocked arm (someone holding it, an obstacle) or a frozen RTDE receive stream both look the
                # same: the command sits ahead of "actual" and actual stops changing. The ramps' lead clamp
                # would wait forever (two 53 s stalls, 2026-11-01), so give up after a second of no progress.
                now = time.monotonic()
                if progress["q"] is None or max(abs(a - b) for a, b in zip(actual, progress["q"])) > 0.005:
                    progress["q"], progress["t"] = list(actual), now
                elif lag > 0.08 and now - progress["t"] > 1.0:
                    j = lags.index(lag)
                    raise _Abort(f"arm stopped following the command during the {phase[0]} (joint {j} actual "
                                 f"{math.degrees(actual[j]):.0f}, commanded {math.degrees(q_target[j]):.0f} deg, no motion "
                                 f"for {now - progress['t']:.1f}s) — blocked, or the RTDE stream froze")
                if ok and rs in (2, 5):
                    script_bad[0] = 0
                else:
                    script_bad[0] += 1
                    if script_bad[0] >= 25:     # half a second with no control script: the arm just holds
                        if reuploaded[0]:
                            raise _Abort(f"UR control script stopped again (runtime state {rs})")
                        reuploaded[0] = True
                        self._log(f"UR control script stopped (runtime state {rs}, servoJ {'accepted' if ok else 'refused'}) "
                                  f"during the {phase[0]}; re-uploading it like the teleop panel's Reconnect")
                        try:
                            c.reuploadScript()
                        except Exception as e:  # noqa: BLE001
                            raise _Abort(f"UR control script stopped and the re-upload failed: {type(e).__name__}: {e}")
                        t_w = time.monotonic()
                        while time.monotonic() - t_w < 3.0 and r.getRuntimeState() not in (2, 5):
                            time.sleep(0.05)
                        if r.getRuntimeState() not in (2, 5):
                            raise _Abort(f"UR control script stopped and did not restart (runtime state {r.getRuntimeState()})")
                        script_bad[0] = 0
                        clock["next"] = time.monotonic()
                stats["ticks"] += 1
                clock["next"] += dt_tick
                delay = clock["next"] - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    clock["next"] = time.monotonic()

            def settle(q_to, tol=0.009, max_s=2.0):
                """Keep commanding q_to until the arm is within tol (~0.5 deg) of it: servoJ trails
                the command by ~0.3 s, so stopping right after the last tick leaves it short."""
                t_s = time.monotonic()
                while time.monotonic() - t_s < max_s:
                    tick(list(q_to), dt)
                    if max(abs(a - b) for a, b in zip(r.getActualQ(), q_to)) < tol:
                        return

            def ramp(q_from, q_to, stages):
                """Joint-space walk at <= approach_speed per joint, one stage of joints after another,
                streamed at the motion rate, then settled on the target. Like VLA's rtde_arm.home_step
                it is closed on the ARM: the command is never more than max_lead ahead of the actual
                joints, so on a long wrist swing (home <-> wave pose is 203 deg) the profile waits for
                the arm instead of running away from it and tripping the lag abort (which an open-loop
                ramp did at 20 deg behind, 2026-11-01)."""
                step = self.approach_speed * dt
                lead = self.max_lead
                q = list(q_from)
                for st in stages:
                    while True:
                        d = [(b - a) if j in st else 0.0 for j, (a, b) in enumerate(zip(q, q_to))]
                        if max(abs(x) for x in d) <= step:
                            for j in st:
                                q[j] = q_to[j]
                            break
                        q = [a + max(-step, min(step, x)) for a, x in zip(q, d)]
                        actual = r.getActualQ()
                        q = [max(ac - lead, min(ac + lead, x)) for ac, x in zip(actual, q)]
                        tick(q, dt)
                settle(q_to)

            def stream(samples, start):
                """servoJ the samples at the motion rate, PACED BY THE ARM: a sample is only advanced past
                once the arm is within max_lead of it, so a recording faster than the UR will follow
                (its safety speed limits scale servoJ down) plays at the arm's own pace instead of
                running away and tripping the lag abort — "full speed" is then whatever the controller
                allows. A per-tick joint step cap bounds a corrupt sample."""
                last = list(start)
                lead = self.max_lead
                for s in samples:
                    q = [float(v) for v in s]
                    jump = max(abs(a - b) for a, b in zip(q, last))
                    if jump > self.max_dq:                 # a corrupt sample can never yank the arm
                        q = [a + max(-self.max_dq, min(self.max_dq, b - a)) for a, b in zip(last, q)]
                        stats["capped"] += 1
                    while True:
                        actual = r.getActualQ()
                        held = [max(ac - lead, min(ac + lead, x)) for ac, x in zip(actual, q)]
                        tk = time.monotonic()
                        tick(held, dt)
                        ms = (time.monotonic() - tk) * 1000.0
                        stats["max_ms"] = max(stats["max_ms"], ms)
                        if ms > 2.0 * dt * 1000.0:
                            stats["late"] += 1
                        if held == q:
                            break
                        stats["paced"] += 1                # the arm needed another tick to get within reach
                    last = q
                return last

            retrace_s = 0.0
            try:
                notify("approach")
                ramp(q_before, m["q"][0], st_in)
                phase[0] = "wave"
                notify("wave")
                t_wave = time.monotonic()
                last = stream(m["q"], m["q"][0])
                settle(last)
                wave_s = time.monotonic() - t_wave
                if retrace_from is not None:
                    phase[0] = "retrace"
                    notify("retrace")
                    t_r = time.monotonic()
                    ramp(last, m["q"][retrace_from], [tuple(range(6))])
                    step = self.retrace_speedup
                    idx, back = retrace_from, []
                    while idx > 0:
                        idx -= step
                        back.append(m["q"][max(0, int(round(idx)))])
                    last = stream(back, m["q"][retrace_from])
                    settle(m["q"][0])
                    last = list(m["q"][0])
                    retrace_s = time.monotonic() - t_r
                if q_after is not None:
                    phase[0] = "return"
                    notify("return")
                    ramp(last, q_after, st_out)
            except _Abort as e:
                return False, f"{m['name']} stopped: {e} after {time.monotonic() - t0:.1f}s"
            finally:
                notify("done")
                for fn in (c.servoStop, c.stopScript):
                    try:
                        fn()
                    except Exception:  # noqa: BLE001
                        pass
                if trace is not None:
                    trace.close()
            capped = f", {stats['capped']} samples capped" if stats["capped"] else ""
            paced = (f"; the arm paced it: {wave_s:.1f}s for {self.duration(m):.1f}s recorded, {stats['paced']} extra ticks"
                     if stats["paced"] > 10 else "")
            lags = ", ".join(f"{k} {math.degrees(v):.0f}" for k, v in stats["lag"].items())
            retraced = (f"; retraced {retrace_from / m['rate']:.1f}s of it backwards at {self.retrace_speedup:g}x in "
                        f"{retrace_s:.1f}s" if retrace_from is not None else "")
            return True, (f"{note}{m['name']} played in {wave_s:.1f}s{retraced} ({len(m['q'])} samples at {m['rate']:.0f} Hz, "
                          f"{stats['ticks']} servoJ ticks, max {stats['max_ms']:.0f} ms, {stats['late']} late"
                          f"{capped}; max lag deg: {lags}){paced}; total {time.monotonic() - t0:.1f}s"
                          f"{' incl. the return to ' + where if where else ''}"
                          f"{' (started from home)' if at_home else ''}"
                          f"{'; staged ' + ('in ' + self.approach_order if at_home and self.approach_order else '') + (' / ' if at_home and self.approach_order and where == 'home' and self.return_order else '') + ('out ' + self.return_order if where == 'home' and self.return_order else '') if (at_home and self.approach_order) or (where == 'home' and self.return_order) else ''}")
        except Exception as e:  # noqa: BLE001
            return False, f"ur_rtde error: {type(e).__name__}: {e}"
        finally:
            if c is None:
                notify("done")               # never got going: let the caller clean up anyway
            for iface in (c, r):
                try:
                    if iface is not None:
                        iface.disconnect()
                except Exception:  # noqa: BLE001
                    pass
            self.busy = False
            self._lock.release()

    def stop(self) -> None:
        """Abort a motion in progress (checked every tick)."""
        self.abort.set()


class _Abort(Exception):
    pass


if __name__ == "__main__":      # python3 -m robot_face.ur_motion <motion.json> [ip]
    import sys
    mo = UrMotion(sys.argv[2] if len(sys.argv) > 2 else DEFAULT_UR_IP, log=print)
    print(mo.play(sys.argv[1]))
