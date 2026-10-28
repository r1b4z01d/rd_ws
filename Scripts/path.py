#!/usr/bin/env python3
"""Waypoint runner: drive a route from a YAML file with Nav2 and run each waypoint's post_actions.

    cd ~/rd_ws/Scripts && python3 path.py [route.yaml]      # default: waypoints.yaml next to this file
    ros2 service call /start_waypoints std_srvs/srv/Trigger {}
    python3 path.py --check route.yaml                      # parse, print the segments and commands, no ROS

post_actions entries (per waypoint, and for initial_pose):
  - "shell command"                 fired on arrival, in the background (the robot keeps driving)
  - run: "shell command"            the same, with options:
    wait: true                      blocking: the robot STOPS at this waypoint until the command exits
    on_fail: stop | continue        blocking only: a non-zero exit stops the route (default) or not
  - vla: <model>                    a VLA policy run on the arm (always blocking): model = a training run
    task: "..."                       name, run@step, or a path under ~/VLA/outputs/train; task = the
    seconds: 120                      string it was trained with. Cap on the playing phase (wall clock).
    still: 0                        finish early after N s without commanded motion (0 = off)
    done: none                      closed | open — end by the GRASP LEVEL: 'closed' once the mean
                                      commanded curl has averaged >= 0.7 over `still` seconds (a pick),
                                      'open' once it averages <= 0.2 after having been closed (a place)
    still_mode: all                 all | fingers — motion-based alternative (noisy; prefer done:)
    wait_go: false                  launch paused and wait for PLAY (deck / page) as the go
    steps: 0                        cap in control steps (0 = off)
    keep_grasp: false               carry the object: hand stays closed, UR stays powered for the drive
    home: true                      return the arm to its home (transport) pose at the end
    on_fail: stop
    extra: ""                       extra vla_action.py arguments

A waypoint with a blocking action ends a Nav2 FollowWaypoints segment: the robot arrives, the waypoint's
background actions fire, its blocking actions run in order (the node keeps spinning), then the next
segment starts. A failed blocking action with on_fail: stop ends the route where the robot stands.
DEADMAN: once the route is running, pressing the gamepad's drive-enable or turbo button (the indices in
src/robot_disco/config/joystick.yaml) cancels the Nav2 goal and ends the route; a blocking action running
at that moment is sent SIGTERM (the eval's clean exit) first. --no-deadman-stop disables it.
`vla:` actions go through ~/VLA/scripts/vla_action.py, which stops the recording panel if it runs,
switches the inference server to the model, runs scripts/eval_hw.py --auto and returns when the arm is
back home; see VLA/wiki/Deploy-on-Hardware.md.
"""
from __future__ import annotations

import os
import shlex
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import yaml

VLA_DIR = os.environ.get("VLA_DIR", os.path.expanduser("~/VLA"))
JOYSTICK_YAML = Path(__file__).resolve().parent.parent / "src" / "robot_disco" / "config" / "joystick.yaml"


def stop_buttons() -> list[int]:
    """The gamepad buttons that stop a running route: the drive-enable and turbo buttons from joystick.yaml."""
    try:
        d = yaml.safe_load(open(JOYSTICK_YAML)) or {}
        pr = (d.get("teleop_node") or {}).get("ros__parameters") or {}
        out = [int(pr[k]) for k in ("enable_button", "enable_turbo_button") if k in pr and int(pr[k]) >= 0]
        if out:
            return out
    except Exception:  # noqa: BLE001
        pass
    return [7, 6]


@dataclass
class Action:
    kind: str                 # "shell" | "vla"
    cmd: "str | list[str]"    # a shell string, or an argv list
    blocking: bool
    on_fail: str = "stop"     # "stop" | "continue" (blocking actions only)
    label: str = ""


@dataclass
class Waypoint:
    name: str
    position: dict
    orientation: dict
    actions: list = field(default_factory=list)

    @property
    def blocking(self) -> list:
        return [a for a in self.actions if a.blocking]

    @property
    def background(self) -> list:
        return [a for a in self.actions if not a.blocking]


def vla_command(raw: dict, where: str) -> list[str]:
    model = str(raw["vla"]).strip()
    task = raw.get("task")
    if not model or not task:
        raise ValueError(f"{where}: a vla action needs 'vla: <model>' and 'task: ...'")
    cmd = [os.path.join(VLA_DIR, "run.sh"), "scripts/vla_action.py", "--model", model, "--task", str(task),
           "--seconds", str(float(raw.get("seconds", 120)))]
    if raw.get("still"):
        cmd += ["--still", str(float(raw["still"]))]
    if raw.get("steps"):
        cmd += ["--steps", str(int(raw["steps"]))]
    if raw.get("min_seconds") is not None:
        cmd += ["--min-seconds", str(float(raw["min_seconds"]))]
    if raw.get("keep_grasp"):
        cmd.append("--keep-grasp")
    if raw.get("home", True) is False:
        cmd.append("--no-home")
    if raw.get("still_mode"):
        cmd += ["--still-mode", str(raw["still_mode"])]
    if raw.get("done"):
        cmd += ["--done-when", str(raw["done"])]
    if raw.get("wait_go"):
        cmd.append("--wait-go")
    if raw.get("extra"):
        cmd += shlex.split(str(raw["extra"]))
    return cmd


def parse_action(raw, where: str) -> Action:
    if isinstance(raw, str):
        return Action("shell", raw, False, "continue", raw)
    if not isinstance(raw, dict):
        raise ValueError(f"{where}: an action is a string, or a mapping with 'run:' or 'vla:'")
    on_fail = str(raw.get("on_fail", "stop")).lower()
    if on_fail not in ("stop", "continue"):
        raise ValueError(f"{where}: on_fail must be 'stop' or 'continue'")
    if "vla" in raw:
        return Action("vla", vla_command(raw, where), True, on_fail, f"vla {raw['vla']}: {raw.get('task')}")
    if "run" in raw:
        return Action("shell", str(raw["run"]), bool(raw.get("wait", False)), on_fail, str(raw["run"]))
    raise ValueError(f"{where}: an action is a string, or a mapping with 'run:' or 'vla:'")


def parse_actions(raw, where: str) -> list[Action]:
    if raw is None:
        return []
    if not isinstance(raw, list):
        raise ValueError(f"{where}: post_actions must be a list")
    return [parse_action(a, f"{where} action {i + 1}") for i, a in enumerate(raw)]


def load_route(path: Path) -> tuple[list[Waypoint], Optional[Waypoint]]:
    """The route file -> (waypoints, initial_pose or None). Pure parsing, no ROS."""
    if not path.exists():
        raise FileNotFoundError(f"Waypoints file not found: {path}")
    with path.open() as stream:
        data = yaml.safe_load(stream) or {}
    entries = data.get("waypoints") if isinstance(data, dict) else None
    if not isinstance(entries, list):
        raise ValueError(f"Waypoints file has unexpected format: {path}")
    waypoints = []
    for i, entry in enumerate(entries):
        name = str(entry.get("name", f"waypoint_{i + 1}"))
        waypoints.append(Waypoint(name, entry.get("position", {}) or {}, entry.get("orientation", {}) or {},
                                  parse_actions(entry.get("post_actions"), f"waypoint '{name}'")))
    initial = None
    pose_entry = data.get("initial_pose") if isinstance(data, dict) else None
    if isinstance(pose_entry, dict):
        initial = Waypoint("initial_pose", pose_entry.get("position", {}) or {},
                           pose_entry.get("orientation", {}) or {},
                           parse_actions(pose_entry.get("post_actions"), "initial_pose"))
    return waypoints, initial


def segments(waypoints: list[Waypoint]) -> list[tuple[int, int]]:
    """(first, last) index ranges: a segment ends at a waypoint with a blocking action, or at the end."""
    segs, start = [], 0
    for i, wp in enumerate(waypoints):
        if wp.blocking or i == len(waypoints) - 1:
            segs.append((start, i))
            start = i + 1
    return segs


def run_action(action: Action, log, spin: Optional[Callable[[], None]] = None,
               abort: Optional[Callable[[], bool]] = None) -> int:
    """Run one action, streaming its output into the log. With `spin`, keep the node alive meanwhile;
    with `abort`, terminate the action (SIGTERM, then SIGKILL after 30 s) when it returns True."""
    shell = isinstance(action.cmd, str)
    try:
        proc = subprocess.Popen(action.cmd, shell=shell, cwd=VLA_DIR if action.kind == "vla" else None,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    except OSError as e:
        log.error(f"[post-action] could not start: {action.label}: {e}")
        return 127

    def _pump() -> None:
        for line in proc.stdout:
            log.info(f"[{action.kind}] {line.rstrip()}")

    t = threading.Thread(target=_pump, daemon=True)
    t.start()
    killed_at = None
    while proc.poll() is None:
        if spin is not None:
            spin()
        else:
            time.sleep(0.2)
        if abort is not None and killed_at is None and abort():
            log.warning(f"[post-action] aborting: {action.label} (SIGTERM)")
            proc.terminate()
            killed_at = time.time()
        elif killed_at is not None and time.time() - killed_at > 30.0:
            log.error("[post-action] still running 30 s after SIGTERM — killing it")
            proc.kill()
            killed_at = float("inf")
    t.join(timeout=2.0)
    rc = proc.returncode
    if rc == 0:
        log.info(f"[post-action] ok: {action.label}")
    else:
        log.error(f"[post-action] FAILED (exit {rc}): {action.label}")
    return rc


def check(path: Path) -> int:
    """--check: print what the route would do, without ROS."""
    waypoints, initial = load_route(path)
    print(f"{path}: {len(waypoints)} waypoints, {len(segments(waypoints))} segment(s)")
    if initial is not None:
        print(f"  initial_pose ({initial.position.get('x')}, {initial.position.get('y')})"
              + (f"  {len(initial.actions)} action(s)" if initial.actions else ""))
    for s, (a, b) in enumerate(segments(waypoints)):
        print(f"  segment {s + 1}: {' -> '.join(w.name for w in waypoints[a:b + 1])}")
        for wp in waypoints[a:b + 1]:
            for act in wp.actions:
                mode = "BLOCKING" if act.blocking else "background"
                cmd = shlex.join(act.cmd) if isinstance(act.cmd, list) else act.cmd
                print(f"      {wp.name}: [{mode}{', on_fail=' + act.on_fail if act.blocking else ''}] {cmd}")
    return 0


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--ros-args") and a != "--no-deadman-stop"]
    if args and args[0] == "--check":
        sys.exit(check(Path(args[1]) if len(args) > 1 else Path(__file__).with_name("waypoints.yaml")))
    route_path = Path(args[0]) if args else Path(__file__).with_name("waypoints.yaml")

    deadman_enabled = "--no-deadman-stop" not in sys.argv

    import rclpy
    import nav2_simple_commander.robot_navigator as rn
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy
    from nav_msgs.msg import Path as NavPath
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_srvs.srv import Trigger

    rclpy.init()
    navigator = BasicNavigator()
    logger = navigator.get_logger()

    def pose_of(wp: Waypoint, default_w: float = 0.0) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = navigator.get_clock().now().to_msg()
        msg.pose.position.x = float(wp.position.get("x", 0.0))
        msg.pose.position.y = float(wp.position.get("y", 0.0))
        msg.pose.position.z = float(wp.position.get("z", 0.0))
        msg.pose.orientation.x = float(wp.orientation.get("x", 0.0))
        msg.pose.orientation.y = float(wp.orientation.get("y", 0.0))
        msg.pose.orientation.z = float(wp.orientation.get("z", 0.0))
        msg.pose.orientation.w = float(wp.orientation.get("w", default_w))
        return msg

    path_qos = QoSProfile(depth=1)
    path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    path_qos.reliability = ReliabilityPolicy.RELIABLE
    path_publisher = navigator.create_publisher(NavPath, "waypoints_path", path_qos)

    navigator.waitUntilNav2Active()

    waypoints, initial = load_route(route_path)
    poses = [pose_of(wp) for wp in waypoints]
    segs = segments(waypoints)
    logger.info(f"Loaded {len(waypoints)} waypoints from {route_path}: {', '.join(w.name for w in waypoints)}")
    if len(segs) > 1:
        logger.info("Blocking actions at: " + ", ".join(waypoints[b].name for _, b in segs if waypoints[b].blocking))

    if initial is not None:
        navigator.setInitialPose(pose_of(initial, default_w=1.0))
        logger.info("Published initial pose from the route file to set localization estimate.")

    path_msg = NavPath()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = navigator.get_clock().now().to_msg()
    path_msg.poses = poses
    path_publisher.publish(path_msg)
    logger.info(f"Published {len(poses)} waypoints to 'waypoints_path'.")

    started = False
    start_requested = False
    running_threads: list[threading.Thread] = []
    executed: set[int] = set()

    def spin() -> None:
        rclpy.spin_once(navigator, timeout_sec=0.1)

    def handle_start(req, res):
        nonlocal start_requested
        if started or start_requested:
            res.success = False
            res.message = "Waypoint following already started or requested."
            logger.warning(res.message)
            return res
        start_requested = True
        res.success = True
        res.message = "Waypoint following start requested."
        logger.info("Start service called: will start following waypoints.")
        return res

    navigator.create_service(Trigger, "start_waypoints", handle_start)
    logger.info("Waiting for 'start_waypoints' Trigger service call to begin following waypoints.")

    # Deadman: the gamepad's drive-enable / turbo buttons stop a running route (operator takes over).
    deadman = {"buttons": stop_buttons(), "armed": False, "pressed": False}

    def _joy(msg) -> None:
        if deadman["armed"] and not deadman["pressed"] and any(
                i < len(msg.buttons) and msg.buttons[i] for i in deadman["buttons"]):
            deadman["pressed"] = True
            logger.warning(f"[deadman] gamepad button pressed (indices {deadman['buttons']}) — stopping the route")

    if deadman_enabled:
        navigator.create_subscription(Joy, "/joy", _joy, 10)
        logger.info(f"[deadman] gamepad buttons {deadman['buttons']} stop the route once it is running")

    # Localization watchdog: Nav2 keeps "driving" (recovery spins) on a stale map pose — on 2026-10-27
    # the map->odom transform froze for 11 minutes and the robot spun in place. Watch the map pose with
    # our own TF listener while a segment runs; if it goes stale, cancel and stop.
    from tf2_ros import Buffer, TransformListener

    tf_buf = Buffer()
    TransformListener(tf_buf, navigator)
    STALE_S = 3.0
    stale = {"since": None, "tripped": False}

    def map_pose_age() -> Optional[float]:
        try:
            tf = tf_buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        st = tf.header.stamp
        return navigator.get_clock().now().nanoseconds / 1e9 - (st.sec + st.nanosec / 1e9)

    def localization_stale() -> bool:
        age = map_pose_age()
        now = time.time()
        if age is None or age > STALE_S:
            stale["since"] = stale["since"] or now
            if now - stale["since"] > STALE_S and not stale["tripped"]:
                stale["tripped"] = True
                logger.error(f"[localization] map pose {'missing' if age is None else f'{age:.1f} s old'} for "
                             f"{STALE_S:.0f} s — AMCL is not updating; stopping the route")
        else:
            stale["since"] = None
        return stale["tripped"]

    def stopped() -> bool:
        return deadman["pressed"] or stale["tripped"]

    def fire_background(idx: int, reason: str) -> None:
        """The waypoint's background actions, once, on (inferred) arrival."""
        if not (0 <= idx < len(waypoints)) or idx in executed:
            return
        executed.add(idx)
        wp = waypoints[idx]
        logger.info(f"[waypoint] Confirmed arrival at '{wp.name}' (index {idx}) via {reason}.")
        for act in wp.background:
            logger.info(f"[post-action] waypoint {idx} ({wp.name}): starting command async: {act.label}")
            t = threading.Thread(target=run_action, args=(act, logger), daemon=True)
            t.start()
            running_threads.append(t)

    def run_blocking(idx: int) -> bool:
        """The waypoint's blocking actions, in order, with the robot stopped. False = stop the route."""
        wp = waypoints[idx]
        for act in wp.blocking:
            if stopped():
                return False
            logger.info(f"[post-action] waypoint {idx} ({wp.name}): running and WAITING: {act.label}")
            rc = run_action(act, logger, spin=spin, abort=stopped)
            if stopped():
                logger.error(f"[post-action] route stopped by the deadman during '{wp.name}'.")
                return False
            if rc != 0 and act.on_fail == "stop":
                logger.error(f"[post-action] route stopped at '{wp.name}' (on_fail: stop).")
                return False
        return True

    def follow(first: int, last: int):
        """One FollowWaypoints segment; arrival at each waypoint fires its background actions."""
        if not navigator.followWaypoints(poses[first:last + 1]):
            logger.error("FollowWaypoints goal was rejected.")
            return rn.TaskResult.FAILED
        names = [w.name for w in waypoints[first:last + 1]]
        logger.info(f"Following {len(names)} goal(s): {', '.join(names)}")
        last_index = None
        last_feedback_log = last_status_log = time.time()
        cancelled = False
        while rclpy.ok() and not navigator.isTaskComplete():
            spin()
            now = time.time()
            if (deadman["pressed"] or localization_stale()) and not cancelled:
                navigator.cancelTask()          # Nav2 stops the base; the loop ends on the cancel result
                cancelled = True
            feedback = navigator.getFeedback()
            if feedback and feedback.current_waypoint is not None:
                current = first + int(feedback.current_waypoint)
                dist_rem = getattr(feedback, "distance_remaining", None)
                if dist_rem is not None:
                    logger.info(f"[feedback] current_waypoint={current}, distance_remaining={dist_rem:.3f} m")
                if last_index is None:
                    logger.info(f"[waypoint] Started processing '{waypoints[current].name}' (index {current}).")
                elif current != last_index:
                    logger.info(f"[waypoint] Switching to '{waypoints[current].name}' (index {current}).")
                    # Nav2's current_waypoint is the one being processed: when it advances, the previous
                    # index has just been reached.
                    if current > last_index:
                        fire_background(current - 1, "feedback transition")
                    else:
                        logger.warning(f"[waypoint] Feedback index moved backward {last_index}->{current}; "
                                       "skipping inferred arrival trigger.")
                last_index = current
            elif now - last_feedback_log > 2.0:
                logger.info("[waypoint] No feedback yet from FollowWaypoints action...")
                last_feedback_log = now
            if now - last_status_log > 5.0:
                logger.info(f"[status] segment {first}-{last}, feedback_index={last_index}, "
                            f"executed_actions={sorted(executed)}")
                last_status_log = now
        result = navigator.getResult()
        if cancelled:
            return rn.TaskResult.CANCELED
        # Nav2's waypoint follower skips a waypoint whose NavigateToPose failed (stop_on_failure is
        # off) and still reports SUCCEEDED, listing the skipped ones in missed_waypoints. On 2026-10-27
        # every goal failed within a second (the controller had no map frame) and the whole 19 m route
        # "succeeded" in 4 s without the base moving; treat any skipped waypoint as a failure.
        missed = []
        try:
            missed = list(navigator.result_future.result().result.missed_waypoints)
        except Exception:  # noqa: BLE001 — older nav2 / no result object
            pass
        if missed:
            names = [waypoints[first + i].name for i in missed if 0 <= first + i < len(waypoints)]
            logger.error(f"[waypoint] Nav2 skipped {len(missed)} waypoint(s) it could not reach: {names} "
                         "— stopping the route (check the navigation log: planner/controller errors, TF)")
            return rn.TaskResult.FAILED
        if result == rn.TaskResult.SUCCEEDED:
            fire_background(last, "segment completion")
        return result

    # initial_pose actions: background ones now; a blocking one runs before the route is armed
    if initial is not None:
        for act in initial.background:
            logger.info(f"[post-action] initial_pose: starting command async: {act.label}")
            t = threading.Thread(target=run_action, args=(act, logger), daemon=True)
            t.start()
            running_threads.append(t)
        for act in initial.blocking:
            logger.info(f"[post-action] initial_pose: running and WAITING: {act.label}")
            run_action(act, logger, spin=spin)

    while rclpy.ok() and not start_requested:
        spin()

    result = None
    stopped_early = False
    if rclpy.ok():
        started = True
        deadman["armed"] = True
        for first, last in segs:
            result = follow(first, last)
            if result != rn.TaskResult.SUCCEEDED:
                break
            if not run_blocking(last):
                stopped_early = True
                break

    if not started:
        logger.warning("Shutting down without starting waypoint following (no service call received).")
    elif stale["tripped"]:
        logger.error("Route STOPPED: localization went stale (no fresh map pose).")
    elif stopped():
        logger.warning("Route STOPPED by the deadman button (operator took over).")
    elif stopped_early:
        logger.warning("Route stopped by a failed blocking action.")
    elif result == rn.TaskResult.SUCCEEDED:
        logger.info("Waypoints followed successfully!")
    elif result == rn.TaskResult.FAILED:
        logger.warning("Waypoint following failed.")
    elif result == rn.TaskResult.CANCELED:
        logger.warning("Waypoint following was canceled.")
    else:
        logger.warning(f"Waypoint following completed with status: {result}.")

    # Allow background post-actions to finish logging (non-blocking for long ones)
    for t in running_threads:
        t.join(timeout=0.5)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
