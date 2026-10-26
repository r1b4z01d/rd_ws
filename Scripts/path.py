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
    steps: 0                        cap in control steps (0 = off)
    keep_grasp: false               carry the object: hand stays closed, UR stays powered for the drive
    home: true                      return the arm to its home (transport) pose at the end
    on_fail: stop
    extra: ""                       extra vla_action.py arguments

A waypoint with a blocking action ends a Nav2 FollowWaypoints segment: the robot arrives, the waypoint's
background actions fire, its blocking actions run in order (the node keeps spinning), then the next
segment starts. A failed blocking action with on_fail: stop ends the route where the robot stands.
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


def run_action(action: Action, log, spin: Optional[Callable[[], None]] = None) -> int:
    """Run one action, streaming its output into the log. With `spin`, keep the node alive meanwhile."""
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
    while proc.poll() is None:
        if spin is not None:
            spin()
        else:
            time.sleep(0.2)
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
    args = [a for a in sys.argv[1:] if not a.startswith("--ros-args")]
    if args and args[0] == "--check":
        sys.exit(check(Path(args[1]) if len(args) > 1 else Path(__file__).with_name("waypoints.yaml")))
    route_path = Path(args[0]) if args else Path(__file__).with_name("waypoints.yaml")

    import rclpy
    import nav2_simple_commander.robot_navigator as rn
    from geometry_msgs.msg import PoseStamped
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
            logger.info(f"[post-action] waypoint {idx} ({wp.name}): running and WAITING: {act.label}")
            rc = run_action(act, logger, spin=spin)
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
        while rclpy.ok() and not navigator.isTaskComplete():
            spin()
            now = time.time()
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
        for first, last in segs:
            result = follow(first, last)
            if result != rn.TaskResult.SUCCEEDED:
                break
            if not run_blocking(last):
                stopped_early = True
                break

    if not started:
        logger.warning("Shutting down without starting waypoint following (no service call received).")
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
