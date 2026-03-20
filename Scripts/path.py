from pathlib import Path
from typing import Optional
import time
import subprocess
import threading

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from nav2_simple_commander.robot_navigator import BasicNavigator
import nav2_simple_commander.robot_navigator as rn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger


def create_waypoints(navigator: BasicNavigator, yaml_path: Optional[Path] = None):
    waypoints = []
    waypoint_names = []
    initial_pose = None
    initial_pose_actions = []
    post_actions = []
    path = Path(__file__).with_name("waypoints.yaml") if yaml_path is None else Path(yaml_path)

    if not path.exists():
        raise FileNotFoundError(f"Waypoints file not found: {path}")

    with path.open() as stream:
        data = yaml.safe_load(stream) or {}

    entries = data.get("waypoints") if isinstance(data, dict) else None
    if not isinstance(entries, list):
        raise ValueError(f"Waypoints file has unexpected format: {path}")

    for entry in entries:
        position = entry.get("position", {})
        orientation = entry.get("orientation", {})

        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.header.stamp = navigator.get_clock().now().to_msg()
        waypoint.pose.position.x = float(position.get("x", 0.0))
        waypoint.pose.position.y = float(position.get("y", 0.0))
        waypoint.pose.position.z = float(position.get("z", 0.0))
        waypoint.pose.orientation.x = float(orientation.get("x", 0.0))
        waypoint.pose.orientation.y = float(orientation.get("y", 0.0))
        waypoint.pose.orientation.z = float(orientation.get("z", 0.0))
        waypoint.pose.orientation.w = float(orientation.get("w", 0.0))

        waypoints.append(waypoint)
        waypoint_names.append(entry.get("name", f"waypoint_{len(waypoints)}"))
        actions = entry.get("post_actions", [])
        if actions is None:
            actions = []
        if not isinstance(actions, list):
            raise ValueError(f"post_actions for waypoint '{waypoint_names[-1]}' must be a list")
        post_actions.append([str(a) for a in actions])

    while len(post_actions) < len(waypoints):
        post_actions.append([])

    pose_entry = data.get("initial_pose") if isinstance(data, dict) else None
    if isinstance(pose_entry, dict):
        position = pose_entry.get("position", {})
        orientation = pose_entry.get("orientation", {})
        actions = pose_entry.get("post_actions", [])
        if actions is None:
            actions = []
        if not isinstance(actions, list):
            raise ValueError("initial_pose.post_actions must be a list")
        initial_pose_actions = [str(a) for a in actions]
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = navigator.get_clock().now().to_msg()
        msg.pose.position.x = float(position.get("x", 0.0))
        msg.pose.position.y = float(position.get("y", 0.0))
        msg.pose.position.z = float(position.get("z", 0.0))
        msg.pose.orientation.x = float(orientation.get("x", 0.0))
        msg.pose.orientation.y = float(orientation.get("y", 0.0))
        msg.pose.orientation.z = float(orientation.get("z", 0.0))
        msg.pose.orientation.w = float(orientation.get("w", 1.0))
        initial_pose = msg

    return waypoints, waypoint_names, initial_pose, initial_pose_actions, post_actions


def main():
    rclpy.init()
    navigator = BasicNavigator()
    logger = navigator.get_logger()

    path_qos = QoSProfile(depth=1)
    path_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    path_qos.reliability = ReliabilityPolicy.RELIABLE
    path_publisher = navigator.create_publisher(NavPath, "waypoints_path", path_qos)
    
    navigator.waitUntilNav2Active()
    
    waypoints_list, waypoint_names, initial_pose, initial_pose_actions, post_actions = create_waypoints(navigator)
    logger.info(f"Loaded {len(waypoints_list)} waypoints: {', '.join(waypoint_names)}")

    if initial_pose:
        navigator.setInitialPose(initial_pose)
        logger.info("Published initial pose from waypoints.yaml to set localization estimate.")

    path_msg = NavPath()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = navigator.get_clock().now().to_msg()
    path_msg.poses = waypoints_list
    path_publisher.publish(path_msg)
    logger.info(f"Published {len(waypoints_list)} waypoints to 'waypoints_path'.")

    started = False
    start_requested = False
    running_threads = []

    def handle_start(req, res):
        nonlocal start_requested, started
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

    last_feedback_index = None
    executed_actions = set()
    last_feedback_log = time.time()
    last_status_log = time.time()

    def run_cmd(command: str):
        try:
            completed = subprocess.run(
                command,
                shell=True,
                check=True,
                capture_output=True,
                text=True,
            )
            stdout = (completed.stdout or "").strip()
            stderr = (completed.stderr or "").strip()
            if stdout:
                logger.info(f"[post-action] stdout: {stdout}")
            if stderr:
                logger.warning(f"[post-action] stderr: {stderr}")
            logger.info(f"[post-action] command succeeded (exit 0): {command}")
        except subprocess.CalledProcessError as exc:
            stdout = (exc.stdout or "").strip()
            stderr = (exc.stderr or "").strip()
            if stdout:
                logger.info(f"[post-action] stdout: {stdout}")
            if stderr:
                logger.error(f"[post-action] stderr: {stderr}")
            logger.error(f"[post-action] command failed (exit {exc.returncode}): {command}")

    def trigger_initial_pose_actions():
        for cmd in initial_pose_actions:
            logger.info(
                f"[post-action] initial_pose: starting command async: {cmd}"
            )
            t = threading.Thread(target=run_cmd, args=(cmd,), daemon=True)
            t.start()
            running_threads.append(t)

    def trigger_post_actions(arrived_index: int, reason: str):
        if not (0 <= arrived_index < len(post_actions)):
            return
        if arrived_index in executed_actions:
            return

        waypoint_name = waypoint_names[arrived_index] if arrived_index < len(waypoint_names) else str(arrived_index)
        logger.info(
            f"[waypoint] Confirmed arrival at '{waypoint_name}' (index {arrived_index}) via {reason}."
        )
        for cmd in post_actions[arrived_index]:
            logger.info(
                f"[post-action] waypoint {arrived_index} ({waypoint_name}): "
                f"starting command async: {cmd}"
            )
            t = threading.Thread(target=run_cmd, args=(cmd,), daemon=True)
            t.start()
            running_threads.append(t)
        executed_actions.add(arrived_index)

    if initial_pose_actions:
        trigger_initial_pose_actions()

    while rclpy.ok():
        rclpy.spin_once(navigator, timeout_sec=0.1)

        if start_requested and not started:
            ok = navigator.followWaypoints(waypoints_list)
            if not ok:
                logger.error("FollowWaypoints goal was rejected.")
                start_requested = False
            else:
                started = True
                logger.info(f"Following {len(waypoints_list)} goals....")
            continue

        if not started:
            continue

        now = time.time()
        feedback = navigator.getFeedback()
        if feedback and feedback.current_waypoint is not None:
            current_index = int(feedback.current_waypoint)
            dist_rem = getattr(feedback, "distance_remaining", None)
            if dist_rem is not None:
                logger.info(f"[feedback] current_waypoint={current_index}, distance_remaining={dist_rem:.3f} m")
            if last_feedback_index is None:
                if 0 <= current_index < len(waypoint_names):
                    logger.info(
                        f"[waypoint] Started processing '{waypoint_names[current_index]}' (index {current_index})."
                    )
                else:
                    logger.info(f"[waypoint] Started processing index {current_index}.")
            elif current_index != last_feedback_index:
                if 0 <= current_index < len(waypoint_names):
                    logger.info(
                        f"[waypoint] Switching to '{waypoint_names[current_index]}' (index {current_index})."
                    )
                else:
                    logger.info(f"[waypoint] Switching to index {current_index}.")

                # Nav2 feedback current_waypoint points to the waypoint being processed.
                # When it advances, the previous index has just been reached.
                if current_index > last_feedback_index:
                    trigger_post_actions(current_index - 1, "feedback transition")
                else:
                    logger.warning(
                        f"[waypoint] Feedback index moved backward {last_feedback_index}->{current_index}; "
                        "skipping inferred arrival trigger."
                    )

            last_feedback_index = current_index
        elif now - last_feedback_log > 2.0:
            logger.info("[waypoint] No feedback yet from FollowWaypoints action...")
            last_feedback_log = now

        if now - last_status_log > 5.0:
            logger.info(
                f"[status] started={started}, task_complete={navigator.isTaskComplete()}, "
                f"feedback_index={last_feedback_index}, executed_actions={sorted(executed_actions)}"
            )
            last_status_log = now

        if started and navigator.isTaskComplete():
            break

    if started and navigator.isTaskComplete():
        final_index = len(waypoints_list) - 1
        trigger_post_actions(final_index, "task completion")

    result = navigator.getResult()
    if started:
        if result == rn.TaskResult.SUCCEEDED:
            logger.info("Waypoints followed successfully!")
        elif result == rn.TaskResult.FAILED:
            logger.warning("Waypoint following failed.")
        elif result == rn.TaskResult.CANCELED:
            logger.warning("Waypoint following was canceled.")
        else:
            logger.warning(f"Waypoint following completed with status: {result}.")
    else:
        logger.warning("Shutting down without starting waypoint following (no service call received).")

    # Allow background post-actions to finish logging (non-blocking for long ones)
    for t in running_threads:
        t.join(timeout=0.5)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
