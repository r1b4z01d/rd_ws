#!/usr/bin/env python3
"""
ROS 2 node wrapper around the original combined_face_hand.py.

It subscribes to a color image topic (default: /camera/camera/color/image_raw),
runs MediaPipe pose + hand tracking, renders the animated eyes window, and
optionally relays joint angles over TCP. All runtime configuration is exposed
as ROS parameters (see config/combined_face_hand_defaults.yaml).
"""

import math
import socket
import time
from typing import Optional

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .eye_renderer import draw_eye
from .face_utils import box_top_center, pick_face, pose_box_from_landmarks, schedule_blink
from .hand_processing import JOINT_OPEN_OFFSETS, format_joint_command, generate_joint_offsets
from .window_utils import (
    is_global_key_pressed,
    is_hotkey_pressed,
    set_windows_window_frame_color,
    supports_global_hotkeys,
)

# ---------- MediaPipe Hands setup ----------
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose


class FaceHandNode(Node):
    def __init__(self):
        super().__init__("combined_face_hand")

        # Declare ROS parameters (defaults mirror config/combined_face_hand_defaults.yaml)
        self.declare_parameters(
            "",
            [
                ("image_topic", "/camera/camera/color/image_raw"),
                ("hand_in_eyes", False),
                ("mirror", False),
                ("width", 1280),
                ("height", 720),
                ("fps", 30.0),
                ("relay_hand", False),
                ("hand_hotkey", "ctrl+shift+h"),
                ("hand_host", "192.168.1.194"),
                ("hand_port", 8765),
                ("hand_rate", 20.0),
                ("hand_speed", 1000),
                ("hand_panel_height", 400),
                ("hand_panel_fraction", 0.5),
                ("hand_panel_scale", 1.0),
                ("blink_min", 2.5),
                ("blink_max", 15.5),
                ("blink_duration", 0.18),
                ("blink_closed_hold", 0.04),
                ("gaze_speed", 0.3),
                ("gaze_idle_speed", 0.01),
                ("save_raw_video", "/tmp/raw_camera.mp4"),
                ("save_video", "/tmp/combined_output.mp4"),
            ],
        )

        def p(name):
            return self.get_parameter(name).value

        self.bridge = CvBridge()

        # Camera/image state
        self.camera_width = int(p("width"))
        self.camera_height = int(p("height"))
        self.capture_fps = float(p("fps")) if float(p("fps")) > 0 else 30.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, p("image_topic"), self.image_callback, qos)

        # Hand relay state
        self.hand_stream_enabled = bool(p("relay_hand"))
        self.hand_stream_hotkey = str(p("hand_hotkey")) or "s"
        self.global_hotkey_supported = supports_global_hotkeys()
        self.hand_stream_toggle_active = False
        self.hotkey_prev_state = False
        self.hand_target_speed = int(min(max(int(p("hand_speed")), 50), 2000))
        self.hand_socket: Optional[socket.socket] = None
        self.last_hand_send = 0.0
        self.hand_send_interval = 1.0 / max(float(p("hand_rate")), 1e-5)
        self.hand_idle_timeout = 0.75
        self.hand_last_detection_time = time.monotonic()
        self.hand_open_sent = False

        if self.hand_stream_enabled:
            if self.global_hotkey_supported:
                self.get_logger().info(
                    f"Hand relay armed. Press '{self.hand_stream_hotkey}' to toggle streaming "
                    "(works even when the window is unfocused)."
                )
            else:
                self.hand_stream_toggle_active = True
                self.get_logger().info(
                    "Hand relay armed but global hotkeys unavailable; streaming continuously."
                )

        # Eyes canvas params
        self.eye_canvas_width = 1080
        self.eye_canvas_height = 1900
        self.window_name = "Robot Disco"
        self.eye_radius = 90
        self.eye_white_radius = 220
        self.eye_top_margin = 80
        self.eye_side_margin = 80
        self.eyelid_color = (30, 30, 30)
        self.draw_eyelashes = True
        self.eyelash_color = (50, 50, 55)
        self.eyelash_count = 7
        self.eyelash_length_top = 55
        self.eyelash_length_side = 20
        self.eyelash_thickness = 4

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(self.window_name, self.eye_canvas_width, self.eye_canvas_height)
        set_windows_window_frame_color(self.window_name, (0, 0, 0))

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        raw_video_path = p("save_raw_video")
        if raw_video_path in (None, "", "null"):
            raw_video_path = None
        self.raw_video_writer = (
            cv2.VideoWriter(
                raw_video_path, fourcc, self.capture_fps, (self.camera_width, self.camera_height)
            )
            if raw_video_path
            else None
        )
        if raw_video_path and self.raw_video_writer is not None and not self.raw_video_writer.isOpened():
            self.get_logger().warning(f"Could not open raw video writer at {raw_video_path}")
            self.raw_video_writer = None

        video_path = p("save_video")
        if video_path in (None, "", "null"):
            video_path = None
        self.video_writer = (
            cv2.VideoWriter(
                video_path,
                fourcc,
                self.capture_fps,
                (self.eye_canvas_width, self.eye_canvas_height),
            )
            if video_path
            else None
        )
        if video_path and self.video_writer is not None and not self.video_writer.isOpened():
            self.get_logger().warning(f"Could not open video writer at {video_path}")
            self.video_writer = None

        # Face tracking state
        self.tracked_face = None
        self.last_seen_time = time.time()
        self.face_stick_seconds = 1.0
        self.idle_scan_delay_seconds = 4.0
        self.idle_scan_period_seconds = 8.0
        self.idle_scan_horizontal_fraction = 0.25
        self.idle_scan_vertical_fraction = 0.12
        self.smoothed_center = None

        # Blink state
        self.blink_min_interval_seconds = float(p("blink_min"))
        self.blink_max_interval_seconds = float(p("blink_max"))
        self.blink_duration_seconds = float(p("blink_duration"))
        self.blink_closed_hold = float(p("blink_closed_hold"))
        self.blink_start_time = None
        self.next_blink_time = schedule_blink(
            self.blink_min_interval_seconds, self.blink_max_interval_seconds
        )

        # Mediapipe detectors
        self.pose_detector = mp_pose.Pose(
            model_complexity=0,
            enable_segmentation=False,
            smooth_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.hands_detector = mp_hands.Hands(
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        # Rendering options
        self.hand_in_eyes = bool(p("hand_in_eyes"))
        self.mirror = bool(p("mirror"))
        self.hand_panel_height = int(p("hand_panel_height"))
        self.hand_panel_fraction = min(max(float(p("hand_panel_fraction")), 0.05), 0.5)
        self.hand_panel_scale = float(p("hand_panel_scale"))
        self.gaze_speed = float(p("gaze_speed"))
        self.gaze_idle_speed = float(p("gaze_idle_speed"))

    # ---------- Utility ----------
    def clamp_speed(self, val):
        return min(max(val, 0.01), 1.0)

    # ---------- Image callback ----------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"CvBridge failed: {exc}")
            return

        self.camera_height, self.camera_width = frame.shape[:2]
        if self.raw_video_writer is not None:
            self.raw_video_writer.write(frame)

        current_time = time.time()
        current_mono = time.monotonic()

        # ---- Person detection for eyes ----
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        pose_results = self.pose_detector.process(rgb_frame)
        rgb_frame.flags.writeable = True
        faces = []
        if pose_results and pose_results.pose_landmarks:
            pose_box = pose_box_from_landmarks(
                pose_results.pose_landmarks, self.camera_width, self.camera_height
            )
            if pose_box is not None:
                faces.append(pose_box)

        selected_face = pick_face(faces, self.tracked_face)
        if selected_face is not None:
            self.tracked_face = selected_face
            self.last_seen_time = current_time
        elif current_time - self.last_seen_time > self.face_stick_seconds:
            self.tracked_face = None

        time_since_face = current_time - self.last_seen_time
        if self.tracked_face is not None:
            center = box_top_center(self.tracked_face)
        elif time_since_face > self.idle_scan_delay_seconds:
            scan_elapsed = time_since_face - self.idle_scan_delay_seconds
            sweep_angle = (2 * math.pi / self.idle_scan_period_seconds) * scan_elapsed
            center = (
                self.camera_width / 2.0
                + math.sin(sweep_angle) * self.camera_width * self.idle_scan_horizontal_fraction,
                self.camera_height / 2.0
                + math.cos(sweep_angle * 0.7) * self.camera_height * self.idle_scan_vertical_fraction,
            )
        else:
            center = (self.camera_width / 2.0, self.camera_height / 2.0)

        if self.smoothed_center is None:
            self.smoothed_center = center
        else:
            smoothing_factor_active = self.clamp_speed(self.gaze_speed)
            smoothing_factor_idle = self.clamp_speed(self.gaze_idle_speed)
            current_smoothing = smoothing_factor_active if self.tracked_face is not None else smoothing_factor_idle
            self.smoothed_center = (
                self.smoothed_center[0] + (center[0] - self.smoothed_center[0]) * current_smoothing,
                self.smoothed_center[1] + (center[1] - self.smoothed_center[1]) * current_smoothing,
            )

        if self.blink_start_time is None and current_mono >= self.next_blink_time:
            self.blink_start_time = current_mono

        if self.blink_start_time is not None:
            elapsed = current_mono - self.blink_start_time
            closing = self.blink_duration_seconds / 2.0
            opening = closing
            hold = max(self.blink_closed_hold, 0.0)

            total = closing + hold + opening
            if elapsed >= total:
                self.blink_start_time = None
                self.next_blink_time = schedule_blink(
                    self.blink_min_interval_seconds, self.blink_max_interval_seconds
                )
                blink_amount = 0.0
            else:
                if elapsed <= closing:
                    blink_amount = min(1.0, elapsed / closing)
                elif elapsed <= closing + hold:
                    blink_amount = 1.0
                else:
                    elapsed_open = elapsed - closing - hold
                    blink_amount = max(0.0, 1.0 - (elapsed_open / opening))
        else:
            blink_amount = 0.0

        # Gaze mapping normalized by camera dims
        norm_x = (self.smoothed_center[0] - self.camera_width / 2.0) / (self.camera_width / 2.0)
        norm_y = (self.smoothed_center[1] - self.camera_height / 2.0) / (self.camera_height / 2.0)
        norm_x = max(min(norm_x, 1.0), -1.0)
        norm_y = max(min(norm_y, 1.0), -1.0)

        # Scale by usable pupil travel within the sclera
        max_travel = max(self.eye_white_radius - self.eye_radius, 0)
        gaze_gain_x = 0.7
        gaze_gain_y = 0.6
        offsetX = -norm_x * max_travel * gaze_gain_x  # mirror horizontally
        offsetY = norm_y * max_travel * gaze_gain_y

        # Determine if we need hands processing (for in-eyes overlay or hotkey-triggered streaming)
        if self.hand_stream_enabled and self.global_hotkey_supported:
            # Support multi-key chords like "ctrl+shift+h" while keeping single-key hotkey compatibility
            if "+" in self.hand_stream_hotkey:
                hotkey_pressed = is_hotkey_pressed(self.hand_stream_hotkey)
            else:
                hotkey_pressed = is_global_key_pressed(self.hand_stream_hotkey)
            if hotkey_pressed and not self.hotkey_prev_state:
                self.hand_stream_toggle_active = not self.hand_stream_toggle_active
                state_str = "enabled" if self.hand_stream_toggle_active else "paused"
                self.get_logger().info(f"Hand relay {state_str} via '{self.hand_stream_hotkey}' toggle.")
            self.hotkey_prev_state = hotkey_pressed
        elif self.hand_stream_enabled and not self.global_hotkey_supported:
            self.hand_stream_toggle_active = True

        hand_stream_active = self.hand_stream_enabled and self.hand_stream_toggle_active
        need_hands = self.hand_in_eyes or hand_stream_active
        results = None
        if need_hands:
            rgb_frame.flags.writeable = False
            results = self.hands_detector.process(rgb_frame)
            rgb_frame.flags.writeable = True

        if hand_stream_active:
            hand_payload = None
            if results and results.multi_hand_landmarks:
                joint_offsets = generate_joint_offsets(results.multi_hand_landmarks[0])
                hand_payload = format_joint_command(joint_offsets, self.hand_target_speed)
                self.hand_last_detection_time = current_mono
                self.hand_open_sent = False
            elif (current_mono - self.hand_last_detection_time) > self.hand_idle_timeout and not self.hand_open_sent:
                hand_payload = format_joint_command(JOINT_OPEN_OFFSETS, self.hand_target_speed)
                self.hand_open_sent = True

            if hand_payload is not None and (current_mono - self.last_hand_send) >= self.hand_send_interval:
                if self.hand_socket is None:
                    try:
                        self.hand_socket = socket.create_connection(
                            (self.get_parameter("hand_host").value, int(self.get_parameter("hand_port").value)),
                            timeout=0.5,
                        )
                        self.hand_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    except OSError:
                        self.hand_socket = None
                if self.hand_socket is not None:
                    try:
                        self.hand_socket.sendall(hand_payload.encode("ascii"))
                        self.last_hand_send = current_mono
                    except OSError:
                        self.hand_socket.close()
                        self.hand_socket = None

        # Prepare eyes canvas
        eye_canvas = np.zeros((self.eye_canvas_height, self.eye_canvas_width, 3), dtype=np.uint8)
        eye_vertical_center = self.eye_top_margin + self.eye_white_radius
        left_eye_center = (
            self.eye_side_margin + self.eye_white_radius,
            eye_vertical_center,
        )
        right_eye_center = (
            self.eye_canvas_width - self.eye_side_margin - self.eye_white_radius,
            eye_vertical_center,
        )
        right_center_with_offset = (
            int(right_eye_center[0] + offsetX),
            int(right_eye_center[1] + offsetY),
        )
        left_center_with_offset = (
            int(left_eye_center[0] + offsetX),
            int(left_eye_center[1] + offsetY),
        )
        draw_eye(
            eye_canvas,
            right_eye_center,
            self.eye_white_radius,
            self.eye_radius,
            (
                right_center_with_offset[0] - right_eye_center[0],
                right_center_with_offset[1] - right_eye_center[1],
            ),
            blink_amount,
            self.eyelid_color,
            self.draw_eyelashes,
            self.eyelash_color,
            self.eyelash_count,
            self.eyelash_length_top,
            self.eyelash_length_side,
            self.eyelash_thickness,
        )
        draw_eye(
            eye_canvas,
            left_eye_center,
            self.eye_white_radius,
            self.eye_radius,
            (
                left_center_with_offset[0] - left_eye_center[0],
                left_center_with_offset[1] - left_eye_center[1],
            ),
            blink_amount,
            self.eyelid_color,
            self.draw_eyelashes,
            self.eyelash_color,
            self.eyelash_count,
            self.eyelash_length_top,
            self.eyelash_length_side,
            self.eyelash_thickness,
        )

        if self.hand_in_eyes and results is not None and results.multi_hand_landmarks:
            half_height = max(int(self.eye_canvas_height * 0.5), 1)
            desired_from_fraction = int(self.eye_canvas_height * self.hand_panel_fraction)
            panel_h = min(half_height, max(desired_from_fraction, self.hand_panel_height, 1))
            y0 = self.eye_canvas_height - panel_h
            x0 = 0
            hand_panel = eye_canvas[y0 : self.eye_canvas_height, x0 : self.eye_canvas_width]
            panel_img = np.zeros_like(hand_panel)
            mp_drawing.draw_landmarks(
                panel_img,
                results.multi_hand_landmarks[0],
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style(),
            )
            if self.mirror:
                panel_img = cv2.flip(panel_img, 1)
            panel_scale = max(self.hand_panel_scale, 1.0)
            if panel_scale != 1.0:
                scaled_img = cv2.resize(
                    panel_img,
                    None,
                    fx=panel_scale,
                    fy=panel_scale,
                    interpolation=cv2.INTER_LINEAR,
                )
                target_h, target_w = hand_panel.shape[:2]
                if scaled_img.shape[0] >= target_h and scaled_img.shape[1] >= target_w:
                    start_y = (scaled_img.shape[0] - target_h) // 2
                    start_x = (scaled_img.shape[1] - target_w) // 2
                    panel_img = scaled_img[start_y : start_y + target_h, start_x : start_x + target_w]
                else:
                    panel_img = cv2.resize(scaled_img, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
            hand_panel[:] = panel_img

        if self.video_writer is not None:
            self.video_writer.write(eye_canvas)

        cv2.imshow(self.window_name, eye_canvas)
        key_code = cv2.waitKey(1) & 0xFF
        if key_code == ord("q"):
            self.get_logger().info("Quit requested via keypress; shutting down.")
            rclpy.shutdown()

    # ---------- Cleanup ----------
    def destroy_node(self):
        if self.hand_socket is not None:
            self.hand_socket.close()
        if self.raw_video_writer is not None:
            self.raw_video_writer.release()
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        self.pose_detector.close()
        self.hands_detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceHandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
