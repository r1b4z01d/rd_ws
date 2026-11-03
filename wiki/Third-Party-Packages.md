# apriltag, roboclaw, lidar drivers, VLA

## apriltag / apriltag_ros

**Paths**: `src/apriltag/`, `src/apriltag_ros/`, `src/AprilTag_Detection/`
**Repos**: github.com/AprilRobotics/apriltag, github.com/Adlink-ROS/apriltag_ros

Fiducial marker detection for robot localization and object pose estimation. Used in conjunction with the RealSense D435 camera.

```bash
# NOTE: the stock launch defaults to /camera/color/...; the robot's topics are /camera/camera/color/...
ros2 launch apriltag_ros tag_realsense.launch.py camera_name:=/camera/camera/color
```

**Jetson build note.** The checkout carries `provision/patches/apriltag_ros-single-opencv.patch` (applied by `provision.sh workspace`, or by hand with `git -C src/apriltag_ros apply ../../provision/patches/apriltag_ros-single-opencv.patch`): it removes the node's `cv_bridge` / `image_geometry` use. On JetPack those apt libraries link Ubuntu's OpenCV 4.5 while the node links NVIDIA's 4.8, and the unpatched component container aborts on its first camera frame with `OpenCV(4.8.0) ... Assertion failed s >= 0 in function 'setSize'`. The `tag_sizes`/`tag_frames` parameters and the published topics are unchanged.

### AprilTag map relocalization

`apriltag_relocalization_launch.py` runs the detector plus `scripts/apriltag_relocalizer.py`, which corrects AMCL from landmark tags with known map-frame poses. Needs `launch_robot.launch.py` (camera + TF) and `localization_launch.py` (AMCL) running.

```
detections ──► for each taught tag within max_range:
               T_base_tag = T_base_camera (static TF) · T_camera_tag (the detection's pose)
               T_map_base = T_map_tag · T_base_tag⁻¹
            ──► N agreeing estimates in the last second
            ──► differs from AMCL by > 0.30 m or 12° ──► publish /initialpose (rate-limited)
```

**Tags**: 36h11, ~15–20 cm, mounted ~1.2–1.8 m high (the D435 is pitched 46° up in the URDF). IDs 0–9 are pre-listed in `config/apriltag_landmarks.yaml`; set each tag's real black-border edge length in `tag_sizes`. A print-ready set is in `documents/apriltag_landmarks_36h11_150mm.pdf` (one tag per Letter page, 150 mm black square, 100 mm scale bar; print at 100 %, not fit-to-page). Regenerate for other IDs/sizes with `python3 Scripts/make_apriltag_pdf.py --ids 10 11 --size-mm 200` (reads the codes from `src/apriltag/tag36h11.c`, so that submodule must be initialized).

**Teaching tag poses** (no measuring): with AMCL well localized and the tag in view,
```bash
ros2 service call /apriltag_relocalizer/teach_landmarks std_srvs/srv/Trigger
```
writes `map → tag` for every visible tag to `config/apriltag_landmark_poses.yaml` (through the install-space symlink, so the copy in `src/` is the one that changes; copy it to the workstation to keep it). Re-teach after re-mapping or moving a tag.

The node uses the pose inside each detection, not the `tag_N` TF frames the detector also broadcasts: those reach `/tf` just after the detection message, and a lookup at the detection's stamp failed with "extrapolation into the future" on every frame (RD, 2026-11-01). The frames are still published for Foxglove and the sanity check below.

**Other controls**: `ros2 service call /apriltag_relocalizer/relocalize_now std_srvs/srv/Trigger` forces a correction; `auto_correct:=false` makes the node passive (service only). `/apriltag_relocalizer/estimated_pose` carries the tag-derived pose. The 3D page's **AprilTags** layer draws the taught poses (amber) and the tags currently in view (green) from the same file (`route_editor.py --apriltags-file`).

Launch arguments: `camera` (`/camera/camera/color`), `detector_config` (`config/apriltag_landmarks.yaml`), `landmarks_file` (`config/apriltag_landmark_poses.yaml`), `auto_correct`. Node parameters: `detections_topic`, `landmarks_file`, `map_frame`, `base_frame`, `max_range` (3.0 m camera → tag), `min_detections` (3 agreeing detections), `window_sec` (1.0), `spread_xy` (0.15 m) / `spread_yaw` (0.10 rad) allowed disagreement inside the window, `position_threshold` (0.30 m) / `yaw_threshold` (0.20 rad) deviation from AMCL before correcting, `min_interval` (5 s between automatic corrections), `initial_pose_std_xy` / `initial_pose_std_yaw` (0.10, the covariance handed to AMCL), `tilt_warn` (0.25 rad).

**Sanity check** before trusting it: with a tag in view, `ros2 run tf2_ros tf2_echo base_footprint tag_0` should report a position that matches where the tag physically is relative to the robot. If it doesn't, the camera mount in `description/realsense_d435.xacro` is wrong; the node also warns when the estimated base roll/pitch exceeds `tilt_warn`.

## roboclaw (Motor Controller)

**Paths**: `src/roboclaw_hardware_interface/`, `src/roboclaw_serial/`, `src/roboclaw_ros/`
**Repos**: dumbotics/roboclaw_hardware_interface, dumbotics/roboclaw_serial, norlab-ulaval/roboclaw_ros

RoboClaw motor controller integration for the Robot Disco differential-drive base.

- `roboclaw_serial`: Low-level serial communication library
- `roboclaw_ros`: ROS2 node exposing velocity and odometry
- `roboclaw_hardware_interface`: `ros2_control` hardware interface plugin for use with the ROS2 control framework

## velodyne / rplidar_ros

**Paths**: `src/velodyne/`, `src/rplidar_ros/`
**Repos**: github.com/ros-drivers/velodyne, standard RPLiDAR ROS2 driver

LiDAR drivers for the dual-lidar navigation setup.

- `velodyne`: Full 3D point cloud driver for Velodyne sensors (VLP-16, HDL-32E, etc.)
- `rplidar_ros`: 2D scan driver for the two RPLiDARs: an **S1** (front) and an **S2E** (rear), launched by `rplidar_s1_launch.py` / `rplidar_s2e_launch.py` with box filters (`rplidar_*_filter_launch.py`) that cut the robot's own body out of each scan; the filtered scans (`/rp_s1_lidar/scan_filtered`, `/rp_s2e_lidar/scan_filtered`) are what the costmaps and the browser pages use

The three lidars run together: the two RPLiDARs feed both costmaps, the Velodyne is the local costmap's close guard and the 3D view's point cloud.

## VLA (submodule)

**Path**: `VLA/` (git submodule, github.com/r1b4z01d/VLA, since 2026-11-02; on the robot `~/rd_ws/VLA`)

The arm's learning stack, documented in `VLA/README.md` and `VLA/wiki/`: LeRobot teleop and recording with the UR5e + AmazingHand (`ur5e_lerobot/`), the camera service that owns the three USB cameras (`vla_cameras`, multicast + MJPEG; `vla_camera_ros_bridge` republishes them as `/vla/cameras/<name>/image/compressed` for the browser pages), the SmolVLA inference server that runs on the robot's own GPU (`vla_infer_server`, loopback :8777), the policy eval (`scripts/eval_hw.py`, `vla_action.py`, the units `vla_teleop` / `vla_eval`), training on the GPU box, and the web manager (`webui/`, :8081 on the GPU box and on the robot) whose Map / 3D / Routes / Landmarks / Folks / Services tabs embed or drive the pages in this workspace. rd_ws reaches it in three places: `Scripts/path.py` and `robot_voice` run `VLA/run.sh scripts/vla_action.py` for `vla:` actions and landmark tasks, the face's wave reads `VLA/outputs/home_pose.json`, and `provision/` builds its venv and installs its units (`VLA/services/rd/`, `VLA/scripts/install_services.sh rd`). A `COLCON_IGNORE` in it keeps `colcon build` out of its 8 GB of outputs and its Jazzy-only `ros2_ws`.
