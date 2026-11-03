# robot_disco (the mobile base)

## robot_disco

**Path**: `src/robot_disco/` (git submodule, branch `humble`)
**Repo**: github.com/r1b4z01d/Robot_Disco

The mobile base: robot description (URDF/xacro), bringup, localization, navigation, the joystick, the lidars and the AprilTag relocalizer. On the robot every launch file below is a supervisor unit (`robot_disco*`, see [Supervisor Services](Supervisor-Services#supervisor-services)).

**Launch files** (`launch/`):

```bash
ros2 launch robot_disco launch_robot.launch.py            # bringup: rsp + ros2_control (RoboClaw diff drive), twist_mux, joystick, D435, both RPLiDARs + filters, Velodyne, UR joint-state publisher
ros2 launch robot_disco localization_launch.py            # AMCL on maps/zone_one.yaml (map:=… to override)
ros2 launch robot_disco online_async_launch.py            # slam_toolbox instead of AMCL (mapping)
ros2 launch robot_disco navigation_launch.py              # Nav2 (config/nav2_params.yaml, behavior_trees/)
ros2 launch robot_disco apriltag_relocalization_launch.py # detector + AMCL corrector (see below)
ros2 launch robot_disco ur5e.launch.py                    # the ROS UR driver, namespaced /ur5e (unit robot_disco_ur5e_driver; not used during VLA runs, which drive the arm over RTDE)
ros2 launch robot_disco foxglove_bridge_launch.xml        # Foxglove WebSocket bridge, whitelisted topics
ros2 launch robot_disco joystick.launch.py                # joy + teleop_twist_joy alone (launch_robot includes it)
ros2 launch robot_disco rplidar_s1_launch.py | rplidar_s2e_launch.py | rplidar_s1_filter_launch.py | rplidar_s2e_filter_launch.py | velodyne.launch.py | realsense_launch.py | rsp.launch.py
```

**Configs** (`config/`): `nav2_params.yaml` (both costmaps take the two filtered RPLiDAR scans; the local one also the Velodyne as a close guard), `collision_monitor_params.yaml` (stop / slowdown polygons), `twist_mux.yaml`, `my_controllers.yaml` (ros2_control diff drive), `mapper_params_online_async.yaml`, `rplidar_s1_filter.yaml` / `rplidar_s2e_filter.yaml` (the box filters whose output the costmaps and the browser pages use), `velodyne.yaml`, `joystick.yaml`, `apriltag_landmarks.yaml` (detector: tag ids and sizes) and `apriltag_landmark_poses.yaml` (taught map poses; empty until taught). `behavior_trees/` holds the Nav2 trees.

**Joystick** (`config/joystick.yaml`): an 8BitDo Ultimate 2C on its 2.4 GHz receiver, matched by `device_name` ("8BitDo Ultimate 2C Wireless Controller" under the rebuilt xpad from `provision/xpad/`; stock Tegra xpad never makes the receiver stream). Hold **RB (button 5)** to drive, **LB (4)** for turbo (1.0 m/s, 2.0 rad/s). The same two buttons are `path.py`'s deadman: pressing either while a route runs cancels it.

**Scripts** (`scripts/`): `apriltag_relocalizer.py` (below); `ur_joint_state_publisher.py` publishes the UR5e's joints as `/joint_states` from a read-only RTDE connection so the arm moves in the robot description during VLA runs when the ROS UR driver is not running, with a fallback joint state (`fallback: home|last|zeros|none`, `home_pose_file` `~/rd_ws/VLA/outputs/home_pose.json`) while the arm is off; `bake_amazing_hand.py` bakes the hand meshes for the description. The copies of the face node's files under `scripts/` are stale leftovers from before `robot_face` became its own package.

**Maps** (`maps/`): `zone_one` is the active map (`.pgm`/`.yaml` for AMCL, `.posegraph`/`.data` for slam_toolbox, `.xcf` the GIMP source); `zone1`, `zone_1` and `my_map` are older. See [Maps](Hardware-Bringup#maps).
