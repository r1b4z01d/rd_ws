# Hardware bringup and utilities



## UR5e Arm

```bash
# Start UR5e hardware interface
ros2 launch ur_robot_driver ur_control.launch.py \
  launch_rviz:=false \
  ur_type:=ur5e \
  robot_ip:=192.168.11.21 \
  use_tool_communication:=false

# Start MoveIt2 with RViz
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true \
  robot_ip:=192.168.11.21

# Load and run a UR program
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "filename: remote.urp"
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "filename: dball.urp"
ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}
```

On the robot the ROS driver is the manual-start unit `robot_disco_ur5e_driver` (`ur5e.launch.py`, namespaced `/ur5e`). The VLA teleop, eval and the face's wave do **not** use it: they drive the arm directly over RTDE (ur_rtde), and `scripts/ur_joint_state_publisher.py` in `robot_disco` keeps `/joint_states` flowing from a read-only RTDE connection meanwhile. `Scripts/ur_power.py` powers the arm up or down and clears protective stops through the Dashboard Server (TCP 29999) from the command line; the same logic runs inside the VLA tools and the wave.

## Static Transforms

```bash
# Camera mounted to flange
ros2 run tf2_ros static_transform_publisher \
  0.00175 0.002985 0.0068 0.0 0.0 3.14 flange camera_link

# Grasp location relative to cube
ros2 run tf2_ros static_transform_publisher \
  0.20 0.00 0.0 3.15 1.6 0.0 purple_cube grasp_loc
```

The `flange → camera_link` transform is for the arm-mounted camera demo only. `launch_robot.launch.py` already publishes `base_link → camera_link` from the URDF (`description/realsense_d435.xacro`); running both gives `camera_link` two parents and breaks TF for everything that uses the camera.

## RealSense Camera

`launch_robot.launch.py` already includes the D435 (`launch/realsense_launch.py`, point cloud enabled). Standalone:

```bash
# D435 with point cloud enabled
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  device_type:=d435
```

Topics are under `/camera/camera/...` (namespace and name are both `camera`), e.g. `/camera/camera/color/image_raw` and `/camera/camera/color/camera_info`.

## Navigation Stack

```bash
ros2 launch robot_disco launch_robot.launch.py
ros2 launch robot_disco localization_launch.py          # or online_async_launch.py to map
ros2 launch robot_disco navigation_launch.py
ros2 launch nav2_collision_monitor collision_monitor_node.launch.py \
  params_file:=$HOME/rd_ws/src/robot_disco/config/collision_monitor_params.yaml use_sim_time:=false
```

On the robot these are the `robot_disco*` supervisor services (see [Supervisor Services](Supervisor-Services#supervisor-services)). The collision monitor is in the drive path, so without it the robot does not move.

### Maps

Maps live in `src/robot_disco/maps/`. The active map is **`zone_one`**: `localization_launch.py` loads `maps/zone_one.yaml` (override with `map:=/path/to/other.yaml`), and slam_toolbox continues from `maps/zone_one` (`map_file_name` in `config/mapper_params_online_async.yaml`).

| Files | Used by |
|---|---|
| `<name>.pgm` + `<name>.yaml` | `map_server` / AMCL (occupancy grid) |
| `<name>.posegraph` + `<name>.data` | slam_toolbox, to continue or extend an existing map |

**Recording or updating a map** (on the robot, with `robot_disco_localization` stopped):

```bash
sudo supervisorctl start robot_disco robot_disco_slam
# drive the robot around; watch /map on the 2D map page (http://192.168.11.80:8092/?view=map)

# 1. save the occupancy grid that AMCL uses (overwrites zone_one.pgm/.yaml)
ros2 run nav2_map_server map_saver_cli -f ~/rd_ws/src/robot_disco/maps/zone_one

# 2. save the pose graph so slam_toolbox can continue this map later
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/rd/rd_ws/src/robot_disco/maps/zone_one'}"
```

Notes:
- `maps/` is symlink-installed, so no rebuild is needed; `map_file_name` points straight into `src/`.
- With a `.posegraph` present and `map_start_at_dock: true`, start the robot at the dock before launching SLAM so the map frame stays where it was.
- Starting SLAM without a pose graph makes a **fresh map with a new map-frame origin**. Every pose stored in the map frame — `Scripts/waypoints.yaml` (waypoints and `initial_pose`) and taught AprilTag landmarks — must then be re-recorded.



## AprilTag detection with RealSense

```bash
# camera only if launch_robot.launch.py is NOT running (it already starts the D435)
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true device_type:=d435

# the stock launch defaults to /camera/color/...; the robot's topics are /camera/camera/color/...
ros2 launch apriltag_ros tag_realsense.launch.py camera_name:=/camera/camera/color
```

Detections are published on `/apriltag/apriltag_detections` and as TF frames (`<family>:<id>`, or the names in `tag_frames`). For map relocalization use `apriltag_relocalization_launch.py` instead (see [AprilTag map relocalization](Third-Party-Packages#apriltag-map-relocalization)).

## Publish a battery state for testing

```bash
ROS_DOMAIN_ID=0 ros2 topic pub /battery sensor_msgs/BatteryState \
  "{percentage: 0.82, power_supply_status: 0}"
```
