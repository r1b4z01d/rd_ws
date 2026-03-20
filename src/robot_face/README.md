robot_face
==========

ROS 2 Python package that wraps the MediaPipe eyes + hand demo into a node which subscribes to a color image topic and renders animated eyes (optionally streaming hand joint commands).

## Run
```
colcon build --packages-select robot_face
source install/setup.bash
ros2 run robot_face combined_face_hand_node --ros-args -p image_topic:=/camera/camera/color/image_raw -p hand_in_eyes:=true
```

All configuration is exposed as ROS parameters (see `config/combined_face_hand_defaults.yaml`). Use `--ros-args -p param:=value` to override at runtime, or supply a YAML via the launch file:
```
ros2 launch robot_face combined_face_hand.launch.py \
  param_file:=/path/to/combined_face_hand_defaults.yaml \
  image_topic:=/camera/camera/color/image_raw \
  hand_in_eyes:=true
```
