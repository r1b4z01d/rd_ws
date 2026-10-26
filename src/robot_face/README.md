robot_face
==========

ROS 2 Python package that wraps the MediaPipe eyes + hand demo into a node which subscribes to a color image topic and renders animated eyes and a mouth waveform (optionally streaming hand joint commands).

The mouth is a glowing waveform under the eyes. Publish speech loudness (0..1) on `~/mouth_level` (`std_msgs/Float32`) or raw audio chunks (-1..1) on `~/mouth_audio` (`std_msgs/Float32MultiArray`) and it animates; with no input it rests as a flat line. See the `mouth_*` parameters in `config/combined_face_hand_defaults.yaml`.

The node also learns to tell people apart (`face_id.py`, OpenCV YuNet + SFace, models in `models/`). Strangers are enrolled automatically as `person_N`; name them with `ros2 topic pub --once ~/set_person_name std_msgs/msg/String "{data: 'Name'}"`, list them with the `~/list_people` service, and watch `~/person` for live recognitions. Named people are greeted by name.

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
