robot_face
==========

ROS 2 Python package for Robot Disco's face: a GPU-rendered pair of eyes and a mouth on the robot's portrait display, driven by the camera. The one node, `combined_face_hand_node`, does all of it:

- **Eyes and mouth** (`gl_eye_renderer.py`, OpenGL): moods (`~/set_mood/*` services), blinking, gaze that follows the person in view and wanders when nobody is, a mouth that rests in a smile, traces the robot's speech as a waveform and pulls idle gestures (`mouth_gestures.py`). Publish loudness (0..1) on `~/mouth_level` or raw samples on `~/mouth_audio` to drive it from your own audio. `fullscreen` and `render_fps` parameters; a click or F toggles fullscreen.
- **Who is there** (`face_id.py`): the robot learns faces on the fly. Default back-end on the robot is ArcFace (SCRFD + ArcFace through `faceid_worker.py` in its own venv, `provision/setup_faceid.sh`; GPU on the Orin), fallback OpenCV YuNet + SFace (models in `models/`). Strangers are enrolled as `person_N`; name them with `ros2 topic pub --once /combined_face_hand/set_person_name std_msgs/msg/String "{data: 'Name'}"` (or `person_2=Name`), list with the `~/list_people` service, forget with `~/forget_person`, watch `~/person`.
- **Speech**: greetings by name ("hello Bryan", a wink instead within `greeting_cooldown`), anything published on `~/say` (`std_msgs/String`), through Kokoro-82M (`kokoro_worker.py` in its own venv, `provision/setup_kokoro.sh`), else Coqui, else espeak; `~/speaking` is true while the robot talks so the voice assistant mutes its microphone. The USB speaker (C-Media adapter) is kept the default sink by `audio_sink.py`.
- **The wave** (`ur_motion.py`, `ur_dashboard.py`, `hand_wiggle.py`): recognising a known person plays the recorded wave on the UR5e over ur_rtde (powering the arm up first if needed) while the AmazingHand's fingers ripple over UDP; `~/wave` and `~/wink` services play them on demand. `scripts/ur_record_motion.py` records the wave and captures the home pose; `scripts/ur_path_eval.py` checks the staged approach orders.
- **Folks page** (`people_web.py`): `http://<robot>:8096/`, the people the robot knows with pictures, editable names, last hello / reset, snap, forget; the VLA manager embeds it as its Folks tab.
- **Legacy hand relay** (`hand_processing.py`): MediaPipe hand joints to the AmazingHand over TCP (`relay_hand`, `hand_relay_*`); off on the robot, which runs `face_only: true`.

Every setting is a ROS parameter with its defaults in `config/combined_face_hand_defaults.yaml`; the launch file and the supervisor unit load that file. The full parameter tables, the wave's details and the TTS chain are in the workspace wiki ([Robot-Face](https://github.com/r1b4z01d/rd_ws/wiki/Robot-Face)).

## Run

```
colcon build --packages-select robot_face --symlink-install
source install/setup.bash
ros2 launch robot_face combined_face_hand.launch.py image_topic:=/camera/camera/color/image_raw   # node name: combined_face_hand
```

Overrides: `--ros-args -p param:=value`, or your own YAML as `param_file:=`. On the robot the face is the supervisor unit `robot_face` (`sudo supervisorctl start robot_face`; the unit carries the X display environment the GL window needs). `ros2 run robot_face combined_face_hand_node` works too, but then the node is called `robot_face` and its topics move to `/robot_face/...`, where the voice assistant does not look. Deploy with `provision/deploy_to_rd.sh`.

`python3 test_eye_renderer.py` shows the renderer without ROS (SPACE presets, WASD gaze, B blink, 0-4 moods, M/N talking, G gestures).
