# Robot Disco V3 - ROS2 Workspace

![Robot Disco System Architecture](documents/Robot%20Disco%20System%20Architecture.jpg)

A ROS2 workspace for a mobile robot platform with a UR5e arm, animated robot face, AprilTag detection, LiDAR-based navigation, AmazingHand end-effector, WLED LED matrix panels, RealSense T265 tracking camera, and 2.1 Bluetooth audio. The workspace integrates real-time vision, GPU-accelerated face animation (eyes plus a speech-driven mouth), on-robot face recognition that learns who it meets, and multi-robot hardware drivers.

---

## Table of Contents

- [Workspace Structure](#workspace-structure)
- [Packages](#packages)
  - [robot_face](#robot_face)
  - [AmazingHand](#amazinghand)
  - [robot_disco](#robot_disco)
  - [apriltag / apriltag_ros](#apriltag--apriltag_ros)
  - [roboclaw](#roboclaw-motor-controller)
  - [velodyne / rplidar_ros](#velodyne--rplidar_ros)
- [Hardware Peripherals](#hardware-peripherals)
  - [WLED LED Matrix Panels](#wled-led-matrix-panels)
  - [RealSense T265 Tracking Camera](#realsense-t265-tracking-camera)
  - [2.1 Bluetooth Audio](#21-bluetooth-audio)
- [Foxglove Studio](#foxglove-studio)
- [Supervisor Services](#supervisor-services)
- [Provisioning a New Robot PC](#provisioning-a-new-robot-pc)
- [Quick Start](#quick-start)
- [Hardware Bringup](#hardware-bringup)
  - [UR5e Arm](#ur5e-arm)
  - [RealSense Camera](#realsense-camera)
  - [Navigation Stack](#navigation-stack)
  - [Maps](#maps)
- [Utilities](#utilities)
  - [Waypoint runner (`Scripts/path.py`)](#waypoint-runner-scriptspathpy)

---

## Workspace Structure

```
rd_ws/
├── src/
│   ├── robot_face/             # Animated face (eyes + mouth), face recognition, TTS greeting, hand mirroring (active)
│   ├── robot_disco/            # Mobile robot base platform (submodule)
│   ├── apriltag/               # AprilTag C library (submodule)
│   ├── apriltag_ros/           # ROS2 AprilTag wrapper (submodule)
│   ├── AprilTag_Detection/     # AprilTag demo package (submodule)
│   ├── roboclaw_hardware_interface/  # RoboClaw ros2_control HW interface (submodule)
│   ├── roboclaw_serial/        # RoboClaw serial library (submodule)
│   ├── roboclaw_ros/           # RoboClaw ROS2 node (submodule)
│   ├── norlab_custom_interfaces/    # Custom ROS message types (submodule)
│   ├── velodyne/               # Velodyne 3D LiDAR driver (submodule)
│   └── rplidar_ros/            # RPLiDAR 2D driver (submodule)
├── AmazingHand-main/           # Custom 8-DOF robotic hand (hardware + software)
├── foxglove_studio/            # Open-source Foxglove Studio (customized)
├── services/                   # Supervisord service configs
├── Scripts/                    # Waypoint runner (path.py + waypoints.yaml), UR5e demos, install_services.sh
├── rviz/                       # view_robot.rviz
├── documents/                  # Datasheets (RoboClaw, RPLiDAR, TeraRanger), architecture diagram, notes.txt
└── URDF.{step,stl,obj,f3z}     # Robot CAD exports
```

Initialize all submodules with:

```bash
git submodule update --init --recursive
```

---

## Packages

---

### robot_face

**Path**: `src/robot_face/`
**Maintainer**: Bryan Ribas (bryanribas@gmail.com)
**License**: CC BY-NC-SA

Provides a real-time animated robot face driven by the camera. The face is drawn in a GPU-accelerated OpenGL window: two eyes with moods, blinking and gaze tracking, and a mouth that rests in a slight smile, traces the robot's speech as a waveform, and pulls random expressions while nobody is around. The node recognises people it has met (OpenCV face embeddings, learned on the fly and persisted to disk), greets them by name through TTS, and can optionally stream detected hand joint angles to the AmazingHand over TCP so the robot mirrors the operator's hand live.

#### Architecture

```
Camera Image Topic
       │
       ▼
 combined_face_hand_node
 ├── MediaPipe Pose detector   → person box → gaze target, "someone is here"
 ├── FaceIdentifier (face_id)  → YuNet face detect → SFace embedding → people gallery (JSON)
 ├── TTS greeting              → "hello" / "hello <name>" → WAV → aplay → mouth waveform
 ├── Mouth state               → smile (wider when seeing someone), speech level, idle gestures
 ├── MediaPipe Hands detector  → finger curl angles → TCP hand commands
 ├── GlEyeRenderer (OpenGL)    → eyes + mouth window (GPU)
 └── ROS interfaces            → mood services, mouth topics, people naming / listing
```

#### Source Files

| File | Purpose |
|---|---|
| `combined_face_hand_node.py` | Main ROS2 node — orchestrates all subsystems |
| `gl_eye_renderer.py` | GPU-accelerated eye renderer (OpenGL 3.3 + GLSL) |
| `eye_renderer.py` | CPU-based eye renderer (OpenCV fallback) |
| `audio_utils.py` | WAV loading, chunking and loudness helpers for the mouth waveform |
| `face_id.py` | Face identification: YuNet detection, SFace embeddings, persistent people gallery |
| `mouth_gestures.py` | Idle mouth gesture library: timings, shapes and morph envelopes (no ROS dependency) |
| `face_utils.py` | Pose landmark → bounding box, face tracking/hysteresis |
| `hand_processing.py` | Hand landmarks → servo joint angles → TCP command string |
| `window_utils.py` | Cross-platform global hotkey detection (Win/Linux/macOS) |
| `models/` | YuNet + SFace ONNX models for face ID (installed to `share/robot_face/models`) |
| `test_eye_renderer.py` | Standalone visual test of the renderer, no ROS required |

#### Node: `combined_face_hand_node`

**Entry point**: `robot_face.combined_face_hand_node:main`

**Subscriptions**:

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/camera/camera/color/image_raw` (configurable) | `sensor_msgs/Image` | BEST_EFFORT, depth=1 | Live camera frames for detection |
| `~/mouth_level` (configurable) | `std_msgs/Float32` | BEST_EFFORT, depth=1 | Speech loudness 0..1; drives the mouth waveform amplitude |
| `~/mouth_audio` (configurable) | `std_msgs/Float32MultiArray` | BEST_EFFORT, depth=1 | Raw audio chunk, samples in -1..1; drawn as the mouth waveform |
| `~/set_person_name` | `std_msgs/String` | default | Name the person in view (`"Bryan"`), or by id (`"person_3=Bryan"`) |
| `~/forget_person` | `std_msgs/String` | default | Forget a person by id or name, or `"all"` |

**Publications**:

| Topic | Type | Description |
|---|---|---|
| `~/person` | `std_msgs/String` | JSON `{id, name, score, new, box}` each time face ID runs on the tracked person |

**Services Provided**:

| Service | Type | Description |
|---|---|---|
| `~/set_mood/neutral` | `std_srvs/Trigger` | Neutral eye expression |
| `~/set_mood/tired` | `std_srvs/Trigger` | Drooping tired eyes |
| `~/set_mood/angry` | `std_srvs/Trigger` | Angled angry brows |
| `~/set_mood/happy` | `std_srvs/Trigger` | Squinting happy eyes |
| `~/set_mood/suspicious` | `std_srvs/Trigger` | One-eyebrow-up suspicious look |
| `~/list_people` | `std_srvs/Trigger` | List learned people (id, name, views, times seen) |

**ROS Parameters**:

*Vision & Display*

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/camera/camera/color/image_raw` | Camera image subscription topic |
| `width` | `1280` | Camera frame width (px) |
| `height` | `720` | Camera frame height (px) |
| `fps` | `30.0` | Expected capture framerate |
| `mirror` | `false` | Mirror hand overlay for selfie-mode |

*Eye Rendering*

| Parameter | Default | Description |
|---|---|---|
| `hand_in_eyes` | `false` | Draw hand skeleton inside eye window |
| `hand_panel_height` | `400` | Hand panel pixel height |
| `hand_panel_fraction` | `0.5` | Fraction of screen height for hand panel (0.05–0.5) |
| `hand_panel_scale` | `1.0` | Scale factor for hand panel |

*Blink Behavior*

| Parameter | Default | Description |
|---|---|---|
| `blink_min` | `2.5` | Minimum seconds between blinks |
| `blink_max` | `15.5` | Maximum seconds between blinks |
| `blink_duration` | `0.18` | Total blink cycle duration (s) |
| `blink_closed_hold` | `0.04` | Time held fully closed during blink (s) |

*Gaze & Tracking*

| Parameter | Default | Description |
|---|---|---|
| `gaze_speed` | `0.3` | Gaze smoothing when face is detected (0.01–1.0) |
| `gaze_idle_speed` | `0.01` | Gaze smoothing when no face (idle scan) |

*Mouth Waveform*

| Parameter | Default | Description |
|---|---|---|
| `mouth_enabled` | `true` | Draw the mouth waveform below the eyes |
| `mouth_level_topic` | `~/mouth_level` | `std_msgs/Float32` loudness input (0..1) |
| `mouth_audio_topic` | `~/mouth_audio` | `std_msgs/Float32MultiArray` raw sample input (-1..1) |
| `mouth_gap` | `550.0` | Eye bottom edge → waveform centre line (px) |
| `mouth_amplitude` | `70.0` | Peak displacement at full loudness (px) |
| `mouth_width` | `0.0` | Waveform width in px (0 = span between eye centres) |
| `mouth_gain` | `1.0` | Multiplier applied to incoming level / samples |
| `mouth_attack` | `0.6` | Per-frame smoothing when getting louder (0.01–1.0) |
| `mouth_release` | `0.12` | Per-frame smoothing when getting quieter (0.01–1.0) |
| `mouth_timeout` | `0.4` | Seconds without input before the mouth returns to idle |
| `mouth_smile` | `25.0` | Resting upward curl of the mouth ends (px); negative frowns |
| `mouth_smile_happy` | `70.0` | Curl while a person is in view (px) |
| `mouth_gestures` | `true` | Random idle gestures (see list below) when nobody is around |
| `mouth_gesture_min_interval` | `4.0` | Minimum seconds between idle gestures |
| `mouth_gesture_max_interval` | `12.0` | Maximum seconds between idle gestures |

The mouth is a glowing line centred under the eyes whose ends curl up into a slight smile at rest, and a wider one while someone is in view. With no audio it holds perfectly still; when nobody is around it occasionally performs a short random gesture, which stops as soon as a person appears. Gestures live in `mouth_gestures.py`; shape gestures morph the resting line into a polyline and back:

| Gesture | Look |
|---|---|
| `hum`, `wiggle` | brief waveform motion |
| `grin`, `pout`, `smirk` | ends curl up / down, or tilt to one side |
| `skeptical` | straight line slanted down to one side |
| `shocked` | small "O" |
| `puckered` | sideways "3" / "w" |
| `anxious` | sharp zig-zag with a nervous shake |
| `slack_jawed` | deep drooping "U" |
| `playful` | flat line with a small tongue loop that wags |
| `grimace` | upside-down "V" |
| `shouting` | large "V" |
| `quivering` | gentle curve with shaky stutters |
| `puzzled` | asymmetric check mark |

Preview them all in the standalone test with the `G` key. Publish a loudness level and it becomes a procedural travelling waveform whose amplitude, stroke width and glow scale with the level; publish raw audio chunks and the line traces the samples themselves (resampled to 512 points). The level is smoothed with a fast attack and slow release so syllables read clearly, and it decays to idle after `mouth_timeout` seconds of silence. The waveform is redrawn with each camera frame, so it animates at the camera rate.

```bash
# Quick check: make the mouth "talk" at 80% loudness
ros2 topic pub -r 20 /robot_face/mouth_level std_msgs/msg/Float32 "{data: 0.8}"
```

*Greeting (TTS)*

| Parameter | Default | Description |
|---|---|---|
| `greeting_enabled` | `true` | Say hello and look happy when a new person is detected |
| `greeting_text` | `hello` | Phrase spoken for strangers and unnamed people |
| `greeting_known_text` | `hello {name}` | Phrase spoken for people you have named |
| `greeting_late_text` | `oh, hi {name}` | Follow-up when the name is recognised after the generic greeting |
| `greeting_wait` | `2.0` | Seconds to wait for face ID before falling back to the generic greeting |
| `greeting_name_window` | `10.0` | Seconds after a generic greeting during which a recognised name is still spoken |
| `greeting_happy_seconds` | `30.0` | How long the happy mood lasts after a greeting |
| `speech_lead_silence` | `1.5` | Seconds of lead-in prepended to every utterance so the Bluetooth speaker is awake before the first word |
| `speech_lead_level` | `0.01` | The lead-in is a faint 50 Hz tone at this level (about -40 dBFS) because many speakers only wake on a real signal; `0` makes it pure silence |

*Text-to-Speech*

| Parameter | Default | Description |
|---|---|---|
| `tts_engine` | `auto` | `auto` picks Kokoro if installed, else Coqui, else espeak; or force `kokoro` / `coqui` / `espeak` |
| `tts_voice` | `af_heart` | Kokoro voice id (`af_*`/`am_*` American female/male, `bf_*`/`bm_*` British); 54 available |
| `tts_speed` | `1.0` | Kokoro speaking rate |
| `tts_lang` | `auto` | Phonemisation language; `auto` derives it from the voice prefix (a/b English, e Spanish, f French, h Hindi, i Italian, j Japanese, p Portuguese, z Mandarin) |
| `kokoro_python` | `~/.robot_face/kokoro-venv/bin/python` | Python of the venv holding `kokoro-onnx` |
| `kokoro_model_dir` | `~/.robot_face/kokoro` | Folder with `kokoro-v1.0.onnx` and `voices-v1.0.bin` |

The preferred voice is [Kokoro-82M](https://github.com/hexgrad/kokoro) through [kokoro-onnx](https://github.com/thewh1teagle/kokoro-onnx): on RD's CPU it loads in under a second and synthesises a short phrase in about 0.4 s. `kokoro-onnx` needs numpy 2, which the ROS/MediaPipe stack cannot have, so it is installed in its own virtualenv and the node runs `kokoro_worker.py` there as a long-lived worker process, talking to it over a pipe (one JSON request per line, model loaded once). If the worker is missing or dies, the node falls back to Coqui, then espeak. Install it with `provision/setup_kokoro.sh` (online, or offline with a wheel folder plus the two model files). Anything published on `~/say` (`std_msgs/String`) is spoken with the current engine, which is the quickest way to audition voices:

```bash
ros2 topic pub --once /robot_face/say std_msgs/msg/String "{data: 'Hi, I am Robot Disco.'}"
ros2 run robot_face combined_face_hand_node --ros-args -p tts_voice:=bm_george   # at launch
ros2 param set /robot_face tts_voice bm_george                                    # live, no restart
```

These parameters can be changed live with `ros2 param set` and take effect immediately: `tts_voice`, `tts_speed`, `tts_lang`, `greeting_enabled`, `greeting_text`, `greeting_known_text`, `greeting_late_text`, `greeting_wait`, `speech_lead_silence`, `speech_lead_level`, `mouth_smile`, `mouth_smile_happy`, `mouth_gestures`, `mouth_gain`. An unknown voice is rejected with the list of valid ones. Everything else is read once at startup. To make a change permanent, put it in `config/combined_face_hand_defaults.yaml` (used by the launch file) or your own YAML passed as `param_file:=`.

*Face Identification*

| Parameter | Default | Description |
|---|---|---|
| `face_id_enabled` | `true` | Learn to tell people apart |
| `face_id_models_dir` | `""` | Folder with the ONNX models (empty = `share/robot_face/models`) |
| `face_id_gallery` | `~/.robot_face/people.json` | Where learned people are stored |
| `face_id_interval` | `3` | Run face ID every N camera frames |
| `face_id_match_threshold` | `0.363` | Cosine similarity needed to match; higher = stricter |
| `face_id_auto_enroll` | `true` | Enroll strangers automatically as `person_N` |
| `face_id_min_face_px` | `40` | Ignore faces narrower than this |
| `face_id_unknown_threshold` | `0.28` | Below this against everyone known counts as a stranger; between this and the match threshold is "unsure" |
| `face_id_enroll_min_face_px` | `64` | Only learn new people when the face is at least this wide (close enough for a clean embedding) |
| `face_id_enroll_frames` | `6` | Consistent passes needed before a stranger is enrolled |

Face ID uses two models bundled in `models/` and run by OpenCV itself: YuNet finds the face and SFace turns it into a 128-number embedding. The embedding is compared against everyone in the gallery; a cosine similarity above `face_id_match_threshold` is a match. Scores in the "unsure" band are neither matched nor used to enroll. A clear stranger who is seen consistently for `face_id_enroll_frames` passes, close enough to the camera, is enrolled as `person_N` and the node logs how to name them. Confident matches add fresh views (up to 10 per person), and once someone has been confidently recognised the identity sticks for a couple of seconds through turned heads and distance, learning those harder views too. Naming a person with a name that already exists merges the two records, which is the fix when the same person gets enrolled twice. The gallery is a JSON file and survives restarts.

```bash
# Who does the robot know?
ros2 service call /robot_face/list_people std_srvs/srv/Trigger

# Name the person currently in front of the camera
ros2 topic pub --once /robot_face/set_person_name std_msgs/msg/String "{data: 'Bryan'}"

# Name a specific id, or forget someone
ros2 topic pub --once /robot_face/set_person_name std_msgs/msg/String "{data: 'person_2=Dan'}"

# Same person enrolled twice? Give the duplicate the existing name and they merge
ros2 topic pub --once /robot_face/set_person_name std_msgs/msg/String "{data: 'person_2=Bryan'}"
ros2 topic pub --once /robot_face/forget_person std_msgs/msg/String "{data: 'person_2'}"

# Watch recognitions live
ros2 topic echo /robot_face/person
```

Once a person is named, the greeting becomes `greeting_known_text` with their name. When a new person appears the node waits up to `greeting_wait` seconds for face ID so it can greet them by name. People are usually spotted by the pose tracker from across the room, before the face is large enough to identify, so if the wait runs out it says the generic greeting and then, if the name is recognised within `greeting_name_window` seconds, follows up with `greeting_late_text`.

Speech uses Coqui TTS (`tts_models/en/ljspeech/vits`) when the `TTS` package is installed, otherwise `espeak`. Either way the phrase is rendered to a WAV, played with `aplay`, and the WAV samples are streamed to the mouth waveform in 40 ms chunks timed to playback, so the mouth moves with the words coming out of the speaker.

*Hand Relay (TCP to AmazingHand)*

| Parameter | Default | Description |
|---|---|---|
| `relay_hand` | `false` | Enable TCP hand control streaming |
| `hand_hotkey` | `ctrl+shift+h` | Keyboard chord to toggle hand streaming |
| `hand_host` | `192.168.1.194` | IP of AmazingHand controller |
| `hand_port` | `8765` | TCP port for hand controller |
| `hand_rate` | `20.0` | Command frequency (Hz) |
| `hand_speed` | `1000` | Servo speed sent to hand (50–2000) |

*Video Recording*

| Parameter | Default | Description |
|---|---|---|
| `save_raw_video` | `/tmp/raw_camera.mp4` | Path for raw camera recording (empty = disabled) |
| `save_video` | `/tmp/combined_output.mp4` | Path for composite output recording (empty = disabled) |

#### OpenGL Eye Renderer (`gl_eye_renderer.py`)

Renders a 1080×1900 GLFW window titled **"Robot Disco"** using a GLSL fragment shader. All geometry is computed per-pixel on the GPU.

Visual features:
- Dark purple background with animated electric-purple horizontal scanlines
- Scanline wave: `sin(p.y * 0.88 + sin(p.x * 0.020) * 3.2 + u_time * 0.7)`
- Radial dimming toward eye edges, bright lavender rim highlight at top
- Outer bloom/glow using exponential falloff
- Pupils constrained within sclera bounds, offset by `u_gaze`
- Smooth antialiasing via `smoothstep`

Mood expressions (controlled by `u_mood` uniform):

| ID | Mood | Shape |
|---|---|---|
| 0 | Neutral | No corner cuts |
| 1 | Tired | Shallow droop at top-outer corners |
| 2 | Angry | Steep cut at top-inner corners |
| 3 | Happy | Bottom corners squint |
| 4 | Suspicious | Top-outer droop + bottom-inner slight cut |

Blink animation: smooth vertical compression (`max(0, 1 - u_blink)` applied to eye half-height), easing in/out over `blink_duration` seconds.

Mouth (`u_mouth_*` uniforms): a glowing stroke `mouth_gap` px below the eyes spanning the distance between the eye centres. Its resting shape is a parabola whose ends curl up by `u_mouth_smile` px (negative = frown) plus a `u_mouth_skew` tilt for smirks. On top of that, `u_mouth_mode` selects hidden / procedural / sampled speech motion: procedural mode sums three travelling sines scaled by `u_mouth_level` (perfectly still at level 0), sampled mode reads an `N×1` float texture of audio samples. A window pins both ends, and the stroke thickness uses the local slope (`dFdx`) so steep sections stay even. For gesture shapes the renderer also carries a `K×1` polyline texture (`u_mouth_path`); `morph_from_rest()` bends the resting curve into the shape point by point so the mouth visibly transforms rather than cross-fading.

The eye palette constants (`C_BASE`, `C_GLOW`, `C_RIM`) are the one place the robot's copy differs from this repo: RD runs a black fill with no eye glow or rim. See *Deploying to Robot Disco* below.

#### Hand Processing (`hand_processing.py`)

Converts MediaPipe 21-landmark hand skeleton into 8 servo joint angles sent over TCP to the AmazingHand.

Finger → servo mapping:

| Finger | Servos |
|---|---|
| Index | 0, 1 |
| Middle | 2, 3 |
| Ring | 4, 5 |
| Thumb | 6, 7 |

Curl detection uses 3D vector angles between joints (threshold 60°). Thumb uses a blended metric: 40% angle + 60% distance-to-index-tip.

TCP command format: `J:val0,val1,val2,val3,val4,val5,val6,val7,speed\n`

Open-hand timeout: if no hand is detected for 0.75 s, the node sends a fully-open command and waits.

#### Face Tracking (`face_utils.py`)

- Uses MediaPipe Pose (model_complexity=0 for speed) to derive a face bounding box from upper-body landmarks filtered by visibility > 0.4
- Hysteresis: tracked face must move more than `0.8 × box_dimension + 40px` to switch to a new face candidate
- Stick time: face considered lost after 1 s of no detection
- Idle behavior: after 4 s with nobody in view, a slow figure-of-eight gaze sweep (7 s period, ±33% of the frame horizontally, ±17% vertically) with a smaller, faster wander layered on top so the eyes don't look like they're on rails
- A newly tracked person triggers the happy mood for `greeting_happy_seconds`, the greeting, and face identification

#### Launch

```bash
ros2 launch robot_face combined_face_hand.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  hand_in_eyes:=true \
  mirror:=false
```

#### Running Without ROS (standalone renderer test)

```bash
python3 src/robot_face/test_eye_renderer.py
```

Controls: `SPACE` next preset, `WASD`/arrows gaze, `B` blink, `0-4` mood, `M` simulated talking (procedural mouth), `N` simulated talking (synthesised audio samples), `G` cycle through every mouth gesture, `Q` quit. Presets include `talking`, `whisper`, and `happy + talking`. Requires `moderngl`, `glfw`, `numpy` and `opencv-python`.

#### Deploying to Robot Disco (RD)

The robot's checkout at `/home/rd/rd_ws` is **not** kept in sync through git; it carries local edits (notably the black eye palette in `gl_eye_renderer.py`). Deploy by copying files and rebuilding:

```bash
# From this repo, on your workstation (SSH host "RD" = 192.168.11.80, user rd)
rsync -av src/robot_face/ RD:/home/rd/rd_ws/src/robot_face/   # diff first if RD has local edits
ssh RD 'cd ~/rd_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select robot_face'
```

Notes for RD:
- `install/` is a copy, not a symlink install, so every change needs the `colcon build` above before a restart picks it up.
- The face window needs the robot's Wayland/X session. `/home/rd/start_face.sh` exports the display environment and runs `ros2 run robot_face combined_face_hand_node` detached, logging to `~/rd_ws/log/robot_face-manual.log`. The supervisord `robot_face` service has no `DISPLAY` and fails with `glfw.init() failed`.
- RD has no internet access, so the Coqui TTS model cannot download and speech falls back to `espeak`. To get the Coqui voice, copy the `tts_models--en--ljspeech--vits` model into RD's TTS cache (`~/.local/share/tts/`).
- RD's OpenCV is 4.11 from pip, which is what the YuNet 2023 model requires (≥ 4.8). `setup.py` deliberately lists only `setuptools` in `install_requires`; naming pip packages there broke the TTS import on RD, whose OpenCV is not the `opencv-python` wheel.
- When stopping the node over SSH, avoid a `pkill` pattern that appears in your own command line (`pkill -f "lib/robot_face/[c]ombined"` works).
- Learned people live in `/home/rd/.robot_face/people.json`.

---

### AmazingHand

**Path**: `AmazingHand-main/`

A custom open-source 8 degree-of-freedom robotic hand end-effector. 4 fingers (index, middle, ring, thumb) × 2 SCS0009 smart servos each. Both left and right hand variants are available with dedicated mounting plates. CAD files (STL/STEP), assembly PDFs, and 3D printing guides are included in `AmazingHand-main/docs/` and `AmazingHand-main/cad/`.

#### Hardware

- **Servos**: SCS0009 (serial smart servo, PhantomX-style protocol)
- **Interface**: UART at 1 Mbit/s via USB-serial adapter
- **Control Library**: `rustypot` Python package (SCS0009 protocol)
- **DOF**: 8 total (2 per finger)
- **Variants**: Right hand (servo IDs 1–8), Left hand (servo IDs 11–18)

#### Servo Layout

| Finger | Servo IDs (right) | Servo IDs (left) |
|---|---|---|
| Index | 1, 2 | 11, 12 |
| Middle | 3, 4 | 13, 14 |
| Ring | 5, 6 | 15, 16 |
| Thumb | 7, 8 | 17, 18 |

#### Python Examples (`AmazingHand-main/PythonExample/`)

| File | Description |
|---|---|
| `AmazingHand_Demo.py` | Single hand — open, close, spread, and named gesture presets |
| `AmazingHand_Demo_Both.py` | Simultaneous left + right hand control |
| `AmazingHand_FingerTest.py` | Per-servo calibration workflow |
| `AmazingHand_Hand_FingerMiddlePos.py` | Middle-position calibration for individual fingers |

Available gesture presets: `OpenHand`, `CloseHand`, `OpenHand_Progressive`, `SpreadHand`, `ClenchHand`, `Index_Pointing`, `Nonono`, `Perfect`, `Victory`, `Pinched`, `Scissors`.

Middle (neutral) positions differ per hand to account for mechanical asymmetry:
- Right: `[3, 0, -5, -8, -2, 5, -12, 0]`
- Left: `[3, -3, -1, -10, 5, 2, -7, 3]`

#### ROS2 Integration (via robot_face)

The `combined_face_hand_node` streams live gesture commands to the AmazingHand over TCP when `relay_hand:=true`. MediaPipe detects the operator's hand, `hand_processing.py` converts 21 landmarks into 8 servo angles, and the node sends `J:val0,...,val7,speed\n` at 20 Hz to the hand's TCP server at `192.168.1.194:8765`.

Toggle hand relay at runtime with the configured hotkey (default `ctrl+shift+h`).

#### MuJoCo Simulation (`AmazingHand-main/Demo/AHSimulation/`)

Physics simulation of both hand variants using MuJoCo + mink inverse kinematics.

| File | Description |
|---|---|
| `mj_mink_right.py` | Right hand sim — 4 fingertip frame tasks, Dora dataflow |
| `mj_mink_left.py` | Left hand sim — same architecture, mirrored geometry |

Scene files: `AH_Right/mjcf/scene.xml`, `AH_Left/mjcf/scene.xml`

Note: The simulation uses Dora dataflow (not ROS2) and runs independently of the ROS stack.

#### Standalone Hand Tracking Demo (`AmazingHand-main/Demo/HandTracking/`)

`combined_face_hand.py` — MediaPipe hand tracking with direct TCP relay to the physical hand. Runs without ROS, useful for quick hardware testing outside the full system.

---

### robot_disco

**Path**: `src/robot_disco/` (git submodule)
**Repo**: github.com/r1b4z01d/Robot_Disco

Main mobile robot platform package. Provides bringup, localization, and navigation launch files for the Robot Disco differential-drive robot.

**Key launch files**:

```bash
# Full robot bringup (hardware drivers, transforms)
ros2 launch robot_disco launch_robot.launch.py

# AMCL localization against a saved map
ros2 launch robot_disco localization_launch.py

# Nav2 navigation stack
ros2 launch robot_disco navigation_launch.py

# Foxglove visualization bridge
ros2 launch robot_disco foxglove_bridge_launch.xml

# AprilTag map relocalization (detector + AMCL corrector; see below)
ros2 launch robot_disco apriltag_relocalization_launch.py
```

---

### apriltag / apriltag_ros

**Paths**: `src/apriltag/`, `src/apriltag_ros/`, `src/AprilTag_Detection/`
**Repos**: github.com/AprilRobotics/apriltag, github.com/Adlink-ROS/apriltag_ros

Fiducial marker detection for robot localization and object pose estimation. Used in conjunction with the RealSense D435 camera.

```bash
# NOTE: the stock launch defaults to /camera/color/...; the robot's topics are /camera/camera/color/...
ros2 launch apriltag_ros tag_realsense.launch.py camera_name:=/camera/camera/color
```

#### AprilTag map relocalization

`apriltag_relocalization_launch.py` runs the detector plus `scripts/apriltag_relocalizer.py`, which corrects AMCL from landmark tags with known map-frame poses. Needs `launch_robot.launch.py` (camera + TF) and `localization_launch.py` (AMCL) running.

```
detections ──► for each taught tag within max_range:
               T_map_base = T_map_tag · (T_base_tag from TF)⁻¹
            ──► N agreeing estimates in the last second
            ──► differs from AMCL by > 0.30 m or 12° ──► publish /initialpose (rate-limited)
```

**Tags**: 36h11, ~15–20 cm, mounted ~1.2–1.8 m high (the D435 is pitched 46° up in the URDF). IDs 0–9 are pre-listed in `config/apriltag_landmarks.yaml`; set each tag's real black-border edge length in `tag_sizes`. A print-ready set is in `documents/apriltag_landmarks_36h11_150mm.pdf` (one tag per Letter page, 150 mm black square, 100 mm scale bar; print at 100 %, not fit-to-page). Regenerate for other IDs/sizes with `python3 Scripts/make_apriltag_pdf.py --ids 10 11 --size-mm 200` (reads the codes from `src/apriltag/tag36h11.c`, so that submodule must be initialized).

**Teaching tag poses** (no measuring): with AMCL well localized and the tag in view,
```bash
ros2 service call /apriltag_relocalizer/teach_landmarks std_srvs/srv/Trigger
```
writes `map → tag` for every visible tag to `config/apriltag_landmark_poses.yaml`. Re-teach after re-mapping or moving a tag.

**Other controls**: `ros2 service call /apriltag_relocalizer/relocalize_now std_srvs/srv/Trigger` forces a correction; `auto_correct:=false` makes the node passive (service only). Thresholds, `max_range`, `min_detections` and the `/initialpose` covariance are node parameters. `/apriltag_relocalizer/estimated_pose` shows the tag-derived pose in Foxglove.

**Sanity check** before trusting it: with a tag in view, `ros2 run tf2_ros tf2_echo base_footprint tag_0` should report a position that matches where the tag physically is relative to the robot. If it doesn't, the camera mount in `description/realsense_d435.xacro` is wrong; the node also warns when the estimated base roll/pitch exceeds `tilt_warn`.

---

### roboclaw (Motor Controller)

**Paths**: `src/roboclaw_hardware_interface/`, `src/roboclaw_serial/`, `src/roboclaw_ros/`
**Repos**: dumbotics/roboclaw_hardware_interface, dumbotics/roboclaw_serial, norlab-ulaval/roboclaw_ros

RoboClaw motor controller integration for the Robot Disco differential-drive base.

- `roboclaw_serial`: Low-level serial communication library
- `roboclaw_ros`: ROS2 node exposing velocity and odometry
- `roboclaw_hardware_interface`: `ros2_control` hardware interface plugin for use with the ROS2 control framework

---

### velodyne / rplidar_ros

**Paths**: `src/velodyne/`, `src/rplidar_ros/`
**Repos**: github.com/ros-drivers/velodyne, standard RPLiDAR ROS2 driver

LiDAR drivers for the dual-lidar navigation setup.

- `velodyne`: Full 3D point cloud driver for Velodyne sensors (VLP-16, HDL-32E, etc.)
- `rplidar_ros`: 2D scan driver for RPLIDAR units

The system runs dual lidars simultaneously for navigation and obstacle detection.

---

## Hardware Peripherals

---

### WLED LED Matrix Panels

**Location**: Left and right sides of the robot body
**Quantity**: 2 panels (one per side)
**Resolution**: 32 × 48 pixels (W × H) per panel — 1,536 LEDs each

Each panel is an addressable RGB LED matrix running [WLED](https://kno.wled.ge/) firmware over Wi-Fi. WLED exposes an HTTP/JSON API and a WebSocket interface, allowing real-time control of animations, colors, and brightness from any device on the network. The panels can be driven independently or mirrored for symmetric effects.

#### Integration Points

- **WLED HTTP API**: `POST /json/state` to set colors, effects, brightness, and segments
- **WLED WebSocket**: `ws://<ip>/ws` for real-time streaming updates
- **E1.31 / Art-Net**: For high-framerate pixel-perfect content streamed from a host PC or Raspberry Pi
- **Foxglove / ROS2**: Can be commanded via a ROS2 node publishing to a WLED HTTP bridge (not yet implemented)

#### Example: Set both panels to solid purple

```bash
# Left panel
curl -X POST http://<left-panel-ip>/json/state \
  -H "Content-Type: application/json" \
  -d '{"on":true,"bri":200,"seg":[{"col":[[138,0,96]]}]}'

# Right panel
curl -X POST http://<right-panel-ip>/json/state \
  -H "Content-Type: application/json" \
  -d '{"on":true,"bri":200,"seg":[{"col":[[138,0,96]]}]}'
```

#### Panel Layout

| Property | Value |
|---|---|
| Width | 32 pixels |
| Height | 48 pixels |
| Total LEDs per panel | 1,536 |
| Firmware | WLED |
| Interface | Wi-Fi (HTTP + WebSocket + E1.31) |
| Mounting | Left side and right side of robot body |

---

### RealSense T265 Tracking Camera

**Location**: Left side of the robot
**Driver**: `realsense2_camera` (same ROS2 package as D435)

The Intel RealSense T265 is a standalone visual-inertial odometry (VIO) tracking camera. It uses two fisheye lenses and an onboard IMU to produce 6-DOF pose estimates without any external infrastructure. On Robot Disco it supplements wheel odometry from the RoboClaw with drift-free visual tracking, improving localization robustness in featureless or slippery environments.

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/t265/odom/sample` | `nav_msgs/Odometry` | 6-DOF pose + velocity at ~200 Hz |
| `/t265/accel/sample` | `sensor_msgs/Imu` | Raw accelerometer data |
| `/t265/gyro/sample` | `sensor_msgs/Imu` | Raw gyroscope data |
| `/t265/fisheye1/image_raw` | `sensor_msgs/Image` | Left fisheye camera |
| `/t265/fisheye2/image_raw` | `sensor_msgs/Image` | Right fisheye camera |

#### Launch

```bash
ros2 launch realsense2_camera rs_launch.py \
  device_type:=t265 \
  enable_pose:=true \
  publish_odom_tf:=true
```

#### Notes

- The T265 runs its SLAM pipeline entirely onboard — no host CPU cost for tracking
- Pose frame: starts at identity on boot; use a static TF to relate it to the robot base frame
- Best combined with wheel odometry via a robot localization EKF (`robot_localization` package)

---

### 2.1 Bluetooth Audio

**Configuration**: 2.1 stereo + subwoofer
**Interface**: Bluetooth (A2DP profile)

The robot has a 2.1 Bluetooth audio system for sound output — music, speech synthesis (TTS), and event feedback. The speaker connects over A2DP for high-quality stereo audio. On the host (Raspberry Pi / robot PC), audio is routed through PulseAudio or PipeWire with the BlueALSA or bluez-alsa backend.

#### System Setup (Ubuntu / Debian)

```bash
# Install Bluetooth audio backend
sudo apt install bluez pulseaudio pulseaudio-module-bluetooth

# Pair and connect the speaker
bluetoothctl
> power on
> scan on
> pair <SPEAKER_MAC>
> trust <SPEAKER_MAC>
> connect <SPEAKER_MAC>

# Set as default sink
pactl set-default-sink bluez_sink.<SPEAKER_MAC_underscored>
```

#### Playing Audio from ROS2

```bash
# Play a WAV file (any ROS node can shell out or use sound_play)
ros2 run sound_play soundplay_node.py

ros2 topic pub /robotsound sound_play/msg/SoundRequest \
  "{sound: -2, command: 1, volume: 1.0, arg: 'Hello, I am Robot Disco'}"
```

#### Audio Subsystem Summary

| Property | Value |
|---|---|
| Profile | Bluetooth A2DP |
| Configuration | 2.1 (stereo satellites + subwoofer) |
| Linux backend | PulseAudio + bluez |
| ROS2 integration | `sound_play` package |

---

## Foxglove Studio

**Path**: `foxglove_studio/`

A full local clone of the open-source Foxglove Studio TypeScript monorepo, customized for Robot Disco. This is the web-based visualization and control interface for the robot.

### Custom Panel: `CallPartyService`

**Path**: `foxglove_studio/packages/studio-base/src/panels/CallPartyService/`

A bespoke Foxglove panel that calls ROS2 services directly from the browser UI. Fully configurable via the Foxglove settings sidebar:

| Setting | Description |
|---|---|
| Service name | ROS2 service to call (e.g. `/start_waypoints`) |
| Button text | Label shown on the button |
| Tooltip | Hover text |
| Button color | RGBA color picker |
| Request payload | JSON body sent with the service call |
| Layout | Horizontal or vertical button arrangement |

State tracking: button reflects `requesting` / `success` / `error` states.

### Dashboard Layout (`foxglove-layout.json`)

The default layout loaded into Foxglove Studio at startup:

| Panel | Topic / Service | Description |
|---|---|---|
| Battery Gauge | `/battery.percentage` | 0–1 range, red-yellow-green colormap |
| Power Supply Indicator | `/battery.power_supply_status` | Charging state display |
| E-Stop Button | `/stop_motor` (service) | Large red emergency stop button |
| 3D View | `/map`, `/velodyne_points`, `/scan`, `/robot_description`, `/camera/camera/color/image_raw`, `/waypoints_path` | Full sensor + map visualization |
| Image Panel | `/camera/camera/color/image_raw` | Camera feed with pose estimation overlay |
| Party Buttons | `/start_waypoints`, `/Chargers` | "Party In Living Room" (purple), "Go To Charger" (green) |
| ROS Out | — | Log viewer |

3D view layers:
- `/global_costmap/costmap` at 40% alpha
- `/velodyne_points` — rainbow colormap, circle points, 2 px size
- `/scan` — turbo colormap
- Camera image projected into 3D scene with calibration from `/camera/camera/color/camera_info`
- Waypoint path and goal pose topics

How the default is applied: the container entrypoint injects `/foxglove/default-layout.json` (bind-mounted from `foxglove_studio/foxglove-layout.json` by the supervisor conf) into `index.html` as `FOXGLOVE_STUDIO_DEFAULT_LAYOUT`. Studio uses it **only for browsers that have no saved layout** — it persists the current layout in `localStorage` under `studio.layout`. After changing `foxglove-layout.json`, restart `robot_disco_foxglove_studio` and have existing clients clear that key (`localStorage.removeItem('studio.layout')` in the devtools console, then reload) or open the page in a fresh browser profile. To check the injection worked: `curl -s localhost:8080 | grep -o 'FOXGLOVE_STUDIO_DEFAULT_LAYOUT = .\{0,40\}'` on the robot should show `[{"configById"…`, not `[][0]`.

### Docker Deployment (`compose.foxglove.yaml`)

```yaml
services:
  foxglove:
    image: icon/foxglove
    ports:
      - 80:8080
    volumes:
      - ./foxglove-layout.json:/foxglove/default-layout.json
    environment:
      - DS_TYPE=foxglove-websocket
      - DS_PORT=8765
      - UI_PORT=8080
      - DISABLE_CACHE=true
```

Web UI is served on port 8080 (`--publish 8080:8080` in the supervisor conf; `compose.foxglove.yaml` maps it to 80 instead). Connects to the ROS2 Foxglove WebSocket bridge on port 8765.

```bash
# Start the Foxglove web UI
docker run --rm --name foxglove \
  --publish 8080:8080 \
  --volume ./foxglove_studio/foxglove-layout.json:/foxglove/default-layout.json \
  robotdisco/foxglove

# Or via compose
docker compose -f foxglove_studio/compose.foxglove.yaml up
```

---

## Supervisor Services

**Path**: `services/`
**Install script**: `Scripts/install_services.sh`

[Supervisord](http://supervisord.org/) manages all major robot subsystems as persistent background processes. Services auto-restart on failure where appropriate and write rotating logs to `~/rd_ws/log/`.

### Installation

```bash
sudo Scripts/install_services.sh      # runs from any directory
```

This copies every `services/*.conf` to `/etc/supervisor/conf.d/` and reloads the daemon, so a new `.conf` needs no changes to the script. Re-run it after editing any conf, then `sudo supervisorctl restart <service>`.

### Service Overview

| Service | Command | Autostart | Autorestart |
|---|---|---|---|
| `robot_disco` | `ros2 launch robot_disco launch_robot.launch.py` | false | false |
| `robot_disco_foxglove_bridge` | `ros2 launch robot_disco foxglove_bridge_launch.xml` | **true** | **true** |
| `robot_disco_foxglove_studio` | `docker run ... robotdisco/foxglove` | **true** | **true** |
| `robot_disco_ur5e_driver` | `ros2 launch robot_disco ur5e.launch.py` | false | true |
| `robot_disco_localization` | `ros2 launch robot_disco localization_launch.py` | false | false |
| `robot_disco_slam` | `ros2 launch robot_disco online_async_launch.py` | false | false |
| `robot_disco_navigation` | `ros2 launch robot_disco navigation_launch.py` | false | false |
| `robot_disco_collision_monitor` | `ros2 launch nav2_collision_monitor collision_monitor_node.launch.py` | **true** | **true** |
| `robot_face` | `ros2 launch robot_face combined_face_hand.launch.py` | false | false |
| `robot_disco_apriltag_relocalization` | `ros2 launch robot_disco apriltag_relocalization_launch.py` | false | true |

Foxglove bridge, Studio and the collision monitor are the only services that start automatically on boot — everything else is started manually once the robot is ready.

The `robot_face` service currently fails on RD because supervisord gives it no `DISPLAY`; start the face with `/home/rd/start_face.sh` instead (see *Deploying to Robot Disco* under robot_face).

### Service Details

**`robot_disco_foxglove_studio`** runs the Foxglove web UI inside Docker:
```
docker run --rm --name foxglove \
  --publish 8080:8080 \
  --volume /home/rd/rd_ws/foxglove_studio/foxglove-layout.json:/foxglove/default-layout.json:ro \
  robotdisco/foxglove
```
- `startsecs=10` — waits 10 s before marking healthy
- `stopsignal=INT` — graceful Docker shutdown
- `stoptimeout=10`

**`robot_disco_slam`** runs slam_toolbox for building or updating a map. It publishes the `map → odom` transform, as does `robot_disco_localization`, so run one or the other, never both. After mapping, save with `map_saver_cli` and serialize the pose graph (see `maps/`).

**`robot_disco_collision_monitor`** sits in the drive path between `twist_mux` and the diff drive controller, so every velocity source (joystick, Nav2, behavior server) is gated by it. If it is not running the robot will not move.
```
joystick → /cmd_vel_joy ─┐
nav      → /cmd_vel_nav ─┼→ twist_mux → /cmd_vel_raw → collision_monitor → /diff_cont/cmd_vel_unstamped
behavior → /cmd_vel     ─┘
```
`twist_mux` priorities (`config/twist_mux.yaml`): joystick 100 > `cmd_vel_nav` 20 > `cmd_vel` 10, each with a 0.5 s timeout, so the joystick always overrides Nav2.

Launch args (the nav2 launch file's argument is `params_file`, plural; unknown args are silently ignored):
```
params_file:=/home/rd/rd_ws/src/robot_disco/config/collision_monitor_params.yaml use_sim_time:=false
```

All services share these settings:
- `ROS_DOMAIN_ID=0`
- Working directory: `/home/rd/rd_ws`
- User: `rd`
- `stopasgroup=true`, `killasgroup=true` — ensures all child processes are cleaned up
- Log rotation: 20 MB max, 2 backups per service

### Supervisorctl Commands

`supervisorctl` needs `sudo` on the robot.

```bash
# View all service states
sudo supervisorctl status

# Typical navigation session
sudo supervisorctl start robot_disco                 # bringup: drivers, camera, TF, twist_mux
sudo supervisorctl start robot_disco_localization    # AMCL on the saved map (not with robot_disco_slam)
sudo supervisorctl start robot_disco_navigation
sudo supervisorctl start robot_disco_apriltag_relocalization   # optional

# Mapping session instead of localization
sudo supervisorctl start robot_disco_slam

# Others
sudo supervisorctl start robot_disco_ur5e_driver
sudo supervisorctl start robot_face

# Stop / restart
sudo supervisorctl stop robot_disco_navigation
sudo supervisorctl restart robot_disco_ur5e_driver

# Stream logs
sudo supervisorctl tail -f robot_disco
sudo supervisorctl tail -f robot_disco_foxglove_bridge
```

---

## Provisioning a New Robot PC

`provision/` rebuilds the robot PC from a fresh Ubuntu 22.04 install: apt + ROS 2 Humble packages, udev
rules, groups, the workspace build, `robot_face` python deps, supervisor units, GNOME autologin, the
Foxglove image, optionally the VLA venv, and finally the static `192.168.11.80` address.

```bash
provision/capture_rd_state.sh          # workstation: snapshot/backup the live robot first
~/rd_ws/provision/provision.sh all     # new machine, as user rd, with internet
```

See [provision/README.md](provision/README.md) for the phases, the snapshot/restore flow, how to
provision a box that is already on the internet-less robot subnet, and the Jetson Orin variant
(`provision/jetson/provision_jetson.sh`) that also runs the VLA inference server on the robot's GPU.

---

## Quick Start

### Build

```bash
# Build everything
colcon build
source install/setup.bash

# Build only robot_face
colcon build --packages-select robot_face
source install/setup.bash
```

On the robot (`rd@RD`, workspace `/home/rd/rd_ws`) the workspace is built with `--symlink-install`, so edits to existing launch files, configs and Python scripts take effect on the next launch without a rebuild. **New** files still need `colcon build --packages-select <pkg> --symlink-install` to get their symlinks.

If a rebuild "finishes" in a fraction of a second without installing anything, check the clock: make skips work when build files look newer than now (`timedatectl` should say `System clock synchronized: yes`; `sudo timedatectl set-ntp true`). `colcon build --cmake-force-configure` gets past a one-off skew.

### Run robot_face

```bash
# Minimal run against RealSense
ros2 launch robot_face combined_face_hand.launch.py \
  image_topic:=/camera/camera/color/image_raw

# With hand skeleton overlay and live AmazingHand mirroring
ros2 launch robot_face combined_face_hand.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  hand_in_eyes:=true \
  --ros-args -p relay_hand:=true -p hand_host:=192.168.1.194
```

> **Node name:** the launch file names the node `combined_face_hand`, so its topics and services are under `/combined_face_hand/...`. When started with `ros2 run robot_face combined_face_hand_node` (as `start_face.sh` does on RD) the node keeps its default name and everything is under `/robot_face/...`. The examples below use `/robot_face`; swap the prefix if you used the launch file.

### Control eye mood at runtime

```bash
ros2 service call /robot_face/set_mood/happy      std_srvs/srv/Trigger
ros2 service call /robot_face/set_mood/angry      std_srvs/srv/Trigger
ros2 service call /robot_face/set_mood/tired      std_srvs/srv/Trigger
ros2 service call /robot_face/set_mood/suspicious std_srvs/srv/Trigger
ros2 service call /robot_face/set_mood/neutral    std_srvs/srv/Trigger
```

### Teach the robot who you are

```bash
# Stand in front of the camera for a couple of seconds; the log reports
# "New person enrolled as person_1". Then, while still in view:
ros2 topic pub --once /robot_face/set_person_name std_msgs/msg/String "{data: 'Bryan'}"

# Check, rename by id, forget, or watch live recognitions
ros2 service call /robot_face/list_people std_srvs/srv/Trigger
ros2 topic pub --once /robot_face/set_person_name std_msgs/msg/String "{data: 'person_2=Dan'}"
ros2 topic pub --once /robot_face/forget_person std_msgs/msg/String "{data: 'person_2'}"
ros2 topic echo /robot_face/person
```

Named people are greeted by name ("hello Bryan"). If the face is only recognised after the generic hello, the robot follows up with "oh, hi Bryan".

### Drive the mouth from your own audio

```bash
# Loudness only (0..1): procedural waveform
ros2 topic pub -r 20 /robot_face/mouth_level std_msgs/msg/Float32 "{data: 0.8}"

# Raw samples in -1..1: the waveform traces the audio itself
ros2 topic pub --once /robot_face/mouth_audio std_msgs/msg/Float32MultiArray "{data: [0.0, 0.5, 0.9, 0.5, 0.0, -0.5, -0.9, -0.5]}"
```

---

## Hardware Bringup

### UR5e Arm

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
  robot_ip:=192.168.1.21

# Load and run a UR program
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "filename: remote.urp"
ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load "filename: dball.urp"
ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}
```

### Static Transforms

```bash
# Camera mounted to flange
ros2 run tf2_ros static_transform_publisher \
  0.00175 0.002985 0.0068 0.0 0.0 3.14 flange camera_link

# Grasp location relative to cube
ros2 run tf2_ros static_transform_publisher \
  0.20 0.00 0.0 3.15 1.6 0.0 purple_cube grasp_loc
```

The `flange → camera_link` transform is for the arm-mounted camera demo only. `launch_robot.launch.py` already publishes `base_link → camera_link` from the URDF (`description/realsense_d435.xacro`); running both gives `camera_link` two parents and breaks TF for everything that uses the camera.

### RealSense Camera

`launch_robot.launch.py` already includes the D435 (`launch/realsense_launch.py`, point cloud enabled). Standalone:

```bash
# D435 with point cloud enabled
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  device_type:=d435
```

Topics are under `/camera/camera/...` (namespace and name are both `camera`), e.g. `/camera/camera/color/image_raw` and `/camera/camera/color/camera_info`.

### Navigation Stack

```bash
ros2 launch robot_disco launch_robot.launch.py
ros2 launch robot_disco localization_launch.py          # or online_async_launch.py to map
ros2 launch robot_disco navigation_launch.py
ros2 launch nav2_collision_monitor collision_monitor_node.launch.py \
  params_file:=$HOME/rd_ws/src/robot_disco/config/collision_monitor_params.yaml use_sim_time:=false
```

On the robot these are the `robot_disco*` supervisor services (see [Supervisor Services](#supervisor-services)). The collision monitor is in the drive path, so without it the robot does not move.

#### Maps

Maps live in `src/robot_disco/maps/`. The active map is **`zone_one`**: `localization_launch.py` loads `maps/zone_one.yaml` (override with `map:=/path/to/other.yaml`), and slam_toolbox continues from `maps/zone_one` (`map_file_name` in `config/mapper_params_online_async.yaml`).

| Files | Used by |
|---|---|
| `<name>.pgm` + `<name>.yaml` | `map_server` / AMCL (occupancy grid) |
| `<name>.posegraph` + `<name>.data` | slam_toolbox, to continue or extend an existing map |

**Recording or updating a map** (on the robot, with `robot_disco_localization` stopped):

```bash
sudo supervisorctl start robot_disco robot_disco_slam
# drive the robot around; watch /map in Foxglove

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

---

## Utilities

### AprilTag detection with RealSense

```bash
# camera only if launch_robot.launch.py is NOT running (it already starts the D435)
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true device_type:=d435

# the stock launch defaults to /camera/color/...; the robot's topics are /camera/camera/color/...
ros2 launch apriltag_ros tag_realsense.launch.py camera_name:=/camera/camera/color
```

Detections are published on `/apriltag/apriltag_detections` and as TF frames (`<family>:<id>`, or the names in `tag_frames`). For map relocalization use `apriltag_relocalization_launch.py` instead (see [AprilTag map relocalization](#apriltag-map-relocalization)).

### Waypoint runner (`Scripts/path.py`)

A plain Python script (not a ROS package) built on `nav2_simple_commander`. Run it on the robot once bringup, localization and navigation are up:

```bash
cd ~/rd_ws/Scripts && python3 path.py                 # waypoints.yaml next to the script
cd ~/rd_ws/Scripts && python3 path.py tigger_route.yaml  # any route file
python3 path.py --check tigger_route.yaml                 # parse only: segments + the commands, no ROS
```

What it does, in order:

1. Waits for Nav2 to become active.
2. Loads `waypoints.yaml` from the same directory, or the route file given as the first argument.
3. If the file has an `initial_pose`, publishes it to `/initialpose` to seed AMCL, then runs its `post_actions`.
4. Publishes the route on `/waypoints_path` (`nav_msgs/Path`, transient-local) so Foxglove's 3D panel shows it.
5. Waits for the `start_waypoints` Trigger service — the Foxglove "Party" button calls it — then runs Nav2 `FollowWaypoints`.
6. Runs each waypoint's background `post_actions` when the robot moves on to the next waypoint (arrival is inferred from the action feedback index advancing), and the last waypoint's on completion. A waypoint with a **blocking** action (`wait: true`, or any `vla:` action) ends a `FollowWaypoints` segment: the robot stops there, the blocking actions run in order while the node keeps spinning, then the next segment starts. A failed blocking action with `on_fail: stop` (the default) ends the route where the robot stands.

Route file format:

```yaml
waypoints:
  - name: charger
    position: {x: 2.99, y: 1.99, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -0.004, w: 1.0}   # always give a full quaternion; a missing w defaults to 0
    post_actions:
      - 'ROS_DOMAIN_ID=0 ros2 topic pub /battery sensor_msgs/BatteryState "{percentage: 0.82, power_supply_status: 0}"'
      - ffplay -nodisp -autoexit /home/rd/Music/track.mp3   # plain strings: shell commands, run in the background
      - run: "some command"                                  # the same with options:
        wait: true                                           #   blocking — the robot stops here until it exits
        on_fail: stop                                        #   stop (default) | continue, blocking only
  - name: bookshelf
    position: {x: 2.54, y: 1.44, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.995, w: 0.102}
    post_actions:
      - vla: pickup_tigger_smolvla_ft    # a VLA policy run on the arm (always blocking): a training run
        task: "pick up tigger"           #   name, run@step or a path under ~/VLA/outputs/train, and the
        seconds: 90                      #   task string it was trained with; cap on the playing phase
        done: closed                     #   end by the grasp level: closed (a pick) | open (a place, after holding)
        still: 2                         #   averaging window (s) for done:, or motion-stillness seconds without it
        keep_grasp: true                 #   carry the object: hand stays closed, UR stays powered
        home: true                       #   return the arm to its home (transport) pose before driving on

initial_pose:
  position: {x: 0.36, y: -0.14, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.012, w: 1.0}
  post_actions: []
```

A `vla:` action runs `~/VLA/scripts/vla_action.py`, which stops the recording panel if it is up, switches the
robot's inference server to that model (a ~30 s restart only when the model changes), runs
`scripts/eval_hw.py --auto` and returns when the arm is back home; the run lands in the eval registry like
any other. `Scripts/tigger_route.yaml` is the fetch-and-carry example (bookshelf → pick up Tigger → banana
bed → place). Details: `VLA/wiki/Deploy-on-Hardware.md`, "Unattended runs".

All poses are in the `map` frame, so they must be re-recorded after a fresh SLAM map (see [Maps](#maps)). The easiest way to capture a pose is to drive there and `ros2 topic echo /amcl_pose --once`.

### Publish a battery state for testing

```bash
ROS_DOMAIN_ID=0 ros2 topic pub /battery sensor_msgs/BatteryState \
  "{percentage: 0.82, power_supply_status: 0}"
```
