# Robot Disco V3 - ROS2 Workspace

![Robot Disco System Architecture](documents/Robot%20Disco%20System%20Architecture.jpg)

A ROS2 workspace for a mobile robot platform with a UR5e arm, animated robot face, AprilTag detection, LiDAR-based navigation, AmazingHand end-effector, WLED LED matrix panels, RealSense T265 tracking camera, and 2.1 Bluetooth audio. The workspace integrates real-time vision, GPU-accelerated eye animation, and multi-robot hardware drivers.

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
- [Quick Start](#quick-start)
- [Hardware Bringup](#hardware-bringup)
  - [UR5e Arm](#ur5e-arm)
  - [RealSense Camera](#realsense-camera)
  - [Navigation Stack](#navigation-stack)
- [Utilities](#utilities)

---

## Workspace Structure

```
rd_ws/
├── src/
│   ├── robot_face/             # Animated robot eyes + hand mirroring (active)
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
└── Scripts/                    # Install/utility scripts
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

Provides a real-time animated robot face display driven by MediaPipe pose and hand detection. The face renders two animated eyes (with moods, blinking, and gaze tracking) in a GPU-accelerated OpenGL window. It can optionally stream detected hand joint angles to the AmazingHand over TCP, making the robot mirror the operator's hand gestures live.

#### Architecture

```
Camera Image Topic
       │
       ▼
 combined_face_hand_node
 ├── MediaPipe Pose detector   → face bounding box → gaze target
 ├── MediaPipe Hands detector  → finger curl angles → TCP hand commands
 ├── GlEyeRenderer (OpenGL)    → animated eye window (GPU)
 └── Eye mood services         → 5 mood expressions via ROS services
```

#### Source Files

| File | Purpose |
|---|---|
| `combined_face_hand_node.py` | Main ROS2 node — orchestrates all subsystems |
| `gl_eye_renderer.py` | GPU-accelerated eye renderer (OpenGL 3.3 + GLSL) |
| `eye_renderer.py` | CPU-based eye renderer (OpenCV fallback) |
| `face_utils.py` | Pose landmark → bounding box, face tracking/hysteresis |
| `hand_processing.py` | Hand landmarks → servo joint angles → TCP command string |
| `window_utils.py` | Cross-platform global hotkey detection (Win/Linux/macOS) |

#### Node: `combined_face_hand_node`

**Entry point**: `robot_face.combined_face_hand_node:main`

**Subscriptions**:

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/camera/camera/color/image_raw` (configurable) | `sensor_msgs/Image` | BEST_EFFORT, depth=1 | Live camera frames for detection |

**Services Provided**:

| Service | Type | Description |
|---|---|---|
| `~/set_mood/neutral` | `std_srvs/Trigger` | Neutral eye expression |
| `~/set_mood/tired` | `std_srvs/Trigger` | Drooping tired eyes |
| `~/set_mood/angry` | `std_srvs/Trigger` | Angled angry brows |
| `~/set_mood/happy` | `std_srvs/Trigger` | Squinting happy eyes |
| `~/set_mood/suspicious` | `std_srvs/Trigger` | One-eyebrow-up suspicious look |

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
- Idle behavior: sinusoidal gaze sweep when no person present

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

Controls: `SPACE` next preset, `WASD`/arrows gaze, `B` blink, `0-4` mood, `Q` quit.

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
```

---

### apriltag / apriltag_ros

**Paths**: `src/apriltag/`, `src/apriltag_ros/`, `src/AprilTag_Detection/`
**Repos**: github.com/AprilRobotics/apriltag, github.com/Adlink-ROS/apriltag_ros

Fiducial marker detection for robot localization and object pose estimation. Used in conjunction with the RealSense D435 camera.

```bash
ros2 launch apriltag_ros tag_realsense.launch.py
```

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

Web UI is served on port 80 (mapped from container port 8080). Connects to the ROS2 Foxglove WebSocket bridge on port 8765.

```bash
# Start the Foxglove web UI
docker run --rm --name foxglove \
  --publish 8080:8080 \
  --volume ./foxglove_studio/foxglove-layout.json:/foxglove/default-layout.json \
  RobotDisco/foxglove

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
cd Scripts
chmod +x install_services.sh
sudo ./install_services.sh
```

This copies all `services/*.conf` files to `/etc/supervisor/conf.d/` and reloads the daemon.

### Service Overview

| Service | Command | Autostart | Autorestart |
|---|---|---|---|
| `robot_disco` | `ros2 launch robot_disco launch_robot.launch.py` | false | false |
| `robot_disco_foxglove_bridge` | `ros2 launch robot_disco foxglove_bridge_launch.xml` | **true** | **true** |
| `robot_disco_foxglove_studio` | `docker run ... RobotDisco/foxglove` | **true** | **true** |
| `robot_disco_ur5e_driver` | `ros2 launch robot_disco ur5e.launch.py` | false | true |
| `robot_disco_localization` | `ros2 launch robot_disco localization_launch.py` | false | false |
| `robot_disco_navigation` | `ros2 launch robot_disco navigation_launch.py` | false | false |
| `robot_disco_collision_monitor` | `ros2 launch nav2_collision_monitor collision_monitor_node.launch.py` | false | true |

Foxglove bridge and Studio are the only services that start automatically on boot — everything else is started manually once the robot is ready.

### Service Details

**`robot_disco_foxglove_studio`** runs the Foxglove web UI inside Docker:
```
docker run --rm --name foxglove \
  --publish 8080:8080 \
  --volume /home/rd/rd_wd/foxglove_studio/foxglove-layout.json:/foxglove/default-layout.json \
  RobotDisco/foxglove
```
- `startsecs=10` — waits 10 s before marking healthy
- `stopsignal=INT` — graceful Docker shutdown
- `stoptimeout=10`

**`robot_disco_collision_monitor`** uses a dedicated param file:
```
param_file:=/home/rd/rd_ws/src/robot_disco/config/collision_monitor_params.yaml
```

All services share these settings:
- `ROS_DOMAIN_ID=0`
- Working directory: `/home/rd/rd_ws`
- User: `rd`
- `stopasgroup=true`, `killasgroup=true` — ensures all child processes are cleaned up
- Log rotation: 20 MB max, 2 backups per service

### Supervisorctl Commands

```bash
# View all service states
supervisorctl status

# Start services individually
supervisorctl start robot_disco
supervisorctl start robot_disco_localization
supervisorctl start robot_disco_navigation
supervisorctl start robot_disco_ur5e_driver
supervisorctl start robot_disco_collision_monitor

# Stop / restart
supervisorctl stop robot_disco_navigation
supervisorctl restart robot_disco_ur5e_driver

# Stream logs
supervisorctl tail -f robot_disco
supervisorctl tail -f robot_disco_foxglove_bridge
```

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

### Control eye mood at runtime

```bash
ros2 service call /combined_face_hand/set_mood/happy      std_srvs/srv/Trigger
ros2 service call /combined_face_hand/set_mood/angry      std_srvs/srv/Trigger
ros2 service call /combined_face_hand/set_mood/tired      std_srvs/srv/Trigger
ros2 service call /combined_face_hand/set_mood/suspicious std_srvs/srv/Trigger
ros2 service call /combined_face_hand/set_mood/neutral    std_srvs/srv/Trigger
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

### RealSense Camera

```bash
# D435 with point cloud enabled
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true \
  device_type:=d435
```

### Navigation Stack

```bash
ros2 launch robot_disco launch_robot.launch.py
ros2 launch robot_disco localization_launch.py
ros2 launch robot_disco navigation_launch.py
```

---

## Utilities

### AprilTag detection with RealSense

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true device_type:=d435
ros2 launch apriltag_ros tag_realsense.launch.py
```

### Publish a battery state for testing

```bash
ROS_DOMAIN_ID=0 ros2 topic pub /battery sensor_msgs/BatteryState \
  "{percentage: 0.82, power_supply_status: 0}"
```
