# AmazingHand (8-DOF hand)

## AmazingHand

**Path**: `AmazingHand-main/`

A custom open-source 8 degree-of-freedom robotic hand end-effector. 4 fingers (index, middle, ring, thumb) × 2 SCS0009 smart servos each. Both left and right hand variants are available with dedicated mounting plates. CAD files (STL/STEP), assembly PDFs, and 3D printing guides are included in `AmazingHand-main/docs/` and `AmazingHand-main/cad/`.

### Hardware

- **Servos**: SCS0009 (serial smart servo, PhantomX-style protocol)
- **Interface**: UART at 1 Mbit/s via USB-serial adapter
- **Control Library**: `rustypot` Python package (SCS0009 protocol)
- **DOF**: 8 total (2 per finger)
- **Variants**: Right hand (servo IDs 1–8), Left hand (servo IDs 11–18)

### Servo Layout

| Finger | Servo IDs (right) | Servo IDs (left) |
|---|---|---|
| Index | 1, 2 | 11, 12 |
| Middle | 3, 4 | 13, 14 |
| Ring | 5, 6 | 15, 16 |
| Thumb | 7, 8 | 17, 18 |

### Python Examples (`AmazingHand-main/PythonExample/`)

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

### ROS2 Integration (via robot_face) and the robot's hand link

On the robot the hand is an ESP32 at `192.168.11.117` on the robot Wi-Fi, powered from the UR tool port, running the firmware in `AmazingHand-main/` (ESP32 folder). It speaks a one-line protocol, `J:val0,...,val7,speed\n` (servo offsets in degrees, speed 50..2000), over **UDP :8766** (the live transport: a lost setpoint is simply superseded by the next one; `Q` is answered with `K:<uptime>` as a liveness probe) and TCP :8765 (legacy). The VLA teleop and eval (`VLA/ur5e_lerobot/hand/amazing_hand_client.py`, the reference implementation), the manager's Hand tab and the face node's finger wiggle during the wave (`hand_wiggle.py`, `hand_host:hand_port`) all use the UDP port.

The legacy relay in `combined_face_hand_node` (`relay_hand:=true`, off on the robot because of `face_only`) streams live gesture commands over TCP instead: MediaPipe detects the operator's hand, `hand_processing.py` converts 21 landmarks into 8 servo angles, and the node sends the same `J:` lines at `hand_rate` (20 Hz) to `hand_relay_host:hand_relay_port`. Toggle it at runtime with the configured hotkey (default `ctrl+shift+h`).

### MuJoCo Simulation (`AmazingHand-main/Demo/AHSimulation/`)

Physics simulation of both hand variants using MuJoCo + mink inverse kinematics.

| File | Description |
|---|---|
| `mj_mink_right.py` | Right hand sim — 4 fingertip frame tasks, Dora dataflow |
| `mj_mink_left.py` | Left hand sim — same architecture, mirrored geometry |

Scene files: `AH_Right/mjcf/scene.xml`, `AH_Left/mjcf/scene.xml`

Note: The simulation uses Dora dataflow (not ROS2) and runs independently of the ROS stack.

### Standalone Hand Tracking Demo (`AmazingHand-main/Demo/HandTracking/`)

`combined_face_hand.py` — MediaPipe hand tracking with direct TCP relay to the physical hand. Runs without ROS, useful for quick hardware testing outside the full system.
