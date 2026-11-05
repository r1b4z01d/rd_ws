# robot_face (eyes, face recognition, speech, Folks)

## robot_face

**Path**: `src/robot_face/`
**Maintainer**: Bryan Ribas (bryanribas@gmail.com)
**License**: CC BY-NC-SA

Provides a real-time animated robot face driven by the camera. The face is drawn in a GPU-accelerated OpenGL window: two eyes with moods, blinking and gaze tracking, and a mouth that rests in a slight smile, traces the robot's speech as a waveform, and pulls random expressions while nobody is around. The node recognises people it has met (ArcFace embeddings from a worker process in its own venv, OpenCV YuNet + SFace as the fallback; learned on the fly and persisted to disk), greets them by name through Kokoro TTS, waves the UR5e arm at people it knows while the AmazingHand's fingers ripple, serves the **Folks** web page of the people it knows, and keeps the USB speaker the default audio sink. An optional legacy mode streams the operator's hand joints to the AmazingHand over TCP (`relay_hand`); the robot runs with `face_only: true`, which never loads the hand model.

### Architecture

```
Camera Image Topic
       │
       ▼
 combined_face_hand_node
 ├── Tracker                    → SCRFD face detector (tracker: face, the default with ArcFace) or MediaPipe Pose
 │                                → person box → gaze target, "someone is here"
 ├── FaceIdentifier (face_id)   → faceid_worker.py (InsightFace venv: SCRFD + ArcFace) or YuNet + SFace → people gallery (JSON)
 ├── Greeting / TTS             → "hello" / "hello <name>" / wink on a repeat visit → kokoro_worker.py (Kokoro-82M venv) → Coqui → espeak
 │                                → WAV → aplay (pinned to the USB sink) → mouth waveform; ~/say speaks anything
 ├── UR wave (ur_motion, ur_dashboard) → replays the recorded wave over ur_rtde, powers the arm up first if needed
 ├── HandWiggle (hand_wiggle)   → UDP J: lines to the hand's ESP32 while the arm waves
 ├── Mouth state                → smile (wider when seeing someone), speech level, idle gestures
 ├── MediaPipe Hands detector   → finger curl angles → TCP hand relay (legacy; off with face_only)
 ├── GlEyeRenderer (OpenGL)     → eyes + mouth window (GPU)
 ├── SinkWatchdog (audio_sink)  → keeps the C-Media USB adapter the default PulseAudio sink
 ├── Folks page (people_web)    → http://<robot>:8096/, the people the robot knows
 └── ROS interfaces             → mood / wave / wink services, mouth and say topics, people naming / listing
```

### Source Files

| File | Purpose |
|---|---|
| `combined_face_hand_node.py` | Main ROS2 node — orchestrates all subsystems |
| `gl_eye_renderer.py` | GPU-accelerated eye renderer (OpenGL 3.3 + GLSL) |
| `eye_renderer.py` | CPU-based eye renderer (OpenCV fallback) |
| `audio_utils.py` | WAV loading, chunking and loudness helpers for the mouth waveform |
| `audio_sink.py` | `SinkWatchdog`: keeps the USB speaker the default PulseAudio sink, pins `aplay` to it |
| `tts_engines.py` | Speech back-ends: Kokoro (worker process), Coqui, espeak; the `~/say` path |
| `kokoro_worker.py` | Long-lived Kokoro-82M synthesiser, run in `~/.robot_face/kokoro-venv` (JSON over a pipe) |
| `face_id.py` | Face identification front end: galleries, matching, enrolment, both back-ends |
| `faceid_worker.py` | SCRFD + ArcFace worker, run in `~/.robot_face/faceid-venv` (InsightFace, onnxruntime-gpu on the Orin) |
| `people_web.py` | The Folks page: people, pictures, names, greetings (stdlib HTTP on :8096) |
| `ur_motion.py` | Replays a recorded joint trajectory on the UR5e over ur_rtde (the wave), staged approach/return, home pose |
| `ur_dashboard.py` | UR Dashboard Server client: power up, brake release, protective-stop / fault recovery, program play |
| `hand_wiggle.py` | Finger ripple over UDP to the AmazingHand ESP32 during the wave |
| `mouth_gestures.py` | Idle mouth gesture library: timings, shapes and morph envelopes (no ROS dependency) |
| `face_utils.py` | Pose landmark → bounding box, face tracking/hysteresis |
| `hand_processing.py` | Hand landmarks → servo joint angles → TCP command string (legacy relay) |
| `window_utils.py` | Cross-platform global hotkey detection (Win/Linux/macOS) |
| `models/` | YuNet + SFace ONNX models for the OpenCV face-ID fallback (installed to `share/robot_face/models`) |
| `motions/wave.json` | The arm's wave, installed to `~/.robot_face/motions/` by provisioning |
| `scripts/ur_record_motion.py` | Record the wave (freedrive demonstration, keyframes, or the pendant program) and capture the home pose |
| `scripts/ur_path_eval.py` | Forward-kinematics check of the staged approach/return orders for the wave |
| `test_eye_renderer.py` | Standalone visual test of the renderer, no ROS required |

### Node: `combined_face_hand_node`

**Entry point**: `robot_face.combined_face_hand_node:main`

**Subscriptions**:

| Topic | Type | QoS | Description |
|---|---|---|---|
| `/camera/camera/color/image_raw` (configurable) | `sensor_msgs/Image` | BEST_EFFORT, depth=1 | Live camera frames for detection |
| `~/mouth_level` (configurable) | `std_msgs/Float32` | BEST_EFFORT, depth=1 | Speech loudness 0..1; drives the mouth waveform amplitude |
| `~/mouth_audio` (configurable) | `std_msgs/Float32MultiArray` | BEST_EFFORT, depth=1 | Raw audio chunk, samples in -1..1; drawn as the mouth waveform |
| `~/set_person_name` | `std_msgs/String` | default | Name the person in view (`"Bryan"`), or by id (`"person_3=Bryan"`) |
| `~/forget_person` | `std_msgs/String` | default | Forget a person by id or name, or `"all"` |
| `~/say` | `std_msgs/String` | default | Speak the text with the current TTS engine (the voice assistant's replies arrive here) |

**Publications**:

| Topic | Type | Description |
|---|---|---|
| `~/person` | `std_msgs/String` | JSON `{id, name, score, new, box}` each time face ID runs on the tracked person |
| `~/speaking` | `std_msgs/Bool` | True while the robot's own voice is playing (the voice assistant mutes its microphone) |

**Services Provided**:

| Service | Type | Description |
|---|---|---|
| `~/set_mood/neutral` | `std_srvs/Trigger` | Neutral eye expression |
| `~/set_mood/tired` | `std_srvs/Trigger` | Drooping tired eyes |
| `~/set_mood/angry` | `std_srvs/Trigger` | Angled angry brows |
| `~/set_mood/happy` | `std_srvs/Trigger` | Squinting happy eyes |
| `~/set_mood/suspicious` | `std_srvs/Trigger` | One-eyebrow-up suspicious look |
| `~/list_people` | `std_srvs/Trigger` | List learned people (id, name, views, times seen) |
| `~/wave` | `std_srvs/Trigger` | Play the UR wave program now (ignores the cooldown; still refuses while the arm is busy) |
| `~/wink` | `std_srvs/Trigger` | Wink one eye now (`wink_eye`) |

**ROS Parameters**. Defaults below are those of `config/combined_face_hand_defaults.yaml`, which the launch file and the supervisor unit load; where the node's own default (used by a bare `ros2 run`) differs it is noted.

*Vision & Display*

| Parameter | Default | Description |
|---|---|---|
| `image_topic` | `/camera/camera/color/image_raw` | Camera topic. `sensor_msgs/Image`, or `sensor_msgs/CompressedImage` when the name ends in `compressed` (e.g. the VLA camera bridge's `/vla/cameras/scene/image/compressed`) |
| `render_fps` | `30.0` | The face animates on its own timer at this rate; camera frames only steer it, so a stopped camera never freezes the face |
| `camera_timeout` | `3.0` | Seconds without frames before the face drops the tracked person and logs a warning |
| `fullscreen` | `true` | Start fullscreen on the primary monitor. Clicking the face, or pressing F, toggles windowed/fullscreen; Esc leaves fullscreen. Live-settable with `ros2 param set` |
| `window_focus_seconds` | `10.0` | How long after start (and after entering fullscreen) the face keeps asking GNOME to focus its window, twice a second. A window that supervisor opened gets no focus from GNOME's focus-stealing prevention, and an unfocused fullscreen window sits behind the desktop; the face also never minimises itself (GLFW's auto-iconify is off). `0` disables |
| `width` | `1280` | Camera frame width (px) |
| `height` | `720` | Camera frame height (px) |
| `fps` | `30.0` | Expected capture framerate |
| `mirror` | `true` | Mirror the camera image (selfie-style gaze); node default `false` |

*Eye Rendering*

| Parameter | Default | Description |
|---|---|---|
| `face_only` | `true` | Eyes, mouth, face ID and speech only: the MediaPipe hand model is never loaded and the hand panel / TCP relay are off (saves a lot of CPU); node default `false` |
| `hand_in_eyes` | `false` | Draw hand skeleton inside eye window |
| `hand_panel_height` | `1000` | Hand panel pixel height (node default 400) |
| `hand_panel_fraction` | `0.8` | Fraction of screen height for hand panel (node default 0.5) |
| `hand_panel_scale` | `1.0` | Scale factor for hand panel |

*Blink Behavior*

| Parameter | Default | Description |
|---|---|---|
| `blink_min` | `2.5` | Minimum seconds between blinks |
| `blink_max` | `15.5` | Maximum seconds between blinks |
| `blink_duration` | `0.14` | Total blink cycle duration (s); node default 0.18 |
| `blink_closed_hold` | `0.05` | Time held fully closed during blink (s); node default 0.04 |

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

Preview them all in the standalone test with the `G` key. Publish a loudness level and it becomes a procedural travelling waveform whose amplitude, stroke width and glow scale with the level; publish raw audio chunks and the line traces the samples themselves (resampled to 512 points). The level is smoothed with a fast attack and slow release so syllables read clearly, and it decays to idle after `mouth_timeout` seconds of silence. The waveform is redrawn on the face's own `render_fps` timer, so it keeps moving even when the camera stalls.

```bash
# Quick check: make the mouth "talk" at 80% loudness
ros2 topic pub -r 20 /combined_face_hand/mouth_level std_msgs/msg/Float32 "{data: 0.8}"
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
| `greeting_cooldown` | `3600.0` | A recognised person (named or not) is greeted at most once per this many seconds; the time of the last hello is stored with the person, so it survives restarts. `0` greets on every visit. The Folks page shows "last hello" per person and a **Reset hello** button that clears it |
| `wink_eye` | `right` | Someone who comes back inside the cooldown gets a wink instead of a hello: this eye (screen `left` / `right`) closes. `ros2 service call /combined_face_hand/wink std_srvs/srv/Trigger` winks on demand |
| `wink_duration` | `0.7` | The whole wink: close, hold, open (s) |
| `wink_depth` | `0.9` | How far the eye closes: `1.0` shuts it, `0.9` leaves a sliver open |
| `audio_sink` | `C-Media` | Speaker: PulseAudio sink name or fragment. A watchdog keeps it the default sink whenever it is present (the dock adapter enumerates after pulse at boot, so `default.pa`'s `set-default-sink` misses it and the voice lands on the onboard jack) and `aplay` is pinned to it with `PULSE_SINK`; `""` leaves the default alone |
| `audio_sink_check` | `5.0` | Seconds between default-sink checks |
| `speech_lead_silence` | `1.5` | Seconds of lead-in prepended to every utterance so the USB speaker is awake before the first word |
| `speech_lead_level` | `0.01` | The lead-in is a faint 50 Hz tone at this level (about -40 dBFS) because many speakers only wake on a real signal; `0` makes it pure silence |

*UR arm gesture (wave)*

When a person is recognised by name the node also plays a program saved on the UR teach pendant, through the
arm's Dashboard Server (TCP 29999): `load wave.urp`, `play`, then it polls `programState` until the program ends
(stopped at 40 s). It only ever starts programs the pendant already has. Before each play it checks that PolyScope is in
Remote Control, the robot mode is RUNNING (powered, brakes released), the safety status is NORMAL and nothing is running;
while the VLA teleop or eval drive the arm through ur_rtde the wave is skipped with a log line ("arm is busy"). Test it
with `ros2 service call /combined_face_hand/wave std_srvs/srv/Trigger`.

| Parameter | Default | Description |
|---|---|---|
| `ur_wave_enabled` | `true` | Wave when someone is recognised |
| `ur_ip` | `192.168.11.21` | UR controller address |
| `ur_wave_program` | `wave.urp` | Program under `/programs` on the pendant |
| `ur_wave_cooldown` | `20.0` | Minimum seconds between waves |
| `ur_wave_known_only` | `true` | Only for people recognised by name; `false` waves on every hello |
| `ur_wave_source` | `auto` | `auto` plays the recording when the file exists, else the pendant program; or force `recording` / `dashboard` |
| `ur_wave_motion_file` | `~/.robot_face/motions/wave.json` | Joint trajectory written by `scripts/ur_record_motion.py` |
| `ur_wave_return` | `home` | Where the arm goes after the wave: `home` = the joints in `ur_wave_home_file`, `start` = where it was before the wave, `none` |
| `ur_wave_home_file` | `~/.robot_face/motions/home.json,~/rd_ws/VLA/outputs/home_pose.json` | Comma-separated, first existing wins: the wave's own forward home (`scripts/ur_record_motion.py --capture-home`), else the eval/training home the teleop panel's *Set Home* captured; none = fall back to `start` |
| `ur_wave_approach_order` | `""` | Joint stages for home → wave pose (digits = joints 0 base … 5 wrist 3, `>` = next stage, `""` = all at once); only needed for a recording that starts far from home, see `scripts/ur_path_eval.py` |
| `ur_wave_return_order` | `""` | Joint stages for wave pose → home |
| `ur_wave_retrace` | `true` | A take that starts at home but ends elsewhere returns along its own samples, backwards, from the earliest sample within 20 deg of the end pose: the path home is one you demonstrated, never a straight joint ramp |
| `ur_wave_retrace_speedup` | `1.5` | Playback speed of that retrace (1 = as demonstrated) |
| `hand_wiggle_enabled` | `true` | AmazingHand: index, middle and ring wave between open and `hand_wiggle_curl` at `hand_wiggle_hz` while the arm waves, each finger `hand_wiggle_phase` of a period behind the previous one (a wave running index → ring), the thumb held at `hand_thumb_curl`; fingers open again for the way home |
| `hand_host` / `hand_port` / `hand_speed` | `192.168.11.117` / `8766` / `500` | The hand's ESP32 (UDP `J:` lines, the VLA teleop's protocol) and the firmware speed (50..2000). A hand that does not answer the `Q` probe is logged and skipped |
| `hand_wiggle_hz` | `0.5` | Finger ripples per second |
| `hand_wiggle_curl` | `0.333` | How far the fingers close (0 open .. 1 closed): between fully open and 2/3 open |
| `hand_thumb_curl` | `0.0` | The thumb's fixed curl during the wave |
| `hand_wiggle_phase` | `0.25` | Fraction of a period between neighbouring fingers (index leads, middle and ring trail) |
| `ur_wave_power_up` | `true` | Arm IDLE, powered off or in a protective stop: run the same startup as eval/training first (`power on`, `brake release`, protective-stop unlock, safety restart after a fault) |

PolyScope sometimes refuses the dashboard `play` with a bare "Failed to execute" and no reason (seen on 5.21 with
everything green). So the wave is normally a **recording** driven the same way the teleop drives the arm. The easiest
recording is a **freedrive demonstration**: `python3 src/robot_face/scripts/ur_record_motion.py --capture-home
--freedrive --say` on RD releases the arm (teach mode) and the face talks you through two phases: first hold the arm
still in the wave's **forward home** (saved to `~/.robot_face/motions/home.json`, separate from the eval/training home
in `~/rd_ws/VLA/outputs/home_pose.json`; skip the flag to keep the home you have), then guide it through the gesture from
that home back to it. The recording starts at your first movement and ends after 4 s of stillness (a nudge under 15 deg
or a take under 2 s is discarded and it waits again); the arm locks, the trace is smoothed (100 ms) and saved, and the
previous file is kept with a timestamp. Or build it **point by point** with `--keyframes`: where the arm is when the
script starts is point one, every pose you then hold still for 4 s is the next point (the face counts them), and holding
10 s longer at the last point finishes; the motion eases between the points at `--speed` (0.6 rad/s), with the home pose
added before the first and after the last point, and the points are kept in `wave_keyframes.json`. `--release` just
frees the arm so you can place it by hand (it locks again when held still). The wave in use since 2026-11-02 is two
keyframes from the front home, `--regen --repeat 2 --speed 1.0`: hand sweeps 26 cm right, 27 cm left, twice, back to
centre. The replay is **paced by the arm**: a sample is only advanced past once the arm is within 11 deg of it, so a
recording faster than the UR allows (its safety speed limits scale servoJ down) plays at the controller's own pace
instead of tripping the lag abort — at 1.0 rad/s the two swings take 12 s, and asking for more would not make it
faster. While the arm waves the AmazingHand's index, middle and ring fingers ripple between fully open and 2/3 open at
0.5 Hz with the thumb still (`hand_wiggle_*`), and open again for the way home. Alternatively record the pendant's
program: run the script without `--freedrive`, press Play on the pendant once (Local Control), and it samples the
joints over RTDE while `wave.urp` runs; a looping program ("loops forever", PolyScope's default) is cut to one cycle.
The node then replays the JSON through ur_rtde exactly like `rtde_arm.py`'s streamer: servoJ at the
recorded rate (20 ms servo time, 0.1 s lookahead, gain 300), a joint-space ramp at 0.5 rad/s from wherever the arm is to
the first sample, the samples, a ramp to the **home pose** (`ur_wave_return: home`, the same joint-space walk the eval's
HOME key does, so the arm ends every wave where the eval and the teleop panel park it), then servoStop/stopScript. A
per-tick joint step cap (0.2 rad) bounds any bad sample, a protective or emergency stop aborts the motion, and it refuses
when the arm is more than 143 deg from the start pose on any joint, unless it is parked at home (home ↔ wave pose is a
known route, 203 deg on wrist 2).

The two ramps are **staged** so the hand stays closer to the robot's width on the way out and back, and never passes
over the robot. Moving every joint at once swept the fingertips 50 cm from the centreline on the way to the wave pose
and 45 cm on the way home (the base turns 48 deg while wrist 2 swings 200 deg with the arm extended); the body is
27 cm to each side and the wave pose itself holds the hand 43 cm out. `scripts/ur_path_eval.py` runs nominal UR5e
forward kinematics (arm base on the back lidar, yawed so the robot's lateral axis is the arm frame's x) over every 1-4
stage joint ordering, drops every ordering in which any part of the arm passes above the chassis footprint (the first
picks, chosen on lateral reach alone, swung the hand over the top of the robot and were stopped there twice), and
ranks the rest by the hand's lateral peak during the transit: `2345>0>1` (elbow and wrists together, then the base,
then the shoulder dips the hand out into the wave pose) peaks at 36 cm, `015>34>2` home peaks at 35 cm, for about
2.3 s more each way than the straight ramps. The 27 cm body width is not reachable without re-recording the wave
closer to the body. Re-run the script after re-recording the wave or capturing a new home pose; it prints the orders
to put in `ur_wave_approach_order` / `ur_wave_return_order`. From any other start pose the approach moves all joints
at once as before. Every play writes a per-tick trace (commanded, actual and target joints, runtime state, speed
scaling) to `~/.robot_face/motions/last_wave_trace.csv`; a ramp that stops making progress for a second while the
command is ahead (someone holding the arm, an obstacle, a frozen RTDE stream) aborts with servoStop instead of
waiting, and a stopped control script is re-uploaded once. Because it takes the RTDE control interface, it never runs while the teleop or eval hold the arm
(the "arm is busy" check) and they reconnect the way they always do if they come up right after a wave.

After a reboot the UR sits in IDLE with the brakes on, and a protective stop leaves it stuck too. With
`ur_wave_power_up` the wave first runs the arm startup eval_hw and the teleop panel use (a port of VLA's
`ur_power.power_up`, kept in step with it): Remote Control required, `close safety popup` + `restart safety` after a
FAULT/VIOLATION, `close safety popup` + `unlock protective stop` after a protective stop, then `power on` and `brake
release`, waiting for RUNNING. Each auto-clear is logged; like the eval, it releases the brakes without knowing whether
the arm is clear, so keep the workspace clear. Set it `false` to have the wave refuse instead.

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
ros2 topic pub --once /combined_face_hand/say std_msgs/msg/String "{data: 'Hi, I am Robot Disco.'}"
ros2 run robot_face combined_face_hand_node --ros-args -p tts_voice:=bm_george   # at launch
ros2 param set /combined_face_hand tts_voice bm_george                                    # live, no restart
```

These parameters can be changed live with `ros2 param set` and take effect immediately: `tts_voice`, `tts_speed`, `tts_lang`, `greeting_enabled`, `greeting_text`, `greeting_known_text`, `greeting_late_text`, `greeting_wait`, `speech_lead_silence`, `speech_lead_level`, `mouth_smile`, `mouth_smile_happy`, `mouth_gestures`, `mouth_gain`. An unknown voice is rejected with the list of valid ones. Everything else is read once at startup. To make a change permanent, put it in `config/combined_face_hand_defaults.yaml` (used by the launch file) or your own YAML passed as `param_file:=`.

*Face Identification*

| Parameter | Default | Description |
|---|---|---|
| `tracker` | `auto` | How people are found for gaze: `face` runs the SCRFD face detector on every frame (needs the ArcFace back-end; ~6 ms on the Orin GPU, no phantom people in furniture), `pose` is MediaPipe; `auto` picks `face` when ArcFace is active |
| `face_id_enabled` | `true` | Learn to tell people apart |
| `face_id_backend` | `auto` | `arcface` (SCRFD + ArcFace via InsightFace, own venv), `opencv` (YuNet + SFace), or `auto` = ArcFace when its venv exists |
| `face_id_python` | `~/.robot_face/faceid-venv/bin/python` | Python of the InsightFace venv (`provision/setup_faceid.sh`) |
| `face_id_arcface_model` | `buffalo_l` | InsightFace model pack (`buffalo_l` = SCRFD-10GF + ArcFace ResNet50) |
| `face_id_det_size` | `640` | SCRFD input size. With the face tracker the worker sees full frames (cheap on the GPU); the MediaPipe path sends a head crop, where 320 is plenty |
| `face_id_gpu` | `true` | Use the CUDA provider for the worker. On the Jetson this needs NVIDIA's `onnxruntime-gpu` build (installed by `provision/setup_faceid.sh`); detection + embedding is ~29 ms per frame |
| `face_id_models_dir` | `""` | OpenCV back-end models (empty = `share/robot_face/models`) |
| `face_id_gallery` | `~/.robot_face/people.json` | Where learned people are stored; ArcFace uses `people.arcface.json` |
| `face_id_interval` | `3` | Run face ID every N camera frames (in a background thread) |
| `face_thumb_dir` | `~/.robot_face/faces` | One face picture per person, kept for the Folks page (first sighting, then a closer or fresher one) |
| `people_web_enabled` | `true` | Serve the **Folks** page: the people the robot knows, with pictures and editable names |
| `people_web_port` | `8096` | Port of the Folks page (the VLA manager's Folks tab embeds `http://192.168.11.80:8096/`) |
| `face_id_min_period` | `0.5` | Minimum seconds between identification (embedding) passes; detection itself runs on every frame with the face tracker |
| `face_id_threads` | `2` | onnxruntime threads for the ArcFace worker; its default pool spins on every core and was eating five of them |
| `face_id_match_threshold` | `0` | Cosine similarity needed to match; `0` = back-end default (SFace 0.363, ArcFace 0.45) |
| `face_id_auto_enroll` | `true` | Enroll strangers automatically as `person_N` |
| `face_id_min_face_px` | `40` | Ignore faces narrower than this |
| `face_id_unknown_threshold` | `0` | Below this against everyone known counts as a stranger, between it and the match threshold is "unsure"; `0` = back-end default (SFace 0.28, ArcFace 0.30) |
| `face_id_enroll_min_face_px` | `64` | Only learn new people when the face is at least this wide (close enough for a clean embedding) |
| `face_id_enroll_frames` | `6` | Consistent passes needed before a stranger is enrolled |

Two back-ends produce the face embeddings. The default on the robot is **ArcFace**: InsightFace's SCRFD detector plus the ArcFace ResNet50 from the `buffalo_l` pack (512-d embeddings), markedly more robust to angle, distance and lighting than the fallback. It needs numpy 2-incompatible pins, so it lives in its own venv and runs as `faceid_worker.py`, a long-lived process the node streams frames to over a pipe (install with `provision/setup_faceid.sh`). The fallback is OpenCV's own YuNet + SFace (128-d) with the two models bundled in `models/`. Identification runs in a background thread so it never stalls the face animation. The embedding is compared against everyone in the gallery; a cosine similarity above `face_id_match_threshold` is a match. The two embedding spaces are incompatible, so each back-end keeps its own gallery file and people must be taught once per back-end. Scores in the "unsure" band are neither matched nor used to enroll. A clear stranger who is seen consistently for `face_id_enroll_frames` passes, close enough to the camera, is enrolled as `person_N` and the node logs how to name them. Confident matches add fresh views (up to 10 per person), and once someone has been confidently recognised the identity sticks for a couple of seconds through turned heads and distance, learning those harder views too. Naming a person with a name that already exists merges the two records, which is the fix when the same person gets enrolled twice. The gallery is a JSON file and survives restarts.

```bash
# Who does the robot know?
ros2 service call /combined_face_hand/list_people std_srvs/srv/Trigger

# Name the person currently in front of the camera
ros2 topic pub --once /combined_face_hand/set_person_name std_msgs/msg/String "{data: 'Bryan'}"

# Name a specific id, or forget someone
ros2 topic pub --once /combined_face_hand/set_person_name std_msgs/msg/String "{data: 'person_2=Dan'}"

# Same person enrolled twice? Give the duplicate the existing name and they merge
ros2 topic pub --once /combined_face_hand/set_person_name std_msgs/msg/String "{data: 'person_2=Bryan'}"
ros2 topic pub --once /combined_face_hand/forget_person std_msgs/msg/String "{data: 'person_2'}"

# Watch recognitions live
ros2 topic echo /combined_face_hand/person
```

Once a person is named, the greeting becomes `greeting_known_text` with their name. When a new person appears the node waits up to `greeting_wait` seconds for face ID so it can greet them by name. People are usually spotted by the pose tracker from across the room, before the face is large enough to identify, so if the wait runs out it says the generic greeting and then, if the name is recognised within `greeting_name_window` seconds, follows up with `greeting_late_text`. Each recognised person is greeted at most once per `greeting_cooldown` (an hour by default): a known person who comes back sooner gets a wink instead: no hello when the face is identified within the wait, no name follow-up after a generic hello, and no wave.

Speech goes through the engine chain above (Kokoro, then Coqui `tts_models/en/ljspeech/vits` if the `TTS` package is installed, then `espeak`). Whichever renders the phrase, it becomes a WAV, is played with `aplay` on the USB sink, and the WAV samples are streamed to the mouth waveform in 40 ms chunks timed to playback, so the mouth moves with the words coming out of the speaker.

*Hand Relay (TCP to AmazingHand, legacy)*

Off on the robot (`face_only`). When enabled, MediaPipe hand landmarks become servo angles sent as `J:` lines over TCP; the same ESP32 that takes the wave's UDP wiggle serves the TCP line protocol on :8765.

| Parameter | Default | Description |
|---|---|---|
| `relay_hand` | `false` | Enable TCP hand control streaming |
| `hand_hotkey` | `ctrl+shift+h` | Keyboard chord to toggle hand streaming |
| `hand_relay_host` | `192.168.11.117` | The hand's ESP32 (TCP line protocol) |
| `hand_relay_port` | `8765` | TCP port of the line protocol (UDP :8766 is the wiggle's) |
| `hand_rate` | `20.0` | Command frequency (Hz) |
| `hand_relay_speed` | `1000` | Servo speed sent with each relay command (50–2000) |

*Video Recording*

| Parameter | Default | Description |
|---|---|---|
| `save_raw_video` | `""` | Path for raw camera recording (empty = disabled; the file grows without bound, so leave it off in normal use). Node default `/tmp/raw_camera.mp4` |
| `save_video` | `/tmp/combined_output.mp4` | Path for composite output recording (empty = disabled) |

### OpenGL Eye Renderer (`gl_eye_renderer.py`)

Renders the face with a GLSL fragment shader in a GLFW window titled **"Robot Disco"**. All geometry is computed per-pixel on the GPU in a fixed **1080×1900 design canvas**; in fullscreen (the default) the canvas is scaled to fit the monitor and letterboxed, so the layout is identical on RD's portrait 1080×1920 display (10 px bars) or a landscape screen. A click on the face or the F key toggles between fullscreen and a window, Esc leaves fullscreen. Started by supervisor the window would not be focused (GNOME only focuses windows the user opened) and GLFW would minimise the unfocused fullscreen window: the renderer turns that auto-iconify off and asks for focus for `window_focus_seconds` after start, logging `window: focused after N request(s)`.

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

### Hand Processing (`hand_processing.py`)

Legacy relay (off on the robot, see `face_only`): converts the MediaPipe 21-landmark hand skeleton into 8 servo joint angles sent over TCP to the AmazingHand's ESP32 (`hand_relay_host:hand_relay_port`, 192.168.11.117:8765). The robot normally talks to the hand over UDP :8766 (the VLA teleop and the wave's wiggle, `hand_wiggle.py`).

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

### Face Tracking (`face_utils.py`)

- Uses MediaPipe Pose (model_complexity=0 for speed) to derive a face bounding box from upper-body landmarks filtered by visibility > 0.4
- Hysteresis: tracked face must move more than `0.8 × box_dimension + 40px` to switch to a new face candidate
- Stick time: face considered lost after 1 s of no detection
- Idle behavior: after 4 s with nobody in view, a slow figure-of-eight gaze sweep (7 s period, ±33% of the frame horizontally, ±17% vertically) with a smaller, faster wander layered on top so the eyes don't look like they're on rails
- A newly tracked person triggers the happy mood for `greeting_happy_seconds`, the greeting, and face identification

### Launch

```bash
ros2 launch robot_face combined_face_hand.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  hand_in_eyes:=true \
  mirror:=false
```

### Running Without ROS (standalone renderer test)

```bash
python3 src/robot_face/test_eye_renderer.py
```

Controls: `SPACE` next preset, `WASD`/arrows gaze, `B` blink, `0-4` mood, `M` simulated talking (procedural mouth), `N` simulated talking (synthesised audio samples), `G` cycle through every mouth gesture, `Q` quit. Presets include `talking`, `whisper`, and `happy + talking`. Requires `moderngl`, `glfw`, `numpy` and `opencv-python`.

### Deploying to Robot Disco (RD)

The robot's checkout at `/home/rd/rd_ws` (really `/mnt/data/rd_ws` on the NVMe) is **not** kept in sync through git. `provision/deploy_to_rd.sh` is the deploy path for this package and `robot_voice`: it rsyncs the two packages, their units and the provision scripts, builds them on RD with `--symlink-install` and restarts the units (see [Deploying to the robot](Provisioning-and-Deploy#deploying-to-the-robot-provisiondeploy_to_rdsh)). By hand, the same thing is:

```bash
# From this repo, on your workstation (SSH host "RD" = 192.168.11.80, user rd)
rsync -av --exclude __pycache__ src/robot_face/ RD:/home/rd/rd_ws/src/robot_face/
ssh RD 'cd ~/rd_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select robot_face --symlink-install && sudo -n supervisorctl restart robot_face'
```

Notes for RD:
- The workspace is a `--symlink-install` build, so edited Python, launch and config files take effect on the next restart; a **new** file needs the `colcon build` above to get its symlink.
- The face runs as the supervisor unit `robot_face` (`services/robot_face.conf`): the unit carries `DISPLAY=:0`, `XAUTHORITY=/run/user/1000/gdm/Xauthority` and `XDG_RUNTIME_DIR=/run/user/1000`, because RD's GDM autologin session is X11 and glfw needs its Xauthority file. `/home/rd/start_face.sh` is the manual fallback (it picks the Wayland or X11 exports from the session and logs to `~/rd_ws/log/robot_face-manual.log`); it starts the node with `ros2 run`, so its topics live under `/robot_face/` instead of `/combined_face_hand/` and the voice assistant cannot reach it.
- Speech is Kokoro (`provision/setup_kokoro.sh`, offline-capable with a wheel folder plus the two model files); RD has no internet, so a Coqui model would have to be copied into `~/.local/share/tts/` by hand.
- The Jetson's OpenCV is NVIDIA's 4.8 build; `setup.py` deliberately lists only `setuptools` in `install_requires`, so no pip wheel can shadow it (see `provision/README.md` on the two-OpenCV trap).
- When stopping the node over SSH, avoid a `pkill` pattern that appears in your own command line (`pkill -f "lib/robot_face/[c]ombined"` works).
- Learned people live in `/home/rd/.robot_face/people.arcface.json` (ArcFace) and `people.json` (OpenCV back-end); their pictures in `~/.robot_face/faces/`, the wave and home pose in `~/.robot_face/motions/`.

## Run robot_face

```bash
# Minimal run against RealSense
ros2 launch robot_face combined_face_hand.launch.py \
  image_topic:=/camera/camera/color/image_raw

# With the hand skeleton overlay and the legacy TCP hand relay (needs face_only:=false)
ros2 launch robot_face combined_face_hand.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  hand_in_eyes:=true \
  --ros-args -p face_only:=false -p relay_hand:=true -p hand_relay_host:=192.168.11.117

# On the robot
sudo supervisorctl start robot_face
```

> **Node name:** the launch file (and so the supervisor unit) names the node `combined_face_hand`, so its topics and services are under `/combined_face_hand/...`; the voice assistant is wired to those names. When started with `ros2 run robot_face combined_face_hand_node` (as `start_face.sh` does) the node keeps its default name and everything is under `/robot_face/...`. The examples here use `/combined_face_hand`.

## Control eye mood at runtime

```bash
ros2 service call /combined_face_hand/set_mood/happy      std_srvs/srv/Trigger
ros2 service call /combined_face_hand/set_mood/angry      std_srvs/srv/Trigger
ros2 service call /combined_face_hand/set_mood/tired      std_srvs/srv/Trigger
ros2 service call /combined_face_hand/set_mood/suspicious std_srvs/srv/Trigger
ros2 service call /combined_face_hand/set_mood/neutral    std_srvs/srv/Trigger
```

## Teach the robot who you are

```bash
# Stand in front of the camera for a couple of seconds; the log reports
# "New person enrolled as person_1". Then, while still in view:
ros2 topic pub --once /combined_face_hand/set_person_name std_msgs/msg/String "{data: 'Bryan'}"

# Check, rename by id, forget, or watch live recognitions
ros2 service call /combined_face_hand/list_people std_srvs/srv/Trigger
ros2 topic pub --once /combined_face_hand/set_person_name std_msgs/msg/String "{data: 'person_2=Dan'}"
ros2 topic pub --once /combined_face_hand/forget_person std_msgs/msg/String "{data: 'person_2'}"
ros2 topic echo /combined_face_hand/person
```

Named people are greeted by name ("hello Bryan"). If the face is only recognised after the generic hello, the robot follows up with "oh, hi Bryan".

**Folks page.** The face node serves `http://<robot>:8096/` (and the VLA manager shows it as the **Folks** tab): one card per known person with their face picture, an editable name (Enter or click Save; giving a card a name that already exists merges the two records), first/last seen, how often they were recognised, a Forget button, and a live "in view" badge on whoever is in front of the camera. Each card also shows the last hello and a **Reset hello** button (greet them again before the cooldown ends) and a **snap** button (replace the picture with the next confirmed sighting). JSON under `/api/people`, `/api/live`, `GET /thumb/<id>.jpg`, `POST /api/people/<id>/name`, `POST /api/people/<id>/forget`, `POST /api/people/<id>/snap`, `POST /api/people/<id>/reset_greeting`, `POST /api/forget_all`. The same server carries the speaker: `GET /api/volume` → `{sink, volume (0-100), muted}` and `POST /api/volume {volume}` / `{muted}` act on the `audio_sink` sink through pactl (the VLA manager's header slider).

<!-- No screenshot on purpose: the page shows people's faces and names. -->

## Drive the mouth from your own audio

```bash
# Loudness only (0..1): procedural waveform
ros2 topic pub -r 20 /combined_face_hand/mouth_level std_msgs/msg/Float32 "{data: 0.8}"

# Raw samples in -1..1: the waveform traces the audio itself
ros2 topic pub --once /combined_face_hand/mouth_audio std_msgs/msg/Float32MultiArray "{data: [0.0, 0.5, 0.9, 0.5, 0.0, -0.5, -0.9, -0.5]}"
```
