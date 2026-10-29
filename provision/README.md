# Provisioning a Robot Disco robot PC

Everything needed to turn a fresh **Ubuntu 22.04** install into the robot PC ("RD", `rd@192.168.11.80`,
ROS 2 Humble). The live robot is *not* git-synced and carries state that is not in this repo, so there
are two scripts:

| Script | Runs on | Purpose |
|---|---|---|
| `capture_rd_state.sh` | your workstation | Snapshot a working robot PC: installed packages, local edits, maps, learned faces, TTS cache, foxglove image. Doubles as a backup. |
| `provision.sh` | the new machine | Idempotent, phased installer. Optionally restores from a snapshot. |
| `jetson/provision_jetson.sh` | a Jetson Orin | Same phases for JetPack 6.x plus CUDA torch and the local VLA inference server. |
| `gpu/provision_gpu.sh` | the GPU box (Ubuntu 24.04, RTX 4090) | Training box + VLA web manager: venv, supervisor units, data from RD, optional Wi-Fi/desktop apps. See `gpu/README.md`. |

Manifests the installer reads: `apt-packages.txt`, `ros-packages.txt`, `pip-robot-face.txt`,
`pip-robot-face-tts.txt`, `pip-vla.txt`, `udev/99-robot-disco.rules`, `netplan/01-robot-disco.yaml`,
`start_face.sh`, `setup_kokoro.sh` (Kokoro-82M voice; offline wheel/model instructions in its header). The supervisor units come from `../services/`, the VLA units from `VLA/services/rd/`.

## 0. Before the old machine dies: take a snapshot

```bash
provision/capture_rd_state.sh            # -> provision/snapshot-YYYYMMDD/  (needs ssh host "RD")
```

Do this now, while the current RD still works. It is the only copy of the robot's uncommitted edits
(`rd_ws.local.diff`), the `zone_one` map, `people.json`, the Coqui voice cache and the foxglove image.
Commit anything from `rd_ws.local.diff` you want to keep; the rest can be re-applied with `git apply`.

## 1. Install Ubuntu 22.04 Desktop

- Create user **`rd`** (the supervisor units, services and paths all assume `/home/rd`).
- Hostname `robotdisco`. Desktop (GNOME) is required: the face is an OpenGL window on the robot's screen.
- Enable OpenSSH during install or `sudo apt install openssh-server` afterwards.
- **Give the machine internet for provisioning** (any DHCP network, or a USB phone tether). The robot
  subnet `192.168.11.x` is isolated; moving onto it is the last step. If the box must already sit on
  the robot subnet, see *Provisioning without internet* below.

## 2. Run the installer

```bash
sudo apt install -y git
git clone https://github.com/r1b4z01d/rd_ws.git ~/rd_ws
~/rd_ws/provision/provision.sh all                      # ~30-60 min, mostly apt + colcon
sudo reboot                                             # groups + GNOME autologin take effect
```

Restoring from a snapshot copied to the new box (e.g. `scp -r provision/snapshot-20261008 rd@<ip>:~/`):

```bash
SNAPSHOT=~/snapshot-20261008 ~/rd_ws/provision/provision.sh all
cd ~/rd_ws && git apply ~/snapshot-20261008/rd_ws.local.diff   # the robot-only edits, if wanted
```

`all` runs: `preflight base ros groups udev sysctl workspace face services display foxglove`.
Each phase can be run alone and re-run safely, e.g. `provision.sh workspace` after fixing a build error.

| Phase | What it does |
|---|---|
| `base` | `apt-packages.txt`; enables ssh, supervisor, docker; hostname; NTP; `/etc/hosts` entries for ur5e/amazinghand/gpu; `AUTHORIZED_KEY` |
| `ros` | ROS 2 apt repo, `ros-packages.txt`, rosdep, sources Humble + the workspace in `~/.bashrc`, realsense udev rules |
| `groups` | adds `rd` to dialout, video, input, plugdev, audio, docker, render |
| `udev` | `udev/99-robot-disco.rules` (SpaceMouse hidraw, RealSense, webcams, gamepads) |
| `sysctl` | UDP receive buffers for the multicast camera stream (same file VLA's installer writes) |
| `workspace` | clones/updates `~/rd_ws`, inits the `src/` submodules over https, `rosdep install`, `colcon build --symlink-install` |
| `face` | `pip-robot-face.txt` into the system python (`--user`); installs Kokoro-82M TTS into its own venv via `setup_kokoro.sh` (`WITH_KOKORO=0` to skip, `KOKORO_WHEELS=dir` for offline); `WITH_TTS=1` adds Coqui; restores Kokoro models, Coqui cache + `people.json` from the snapshot |
| `services` | supervisor web UI on `:9999`, `Scripts/install_services.sh` (supervisor units) and `~/start_face.sh` |
| `display` | GDM autologin for `rd`, no screen blanking or lock |
| `foxglove` | `docker load` from `FOXGLOVE_IMAGE_TAR` or the snapshot, else builds `foxglove_studio/Dockerfile` (slow, needs internet) |
| `vla` | *(not in `all`)* `~/VLA/.venv` from the snapshot freeze or `pip-vla.txt`, `run.sh`, `install_services.sh rd` (camera service + ROS bridge, and the manual-start `vla_teleop` / `vla_eval` units with their `~/VLA/config/*.env` argument files) |
| `network` | *(not in `all`)* static `192.168.11.80/24`, no gateway, from `netplan/01-robot-disco.yaml` |

Environment knobs are listed at the top of `provision.sh` (`RD_USER`, `RD_WS_REF`, `SNAPSHOT`,
`WITH_TTS`, `FOXGLOVE_IMAGE_TAR`, `PROXY`, `AUTHORIZED_KEY`, `NET_IFACE`, `HOSTNAME_NEW`).

## 3. VLA (optional, only if this box also records/evaluates with the arm)

The VLA tree is not a git checkout on the robot; it is pushed from the workstation (see
`VLA/wiki/Sync-and-Layout.md`). Then build the venv and install the camera services:

```bash
# workstation, from the VLA repo
tar czf - --exclude=__pycache__ --exclude=.venv --exclude=outputs ur5e_lerobot scripts services webui \
  | ssh rd@<new-rd> 'mkdir -p ~/VLA && tar xzf - -C ~/VLA'
# new robot PC
SNAPSHOT=~/snapshot-20261008 ~/rd_ws/provision/provision.sh vla
```

## 4. Move onto the robot subnet (last)

```bash
NET_IFACE=enp3s0 ~/rd_ws/provision/provision.sh network     # NET_IFACE optional; auto-detects first en*/eth*
```

From the workstation, `ssh RD` should now work (the `RD` host entry in `~/.ssh/config` points at
`192.168.11.80` with `~/.ssh/alpha_id_rsa`; pass that public key as `AUTHORIZED_KEY` in step 2 or
`ssh-copy-id -i ~/.ssh/alpha_id_rsa.pub rd@<ip>` before switching).

## 5. Verify

```bash
sudo supervisorctl status            # foxglove bridge/studio + collision monitor should be RUNNING
ls -l /dev/ttyACM0 /dev/ttyUSB0      # RoboClaw, RPLiDAR (dialout)
rs-enumerate-devices | head          # RealSense seen
~/start_face.sh                      # face window appears on the robot's screen
sudo supervisorctl start robot_disco && ros2 topic list | head
```

Open Foxglove at `http://192.168.11.80:8080` and check `/scan`, `/odom`, `/camera/camera/color/image_raw`.

## Jetson Orin as the robot PC (`jetson/`)

The Intel box is being replaced by a **Jetson Orin dev kit** so the VLA inference server runs on the
robot's own GPU. `jetson/provision_jetson.sh` is a separate entrypoint that sources `provision.sh` and
overrides only what differs on JetPack 6.x (Ubuntu 22.04, aarch64):

| Phase | Jetson difference |
|---|---|
| `preflight` | requires aarch64, `/etc/nv_tegra_release`, Ubuntu 22.04; prints L4T + CUDA versions |
| `base` | adds `jetson/apt-packages-jetson.txt` (boost/cmake/openblas for the source builds, `nvidia-jetpack`) |
| `jetson` *(new)* | `nvpmodel -m 0` (MAXN), a `jetson-clocks.service`, nvidia docker runtime as default, `jtop` |
| `data` *(new)* | optional second drive (the 512 GB NVMe, ext4 label `data`): fstab `nofail` + mount at `/mnt/data`, then `~/VLA`, `~/rd_ws`, `~/.cache/huggingface` and `~/.cache/robot_descriptions` become **symlinks** onto it (an existing tree is rsync-copied, checksum-verified and removed; refused while something runs from it), plus a `supervisor.service.d` drop-in so the units wait for the mount. No drive → no-op. |
| `vla` | cuSPARSELt, then **torch 2.8.0 / torchvision 0.23.0 from `https://pypi.jetson-ai-lab.io/jp6/cu126`** (CUDA builds; PyPI torch has no Jetson CUDA), then `jetson/pip-vla-jetson.txt` = `lerobot[smolvla]==0.4.4` + the rest. `ur_rtde` and `moderngl` compile from source (no arm64 wheels). Asserts `torch.cuda.is_available()`. |
| `infer` *(new)* | `outputs/serve_ckpt` symlink (`SERVE_CKPT=`), installs `VLA/services/rd/vla_infer_server.conf` via `install_services.sh rd`; the same install drops `vla_teleop` / `vla_eval` (manual start: `sudo supervisorctl start vla_teleop`, arguments in `~/VLA/config/teleop.env` / `eval.env`) |
| `foxglove` | always an arm64 build; refuses to trust an x86 tarball |
| `network` | netplan renderer `NetworkManager` (JetPack desktop default) |

```bash
# flash with SDK Manager: JetPack 6.x + Jetson runtime components, user rd
git clone https://github.com/r1b4z01d/rd_ws.git ~/rd_ws
~/rd_ws/provision/jetson/provision_jetson.sh all        # preflight base jetson data ros ... foxglove
sudo reboot
# copy ~/VLA over (section 3), then
SNAPSHOT=~/snapshot-20261008 ~/rd_ws/provision/jetson/provision_jetson.sh vla infer
~/rd_ws/provision/jetson/provision_jetson.sh network
```

Driving with the local server (same split as before, just over loopback, so `--prefetch --rtc` and
`--n-action-steps` keep living in the unit file):

```bash
rsync -a gpu:VLA/outputs/train/<run>/checkpoints/last/ ~/VLA/outputs/train/<run>/checkpoints/last/
ln -sfn ~/VLA/outputs/train/<run>/checkpoints/last/pretrained_model ~/VLA/outputs/serve_ckpt
sudo supervisorctl start vla_infer_server
~/VLA/run.sh scripts/eval_hw.py --remote 127.0.0.1:8777 --cameras udp --fps 12 --video
```

Jetson-specific caveats:
- The torch pins are deliberate: lerobot 0.4.4 wants `torch<2.11`, and the 2.9+ wheels on the Jetson index
  additionally need cuDSS. Override with `JETSON_TORCH=` / `JETSON_TORCHVISION=` if the index moves on.
- An x86 snapshot's `vla-venv-freeze.txt` is reused minus torch/CUDA/x86-only lines; a few pins may still
  fail on aarch64, which the script reports and then falls back to the loose list.
- Coqui TTS (`WITH_TTS=1`) is untested on aarch64; the face falls back to espeak without it.
- `mediapipe` ships arm64 wheels, `moderngl` and `ur_rtde` build from source (several minutes), `glfw`
  uses the apt `libglfw3`.
- No AMD64-only assumptions remain in `services/*.conf`; the Foxglove image is the only thing that
  must be rebuilt for arm64.

Verified on an AGX Orin dev kit (JetPack 6.2, L4T r36.5.2, CUDA 12.6) on 2026-10-10; the fixes that
run needed are now in the scripts:
- JetPack images ship Docker CE, so `docker.io` is only installed when no docker is present.
- `robot_disco`'s CMake installs a `worlds/` directory git never tracked; `worlds/.gitkeep` fixes the
  symlink-install build.
- ROS setup files trip `set -u`; the installer relaxes it around `source`.
- `AprilTag_Detection`'s upstream repo is gone; submodules are now fetched one by one and a dead one
  is skipped (it is a demo package, nothing launches it).
- Kokoro TTS: onnxruntime ≥ 1.20 aborts on the Orin (cpuinfo assertion); `setup_kokoro.sh` pins
  1.18.1 on aarch64.
- NVIDIA's torch wheel is built against NumPy 1.x, but lerobot's rerun-sdk declares numpy 2, so the
  venv is installed with numpy 2 and then downgraded to `numpy<2` as a post-step (extensions built
  against numpy 2 run on 1.26; only `pip check` complains). `opencv-python<4.13` matches lerobot's
  headless pin.
- cuDNN 9, cuBLAS and the CUDA toolkit come with the flash; only cuSPARSELt was missing, installed from
  NVIDIA's redist tarball. `nvidia-jetpack` is deliberately not installed (several GB on the 64 GB eMMC).
- MAXN cannot be switched live (nvpmodel asks for a reboot), so the boot default in
  `/etc/nvpmodel.conf` is set instead and takes effect at the post-provisioning reboot.
- **USB layout matters.** The dev kit's four USB-A ports sit behind two internal hubs, each with its
  own 480 Mbit/s USB 2.0 budget; an external hub puts everything behind one. Two of the wide-FOV
  webcams on the same hub cannot stream together at any resolution (`Not enough bandwidth for
  altsetting` in dmesg; the uvcvideo bandwidth quirk does not help). Put the scene and wrist cameras
  on different root ports, and give the RealSense a USB 3 port + cable (on USB 2.1 it reports
  "Reduced performance" and limits depth).
- `ur_description` 2.13 (apt) emits a ros2_control block from `ur_macro.xacro` by default; the base
  description passes `generate_ros2_control_tag="false"` so the base controller_manager does not try
  to open the arm (the `robot_disco_ur5e_driver` service owns it).
- `git submodule update --init` can leave a submodule directory empty while reporting it checked out
  (seen after a partial run); `--force` restores the files. `provision.sh workspace` warns on empty dirs.
- The 8BitDo Ultimate 2C gamepad (2.4 GHz receiver) in X-input mode shows up as `2dc8:310a` with
  an Xbox 360 class interface, which xpad on kernels before 6.10 does not list; a udev rule in
  `udev/99-robot-disco.rules` registers the ID with xpad so `/dev/input/js0` appears for joy_node.
  The controller must be in X-input mode (keyboard/mouse mode only yields a Keyboard + Mouse device).
- `foxglove_bridge` comes from `src/foxglove_bridge` (C++ 0.8.5 submodule), not apt: the apt 3.x
  Rust bridge only accepts the `foxglove.sdk.v1` subprotocol, and the vendored Studio 1.87 speaks
  `foxglove.websocket.v1`, so every browser connection was dropped with "handshake failed".
- **Clock.** The AGX Orin dev kit has no battery-backed RTC: after a power cut it boots in 1970 and
  stays there until chrony reaches a server through the subnet gateway. Everything TLS then fails
  ("certificate is not yet valid": pip, curl, HF). Seen 2026-10-11 after a network outage. If the
  uplink is down, set it by hand (`sudo date -u -s "<UTC from a good clock>"`) and check
  `timedatectl` says synchronized once the link is back.
- Stream Deck: the `streamdeck` pip package uses hidapi's libusb backend, so it needs
  `libhidapi-libusb0` (apt list) and a udev rule on the USB device node (udev file), not just hidraw.
- The UR5e in the base description is animated by `robot_disco/scripts/ur_joint_state_publisher.py`
  (read-only RTDE poll into `/joint_states`, launched by `launch_robot.launch.py`); it needs `ur_rtde`
  in the system python (`pip-robot-face.txt`).
- A SmolVLA checkpoint needs its base VLM (`vlm_model_name` in the checkpoint's `config.json`,
  HuggingFaceTB/SmolVLM2-500M-Video-Instruct) in `~/.cache/huggingface/hub`; the unit runs offline.
  `provision_jetson.sh infer` downloads it while the box still has internet; on the robot subnet copy
  the `models--HuggingFaceTB--*` folder from the GPU box instead.

## Provisioning without internet (box already on the robot subnet)

`apt`, `pip` and `git` can be tunnelled through the workstation over SSH. On the workstation:

```bash
ssh -R 1080 rd@192.168.11.80          # OpenSSH >= 7.6: remote dynamic (SOCKS) forward
```

In that session on the robot PC:

```bash
PROXY=socks5h://127.0.0.1:1080 ~/rd_ws/provision/provision.sh all
```

`preflight` writes the apt proxy config and exports `ALL_PROXY`/`PIP_PROXY`; `network` removes the
apt config again. The docker daemon does **not** use it, so bring the foxglove image as a tarball
(from the snapshot, or `docker build -t robotdisco/foxglove foxglove_studio/ && docker save ...` on a
machine with internet) and pass `FOXGLOVE_IMAGE_TAR=`.

## DDS transport

Every ROS unit (and rd's shell) sets `FASTRTPS_DEFAULT_PROFILES_FILE=~/rd_ws/services/fastdds_no_shm.xml`:
Fast DDS over UDPv4 only, no shared-memory transport. With the default SHM+UDP transport, a handful of
unit restarts in one boot left stale `/dev/shm/fastrtps_*` segments and data silently stopped flowing
between some processes (Nav2 lost the map frame, AMCL ignored the initial pose; 2026-10-27). If it ever
happens again anyway: stop all ROS units, `rm /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`, start them
in order (base, localization, navigation, the rest), re-seed `/initialpose`.

## Known gaps / things to confirm against the snapshot

- The vendored `foxglove_studio/` tree was missing its `packages/log` workspace because the top-level
  `.gitignore` ignores any `log/` directory; it is restored (from Lichtblick's `1.86.0-fork-dev`, the
  Studio 1.86 fork point, pinned to this repo's yarn.lock) and un-ignored. The Dockerfile uses a
  heredoc, so it needs BuildKit: on the Mac that is colima + the `docker-buildx` brew plugin
  (`docker buildx build --platform linux/arm64 --load ...`, ~7 min on an M2).
- The Foxglove image is tagged `robotdisco/foxglove` (docker rejects uppercase repository names; the
  unit used to say `RobotDisco/foxglove`, which docker parsed as a registry host). Build it with
  `docker build --platform linux/arm64 -t robotdisco/foxglove foxglove_studio/` on an arm64 Mac
  (colima) and ship it with `docker save robotdisco/foxglove | ssh RD docker load`, or run
  `provision_jetson.sh foxglove` on the robot.
- The RealSense **T265** is unsupported by librealsense ≥ 2.54 (what the Humble apt wrapper ships).
  If T265 odometry is actually used, pin `ros-humble-librealsense2`/`realsense2-camera` to a 2.53.x
  build or build from source.
- `start_face.sh` here is a reconstruction of the one living on RD; the snapshot keeps the original in
  `inventory/start_face.sh`, diff them once.
- Python pins for `robot_face` (`mediapipe`, `moderngl`, `glfw`) and the VLA venv are unpinned in the
  manifests on purpose; the snapshot freezes are the authoritative versions.
- The AmazingHand runs on its own host (`192.168.11.117` / `192.168.1.194`); nothing from
  `AmazingHand-main/` is installed on the robot PC.

## Disk layout on the Jetson (since 2026-10-26)

The 64 GB eMMC is the system disk and was filling up, so a 512 GB NVMe (ext4, label `data`) is mounted
at `/mnt/data` and `provision_jetson.sh data` keeps the big trees there behind symlinks:

| in the home | on the drive |
|---|---|
| `~/VLA` | `/mnt/data/VLA` |
| `~/rd_ws` | `/mnt/data/rd_ws` |
| `~/.cache/huggingface` (base VLM, LeRobot's Arrow copies) | `/mnt/data/cache/huggingface` |
| `~/.cache/robot_descriptions` (robot meshes) | `/mnt/data/cache/robot_descriptions` |

Everything in this kit, the supervisor confs, the venv shebangs and the workstation's rsync targets keep
using `/home/rd/...`, which resolves through the links. The fstab entry is `nofail` (the robot boots
without the drive), so the phase also installs `/etc/systemd/system/supervisor.service.d/nvme-data.conf`
(`RequiresMountsFor=/mnt/data`): without it supervisor's autostart units would start into empty symlink
targets. The phase is idempotent; on a box that already has the drive it only verifies. To migrate a
live box, stop the units that run from the trees first (`vla_cameras`, `vla_camera_ros_bridge`,
`robot_disco_*`), as the phase refuses to move a tree that is in use. `DATA_LABEL` / `DATA_MOUNT`
override the defaults.

## Robot web pages (route editor, port 8092)

`Scripts/route_editor.py` runs as the supervisor unit `robot_disco_route_editor` (autostart) and serves,
with no internet needed on the robot:

| URL | what |
|---|---|
| `http://192.168.11.80:8092/` | route editor: map + 1 m grid, live robot pose, click/drag waypoints, pause + post_actions per waypoint, undo, check / centre / densify, save for `path.py` |
| `…:8092/?view=map` | 2D map view: filtered RPLiDAR scans, Velodyne top-down, global/local costmap overlays (the local one placed through map→odom) |
| `…:8092/3d` | 3D view replicating Foxglove's 3D panel: map plane, costmaps, Velodyne / RPLiDAR clouds, the URDF robot model with live joints (meshes served from the package share dirs), TF frames, nav2 plans / goal / footprint / collision polygons / AMCL particles, the route, follow-robot camera, click tools publishing `/goal_pose` and `/initialpose` |

**Developing without the robot:** `cd rd_ws/Scripts && python3 route_editor.py --sim` serves the same
pages on a laptop with a simulated robot driving the route (`--sim-route NAME`): scans and a Velodyne
cloud raycast against the map, costmaps, TF, joints, plans, goal/initial-pose tools, the URDF
(`sim/robot_sim.urdf` primitives, or the real model after `sim/fetch_assets.sh` with the robot on).
Needs only numpy, Pillow and PyYAML. The manager follows with
`python -m webui.server --port 8081 --route-url http://127.0.0.1:8092/`.

The VLA manager embeds the last two as its Map and 3D tabs. three.js 0.182 and urdf-loader 0.12.6 are
vendored under `Scripts/static/vendor/` (licenses there); upgrade by copying the same files from a
`node_modules` tree.
