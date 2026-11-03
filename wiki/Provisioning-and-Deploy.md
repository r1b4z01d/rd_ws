# Provisioning and deploying

`provision/` rebuilds a robot PC from a fresh install, in phases that can be re-run: apt + ROS 2 Humble packages, udev rules, groups, the workspace clone and build (submodules one at a time, `provision/patches/` applied), `robot_face` python deps plus the face-ID / voice / Kokoro venvs, the PulseAudio output policy, the supervisor units, the local language model (Ollama under supervisor), GDM autologin, the Foxglove image, the VLA venv and units, and finally the static `192.168.11.80` address. The robot has been a **Jetson AGX Orin** since 2026-10-10, so the main path is `provision/jetson/provision_jetson.sh`: it sources `provision.sh` and adds the Jetson phases (MAXN power mode, the `data` phase that puts `~/rd_ws` and the caches on the 512 GB NVMe at `/mnt/data` behind symlinks, CUDA torch wheels for the VLA venv, the local inference server, `vlamove`/`vlacheck` for a robot set up before the VLA submodule). `provision.sh` on its own is the Intel/amd64 variant.

```bash
provision/capture_rd_state.sh                  # workstation: snapshot/backup the live robot first
git clone https://github.com/r1b4z01d/rd_ws.git ~/rd_ws
~/rd_ws/provision/jetson/provision_jetson.sh all   # new Jetson, as user rd, with internet; then reboot
provision/deploy_to_rd.sh --vla                    # workstation: push the VLA tree (the robot cannot fetch the submodule)
~/rd_ws/provision/jetson/provision_jetson.sh vla infer network
```

See [provision/README.md](https://github.com/r1b4z01d/rd_ws/blob/main/provision/README.md) for the phases and their knobs, the snapshot/restore flow, how to provision a box that is already on the internet-less robot subnet, the GPU box kit (`provision/gpu/`), the NVMe layout and the known Jetson traps (two OpenCVs, the gamepad receiver, DDS shared memory).

The robot is **not git-synced** (no internet, and its checkout carries no `.git` history of use): `provision/deploy_to_rd.sh` is how code gets there, run from this repo on a machine with the ssh host `RD` (192.168.11.80, user `rd`).

```bash
provision/deploy_to_rd.sh            # robot_face + robot_voice sources, their units, Scripts/path.py, the UR joint-state publisher,
                                     # the provision scripts and this README → build both packages (--symlink-install) → restart robot_face,
                                     # (re)start robot_voice → tail both logs and check the API key
provision/deploy_to_rd.sh --setup    # first time: also run setup_faceid.sh / setup_voice.sh / setup_kokoro.sh on RD
provision/deploy_to_rd.sh --vla      # also push the VLA submodule tree to RD:~/rd_ws/VLA and reinstall its units
```

The VLA push is an rsync without `--delete` that skips the robot's own `.venv`, `outputs/` (datasets, checkpoints), `log/` and `config/*.env`, then runs `VLA/scripts/install_services.sh rd`, which restarts a running `vla_cameras` / `vla_camera_ros_bridge` only if its unit file changed. It refuses while the robot still has the pre-submodule layout (`~/VLA` beside `~/rd_ws`): move that first with `provision_jetson.sh vlamove`. Anything else (route files, `robot_disco` changes, maps) is rsynced by hand the same way; a Python, launch or config edit takes effect on the next restart because the workspace is a symlink install, a new file needs `colcon build --packages-select <pkg> --symlink-install`.
