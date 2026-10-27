# Provisioning the GPU box (training + VLA web manager)

Built 2026-10-26 on `4090TI` (RTX 4090 24 GB, 32 cores, 62 GB, 1.8 TB NVMe), replacing `bryan-ubuntu`
(Ubuntu 20.04, gone). Everything here was run for real on that box; `provision_gpu.sh` is the same steps.

Role: **training** and the **web manager** (`webui/server.py`, `:8081`, the master eval registry). It does
not serve the robot: the production inference server is RD's (`VLA/services/rd/`, loopback `:8777`);
`vla_infer_server` here is a manual-start fallback that pins VRAM.

## 1. Install Ubuntu 24.04

- User **`bryan`** (units and paths assume `/home/bryan`); tick *Install OpenSSH server* and
  *third-party drivers* (the NVIDIA driver; 24.04 gave 595.99). Plain ext4, whole disk.
- Reserve `192.168.1.135` (wired) in the router. Passwordless sudo makes provisioning unattended:
  `echo 'bryan ALL=(ALL) NOPASSWD: ALL' | sudo tee /etc/sudoers.d/bryan && sudo chmod 440 /etc/sudoers.d/bryan`

## 2. SSH from the Mac (and between the boxes)

```bash
ssh-keygen -R 192.168.1.135            # a reinstall changes the host key
ssh-copy-id -i ~/.ssh/alpha_id_rsa.pub bryan@192.168.1.135
```
`~/.ssh/config` on the Mac **and on RD**:
```
Host GPU
  HostName 192.168.11.119        # the Wi-Fi address, on the robot subnet (wired is 192.168.1.135)
  User bryan
  IdentityFile ~/.ssh/alpha_id_rsa     # RD: its own key, authorised on the GPU box
```
`GPU` also needs `Host RD` (`rd@192.168.11.80`) and a key RD accepts — `ssh GPU ssh RD hostname` is the
test. Add the GPU box's `id_ed25519.pub` to RD's `authorized_keys` **yourself**: an agent session is
(rightly) blocked from granting new SSH access. The repo's `--gpu-host` defaults are now `bryan@GPU`, an
ssh alias, so the address lives in one place instead of a DHCP hostname that moves.

## 3. Push the VLA tree, then provision

```bash
# workstation, from the VLA repo — ~/VLA on the GPU box is NOT a git checkout (see VLA/wiki/Sync-and-Layout.md)
rsync -a --exclude='/outputs/' --exclude='.venv*' --exclude='.git/' --exclude='__pycache__/' --exclude='/log/' \
      --exclude='/ros2_ws/build/' --exclude='/ros2_ws/install/' --exclude='/ros2_ws/log/' ./ GPU:~/VLA/
rsync -a ../rd_ws/provision/gpu/ GPU:~/provision_gpu/
ssh GPU '~/provision_gpu/provision_gpu.sh all && ~/provision_gpu/provision_gpu.sh data'
```
`all` = `base venv services`; `data` pulls `~/VLA/outputs` (datasets, train runs, eval registry, ~6 GB)
from RD and repoints `outputs/serve_ckpt`. Then browse the web manager at `http://192.168.11.119:8081` and the supervisor UI at `:9999`.

| Phase | What it does |
|---|---|
| `base` | `apt-packages-gpu.txt`, supervisor on, `uv` |
| `venv` | `~/VLA/.venv` (Python 3.12) from `vla-venv-freeze.txt`, then asserts CUDA + SmolVLA import |
| `services` | supervisor web UI on `:9999` (drop-in `00-inet-http.conf`, no auth, needs a supervisord restart), then `VLA/scripts/install_services.sh` → `vla_webui` (autostart) + `vla_infer_server` (manual) |
| `data` | rsync `outputs/` from RD, `serve_ckpt` → `train/$SERVE_RUN/checkpoints/$SERVE_STEP/pretrained_model` |
| `wifi PW` | *(not in `all`)* join `RobotDisco`; Ethernet keeps metric 100, Wi-Fi gets 600 |
| `extras` | *(not in `all`)* Steam (Proton), Brave, VLC, Remmina, HandBrake, GIMP, Angry IP Scanner, Bambu Studio |

## Discoveries (each one cost a failed run — keep them)

- **A fresh 24.04 has no `curl`, `git`, `supervisor` or compiler.** My first extras install "succeeded"
  (exit 0) because a trailing `| tail` swallowed `curl: command not found`. Check real exit codes.
- **The venv must be Python 3.12 via `uv`** (system python is 3.12 too, but uv gives a reproducible
  venv). RD is 3.10 / aarch64 — only the *versions* (lerobot 0.4.4, torch 2.8.0, transformers 4.57.6,
  numpy 1.26.4) are mirrored. PyPI's x86 torch 2.8.0 bundles cu128: no `--index-url` needed.
- **pandas/scipy/pyarrow must be pinned** (2.3.3 / 1.15.3 / 25.0.1). uv resolves pandas 3.x, which needs
  numpy 2; once numpy is pinned back to 1.26.4 it dies with `ImportError: C extension: None not built`.
  `uv pip check` still lists three `numpy>=2` complaints (opencv x2, rerun-sdk) — harmless, same as RD.
- **Install the freeze with `--no-deps`**: resolving it as a set fails (`opencv-python==4.12.0.88 depends on
  numpy>=2` vs the pinned 1.26.4). Found by re-running the script on the finished box — do that after edits.
- **`fastapi` + `uvicorn` are not in any RD manifest** (RD never runs the web UI), so `vla_webui` goes
  BACKOFF → FATAL with `No module named 'fastapi'` on a new box. Now in `pip-vla-gpu.txt` and the freeze.
- **The Mac's rsync is openrsync**: `--delete-excluded` against the Ubuntu rsync 3.2.7 aborts with
  `buffer overflow: recv_rules`. Don't use it; the target is empty on first push anyway.
- **`serve_ckpt` is an absolute symlink** (RD's lives under `/mnt/data/VLA/...`), so it is excluded from
  the `data` pull and re-created for `/home/bryan`.
- **Two NICs, two subnets**: Ethernet `192.168.1.x` (internet) and Wi-Fi `192.168.11.x` (robot, isolated).
  Both default routes exist; keep wired at metric 100 / Wi-Fi 600. The Mac and RD reach the box on the
  Wi-Fi address (`192.168.11.119`, DHCP — pin a lease if it moves; every consumer goes via the `GPU` alias).
- **Bambu Studio's asset is `ubuntu24.04`** (no hyphen) — a `ubuntu-24.04` grep silently matches nothing.
- Passwordless `sudo supervisorctl` comes with the NOPASSWD sudoers line above.

## Not automated

- **Fusion 360** runs only under Wine via the community installer
  (`autodesk_fusion_installer_x86-64.sh --install --default`, from the cryinkfly project, moving from GitHub
  to Codeberg). It is interactive, needs the desktop for the Autodesk sign-in, adds WineHQ-staging + Samba,
  and offers `sudo snap remove firefox` at one prompt. Read it first; run it by hand on the desktop.
- **Rocket League** (Steam, Proton): sign in to Steam, enable Steam Play for all titles, install.
- The first Steam launch self-updates; Bambu printer access needs LAN access code or a cloud login.
