# Build and run



## Build

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
