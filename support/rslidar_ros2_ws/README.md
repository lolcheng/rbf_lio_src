# M20 RoboSense Airy reference workspace

This directory keeps the upstream RoboSense packages as Git submodules:

- `src/rslidar_msg`: RoboSense `rslidar_msg`, pinned to commit `fe8a95c`.
- `src/rslidar_sdk`: RoboSense `rslidar_sdk`, pinned to tag `v1.5.19`
  (`78d2abb`).

Initialize all nested dependencies after cloning the parent repository:

```bash
git submodule update --init --recursive
```

The M20 dual-Airy configuration is stored outside the upstream submodule at
`config/m20_airy.yaml`. Pass its absolute path through the SDK launch
`config_path` argument in the complete ROS 2 workspace.

The supplied driver build used `XYZIRT` points and enabled Airy IMU parsing.
Apply `patches/rslidar_sdk_m20_build.patch` to the `rslidar_sdk` submodule, or
make equivalent changes in the real driver workspace, before building.

The parent repository intentionally ignores `build/`, `install/`, `log/` and
rosbag database files. `support/rslidar_start.sh` is retained only as supplied
reference material; its ROS 1 `roslaunch` command does not match the confirmed
ROS 2 command `ros2 launch rslidar_sdk start.py`.
