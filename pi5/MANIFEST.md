# Pi 5 snapshot (2026-11-22)

Read-only copy of the project files on the team Pi 5 (`oscar@RaspberryPi5`,
Debian 13 trixie, kernel 6.18.50+rpt-rpi-2712), taken so the work that only
existed on the Pi is in git. Nothing on the Pi was changed. Files are as found;
none were edited.

Newest work is dated Nov 20. None of it was on GitHub before this branch.

## Where each folder came from

| Here | On the Pi | Notes |
|---|---|---|
| `oscar_ros/OSCAR_rover_prototype_v2/` | `~/oscar_ros/OSCAR_rover_prototype_v2/` | Nov 20 deployment kit. Same content as `~/OSCAR_Ethan_Handoff.tar.gz`. Start with `HANDOFF_ETHAN.md`, then `README_FIRST.md`. |
| `oscar_ros/ros2_ws/src/rover_navigation/` | `~/oscar_ros/ros2_ws/src/rover_navigation/` | Earlier nav package (explorers, waypoint/goal nodes, sim Nav2 params, a sim map). Copied to the Pi from a laptop `~/projects/OSCAR/ros2_ws/...`. |
| `oscar_ros/ros2_ws/src/ldlidar_stl_ros2/` | same | Vendor clone of https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2 (LD19 driver). Its `.git` was not copied, so the exact upstream commit is unknown. |
| `system/oscar.service`, `system/oscar-camera.service` | `~/OSCAR_Service_Backup/` | Backups of the systemd units. |
| `system/oscar-docker-inspect_pre-uart.json` | `~/oscar-docker-backup.json` | `docker inspect` of `oscar-ros-lidar` **before** it was recreated with `/dev/ttyAMA0`. Stale on purpose; see below. |

`~/oscar_ros/ros2_ws/src/oscar_preflight` on the Pi is a symlink to
`/workspace/OSCAR_rover_prototype_v2/oscar_preflight` (the Docker mount path), so
it is not duplicated here. `~/rover/predict_outlet.py` is identical to the copy on
`main` and is also not duplicated.

## Left out, with hashes

Binaries are not committed (matches `main`'s `.gitignore`). SHA256 so anyone can
check what the Pi is running:

| File on the Pi | SHA256 | Notes |
|---|---|---|
| `~/oscar_ros/OSCAR_rover_prototype_v2/models/exp.pt` | `567c2ebf368f23f24fd0c12895ecfbf86a93cc1e777e6a41a89abb5c3355743a` | YOLO baseline. Matches the hash in `README_FIRST.md`. |
| `~/oscar_ros/OSCAR_rover_prototype_v2/models/exp.onnx` | `aa9a09fb16e92f6c65253936106338effa77350f4fc5d718042cde0fc474f910` | What `onnx_ros_detector.py` loads. Same file as `~/Downloads/exp.onnx` (same size and date; not hashed separately). |
| `~/oscar_ros/OSCAR_rover_prototype_v2/firmware/oscar_velocity.uf2` | `224b793420c8561bd9572aa94b58c9ba658eeabaf23bf4af9c99c941d31580c0` | Same file as `~/Downloads/oscar_velocity(1).uf2`. See "Firmware" below. |
| `~/Downloads/oscar_velocity.uf2` | `e2cb22269c041ac8836f9f073ccd06443147c4f019cb4f121d03be7ad06a51fa` | Earlier build, no `HELLO OSCAR_V1` heartbeat. |
| `~/rover/models/best.pt` | not hashed | Copy dropped mid-transfer when the Pi left the network. |

Also not copied: `~/oscar_ros/yolo_env` and `~/rover/.venv` (Python envs, 3.4 GB),
`~/oscar_ros/core.362` (174 MB crash dump), colcon `build/install/log`, camera test
JPEGs, LiDAR `.bin` captures, `~/.bash_history`.

## Firmware: three different versions exist

1. `firmware/oscar_autonomous.c` (this snapshot). Uses `fw/bringup-tests`
   `diag.c`, `oscar_hw.h` and the PIO encoder code. Pins and motor invert flags
   match `test_motor_single.c`. **Never compiled** per its own notes.
2. `firmware/oscar_velocity.uf2`. Built from `firmware/src/motor_velocity.c`,
   which is **not on the Pi and not in any branch**. Its replies (`ARMED`,
   `DISARMED: command ignored`, `WHEELS FL=...`) differ from version 1
   (`OK ARM`, `ERR VEL`). It does send `HELLO OSCAR_V1`. Both
   "Motor outputs disabled for testing" and the handoff's "motor output is compiled
   enabled" can't be checked without the source. Do not flash it.
3. `fw/bringup-tests` `test_motor_single.c`: the keyboard bench tool that drives
   the robot today. Not compatible with the `VEL` protocol.

## Pi system state on 2026-11-22

- `oscar.service` (starts LiDAR, **mock** odometry, SLAM, Nav2, YOLO, outlet
  targeting in Docker): **disabled and inactive**. The handoff says startup is
  automatic; it currently isn't.
- `oscar-camera.service` (rpicam-still to `camera_tests/live.jpg` once a second):
  enabled and active.
- Docker container `oscar-ros-lidar` (ROS 2 Jazzy, `--network host`,
  `~/oscar_ros` mounted at `/workspace`). Shell history shows it was recreated
  from image `oscar-jazzy-nav2:with-lidar-before-uart` with both `/dev/ttyUSB0`
  (LD19) and `/dev/ttyAMA0` (Pico) mapped. The old one was renamed
  `oscar-ros-lidar-backup`.
- UART: `/boot/firmware/config.txt` has `dtoverlay=uart0-pi5` (GPIO14/15 ->
  `/dev/ttyAMA0`) and the typo `dtparam=uart0=onwq` (harmless; the overlay does
  the work). Kernel cmdline has `console=serial0,115200`.
- **`/dev/serial0` -> `ttyAMA10`**, the Pi 5 debug-UART connector, *not* GPIO14/15.
  `serial_bridge.py` defaults to `/dev/serial0` and `README_FIRST.md` /
  `create_uart_container.sh` use it; those are wrong for the Pico. The hardware
  launcher `start_oscar_hardware.sh` was later edited to `/dev/ttyAMA0` (correct).
- Nothing had `/dev/ttyAMA0` open.

## Contradictions between the docs

- Watchdog: `HANDOFF_ETHAN.md` says 300 ms, `oscar_autonomous.c` and
  `INSTALL_IN_ETHAN_WORKTREE.md` say 250 ms.
- Rotation arm 0.20 m in firmware vs `effective_track_width` placeholder 0.40 in
  odometry. Consistent with each other, but neither is measured.
- `oscar_autonomous.c` assumes 0.50 m/s wheel speed at full duty.
  `fw/bringup-tests` `sweep.log` measured ~0.74 m/s (RR, wheels up, slow decay).
