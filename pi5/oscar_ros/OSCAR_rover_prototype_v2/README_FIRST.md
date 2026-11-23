# OSCAR Rover Prototype V2 — deployment kit (November 20, 2026)

**Purpose:** install the available ROS2 sensor, restricted two-encoder odometry, navigation, camera detection, and serial-control software on the Pi, without writing new software when Ethan physically mounts the hardware. The bundle is **prototype-ready source**, not a verified safe autonomous rover. The specific build/runtime/hardware acceptance steps are documented here.

## 1. Key design decision: two encoders

Ethan's rear wheel encoders are **enough for a constrained non-strafing prototype** where `vy=0` and the wheels approximately follow rolling geometry. With rear-left/right travel `dL,dR`:

`forward_distance=(dR+dL)/2`, `heading_change=(dR-dL)/effective_track_width`.

That gives the `/odom` integration used here, with enlarged yaw uncertainty to reflect mecanum slip. The **effective track width is a calibration constant**, and for ideal X-mecanum geometry it is approximately `(wheelbase + wheel track)`, not just the tape-measured wheel track. Under arbitrary mecanum driving, the rear wheel equations are `vRL=vx+vy-K*wz` and `vRR=vx-vy+K*wz`, which cannot distinguish `vy` from `wz`. Nav2 movement is therefore limited to forward/reverse and rotation. This is practical for testing and may be sufficient for indoor exploration with LiDAR/SLAM corrections, but it is not guaranteed accurate physical navigation. **Strafing is intentionally disabled**.

## 2. Inventory and references

* Actual saved baseline model artifact: `models/exp.pt` (SHA256 `567c2ebf368f23f24fd0c12895ecfbf86a93cc1e777e6a41a89abb5c3355743`). This is not the user's later `outlet_candidate_v4.pt`. Replace with the user's validated checkpoint when available.
* Ethan's actual `fw/bringup-tests` files observed: `firmware/inc/oscar_hw.h`, `firmware/src/diag.c`, `firmware/src/test_motor_single.c`, `firmware/CMakeLists.txt`. Reference pin map: FL GP7/6, FR GP9/8, RL GP3/2, RR GP15/14. Rear encoder RL GP12/13, RR GP10/11. UART0 GP0/1 @115200; **never drive GP5**. 2,464 ticks/revolution, 80mm wheel diameter. Encoder and motor signs are **not yet calibrated**.
* Current bench firmware is **keystroke-driven** and **is NOT compatible** with the new `VEL` protocol. Do not connect this bridge to the bench firmware and expect movement.
* Root package `oscar_preflight` is additive; existing `rover_navigation` and LiDAR driver remain unchanged.

## 3. Install on laptop, then transfer to Pi

In WSL, unzip as `~/OSCAR_rover_prototype_v2` and run:

```bash
cd ~/OSCAR_rover_prototype_v2
bash scripts/laptop_tests.sh
scp -r ~/OSCAR_rover_prototype_v2 oscar@172.20.10.6:~/oscar_ros/
```

Use the Pi's current IP instead of `172.20.10.6` if it has changed.

On Pi host:

```bash
sudo docker exec -it oscar-ros-lidar bash
cd /workspace/OSCAR_rover_prototype_v2
bash scripts/install_pi.sh
```

**The existing earlier `OSCAR_morning_bundle` is left untouched**; this new package installs as `oscar_preflight` only if no unrelated conflicting source exists. If you have previously installed the morning package symlink, remove that symlink manually after confirming its target, then rerun. No running Docker container is overwritten or deleted.

For SLAM Toolbox if missing: `apt-get -o Acquire::ForceIPv4=true install -y ros-jazzy-slam-toolbox`. ROS Jazzy's installed Nav2 defaults are used as a base for the generated `config/nav2_rover.yaml` rather than attempting to reproduce all of Nav2's plugin settings.

## 4. Launch sequence: sensor-only, no physical motor activity

**Terminal A**, inside `oscar-ros-lidar`: `source /opt/ros/jazzy/setup.bash; source /workspace/ros2_ws/install/setup.bash; ros2 launch ldlidar_stl_ros2 ld19.launch.py` (LiDAR desk test already verified ~10Hz).

**Terminal B**: `source /opt/ros/jazzy/setup.bash; source /workspace/ros2_ws/install/setup.bash; ros2 launch oscar_preflight preflight.launch.py` (checks topics only; no motion).

**Terminal C**, if the Pico is attached AND is running `oscar_autonomous` protocol firmware, launch the serial bridge *without allowing hardware yet*:

`ros2 launch oscar_preflight rover_serial.launch.py` (disabled; status only)

After wiring/bench safety verification, its explicit hardware option is `ros2 launch oscar_preflight rover_serial.launch.py enable_hardware:=true port:=/dev/serial0`. This requires mapping `/dev/serial0` into the Docker container at container creation; **the existing `oscar-ros-lidar` maps only the LiDAR's `/dev/ttyUSB0` and cannot access the Pi UART unless separately configured**. Stop and adapt the device mapping rather than assuming UART works inside Docker.

**Terminal D** once real `ENC` telemetry is visible (NOT in the desk-only phase):

`ros2 launch oscar_preflight rover_sensors.launch.py effective_track_width:=0.40`

`0.40` is a placeholder consistent with the firmware’s provisional 0.20 m rotation arm (effective width = 2 × rotation arm); collect the real effective track calibration on chassis. The encoder readings should count upward on both rear wheels when pushing/commanding forward. If not, set `encoder_sign_rl` or `encoder_sign_rr` after physical verification. **Never launch mock odometry simultaneously.**

**Terminal E** once /odom + odom->base_link + /scan are publishing and the LiDAR is mounted:

`ros2 launch oscar_preflight slam_sensor_only.launch.py`

**Terminal F** once SLAM publishes map->odom reliably and a valid map is being formed:

`ros2 launch oscar_preflight rover_nav2.launch.py`

Run `ros2 lifecycle get /planner_server`, `/controller_server`, `/bt_navigator`; check active nodes, costmaps, and collision-monitor topic wiring before arming. Do not attempt autonomous driving by merely launching these commands. ROS 2 Nav2 and SLAM **have not been runtime-verified together in this container**.

## 5. Camera / YOLO

`models/exp.pt` is packaged as a baseline checkpoint, not certified to be the final selected detector. The `outlet_detector` publishes JSON bounding boxes on `/outlets/detections`. The `outlet_approach` node publishes candidate PoseStamped goals on `/outlets/candidate_goal`. It cannot reliably estimate depth from an arbitrary box alone; the camera-to-laser orientation/FOV and relevant LiDAR ray must be measured. Nav2 goal sending is disabled by default and requires manual one-shot approval.

On Pi, you must enable a camera-compatible capture path in Docker and install compatible Ultralytics/OpenCV/Torch runtime (not included), and verify an actual camera frame. The model file exists but **Pi inference has not been demonstrated**. `ros2 launch oscar_preflight rover_perception.launch.py` is ONLY for when camera/model runtime is configured. The default camera source `0` assumes V4L2/OpenCV and may not address Raspberry Pi CSI/libcamera; check actual Pi camera interface.

## 6. Motor firmware and protocol

`firmware/oscar_autonomous.c` and `firmware/INSTALL_IN_ETHAN_WORKTREE.md` are a **new autonomous target** using Ethan's `diag.c`, 11/16 pin map, and PIO encoder code. The source has `OSCAR_ENABLE_MOTOR_OUTPUT=0` by default. It emits `HELLO OSCAR_V1`, `ENC rl rr` and accepts `ARM`, `VEL vx vy wz`, `STOP`, `DISARM`, with an independent 250ms motor watchdog. It is not a patch to Ethan's existing bench diagnostic binary.

**Do not flash/autonomously drive until the Pico SDK compiles this new target, serial framing works, motor signs and emergency stop have been validated with wheels off the ground.** Hardware output is an explicit, separate recompile opt-in. Encoder polarity is unconfirmed.

The Pi bridge requires successful version handshake, explicit `/pico/arm` true, fresh `/cmd_vel`, `/scan`, `/odom`, and no latched `/pico/estop`. A `true` message on `/pico/estop` latches stop until bridge restart. The bridge never auto-arms from Nav2.

## 7. What was actually verified in THIS handoff

* Python source compiles.
* Pure-code unit tests run locally, including the two-encoder observability counterexample, range conversion and firmware guard presence.
* Copied baseline YOLO checkpoint has a confirmed hash.

**Not verified here:** actual ROS 2 Jazzy colcon build, Pico SDK cross-compile, camera inference in the Pi Docker image, SLAM/Nav2 bringup with real odometry, UART device passing, physical PWM/encoder response. These MUST be performed before any physical autonomous driving. Shipping source does not certify safety.

## 8. Ethan's hardware acceptance checklist

1. Inspect actual motor-driver wiring and power protection, validate hardwired E-stop.
2. Verify Pico still matches `oscar_hw.h`; GP5 unused. Identify UART physical pins and Pi serial mapping.
3. Cross-compile and flash `oscar_autonomous` with motor output **disabled** first. Confirm `HELLO OSCAR_V1` and encoder data.
4. With battery disconnected and wheels lifted, verify motor output pins are low; check PWM using a logic probe when explicitly enabled.
5. With wheels raised and isolated test area, test low-speed forward, reverse, rotation, and E-stop; validate motor and encoder signs.
6. Measure effective track width experimentally and chassis footprint; measure LiDAR/camera frame transforms.
7. Validate odometry forward and yaw integration, then SLAM TF chain and physical costmap obstacle marking.
8. Validate Nav2 goal navigation at limited velocity with spotter and physical E-stop.
9. Validate plugged/unplugged outlet bounding boxes and their approach goals within 0.6m; verify no unsafe obstacle approach.
10. If sideways movement or significant wheel slip is required, add more odometry information or fuse independent localization; do not silently enable `vy`.

## 9. Hardware interface automation scripts

On WSL: `bash scripts/install_firmware_worktree.sh ~/projects/OSCAR_pico_velocity`. This **adds** the new `oscar_autonomous` target to the isolated firmware worktree and refuses to overwrite an existing file. Compile with Pico SDK. The existing `firmware` diagnostic target is left intact.

On Pi host (when Pico physically connected), identify its UART device using `readlink -f /dev/serial0`, disable a Linux serial console using the Pi OS configuration tools if necessary, and make sure it is not in use by another program. Then from the **Pi host**, run `sudo bash ~/oscar_ros/OSCAR_rover_prototype_v2/scripts/create_uart_container.sh /dev/serial0`. This makes a new `oscar-ros-prototype` container with the actual UART device at `/dev/oscar_pico` while leaving the existing Docker container untouched. **Do not launch two LiDAR drivers at the same time** because both containers map the same USB device. Use `port:=/dev/oscar_pico` in the new container. Additional Pi camera device access is still environment-specific.

Before physical movement: verify `ros2 topic echo /pico/status`, `/pico/telemetry_raw`, `/odom`, `/scan`, and TF, and verify Nav2's active velocity output topic and safety chain. Explicit bridge arming: `ros2 topic pub --once /pico/arm std_msgs/msg/Bool '{data: true}'`. Explicit E-stop: `ros2 topic pub --once /pico/estop std_msgs/msg/Bool '{data: true}'`. **Software E-stop complements but does not replace a hardwired motor-power emergency stop.**
