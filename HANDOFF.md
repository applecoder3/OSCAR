# OSCAR firmware handoff (Ethan -> nav/perception, 2026-11-23)

**Short version:** the interconnect is done. ROS `/cmd_vel` -> your
`serial_bridge` -> Pi 5 `/dev/ttyAMA0` -> Pico (`oscar_autonomous`) -> four
wheels was run end to end on 2026-11-23 (wheels up), along with the
handshake, encoder telemetry, the stale-command disarm and the E-stop.
`tools/bridge_check.sh` reruns all of it in about 30 s. What's left is
navigation: calibration, transforms, SLAM, Nav2.

> The Pico currently has `oscar_autonomous`, **motors-on** build, from this
> branch. It starts disarmed and only drives when armed from the keyboard (`g`)
> or by your bridge (`/pico/arm`).

## Branches

| Branch | What | Base |
|---|---|---|
| `fw/velocity-cmd` | **Use this.** Bench tool + `oscar_autonomous` velocity firmware + Pi 5 flashing. This file. | `fw/bringup-tests` |
| `fw/bringup-tests` | Bench tool only, as it was before the Pi 5 move. | |
| `pi5/snapshot` | Unmodified copy of everything that was only on the Pi 5 (your Nov 20 kit, `rover_navigation`, LD19 driver, systemd units). Read `pi5/MANIFEST.md`. | `main` |

`main`, `asusbranch`, `proto` were not touched.

## State

**Verified on hardware (Pi 5, wheels up)**
- Build on the laptop, flash over SWD from the Pi 5 (Black Magic Probe on `/dev/ttyACM0`), `compare-sections` matched.
- Pi 5 UART to the Pico on `/dev/ttyAMA0`: console works.
- `test_motor_single` (bench tool): all four wheels drive.
- `oscar_autonomous`, motors-off build: banner, keys, arming and the 2 s key timeout (stage 1).
- `oscar_autonomous`, motors-on build, after the fixes below: `g` `8` drives all four wheels forward.
- **Through your bridge** (`tools/bridge_check.sh`, `serial_bridge` in `oscar-ros-lidar`,
  scan/odom guards off): every check PASS.
  - Handshake (`firmware_ready=True`) and `ENC rl rr` telemetry on `/pico/telemetry_raw`.
  - `/cmd_vel` vx=0.05: all four wheels drove forward.
  - `/cmd_vel` wz=0.3: drove, and the wheels looked right for a left turn (left back, right
    forward). That was judged wheels-up, so confirm on the floor.
  - Stopping `/cmd_vel`: the bridge disarmed by itself. `/pico/estop` while driving: stopped
    and latched.
- `oscar_autonomous`, motors-on build (before the fixes below): three wheels drove from the
  keyboard, the rear-right wheel did not turn. It then ran under `test_motor_single`.
- **Motor left/right labels were mirrored. Fixed in `oscar_hw.h` (2026-11-23):** on the bench,
  GP7/6 turned the front-right wheel and GP3/2 the rear-right (left/right as seen from behind the
  robot). They're now named FR and RR. GP9/8 = FL and GP15/14 = RL are **inferred, not checked**
  (keys `1`/`3` were tested, `2`/`4` weren't). The invert flags moved with their pins, so the right
  side is the inverted side now; RR's flag is still a guess. **Encoders were already correct**:
  turning the rear-left wheel by hand changes the `RL` count. The pin map in the Nov 20 kit's
  `README_FIRST.md` / `oscar_autonomous.c` (`pi5/snapshot`) is the old, mirrored one.
- **Rear-right breakaway: cmd 0.10 in slow decay, 0.40 in fast** (bench tool, wheels up).
  `oscar_autonomous` now raises any wheel command between 0.01 and 0.10 to 0.10, and treats
  anything below 0.01 as zero. That explains and should fix the run above. **Not re-tested on
  hardware since the fix.** The other three wheels' breakaway wasn't measured.

**Verified off hardware only (laptop, Pico calls stubbed)**
- Protocol and safety: ARM/VEL/STOP/DISARM/PING replies, 250 ms VEL timeout, rejection of
  over-limit/NaN/malformed VEL, keyboard vs Pi arming kept separate, slew limit, mixing signs.

- **Floor test: good.** Forward and turn drove correctly on the floor at these speeds.
  Encoder signs count the right way for forward, so `rear_odometry`'s default
  `encoder_sign_rl/rr = 1` should be right.

**Not verified**
- Strafe (`4`/`6`). Deliberately skipped for now; your bridge forces vy = 0 anyway.
- The bridge with the scan/odom guards on, i.e. with the LiDAR and `rear_odometry` running.
- Calibration: `ROTATION_ARM_M = 0.20` m in firmware and `track_width_m = 0.40` in
  `rear_odometry` are both guesses. Real speed vs commanded speed hasn't been measured.

**Power cutoff:** each battery box has its own switch. Killing motor power means switching off every box.

## Flash and drive from the Pi 5

Wiring: Pi pin 8 (GPIO14 TX) -> Pico GP1, Pi pin 10 (GPIO15 RX) <- Pico GP0, GND <-> GND.
Black Magic Probe USB into the Pi 5.

From a laptop with the Pico SDK (`PICO_SDK_PATH` set):

```
cmake -S firmware -B firmware/build-motor -DOSCAR_MOTORS=ON   # omit the flag for a no-PWM build
cmake --build firmware/build-motor --target oscar_autonomous
cd firmware && ./flash.sh build-motor/oscar_autonomous.elf    # every section must say "matched"
```

`flash.sh` defaults to `oscar@RaspberryPi5.local`; `PI=user@host ./flash.sh ...` overrides.
The bench tool is target `firmware` (`./flash.sh build/firmware.elf`).

Console on the Pi: `screen /dev/ttyAMA0 115200` (quit: Ctrl-a k y). The banner says whether
motor output is ENABLED in that build.

| Key | Does |
|---|---|
| `g` / `x` | arm / all stop + disarm |
| `space` or `5` | stop, stay armed |
| `8`/`2`, `4`/`6`, `7`/`9` | step forward/back, strafe left/right, turn left/right |
| `e z v a r ?` | encoders, zero, speed stream, pack voltage, A/B sampler, help |

Starts disarmed. If it's moving and gets no key for 2 s, it stops and disarms.

## Pi link (what your bridge talks to)

UART0 115200, one ASCII line each, lines start with a capital letter:

- Pi -> Pico: `ARM`, `DISARM`, `STOP`, `PING`, `VEL vx vy wz` (m/s, m/s, rad/s; +x forward, +y left, +wz CCW)
- Pico -> Pi: `HELLO OSCAR_V1` (500 ms), `ENC rl rr` (100 ms), `OK ...`, `ERR VEL|UNKNOWN|TIMEOUT`

Limits: |vx| <= 0.15, |vy| <= 0.15, |wz| <= 0.40. Outside them -> `ERR VEL` + disarm.
Armed by `ARM` with no VEL for 250 ms -> stop + disarm. HELLO/ENC start after the first Pi
line (your bridge's DISARM-until-ready does that). Wheel speed is open loop (0.74 m/s at full
duty from `firmware/sweep.log`). The encoders are reported, not used for control.

Don't run `screen` and the bridge on `/dev/ttyAMA0` at the same time.

**Re-check the link anytime** (wheels up, `screen` closed, about 30 s, prints PASS/FAIL per step).
On the Pi 5, with `tools/bridge_check.sh` copied to `~`:

```
sudo docker start oscar-ros-lidar        # if it isn't running
sudo docker exec -i oscar-ros-lidar bash < ~/bridge_check.sh
```

It runs `serial_bridge` with `port:=/dev/ttyAMA0` in ROS domain 77, with the scan/odom guards
off. It drives forward, then turns, then E-stops, and leaves the Pico disarmed. When you run the
bridge for real: start publishing `/cmd_vel` *before* sending `/pico/arm`, because the bridge
disarms as soon as `/cmd_vel` goes stale. That also means re-arming after every Nav2 goal.

## Pi 5 traps found

- `/dev/serial0` on the Pi 5 is the **debug connector** (`ttyAMA10`), not GPIO14/15.
  `serial_bridge.py`'s default port, `README_FIRST.md` and `create_uart_container.sh` use
  `serial0`. Use `/dev/ttyAMA0`. (`start_oscar_hardware.sh` already does.)
- **Don't flash `oscar_velocity.uf2`** from the Nov 20 kit. It was built from
  `motor_velocity.c`, whose source isn't on the Pi or in git. Use `oscar_autonomous` from this branch.
- `oscar.service` is disabled and inactive (the kit says startup is automatic).
- `config.txt` has the typo `dtparam=uart0=onwq`; harmless, `dtoverlay=uart0-pi5` does the work.

## Next (suggested order)

Done: ROS -> bridge -> Pico -> wheels, wheels up and on the floor.

1. **Full sensor chain with the guards on.** `OSCAR_HARDWARE_ENABLED=true` with
   `start_oscar_hardware.sh` (LD19, bridge, `rear_odometry`, SLAM, Nav2, YOLO). Check `/scan` at
   ~10 Hz, `/odom` moving when you push the robot, and TF `odom -> base_link`. Then drive by
   teleop on `/cmd_vel` with the guards on. You'll need a machine with RViz on the same network
   and the same `ROS_DOMAIN_ID` to see any of this.
2. **Calibrate odometry** (needs only teleop and a tape measure):
   - Drive 2 m straight and compare `/odom` x -> `wheel_diameter_m` / `counts_per_rev`.
   - Spin in place 5 full turns and compare `/odom` yaw -> `track_width_m`. Then set firmware
     `ROTATION_ARM_M = track_width_m / 2` so commanded turn rate matches real.
   - Command 0.10 m/s for 10 s and measure the distance. If it's well off, tell Ethan:
     `WHEEL_SPEED_FULL` (0.74 m/s) was measured with the wheels in the air.
3. **Measure sensor transforms.** LiDAR and camera position/height/yaw relative to `base_link`.
   Check LiDAR forward-zero: an object straight ahead should appear at angle 0 in RViz.
4. **Map:** teleop around the room with SLAM running; save the map.
5. **Nav2 goals** at low speed, with a spotter and a hand on the battery switches. Before this,
   **match Nav2's limits to the robot**:
   - In `nav2_rover.yaml` the MPPI controller has `vx_max 0.5`, `wz_max 1.9`, and the velocity
     smoother has 0.5 m/s / 2.0 rad/s. The bridge and firmware cap at 0.15 m/s / 0.4 rad/s, and
     the bridge clamps silently, so Nav2 would plan for a robot ~3x faster than it is.
   - `prepare_nav2.py` sets 0.15 but with DWB parameter names, which MPPI doesn't read.
   - The bridge disarms whenever `/cmd_vel` goes quiet (between goals), so expect to re-arm per goal.
6. **Outlet detection -> approach goal.** Camera frame + YOLO on the Pi, then the approach node
   with `enable_navigation_goals` on.

Raising the 0.15 / 0.4 limits later means changing the firmware `MAX_*`, the bridge
`max_linear_x` / `max_angular_z` and Nav2 together.
