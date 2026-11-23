# OSCAR firmware handoff (Ethan -> nav/perception, 2026-11-23)

**Short version:** the Pico firmware and the Pi 5 <-> Pico link are set up for
your ROS stack. Flashing and the serial console work from the Pi 5. The
velocity firmware (`oscar_autonomous`) speaks the exact protocol your
`serial_bridge.py` expects. It was bench-tested from the keyboard. **It has not
been driven by your bridge or by ROS yet**; that's the next step and it's yours.

> The Pico currently has the bench tool (`test_motor_single`, `firmware.elf`)
> flashed, **not** `oscar_autonomous`. Flash `oscar_autonomous` before running
> the bridge (see below).

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
- `oscar_autonomous`, motors-on build: three wheels drove from the keyboard, **the rear-right wheel
  did not turn**. It then ran under `test_motor_single`.
- **Rear labels don't match the wheels:** firmware motor `RL` (GP3/GP2) drives the *physical
  rear-right* wheel. Not yet checked: whether `FL`/`FR` are also mirrored (a left/right viewpoint
  mix-up) or only the rear pair is swapped in `oscar_hw.h`, and whether the encoders are swapped too.
  Until that's fixed, mixing turn/strafe and `/odom` left/right are wrong. Check: key `1` should
  turn the front-left wheel (left as seen from behind the robot). Press `e` while turning the
  physical rear-left wheel by hand; the `RL` count should change.
- **Rear-right breakaway is high:** it starts at cmd 0.40 in fast decay (bench tool). Slow-decay
  breakaway for this wheel wasn't recorded. (`README.md`'s <= 0.05 was a different, pre-rewire
  measurement.) `oscar_autonomous` uses slow decay and sends at most 0.20 at its 0.15 m/s cap.
  If this wheel needs more than that, it can't start, which would explain the run above. Likely
  fix: measure slow-decay breakaway per wheel and add a minimum-duty offset.

**Verified off hardware only (laptop, Pico calls stubbed)**
- Protocol and safety: ARM/VEL/STOP/DISARM/PING replies, 250 ms VEL timeout, rejection of
  over-limit/NaN/malformed VEL, keyboard vs Pi arming kept separate, slew limit, mixing signs.

**Not verified**
- Wheel directions for turn (`7`/`9`) and strafe (`4`/`6`). They depend on the mecanum rollers
  being mounted in the standard X pattern.
- RL motor `invert` flag and both encoder signs after the Nov 16 rewire (noted in `README.md`).
- Anything through `serial_bridge.py` / ROS.
- `ROTATION_ARM_M = 0.20` m (your handoff's guess, matches `effective_track_width 0.40`). Measure it.
- Floor driving with `oscar_autonomous`. At the 0.15 m/s cap a wheel gets 20% of full speed,
  which may be too little to move the robot on carpet.

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

Suggested first ROS test (wheels up, inside `oscar-ros-lidar`). Scan/odom guards are off for this test only:

```
ros2 run oscar_preflight serial_bridge --ros-args -p enable_hardware:=true \
  -p port:=/dev/ttyAMA0 -p enable_scan_guard:=false -p enable_odom_guard:=false
ros2 topic echo /pico/status                      # expect firmware_ready=True
# start publishing /cmd_vel first (bridge disarms as soon as /cmd_vel goes stale), then:
ros2 topic pub --once /pico/arm std_msgs/msg/Bool '{data: true}'
```

## Pi 5 traps found

- `/dev/serial0` on the Pi 5 is the **debug connector** (`ttyAMA10`), not GPIO14/15.
  `serial_bridge.py`'s default port, `README_FIRST.md` and `create_uart_container.sh` use
  `serial0`. Use `/dev/ttyAMA0`. (`start_oscar_hardware.sh` already does.)
- **Don't flash `oscar_velocity.uf2`** from the Nov 20 kit. It was built from
  `motor_velocity.c`, whose source isn't on the Pi or in git. Use `oscar_autonomous` from this branch.
- `oscar.service` is disabled and inactive (the kit says startup is automatic).
- `config.txt` has the typo `dtparam=uart0=onwq`; harmless, `dtoverlay=uart0-pi5` does the work.

## Next (suggested order)

1. Bridge with wheels up (above): ROS -> UART -> Pico -> wheels. Check turn/strafe directions and encoder signs.
2. Calibrate: spin in place to get the real rotation arm / effective track width; drive a measured distance.
3. Measure LiDAR and camera transforms on the chassis.
4. Teleop with SLAM running to build a map.
5. Nav2 goals at low speed with a spotter and a hand on the battery switches.
6. Outlet detection -> approach goal.
