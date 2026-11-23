# OSCAR firmware handoff (Ethan -> nav/perception, 2026-11-23)

**Short version:** the Pico firmware and the Pi 5 <-> Pico link are set up for
your ROS stack. Flashing and the serial console work from the Pi 5. The
velocity firmware (`oscar_autonomous`) speaks the exact protocol your
`serial_bridge.py` expects. It was bench-tested from the keyboard. **It has not
been driven by your bridge or by ROS yet**; that's the next step and it's yours.

> The Pico currently has the bench tool (`test_motor_single`, `firmware.elf`)
> flashed, built **before** the left/right label fix, **not** `oscar_autonomous`.
> Flash `oscar_autonomous` from this branch before running the bridge (see below).

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

**Not verified**
- Wheel directions for turn (`7`/`9`) and strafe (`4`/`6`). They depend on the mecanum rollers
  being mounted in the standard X pattern.
- Forward direction of each wheel after the label fix (press `5` in the bench tool, or `8` in
  `oscar_autonomous`, and watch that all four roll forward), and both encoder signs.
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
