# OSCAR firmware handoff (Ethan, 2026-11-23)

## Status

**The Pi 5 <-> Pico link is done and tested.** ROS `/cmd_vel` -> `serial_bridge` -> Pi 5
`/dev/ttyAMA0` -> Pico -> all four wheels works, on the bench and on the floor. The Pico is ready
to take whatever the nav stack sends and drive the motors. What's left is navigation:
calibration, sensor transforms, SLAM, Nav2, perception.

The Pico is currently flashed with **`oscar_autonomous`, motors-on build** (from this branch). It
starts disarmed and only moves when armed from the keyboard or by the bridge.

Everything below is on branch **`fw/velocity-cmd`**.

## What's what

| File | What it is |
|---|---|
| `firmware/src/test_motor_single.c` -> `build/firmware.elf` | Ethan's bench tool. Drive one motor at a time, duty sweeps, pin toggling. For wiring/motor debugging. |
| `firmware/src/oscar_autonomous.c` -> `build-motor/oscar_autonomous.elf` | **The firmware to run.** Starts from the `oscar_autonomous.c` in the Nov 20 Pi kit (see `pi5/snapshot`), reworked to match the bench tool. Same keyboard style for testing, plus the Pi protocol your `serial_bridge.py` speaks. Mecanum mixing is on the Pico. |
| `firmware/inc/oscar_hw.h` | Pin map, the single source of truth for both programs. Left/right motor labels fixed 2026-11-23. |
| `firmware/flash.sh` | Flashes an `.elf` over SWD through the Black Magic Probe on the Pi 5. The only way to flash; see Power below. |
| `tools/bridge_check.sh` | 30-second test of ROS on the Pi 5 -> Pico -> wheels. See Test 2. |

Not the same thing: `oscar_velocity.uf2` in the Nov 20 kit was built from a `motor_velocity.c` that
isn't on the Pi or in git. **Don't flash it.**

A motors-off build (`build-nomotor`, without `-DOSCAR_MOTORS=ON`) also exists. It runs everything
except the motor PWM, which helps when testing the UART on a desk. You don't need it day to day.

## Power and safety rules

- **Power off:** each battery box has a switch. Switching off either one powers the robot down.
- **One power source only.** Everything runs from the on-chassis rail. **Never plug USB into the
  Pico while the robot is powered.** That would put a second 5 V supply (the laptop) onto the
  same rail, plus a ground path through the laptop. USB flashing is not used at all; flash with
  `flash.sh`. The same goes for the Pi 5: don't add a USB-C supply while it's on the rail.
  (Ethan's understanding; check the power wiring if in doubt.)
- **Battery: don't run it low.** The pack is 4S Li-ion: 16.8 V full, 14.8 V nominal. Press `a` on
  the console for pack voltage (calibrated over about 10-14 V). **Charge at about 14.0 V and stop by
  13.2 V. Below about 12 V (3.0 V per cell) the cells can be damaged.** The firmware only reports
  the voltage. It doesn't cut off, and the Pi bridge doesn't receive it. Under load it reads lower,
  so check it while the robot is stopped.
- **Motion safety in firmware:** boots disarmed. Keyboard arming stops and disarms after 2 s with
  no key. Pi arming stops and disarms after 250 ms with no `VEL`. `x` always disarms and
  `space` always stops.
- First runs after any change: wheels off the ground, hand on a battery switch.

## One-time setup (the machine you build on)

Linux or WSL Ubuntu:

```
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential git python3
git clone https://github.com/raspberrypi/pico-sdk ~/pico/pico-sdk
cd ~/pico/pico-sdk && git submodule update --init
echo 'export PICO_SDK_PATH=~/pico/pico-sdk' >> ~/.bashrc && source ~/.bashrc
ssh-copy-id oscar@RaspberryPi5.local      # so flash.sh can log in without a password
```

Ethan builds with Pico SDK 2.x. On the first configure the SDK may download and build `picotool`,
which needs internet.

## Build

From the repo root. The first time, configure each build folder (`-S`/`-B`); after that only the
`--build` lines are needed:

```
cmake -S firmware -B firmware/build-motor -DOSCAR_MOTORS=ON
cmake --build firmware/build-motor --target oscar_autonomous     # -> build-motor/oscar_autonomous.elf

cmake -S firmware -B firmware/build
cmake --build firmware/build --target firmware                   # -> build/firmware.elf (bench tool)
```

`OSCAR_MOTORS` is remembered per build folder. That's why motors-on lives in its own folder.

## Flash (`flash.sh`)

The robot must be powered (the Pico gets power from the rail, not from the probe). The Black Magic
Probe must be plugged into the Pi 5's USB.

```
cd firmware
./flash.sh build-motor/oscar_autonomous.elf      # or build/firmware.elf for the bench tool
```

It copies the `.elf` to the Pi 5 and runs `gdb-multiarch` there against the probe
(`/dev/ttyACM0`). **Every section must say `matched` at the end**; if not, it didn't flash.
`PI=user@host ./flash.sh ...` targets a different Pi (the default is `oscar@RaspberryPi5.local`).
The Pico reboots disarmed after flashing. Check the boot banner (below) to confirm what's running:
it says `Motor output ENABLED` or `DISABLED` for that build.

## Console: SSH + screen

```
ssh oscar@RaspberryPi5.local
screen /dev/ttyAMA0 115200        # NOT /dev/serial0, which is the Pi 5 debug connector
```

- Quit screen: `Ctrl-a`, then `k`, then `y`.
- **Only one program can use `/dev/ttyAMA0` at a time.** Close screen before running the bridge.
- If SSH drops while screen is open, screen keeps running on the Pi and keeps the port. Fix with
  `screen -ls`, then `screen -X -S <name> quit`, or `pkill screen`.
- Tip: run things inside `tmux` (`tmux new -s oscar`). After a dropped connection,
  `tmux attach -t oscar` gets you back. `Ctrl-b c` opens another window, `Ctrl-b n` switches.

## Test 1: keyboard (what Ethan ran)

In screen, with `oscar_autonomous` flashed. Keys act immediately; no Enter needed.

| Key | Does |
|---|---|
| `g` / `x` | arm / all stop + disarm |
| `space` or `5` | stop, stay armed |
| `8` / `2` | forward / back (0.05 m/s per press, max 0.15) |
| `7` / `9` | turn left / right (0.10 rad/s per press, max 0.40) |
| `4` / `6` | strafe left / right (not checked yet) |
| `e` `z` `a` | encoder counts, zero them, pack voltage |
| `v` `r` `?` | speed stream, encoder pin sampler, help |

Check: `g` `8`, and all four wheels roll forward. `space`, then `7`: left wheels back, right
wheels forward. `e` while pushing forward: both rear counts go up. `x` when done.

The bench tool (`build/firmware.elf`) uses the same style but drives one motor at a time: `1`-`4`
pick FL/FR/RL/RR, `5` all, `g` arm, `w`/`s` +/-10%, `W`/`S` +/-2%, `t` pin-test mode. `?`
lists it all.

## Test 2: ROS -> Pico (`tools/bridge_check.sh`)

This tests the same path Nav2 will use: it runs your `serial_bridge` inside the
`oscar-ros-lidar` container against the Pico. In order, it checks:

1. pyserial is present in the container;
2. the bridge opens `/dev/ttyAMA0` and gets `HELLO OSCAR_V1` from the Pico (`firmware_ready=True`);
3. `ENC rl rr` encoder telemetry arrives on `/pico/telemetry_raw`;
4. it publishes `/cmd_vel` forward and arms via `/pico/arm`. **Watch the wheels: all forward.** It
   stops publishing and checks that the bridge disarms on its own;
5. the same for a left turn. **Watch: left back, right forward;**
6. it arms, drives, and sends `/pico/estop`, which must stop and latch.

It prints PASS/FAIL per step and leaves the Pico disarmed. It uses ROS domain 77, so it can't see
or drive anything else that's running. The scan/odom guards are off, because this tests the link,
not navigation.

On the Pi 5, wheels up, screen closed:

```
scp tools/bridge_check.sh oscar@RaspberryPi5.local:     # once, from the repo (already there now)
sudo docker start oscar-ros-lidar                         # if it isn't running
sudo docker exec -i oscar-ros-lidar bash < ~/bridge_check.sh
```

**2026-11-23 result: all PASS**, wheels up. Forward and turn were then also good on the floor.

Using the bridge for real: start publishing `/cmd_vel` *before* sending `/pico/arm`. The bridge
disarms as soon as `/cmd_vel` goes quiet, so expect to re-arm for each Nav2 goal.

## Pi protocol (reference)

UART0, 115200, one ASCII line per message. Pi lines start with a capital letter; anything else is
treated as a keyboard key.

- Pi -> Pico: `ARM`, `DISARM`, `STOP`, `PING`, `VEL vx vy wz` (m/s, m/s, rad/s; +x forward, +y left, +wz CCW)
- Pico -> Pi: `HELLO OSCAR_V1` (500 ms), `ENC rl rr` (100 ms), `OK ...`, `ERR VEL|UNKNOWN|TIMEOUT`
- Limits: |vx| <= 0.15, |vy| <= 0.15, |wz| <= 0.40. Outside them -> `ERR VEL` + disarm.
- HELLO/ENC start after the first Pi line, so the keyboard console stays quiet.
- Open loop: 0.74 m/s at full duty (measured wheels-up, `firmware/sweep.log`). Wheel commands
  below 0.10 are raised to 0.10 so every wheel breaks away. The encoders are reported, not used
  for control.

## Verified / not verified

**Verified (2026-11-23):** flashing and the console from the Pi 5; all four wheels in both
programs; keyboard arming, timeout, stop; every `bridge_check.sh` step; forward and turn on the
floor; encoder signs count up for forward (so `rear_odometry`'s `encoder_sign_rl/rr = 1` is right).

**Not verified:** strafe (deliberately skipped; the bridge forces vy = 0 anyway); the bridge with
the scan/odom guards on (LiDAR and `rear_odometry` running); any calibration.

## Things to know

- **Camera has no physical mount yet.** Ethan is working on a 3D-printed one. Camera-to-LiDAR
  transforms can't be measured until it's mounted.
- **Guesses to calibrate:** `ROTATION_ARM_M = 0.20` m in firmware and `track_width_m = 0.40` in
  `rear_odometry`. Spin in place a few turns and compare `/odom` yaw to get the real value; the
  firmware value is half of it. `0.74 m/s` full speed was measured wheels-up.
- **Nav2 limits don't match the robot.** `nav2_rover.yaml` lets MPPI command 0.5 m/s and 1.9 rad/s
  (the velocity smoother allows 0.5 / 2.0). The bridge silently caps at 0.15 / 0.4. Set Nav2 to
  match before Nav2 drives. (`prepare_nav2.py` sets 0.15 with DWB parameter names, which MPPI
  ignores.) Raising the caps means changing the firmware `MAX_*`, the bridge and Nav2 together.
- **Pi 5 serial:** the Pico is `/dev/ttyAMA0`. `/dev/serial0` is the debug connector. The kit's
  `serial_bridge.py` default, `README_FIRST.md` and `create_uart_container.sh` use `serial0`;
  pass `port:=/dev/ttyAMA0` (`start_oscar_hardware.sh` already does).
- **Old pin map:** the Nov 20 kit's `README_FIRST.md` / `oscar_autonomous.c` have the old,
  mirrored left/right motor map. `oscar_hw.h` here is current.
- `oscar.service` on the Pi is disabled. The `oscar-ros-lidar` container may need
  `sudo docker start` after a reboot.
