# OSCAR firmware (Pico)

Motor control and low-level sensing for **OSCAR**, the outlet-scanning robot (senior design capstone).
The Pico drives the four mecanum wheel motors and reads the rear wheel encoders and the battery voltage.
The Pi on the chassis talks to it over UART and flashes it through SWD.

> **Status:** bench and early floor bring-up. This is a test and diagnostics tool driven from a serial
> console, not the final control firmware. The Pi doesn't send motion commands yet.

---

## Hardware

| Part | Role |
|---|---|
| Raspberry Pi Pico (RP2040) | Motor PWM, encoders, pack ADC. A plain Pico for bring-up; the Pico W is the planned final board. |
| 4× DRV8871 breakout | One H-bridge per motor, soldered directly to the perfboard |
| 4× OSOYOO 12 V gear motor | Only the **rear two** have encoders (OSOYOO 2024005900, 11 PPR, 56:1) |
| Pi Zero 2 W | Stands in for the team's Pi 5. UART console to the Pico, and it hosts the debugger. |
| Blue Pill running Black Magic Probe | SWD debugger, plugged into the Pi Zero over USB |
| 4S 18650 pack (14.8 V nom / 16.8 V full) | Single shared rail. A 12 V bench supply was used for early bring-up. |

### Pin map (Nov 16 rewire, source of truth: `inc/oscar_hw.h`)

| Signal | GPIO | Notes |
|---|---|---|
| UART0 TX / RX (console to the Pi) | GP0 / GP1 | 115200 baud |
| FL IN1 / IN2 | GP7 / GP6 | PWM slice 3 |
| FR IN1 / IN2 | GP9 / GP8 | PWM slice 4 |
| RL IN1 / IN2 | GP3 / GP2 | PWM slice 1 |
| RR IN1 / IN2 | GP15 / GP14 | PWM slice 7 |
| RL encoder B / A | GP12 / GP13 | PIO0 SM0 |
| RR encoder B / A | GP10 / GP11 | PIO0 SM1 |
| Pack voltage divider | GP28 (ADC2) | Measured ratio 12.08 / 2.12 ≈ 5.70 |
| GP4 / GP5 | **unused** | GP5 reads ~2 Ω to GND on this Pico. Never drive it. |

Each motor's IN1/IN2 pair has to share one PWM slice. A `_Static_assert` enforces this.

---

## Repo layout

```
firmware/
├── CMakeLists.txt          # builds diag.c + test_motor_single.c into firmware.elf
├── inc/oscar_hw.h          # pin map and chassis constants (single source of truth)
├── src/
│   ├── test_motor_single.c # main bench tool: arming, per-motor drive, sweeps, TEST mode
│   ├── diag.c              # PIO encoders, wheel speed, pack voltage
│   ├── test_motor_all.c    # LEGACY: old GP0–GP7 pin map, not built
│   └── test_pico_pins.c    # LEGACY: GPIO toggle test, not built
├── quadrature_encoder.pio  # from pico-examples
├── flash.sh                # copy the ELF to the Pi, flash over SWD
├── sweep.log               # duty-sweep captures (pre-rewire pin map, see below)
└── docs/                   # datasheets, block diagrams, parts request
```

---

## Build and flash

You need the Pico SDK with `PICO_SDK_PATH` set as an environment variable.

```sh
cd firmware
mkdir -p build && cd build
cmake .. && make
cd .. && ./flash.sh build/firmware.elf
```

`flash.sh` copies the ELF to the Pi Zero and runs `gdb-multiarch` against the Black Magic Probe at
`/dev/ttyACM0`. USB/UF2 flashing is not used. Every section in the `compare-sections` output should
say `matched`. If you see `Error writing data to flash` on the first try, just retry. Run
`ssh-copy-id` once so the script stops prompting for a password.

**Console** (on the Pi): `screen /dev/serial0 115200`, and exit with `Ctrl-A k`. If `screen` exits
immediately, an old session still holds the port: run `screen -ls`, then `screen -X -S <id> quit`.

---

## Using the bench tool

It boots **DISARMED** with all IN pins low. Press `?` for help.

| Key | RUN mode |
|---|---|
| `g` | Arm |
| `1`–`4` | Select FL / FR / RL / RR |
| `5` | Select all four. Starts are staggered 150 ms apart to spread out inrush current. |
| `w` / `s` | Command ±10% |
| `W` / `S` | Command ±2%, for finding breakaway |
| `space` | Stop (works even when disarmed) |
| `x` | Stop everything and disarm |
| `d` | Toggle slow/fast decay (slow is the default) |
| `p` | Duty sweep 0 → 1.0 on RL or RR, printed as CSV. **Wheels on blocks**: the timeout is suspended during a sweep. |
| `e` / `z` / `v` | Encoder counts / zero the counts / toggle a 10 Hz speed stream |
| `a` | Pack voltage and the current `duty_max` |
| `r` | RR encoder A/B pin sampler |
| `t` | TEST mode: `1`–`8` toggle single IN pins as plain GPIO for metering at the driver, `0` sets all low, `r` goes back to RUN mode |

### Safety behavior
- **Command timeout:** if nothing arrives for 2 s, the motors stop and the tool disarms. The plan is to drop this to ~200 ms once the Pi streams commands.
- **`duty_max` follows the pack:** `12 V / V_pack`, capped at 1.0 and updated every 100 ms, so the 12 V motors aren't overdriven on a 16.8 V pack. If the reading falls outside 9–17.5 V, it uses a fallback value of 0.70.
- Every mode change sets all pins low first. Leftover UART bytes are flushed at boot.

### Conventions
- A positive command means "this wheel rolls the robot forward." Per-motor `invert` flags hide which motors are mounted mirror-image.
- Encoder counts go **up** for forward motion (`enc_invert` in `diag.c`).
- Slow decay holds one input high and PWMs the other. Off-time then brakes instead of coasting, which lowers breakaway a lot.

---

## Current status

**Working**
- All four wheels spin under PWM after the Nov 16 rewire. The robot powers up and drives on the 4S pack, and the first floor test went well.
- Rear encoders on PIO measured ~2,470 counts/rev against 2,464 from the datasheet. Reversing 10 turns came back within 44 counts.
- Pack ADC is calibrated and linear across ~10–14 V, and it now sets `duty_max` live.
- Arm/disarm, command timeout, staggered starts, and slow/fast decay.

**Measured** (`sweep.log`, RR, pre-rewire pins)

| Decay | Breakaway | Speed at cmd 1.0 |
|---|---|---|
| Slow | ≤ 0.05 (19 mm/s), close to linear | ~742 mm/s (177 rpm) |
| Fast | 0.30 | ~745 mm/s (178 rpm) |

**To confirm**
- [ ] RL `invert` flag (still a guess) and both `enc_invert` flags after the rewire. Drive each wheel forward with the chassis front marked.
- [ ] The scope read IN1 at 50 kHz once, but the firmware sets 20 kHz. Recheck at a known command.

**Next**
- [ ] Velocity command over UART from the Pi (forward, strafe, turn rate), with mecanum mixing on the Pico. This depends on what Nav2 needs: whether it uses holonomic strafe, and what odometry it wants back.
- [ ] Closed-loop wheel speed on the rear wheels. The front wheels have no encoders.
- [ ] Shorter command timeout once commands stream.
- [ ] Sensor board: bring up the LSM6DSOX IMU first. Decide whether the ToFs are needed at all given the LiDAR.
- [ ] Move from the Pi Zero 2 W to the team's Pi 5 (UART on the Pi 5 with the serial console disabled).

---

## Notes and lessons from bring-up
- Most "dead part" faults turned out to be loose or mis-seated jumpers. One DRV8871 really was damaged: a board seated one position off put rail voltage on an input. Its input was fine and its output stage was dead.
- Encoders with no signal turned out to have **unpowered encoder VCC**: the 3V3 link stopped one hop short. The giveaway was 0.6 V on VCC, fed backward through the signal pull-ups.
- For a v2 board: add test points. Probing a socketed perfboard build was the biggest time sink.
