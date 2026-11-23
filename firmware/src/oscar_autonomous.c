// oscar_autonomous.c -- velocity-command firmware: keyboard bench console plus
// the Pi 5 ROS link, on the same UART.
//
// Adapted from the Pi 5 handoff (branch pi5/snapshot,
// pi5/oscar_ros/OSCAR_rover_prototype_v2/firmware/oscar_autonomous.c). The
// Pi line protocol is unchanged so the Pi's serial_bridge.py talks to it as-is.
//
// Console is UART0 on GP0/GP1 at 115200, NOT USB. Read it on the Pi 5 with
//   screen /dev/ttyAMA0 115200
// (/dev/serial0 on the Pi 5 is the debug connector, not GPIO14/15.)
//
// Starts DISARMED. Two ways to arm, each with its own timeout:
//
// Keyboard (like test_motor_single.c). Keys act immediately, no Enter:
//   g arm | x all stop + disarm | space or 5 stop (stays armed) | ? help
//   numpad layout, each press steps the robot velocity:
//     8/2 forward/back   4/6 strafe left/right   7/9 turn left/right
//   diag: e encoders, z zero, v speed stream, a pack voltage + duty_max,
//         r A/B sampler
//   Armed from the keyboard and moving with no byte for KEY_TIMEOUT_MS: all
//   stop, DISARMED.
//
// Pi link. Lines that start with a capital letter, '\n' or '\r' terminated:
//   Pi -> Pico: ARM | DISARM | STOP | PING | VEL vx vy wz
//               vx forward m/s, vy left m/s, wz counter-clockwise rad/s
//               (ROS base_link convention, the same as /cmd_vel).
//   Pico -> Pi: HELLO OSCAR_V1  every 500 ms, and in reply to PING
//               ENC rl rr       rear encoder counts (diag.c sign), every 100 ms
//               OK ARM | OK DISARM | OK STOP
//               ERR VEL | ERR UNKNOWN | ERR TIMEOUT  (each one also disarms)
//   HELLO/ENC only start once a Pi line has arrived, so the keyboard console
//   stays readable. (The bridge sends DISARM until it hears HELLO, which turns
//   them on.)
//   Armed by ARM with no accepted VEL for CMD_TIMEOUT_MS: all stop, DISARMED.
//   The bridge streams VEL every 50 ms, so this is ~5 missed lines. STOP does
//   not refresh it.
//
// Both: VEL is refused, and disarms, unless armed by ARM, finite and inside
// MAX_*. A control byte or an overlong line while armed: all stop, DISARMED.
// Speeding up is slew-limited (ACCEL_PER_S) so four starting currents don't
// land on the pack at once; slowing down and stopping are immediate.
// Motor PWM is compiled out unless OSCAR_ENABLE_MOTOR_OUTPUT=1
// (cmake -DOSCAR_MOTORS=ON); the banner says which. LED on = armed.
//
// Open loop: wheel speed -> duty is a straight line through WHEEL_SPEED_FULL.
// The encoders are reported, not used for control.

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "oscar_hw.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef OSCAR_ENABLE_MOTOR_OUTPUT
#define OSCAR_ENABLE_MOTOR_OUTPUT 0
#endif

#define PWM_FREQ_HZ 20000
#define CMD_TIMEOUT_MS 250  // Pi link
#define KEY_TIMEOUT_MS 2000 // keyboard, same as test_motor_single.c
#define LINE_STALE_MS 200   // drop a half-typed Pi line (stray capital key)
#define TELEMETRY_PERIOD_MS 100
#define HELLO_PERIOD_MS 500
#define SLEW_PERIOD_MS 10
#define CAP_PERIOD_MS 100
#define LED 25 // plain Pico only; on a Pico W this is WL_GPIO0

// Command limits. VEL outside these is rejected, not clamped, so a bridge bug
// shows up as ERR VEL instead of the robot quietly doing something else.
// MAX_VY allows strafing here; the Pi bridge pins vy to 0 for navigation
// because two rear encoders can't observe sideways motion.
#define MAX_VX 0.15f // m/s
#define MAX_VY 0.15f // m/s
#define MAX_WZ 0.40f // rad/s
#define STEP_V 0.05f // m/s per keypress
#define STEP_W 0.10f // rad/s per keypress

// Wheel ground speed at cmd 1.0. sweep.log: RR, wheels off the ground, slow
// decay, ~0.74 m/s at full duty and close to linear below that. On the floor
// the wheels run slower than this, so the robot undershoots, not overshoots.
#define WHEEL_SPEED_FULL 0.74f // m/s
// (wheelbase + track) / 2 for the mecanum rotation term. PROVISIONAL: the
// handoff's guess, matching effective_track_width 0.40 in rear_odometry. Measure.
#define ROTATION_ARM_M 0.20f
// cmd units per second when speeding up: 0 -> full in 0.5 s.
#define ACCEL_PER_S 2.0f
// Breakaway: the rear-right wheel needs cmd 0.10 in slow decay to start
// (bench, wheels up, 2026-11-23; the other three weren't measured). Any wheel
// cmd between CMD_DEADBAND and CMD_MIN is raised to CMD_MIN so every wheel
// turns. Below CMD_DEADBAND it's zero, so float noise from ROS doesn't kick
// a wheel up to CMD_MIN.
#define CMD_MIN 0.10f
#define CMD_DEADBAND 0.01f

#define V_SANE_MIN 9.0f
#define V_SANE_MAX 17.5f
#define DUTY_FALLBACK 0.70f

// diag.c
void diag_init(void);
void diag_poll(void);
bool diag_handle_key(int c);
int32_t diag_count(int i); // 0 = RL, 1 = RR
float diag_pack_v(void);

// ---- Pin map: oscar_hw.h ------------------------------------------------
// Same order and invert flags as test_motor_single.c. Positive cmd means
// "wheel rolls the robot forward".
typedef struct {
  const char *name;
  uint in1, in2;
  bool invert;
} motor_t;

static const motor_t MOTORS[] = {
    {"FL", FL_IN1, FL_IN2, false},
    {"FR", FR_IN1, FR_IN2, true},
    {"RL", RL_IN1, RL_IN2, false},
    {"RR", RR_IN1, RR_IN2, true}, // UNCONFIRMED

};
#define N_MOTORS 4

#define SAME_SLICE(a, b) (((a) >> 1) == ((b) >> 1))
_Static_assert(SAME_SLICE(FL_IN1, FL_IN2) && SAME_SLICE(FR_IN1, FR_IN2) &&
                   SAME_SLICE(RL_IN1, RL_IN2) && SAME_SLICE(RR_IN1, RR_IN2),
               "each motor's IN1/IN2 must be on one PWM slice");

typedef enum { CTRL_NONE, CTRL_KEYS, CTRL_PI } ctrl_t;

static uint16_t pwm_full; // counts per PWM period; level = pwm_full is 100%
static float duty_max = DUTY_FALLBACK;
static ctrl_t ctrl = CTRL_NONE; // who armed it; CTRL_NONE = disarmed
static bool pi_link = false;    // a Pi line has arrived: stream HELLO/ENC
static float target[N_MOTORS];  // what was last asked for, -1..1
static float applied[N_MOTORS]; // what the pins are doing now, -1..1
static float key_vx, key_vy, key_wz;
static absolute_time_t last_vel; // last accepted VEL (or ARM)

// ---- Low-level ----------------------------------------------------------

static void pin_idle(uint gpio) {
  gpio_init(gpio);
  gpio_put(gpio, 0);
  gpio_set_dir(gpio, GPIO_OUT);
}

static void all_idle(void) {
  for (int i = 0; i < N_MOTORS; i++) {
    pin_idle(MOTORS[i].in1);
    pin_idle(MOTORS[i].in2);
  }
}

#if OSCAR_ENABLE_MOTOR_OUTPUT
static void pin_level(uint gpio, uint16_t level) {
  pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), pwm_gpio_to_channel(gpio),
                     level);
}
#endif

static void motors_claim(void) {
#if OSCAR_ENABLE_MOTOR_OUTPUT
  for (int i = 0; i < N_MOTORS; i++) {
    const motor_t *m = &MOTORS[i];
    uint s = pwm_gpio_to_slice_num(m->in1);
    pwm_set_clkdiv(s, 1.0f);
    pwm_set_wrap(s, pwm_full - 1);
    pwm_set_both_levels(s, 0, 0);
    pwm_set_enabled(s, true);
    gpio_set_function(m->in1, GPIO_FUNC_PWM);
    gpio_set_function(m->in2, GPIO_FUNC_PWM);
  }
#endif
}

// Slow decay only (drive pin high, PWM the other at 1 - duty), the mode the
// bench runs use. c == 0 is coast.
static void motor_set(int i, float c) {
#if OSCAR_ENABLE_MOTOR_OUTPUT
  const motor_t *m = &MOTORS[i];
  c = fmaxf(-1.0f, fminf(1.0f, c));
  if (fabsf(c) < CMD_DEADBAND)
    c = 0.0f;
  else if (fabsf(c) < CMD_MIN)
    c = copysignf(CMD_MIN, c);
  if (m->invert)
    c = -c;
  uint16_t level = (uint16_t)(fabsf(c) * duty_max * pwm_full + 0.5f);
  uint drive = c >= 0 ? m->in1 : m->in2;
  uint other = c >= 0 ? m->in2 : m->in1;
  if (level == 0) {
    pin_level(m->in1, 0);
    pin_level(m->in2, 0);
  } else {
    pin_level(drive, pwm_full);
    pin_level(other, pwm_full - level);
  }
#else
  (void)i;
  (void)c;
#endif
}

static void stop_all(void) {
  key_vx = key_vy = key_wz = 0.0f;
  for (int i = 0; i < N_MOTORS; i++) {
    target[i] = applied[i] = 0.0f;
    motor_set(i, 0.0f);
  }
}

static void arm(ctrl_t who) {
  stop_all();
  ctrl = who;
  gpio_put(LED, 1);
}

static void disarm(void) {
  stop_all();
  ctrl = CTRL_NONE;
  gpio_put(LED, 0);
}

static bool moving(void) {
  for (int i = 0; i < N_MOTORS; i++)
    if (target[i] != 0.0f || applied[i] != 0.0f)
      return true;
  return false;
}

static float duty_from_pack(float v) {
  if (v < V_SANE_MIN || v > V_SANE_MAX)
    return DUTY_FALLBACK;
  return fminf(1.0f, MOTOR_V_RATED / v);
}

// Move a toward t. Slowing down is immediate; a reversal drops to coast first
// and then ramps like a start.
static float slew(float a, float t, float step) {
  if (t * a < 0.0f)
    a = 0.0f;
  if (fabsf(t) <= fabsf(a))
    return t;
  return a + copysignf(fminf(step, fabsf(t - a)), t);
}

// ---- Mixing -------------------------------------------------------------
// Mecanum, rollers in an X seen from above. +x forward, +y left, +wz CCW.
//   FL = vx - vy - ARM*wz     FR = vx + vy + ARM*wz
//   RL = vx + vy - ARM*wz     RR = vx - vy + ARM*wz
// If any wheel would exceed full speed, all four scale down together so the
// direction of travel is kept.
static void set_velocity(float vx, float vy, float wz) {
  float w[N_MOTORS] = {
      vx - vy - ROTATION_ARM_M * wz,
      vx + vy + ROTATION_ARM_M * wz,
      vx + vy - ROTATION_ARM_M * wz,
      vx - vy + ROTATION_ARM_M * wz,
  };
  float peak = WHEEL_SPEED_FULL;
  for (int i = 0; i < N_MOTORS; i++)
    peak = fmaxf(peak, fabsf(w[i]));
  for (int i = 0; i < N_MOTORS; i++)
    target[i] = w[i] / peak;
}

static bool vel_ok(float vx, float vy, float wz) {
  return isfinite(vx) && isfinite(vy) && isfinite(wz) &&
         fabsf(vx) <= MAX_VX && fabsf(vy) <= MAX_VY && fabsf(wz) <= MAX_WZ;
}

// ---- Keyboard console ---------------------------------------------------

static void print_help(void) {
  printf("\nKEYS: g arm | x all stop + disarm | space or 5 stop | ? help\n");
  printf("      8/2 forward/back | 4/6 strafe left/right | 7/9 turn "
         "left/right\n");
  printf("      (each press: %.2f m/s or %.2f rad/s, max %.2f m/s, %.2f "
         "rad/s)\n",
         STEP_V, STEP_W, MAX_VX, MAX_WZ);
  printf("      diag: e encoders | z zero | v speed stream | a pack V + "
         "duty_max | r A/B sampler\n");
  printf("Timeout: motors stop and disarm after %d ms with no key\n",
         KEY_TIMEOUT_MS);
  printf("Motor output %s in this build\n\n",
         OSCAR_ENABLE_MOTOR_OUTPUT ? "ENABLED" : "DISABLED (-DOSCAR_MOTORS=ON)");
}

static void print_velocity(void) {
  printf("vx %+.2f vy %+.2f wz %+.2f | ", key_vx, key_vy, key_wz);
  for (int i = 0; i < N_MOTORS; i++)
    printf("%s %+.2f ", MOTORS[i].name, target[i]);
  printf("(of full speed)\n");
}

static float step(float v, float d, float max) {
  v = fmaxf(-max, fminf(max, v + d));
  return fabsf(v) < 0.005f ? 0.0f : v; // kill float drift at zero
}

static void handle_key(int c) {
  if (c == '?') {
    print_help();
  } else if (c == 'x') {
    disarm();
    printf("ALL STOP, DISARMED\n");
  } else if (c == ' ' || c == '5') {
    stop_all(); // stop always works, armed or not
    printf("STOP (%s)\n", ctrl == CTRL_NONE ? "DISARMED" : "armed");
  } else if (c == 'g') {
    arm(CTRL_KEYS);
    printf("ARMED (keyboard, %d ms timeout)%s\n", KEY_TIMEOUT_MS,
           OSCAR_ENABLE_MOTOR_OUTPUT ? "" : " -- motor output DISABLED");
  } else if (strchr("824679", c)) {
    if (ctrl == CTRL_NONE) {
      printf("DISARMED -- press g to arm\n");
      return;
    }
    if (ctrl == CTRL_PI) {
      printf("Pi has control -- x to disarm\n");
      return;
    }
    if (c == '8' || c == '2')
      key_vx = step(key_vx, c == '8' ? STEP_V : -STEP_V, MAX_VX);
    else if (c == '4' || c == '6')
      key_vy = step(key_vy, c == '4' ? STEP_V : -STEP_V, MAX_VY);
    else
      key_wz = step(key_wz, c == '7' ? STEP_W : -STEP_W, MAX_WZ);
    set_velocity(key_vx, key_vy, key_wz);
    print_velocity();
  } else if (diag_handle_key(c)) {
    if (c == 'a')
      printf("duty_max = %.3f\n", duty_max);
  }
  // anything else: ignored, like test_motor_single.c
}

// ---- Pi link ------------------------------------------------------------

static void handle_line(const char *line) {
  pi_link = true;
  if (!strcmp(line, "DISARM")) {
    disarm();
    puts("OK DISARM");
  } else if (!strcmp(line, "STOP")) {
    stop_all();
    puts("OK STOP");
  } else if (!strcmp(line, "ARM")) {
    arm(CTRL_PI);
    last_vel = get_absolute_time();
    puts("OK ARM");
  } else if (!strncmp(line, "VEL ", 4)) {
    float vx, vy, wz;
    char extra;
    if (ctrl != CTRL_PI ||
        sscanf(line + 4, "%f %f %f %c", &vx, &vy, &wz, &extra) != 3 ||
        !vel_ok(vx, vy, wz)) {
      disarm();
      puts("ERR VEL");
      return;
    }
    last_vel = get_absolute_time();
    set_velocity(vx, vy, wz);
  } else if (!strcmp(line, "PING")) {
    puts("HELLO OSCAR_V1");
  } else {
    disarm();
    puts("ERR UNKNOWN");
  }
}

int main(void) {
  all_idle(); // first action, before stdio or anything else
  pwm_full = (uint16_t)(clock_get_hz(clk_sys) / PWM_FREQ_HZ);
  stdio_init_all();
  diag_init();
  duty_max = duty_from_pack(diag_pack_v());
  motors_claim();

  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);
  gpio_put(LED, 0);

  // Discard anything already in the UART (Pi console junk at boot).
  sleep_ms(50);
  while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
  }

  printf("\noscar_autonomous: DISARMED, pack %.2f V, duty_max %.3f\n",
         diag_pack_v(), duty_max);
  print_help();

  absolute_time_t last_rx = get_absolute_time();
  absolute_time_t next_hello = get_absolute_time();
  absolute_time_t next_enc = get_absolute_time();
  absolute_time_t next_slew = get_absolute_time();
  absolute_time_t next_cap = get_absolute_time();
  char line[96];
  size_t len = 0;

  while (1) {
    diag_poll(); // ADC sample + speed stream

    // Timeouts: checked every loop, before any read.
    absolute_time_t now = get_absolute_time();
    if (ctrl == CTRL_PI && absolute_time_diff_us(last_vel, now) >
                               (int64_t)CMD_TIMEOUT_MS * 1000) {
      disarm();
      puts("ERR TIMEOUT");
    }
    if (ctrl == CTRL_KEYS && moving() &&
        absolute_time_diff_us(last_rx, now) > (int64_t)KEY_TIMEOUT_MS * 1000) {
      disarm();
      printf("TIMEOUT: no input for %d ms. All stop, DISARMED\n",
             KEY_TIMEOUT_MS);
    }
    if (len && absolute_time_diff_us(last_rx, now) >
                   (int64_t)LINE_STALE_MS * 1000)
      len = 0;

    if (time_reached(next_cap)) {
      next_cap = make_timeout_time_ms(CAP_PERIOD_MS);
      duty_max = duty_from_pack(diag_pack_v()); // picked up by the next slew
    }

    if (time_reached(next_slew)) {
      next_slew = make_timeout_time_ms(SLEW_PERIOD_MS);
      for (int i = 0; i < N_MOTORS; i++) {
        applied[i] =
            slew(applied[i], target[i], ACCEL_PER_S * SLEW_PERIOD_MS / 1000.0f);
        motor_set(i, applied[i]);
      }
    }

    if (pi_link && time_reached(next_hello)) {
      next_hello = make_timeout_time_ms(HELLO_PERIOD_MS);
      puts("HELLO OSCAR_V1");
    }

    if (pi_link && time_reached(next_enc)) {
      next_enc = make_timeout_time_ms(TELEMETRY_PERIOD_MS);
      printf("ENC %ld %ld\n", (long)diag_count(0), (long)diag_count(1));
    }

    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT)
      continue;
    last_rx = get_absolute_time();

    // Pi lines start with a capital letter. Anything else at the start of a
    // line is a console key.
    if (len == 0 && c >= 32 && c <= 126 && !(c >= 'A' && c <= 'Z')) {
      handle_key(c);
    } else if (c == '\n' || c == '\r') {
      if (len) {
        line[len] = '\0';
        handle_line(line);
        len = 0;
      }
    } else if (c >= 32 && c <= 126 && len + 1 < sizeof(line)) {
      line[len++] = (char)c;
    } else { // control byte or overlong line
      len = 0;
      if (ctrl != CTRL_NONE) {
        disarm();
        printf("ERR UNKNOWN: control byte, DISARMED\n");
      }
    }
  }
}
