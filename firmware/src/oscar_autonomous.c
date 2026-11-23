// oscar_autonomous.c -- velocity-command firmware for the Pi 5 ROS stack.
//
// Adapted from the Pi 5 handoff (branch pi5/snapshot,
// pi5/oscar_ros/OSCAR_rover_prototype_v2/firmware/oscar_autonomous.c). The
// line protocol is unchanged so the Pi's serial_bridge.py talks to it as-is.
//
// UART0 on GP0/GP1 at 115200, NOT USB. One ASCII line per message, '\n' or
// '\r' terminated.
//   Pi -> Pico: ARM | DISARM | STOP | PING | VEL vx vy wz
//               vx forward m/s, vy left m/s, wz counter-clockwise rad/s
//               (ROS base_link convention, the same as /cmd_vel).
//   Pico -> Pi: HELLO OSCAR_V1  at boot, every 500 ms, and in reply to PING
//               ENC rl rr       rear encoder counts (diag.c sign), every 100 ms
//               OK ARM | OK DISARM | OK STOP
//               ERR VEL | ERR UNKNOWN | ERR TIMEOUT  (each one also disarms)
//
// Safety:
//   - Boots DISARMED with every IN pin low, and flushes UART RX first.
//   - VEL is refused, and disarms, unless armed, finite and inside MAX_*.
//   - Armed with no accepted VEL for CMD_TIMEOUT_MS: all stop, DISARMED. The
//     Pi bridge streams VEL every 50 ms, so this is ~5 missed lines. STOP does
//     not refresh it.
//   - A malformed byte, an overlong line or an unknown line: all stop,
//     DISARMED.
//   - Speeding up is slew-limited (ACCEL_PER_S) so four starting currents don't
//     land on the pack at once. Slowing down and stopping are immediate.
//   - Motor PWM is compiled out unless OSCAR_ENABLE_MOTOR_OUTPUT=1
//     (cmake -DOSCAR_MOTORS=ON). Everything else, including the protocol and
//     encoder telemetry, still runs, so the UART link can be tested first.
//   - LED on = armed.
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
#define CMD_TIMEOUT_MS 250
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

// Wheel ground speed at cmd 1.0. sweep.log: RR, wheels off the ground, slow
// decay, ~0.74 m/s at full duty and close to linear below that. On the floor
// the wheels run slower than this, so the robot undershoots, not overshoots.
#define WHEEL_SPEED_FULL 0.74f // m/s
// (wheelbase + track) / 2 for the mecanum rotation term. PROVISIONAL: the
// handoff's guess, matching effective_track_width 0.40 in rear_odometry. Measure.
#define ROTATION_ARM_M 0.20f
// cmd units per second when speeding up: 0 -> full in 0.5 s.
#define ACCEL_PER_S 2.0f

#define V_SANE_MIN 9.0f
#define V_SANE_MAX 17.5f
#define DUTY_FALLBACK 0.70f

// diag.c
void diag_init(void);
void diag_poll(void);
int32_t diag_count(int i); // 0 = RL, 1 = RR
float diag_pack_v(void);

// ---- Pin map: oscar_hw.h ------------------------------------------------
// Same order and invert flags as test_motor_single.c. Positive cmd means
// "wheel rolls the robot forward".
typedef struct {
  uint in1, in2;
  bool invert;
} motor_t;

static const motor_t MOTORS[] = {
    {FL_IN1, FL_IN2, true},
    {FR_IN1, FR_IN2, false},
    {RL_IN1, RL_IN2, true}, // UNCONFIRMED
    {RR_IN1, RR_IN2, false},
};
#define N_MOTORS 4

#define SAME_SLICE(a, b) (((a) >> 1) == ((b) >> 1))
_Static_assert(SAME_SLICE(FL_IN1, FL_IN2) && SAME_SLICE(FR_IN1, FR_IN2) &&
                   SAME_SLICE(RL_IN1, RL_IN2) && SAME_SLICE(RR_IN1, RR_IN2),
               "each motor's IN1/IN2 must be on one PWM slice");

static uint16_t pwm_full; // counts per PWM period; level = pwm_full is 100%
static float duty_max = DUTY_FALLBACK;
static bool armed = false;
static float target[N_MOTORS];  // what the last VEL asked for, -1..1
static float applied[N_MOTORS]; // what the pins are doing now, -1..1

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
  for (int i = 0; i < N_MOTORS; i++) {
    target[i] = applied[i] = 0.0f;
    motor_set(i, 0.0f);
  }
}

static void disarm(void) {
  stop_all();
  armed = false;
  gpio_put(LED, 0);
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

// ---- Protocol -----------------------------------------------------------

static absolute_time_t last_vel;

static void handle_line(const char *line) {
  if (!strcmp(line, "DISARM")) {
    disarm();
    puts("OK DISARM");
  } else if (!strcmp(line, "STOP")) {
    stop_all();
    puts("OK STOP");
  } else if (!strcmp(line, "ARM")) {
    stop_all();
    armed = true;
    gpio_put(LED, 1);
    last_vel = get_absolute_time();
    puts("OK ARM");
  } else if (!strncmp(line, "VEL ", 4)) {
    float vx, vy, wz;
    char extra;
    if (!armed || sscanf(line + 4, "%f %f %f %c", &vx, &vy, &wz, &extra) != 3 ||
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

  printf("oscar_autonomous: DISARMED, motor output %s (build), pack %.2f V, "
         "duty_max %.3f\n",
         OSCAR_ENABLE_MOTOR_OUTPUT ? "ENABLED" : "DISABLED", diag_pack_v(),
         duty_max);
  puts("HELLO OSCAR_V1");

  absolute_time_t next_hello = make_timeout_time_ms(HELLO_PERIOD_MS);
  absolute_time_t next_enc = get_absolute_time();
  absolute_time_t next_slew = get_absolute_time();
  absolute_time_t next_cap = get_absolute_time();
  char line[96];
  size_t len = 0;

  while (1) {
    diag_poll(); // ADC sample

    // Command timeout: checked every loop, before any read.
    if (armed && absolute_time_diff_us(last_vel, get_absolute_time()) >
                     (int64_t)CMD_TIMEOUT_MS * 1000) {
      disarm();
      puts("ERR TIMEOUT");
    }

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

    if (time_reached(next_hello)) {
      next_hello = make_timeout_time_ms(HELLO_PERIOD_MS);
      puts("HELLO OSCAR_V1");
    }

    if (time_reached(next_enc)) {
      next_enc = make_timeout_time_ms(TELEMETRY_PERIOD_MS);
      printf("ENC %ld %ld\n", (long)diag_count(0), (long)diag_count(1));
    }

    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT)
      continue;
    if (c == '\n' || c == '\r') {
      if (len) {
        line[len] = '\0';
        handle_line(line);
        len = 0;
      }
    } else if (c >= 32 && c <= 126 && len + 1 < sizeof(line)) {
      line[len++] = (char)c;
    } else { // control byte or overlong line
      len = 0;
      if (armed) {
        disarm();
        puts("ERR UNKNOWN");
      }
    }
  }
}
