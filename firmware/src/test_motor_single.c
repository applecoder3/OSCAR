// OSCAR pre-battery bench tool. One binary, two modes.
//
// Console is UART0 on GP0/GP1 at 115200, NOT USB. Read it on the Pi with
//   screen /dev/ttyAMA0 115200     (Pi 5; /dev/serial0 there is the debug port)
//
// Starts DISARMED. Drive keys do nothing until g arms.
// RUN mode (default): g arm, 1-4 pick one motor, 5 picks ALL four,
//                     w/s step +/-10%, W/S step +/-2% (breakaway hunting),
//                     space stops, d toggles slow/fast decay, p duty sweep
//                     (RL/RR), x stops everything and disarms, t TEST mode,
//                     ? help
//                     diag: e encoders, z zero, v speed stream, a pack
//                     voltage + duty_max, r A/B sampler
// TEST mode: 1-8 toggle one IN pin as plain GPIO so you can meter it at the
//            driver, 0 drives all low, r returns to RUN mode
//
// ALL mode starts the motors one at a time, STAGGER_MS apart, whenever the
// command leaves zero, so four starting currents don't land on the supply
// at once.
//
// duty_max tracks the pack: 12 V / V_pack, capped at 1.0, updated every
// 100 ms and re-applied to whatever is running. If the reading is outside
// V_SANE_MIN..V_SANE_MAX (divider unplugged, supply collapsed in CC), it
// falls back to DUTY_FALLBACK (~12 / 16.8 V) instead of trusting it.
//
// Safety: running motors stop and the system disarms if no byte arrives on
// the console for CMD_TIMEOUT_MS. Every mode change passes through
// all-pins-low first.
//
// Direction: a positive cmd means "wheel rolls the robot forward". Motors
// mounted mirror-image get invert = true so callers never have to know.
//
// Decay (DRV8871 truth table: 10 fwd, 01 rev, 00 coast, 11 brake):
//   fast: PWM the drive pin, other pin low. Off-time = coast.
//   slow: drive pin held high, PWM the other pin at (1 - duty).
//         Off-time = brake. Much lower breakaway than fast decay.
//         NOTE: if the "other" IN line is open, the DRV8871's pulldown reads
//         it as low and the motor runs at 100% regardless of cmd.
// cmd == 0 is always coast (both low), in either mode.

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "oscar_hw.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>

#define PWM_FREQ_HZ 20000
#define CMD_TIMEOUT_MS 2000 // drop to ~200 ms once the Pi streams commands
#define STAGGER_MS 150
#define LED 25 // plain Pico only; on a Pico W this is WL_GPIO0

#define V_SANE_MIN 9.0f
#define V_SANE_MAX 17.5f
#define DUTY_FALLBACK 0.70f

#define SEL_NONE -1
#define SEL_ALL 4

// diag.c
void diag_init(void);
void diag_poll(void);
bool diag_handle_key(int c);
float diag_speed_mm_s(int i); // 0 = RL, 1 = RR
float diag_pack_v(void);

// ---- Pin map: oscar_hw.h ------------------------------------------------
// invert flags carried over from the Nov 8-9 runs (motor leads weren't
// moved, only GPIOs). RL is a guess (left side, like FL). Re-confirm all
// four against the marked chassis front with '5' before trusting them.
typedef struct {
  const char *name;
  uint in1, in2;
  bool invert;
} motor_t;

static const motor_t MOTORS[] = {
    {"FL", FL_IN1, FL_IN2, true},
    {"FR", FR_IN1, FR_IN2, false},
    {"RL", RL_IN1, RL_IN2, true}, // UNCONFIRMED
    {"RR", RR_IN1, RR_IN2, false},
};
#define N_MOTORS 4
#define N_IN_PINS (N_MOTORS * 2)

// motor_claim() configures one slice per motor, so each pair must share one.
#define SAME_SLICE(a, b) (((a) >> 1) == ((b) >> 1))
_Static_assert(SAME_SLICE(FL_IN1, FL_IN2) && SAME_SLICE(FR_IN1, FR_IN2) &&
                   SAME_SLICE(RL_IN1, RL_IN2) && SAME_SLICE(RR_IN1, RR_IN2),
               "each motor's IN1/IN2 must be on one PWM slice");

static uint in_pin(int i) {
  const motor_t *m = &MOTORS[i / 2];
  return (i % 2 == 0) ? m->in1 : m->in2;
}

static uint16_t pwm_full; // counts per PWM period; level = pwm_full is 100%
static bool slow_decay = true;
static float duty_max = DUTY_FALLBACK; // replaced from the ADC at boot
static bool duty_fallback = true;

static int sel = SEL_NONE;
static float cmd = 0.0f;

// ---- Low-level ----------------------------------------------------------

// Return a pin to SIO, driven low. Value is set before the output is
// enabled so there is no window where a stale high gets pushed out.
static void pin_idle(uint gpio) {
  gpio_init(gpio);
  gpio_put(gpio, 0);
  gpio_set_dir(gpio, GPIO_OUT);
}

static void all_idle(void) {
  for (int i = 0; i < N_IN_PINS; i++)
    pin_idle(in_pin(i));
}

// Hand one motor's two pins to its PWM slice, both at 0% first.
static void motor_claim(const motor_t *m) {
  uint s = pwm_gpio_to_slice_num(m->in1);
  pwm_set_clkdiv(s, 1.0f);
  pwm_set_wrap(s, pwm_full - 1);
  pwm_set_both_levels(s, 0, 0);
  pwm_set_enabled(s, true);
  gpio_set_function(m->in1, GPIO_FUNC_PWM);
  gpio_set_function(m->in2, GPIO_FUNC_PWM);
}

static void pin_level(uint gpio, uint16_t level) {
  pwm_set_chan_level(pwm_gpio_to_slice_num(gpio), pwm_gpio_to_channel(gpio),
                     level);
}

// c: -1.0 (full reverse) to 1.0 (full forward), in robot terms.
static void motor_set(const motor_t *m, float c) {
  c = fmaxf(-1.0f, fminf(1.0f, c));
  if (m->invert)
    c = -c;

  uint16_t level = (uint16_t)(fabsf(c) * duty_max * pwm_full + 0.5f);
  uint drive = c >= 0 ? m->in1 : m->in2;
  uint other = c >= 0 ? m->in2 : m->in1;

  if (level == 0) { // coast
    pin_level(m->in1, 0);
    pin_level(m->in2, 0);
  } else if (slow_decay) {
    pin_level(drive, pwm_full);
    pin_level(other, pwm_full - level);
  } else {
    pin_level(drive, level);
    pin_level(other, 0);
  }
}

// Push the current cmd to the selected motor(s).
static void apply(bool stagger) {
  if (sel == SEL_NONE)
    return;
  for (int i = 0; i < N_MOTORS; i++) {
    if (sel != SEL_ALL && sel != i)
      continue;
    motor_set(&MOTORS[i], cmd);
    if (stagger && sel == SEL_ALL && i < N_MOTORS - 1)
      sleep_ms(STAGGER_MS);
  }
}

static void select_motor(int s) {
  all_idle();
  sel = s;
  cmd = 0.0f;
  for (int i = 0; i < N_MOTORS; i++)
    if (s == SEL_ALL || s == i)
      motor_claim(&MOTORS[i]);
  apply(false);
  if (s == SEL_ALL)
    printf("selected ALL (starts staggered %d ms), cmd = 0.00\n", STAGGER_MS);
  else
    printf("selected %s (IN1 GP%u, IN2 GP%u%s), cmd = 0.00\n", MOTORS[s].name,
           MOTORS[s].in1, MOTORS[s].in2, MOTORS[s].invert ? ", inverted" : "");
}

static void stop_all(void) {
  all_idle();
  sel = SEL_NONE;
  cmd = 0.0f;
}

static float duty_from_pack(float v) {
  duty_fallback = (v < V_SANE_MIN || v > V_SANE_MAX);
  if (duty_fallback)
    return DUTY_FALLBACK;
  return fminf(1.0f, MOTOR_V_RATED / v);
}

static void print_duty(void) {
  printf("duty_max = %.3f%s\n", duty_max,
         duty_fallback ? "  (FALLBACK: pack reading out of range)" : "");
}

// ---- Duty sweep ---------------------------------------------------------
// Steps the selected rear motor 0 -> 1.0 in 0.05 steps. At each step: 1.5 s
// to settle, then 0.5 s of speed averaged, printed as one CSV row. Ramps
// back down at the end. Blocking, ~45 s; duty_max is frozen for the sweep.
// Any key aborts (and still ramps down). The command timeout doesn't run
// during a sweep, so wheels on blocks.

static bool wait_or_key(uint32_t ms) {
  absolute_time_t end = make_timeout_time_ms(ms);
  while (!time_reached(end))
    if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT)
      return true;
  return false;
}

static void sweep(void) {
  int e = sel - 2; // RL -> encoder 0, RR -> encoder 1
  const motor_t *m = &MOTORS[sel];
  printf("# sweep %s, %s decay, duty_max %.3f, any key aborts\n", m->name,
         slow_decay ? "slow" : "fast", duty_max);
  printf("cmd,speed_mm_s,rpm\n");

  float c = 0.0f;
  bool aborted = false;
  for (int k = 0; k <= 20 && !aborted; k++) {
    c = k * 0.05f;
    motor_set(m, c);
    if (wait_or_key(1500)) {
      aborted = true;
      break;
    }
    float sum = 0.0f;
    for (int j = 0; j < 50 && !aborted; j++) {
      sum += diag_speed_mm_s(e);
      aborted = wait_or_key(10);
    }
    if (!aborted) {
      float v = sum / 50.0f;
      printf("%.2f,%.1f,%.1f\n", c, v, v / WHEEL_CIRC_MM * 60.0f);
    }
  }

  for (; c > 0.0f; c -= 0.05f) { // gentle ramp down
    motor_set(m, c);
    sleep_ms(100);
  }
  motor_set(m, 0.0f);
  printf("# sweep %s\n", aborted ? "ABORTED, ramped down" : "done");
}

// ---- Console ------------------------------------------------------------

static void print_help(void) {
  printf("\nRUN:  g arm | 1-4 one motor | 5 ALL | w/s +/-10%% | W/S +/-2%% | "
         "space stop\n");
  printf("      d slow/fast decay (now %s) | p duty sweep (RL/RR) | x all "
         "stop + disarm\n",
         slow_decay ? "SLOW" : "FAST");
  printf("      t -> TEST mode | ? help\n");
  printf("      diag: e encoders | z zero | v speed stream | a pack V + "
         "duty_max | r A/B sampler\n");
  printf("TEST: 1-8 toggle one IN pin | 0 all low | r -> RUN mode\n");
  printf("Timeout: motors stop and disarm after %d ms with no key\n\n",
         CMD_TIMEOUT_MS);
}

static void print_pin_table(void) {
  printf("\nIN pins, in TEST key order:\n");
  for (int i = 0; i < N_IN_PINS; i++)
    printf("  %d = %s IN%d  GP%u\n", i + 1, MOTORS[i / 2].name, (i % 2) + 1,
           in_pin(i));
  printf("\n");
}

int main(void) {
  all_idle(); // first action, before stdio or anything else
  pwm_full = (uint16_t)(clock_get_hz(clk_sys) / PWM_FREQ_HZ);
  stdio_init_all();
  diag_init();
  duty_max = duty_from_pack(diag_pack_v());

  gpio_init(LED);
  gpio_set_dir(LED, GPIO_OUT);
  gpio_put(LED, 1);

  // Discard anything already in the UART (Pi console junk at boot).
  sleep_ms(50);
  while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT) {
  }

  bool test_mode = false;
  bool armed = false;
  bool pin_high[N_IN_PINS] = {false};
  absolute_time_t last_rx = get_absolute_time();
  absolute_time_t next_cap = get_absolute_time();

  printf("\nbench_motors: RUN mode, DISARMED, no motor selected, %s decay, "
         "pack %.2f V\n",
         slow_decay ? "SLOW" : "FAST", diag_pack_v());
  print_duty();
  print_help();

  while (1) {
    diag_poll(); // ADC sample + speed stream

    // Re-derive duty_max from the pack and re-apply if it moved.
    if (time_reached(next_cap)) {
      next_cap = make_timeout_time_ms(100);
      float d = duty_from_pack(diag_pack_v());
      if (fabsf(d - duty_max) > 0.01f) {
        duty_max = d;
        if (!test_mode)
          apply(false);
      }
    }

    // Command timeout: checked every loop, before any read.
    if (!test_mode && cmd != 0.0f &&
        absolute_time_diff_us(last_rx, get_absolute_time()) >
            (int64_t)CMD_TIMEOUT_MS * 1000) {
      stop_all();
      armed = false;
      printf("TIMEOUT: no input for %d ms. All stop, DISARMED\n",
             CMD_TIMEOUT_MS);
    }

    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT)
      continue;
    last_rx = get_absolute_time();

    if (c == '?') {
      print_help();
      continue;
    }

    if (test_mode) {
      if (c == 'r' || c == '0') {
        all_idle();
        for (int i = 0; i < N_IN_PINS; i++)
          pin_high[i] = false;
        if (c == 'r') {
          test_mode = false;
          printf("RUN mode, all low, no motor selected, %s\n",
                 armed ? "ARMED" : "DISARMED");
        } else {
          printf("all IN pins low\n");
        }
      } else if (c >= '1' && c <= '0' + N_IN_PINS) {
        int i = c - '1';
        pin_high[i] = !pin_high[i];
        gpio_put(in_pin(i), pin_high[i]);
        printf("%s IN%d (GP%u) = %s\n", MOTORS[i / 2].name, (i % 2) + 1,
               in_pin(i), pin_high[i] ? "HIGH" : "low");
      }
      continue;
    }

    // RUN mode. Diag keys only here, so TEST mode keeps 'r'.
    if (diag_handle_key(c)) {
      if (c == 'a')
        print_duty();
      continue;
    }

    if (c == 'g') {
      armed = true;
      printf("ARMED\n");
    } else if (c == 'p') {
      if (sel != 2 && sel != 3) {
        printf("sweep needs RL or RR (encoders) selected -- press 3 or 4\n");
      } else if (!armed) {
        printf("DISARMED -- press g to arm\n");
      } else {
        sweep();
        cmd = 0.0f;
        last_rx = get_absolute_time();
      }
    } else if (c == 'd') {
      slow_decay = !slow_decay;
      apply(false); // re-apply in the new mode
      printf("decay = %s\n", slow_decay ? "SLOW" : "FAST");
    } else if (c == 't') {
      stop_all();
      test_mode = true;
      printf("TEST mode. All pins low, plain GPIO. Meter at the driver.\n");
      print_pin_table();
    } else if (c == 'x') {
      stop_all();
      armed = false;
      printf("ALL STOP, pins low, DISARMED\n");
    } else if (c >= '1' && c <= '5') {
      select_motor(c == '5' ? SEL_ALL : c - '1');
    } else if (c == 'w' || c == 's' || c == 'W' || c == 'S' || c == ' ') {
      if (sel == SEL_NONE) {
        printf("no motor selected -- press 1-5\n");
        continue;
      }
      float prev = cmd;
      if (c == ' ') {
        cmd = 0.0f; // stop always works, armed or not
      } else if (!armed) {
        printf("DISARMED -- press g to arm\n");
        continue;
      } else {
        cmd += (c == 'w')   ? 0.1f
               : (c == 's') ? -0.1f
               : (c == 'W') ? 0.02f
                            : -0.02f;
        cmd = fmaxf(-1.0f, fminf(1.0f, cmd));
        if (fabsf(cmd) < 0.005f)
          cmd = 0.0f; // kill float drift at zero
      }
      apply(prev == 0.0f && cmd != 0.0f);
      printf("%s cmd = %.2f (%s)\n", sel == SEL_ALL ? "ALL" : MOTORS[sel].name,
             cmd, slow_decay ? "slow" : "fast");
    }
  }
}
