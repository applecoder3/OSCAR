// test_motor_all.c
// Bring-up test for all four DRV8871 drivers. Each driver's IN1/IN2 pair shares
// one PWM slice (GP0/1 = slice 0 ... GP6/7 = slice 3) at 20 kHz.
// Keys: w/s = +/-0.1, space = stop, 1-4 = drive only that motor, 0 = all motors.

#include <math.h>
#include <stdio.h>
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#define NUM_MOTORS 4
#define PWM_WRAP 6249 // 125 MHz / 6250 = 20 kHz (use 7499 on a Pico 2)

typedef struct {
  uint in1, in2;
} motor_t;

static const motor_t motors[NUM_MOTORS] = {
    {0, 1}, {2, 3}, {4, 5}, {6, 7},
};

static void motor_init(const motor_t *m) {
  gpio_set_function(m->in1, GPIO_FUNC_PWM);
  gpio_set_function(m->in2, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(m->in1);
  pwm_set_wrap(slice, PWM_WRAP);
  pwm_set_gpio_level(m->in1, 0);
  pwm_set_gpio_level(m->in2, 0);
  pwm_set_enabled(slice, true);
}

// cmd: -1.0 (full reverse) to 1.0 (full forward)
static void motor_set(const motor_t *m, float cmd, float duty_max) {
  if (cmd > 1.0f)
    cmd = 1.0f;
  if (cmd < -1.0f)
    cmd = -1.0f;
  uint16_t level = (uint16_t)(fabsf(cmd) * duty_max * (PWM_WRAP + 1));
  if (cmd >= 0) {
    pwm_set_gpio_level(m->in1, level); // IN1 = PWM
    pwm_set_gpio_level(m->in2, 0);     // IN2 = 0
  } else {
    pwm_set_gpio_level(m->in1, 0);
    pwm_set_gpio_level(m->in2, level);
  }
}

int main(void) {
  stdio_init_all();
  while (!stdio_usb_connected())
    sleep_ms(100);

  for (int i = 0; i < NUM_MOTORS; i++)
    motor_init(&motors[i]);

  float cmd = 0.0f;
  float duty_max = 1.0f; // 12 V bench supply. On the 4S pack: 12.0f / V_batt (~0.71 full)
  int sel = -1;          // -1 = all motors, 0-3 = one motor

  printf("w/s = speed, space = stop, 1-%d = single motor, 0 = all\n", NUM_MOTORS);

  while (1) {
    int c = getchar_timeout_us(0);
    if (c == PICO_ERROR_TIMEOUT)
      continue;

    if (c == 'w')
      cmd += 0.1f;
    else if (c == 's')
      cmd -= 0.1f;
    else if (c == ' ')
      cmd = 0.0f;
    else if (c >= '1' && c <= '0' + NUM_MOTORS) {
      sel = c - '1';
      cmd = 0.0f; // always start a newly selected motor from rest
    } else if (c == '0') {
      sel = -1;
      cmd = 0.0f;
    } else
      continue;

    if (cmd > 1.0f)
      cmd = 1.0f;
    if (cmd < -1.0f)
      cmd = -1.0f;
    if (fabsf(cmd) < 0.05f)
      cmd = 0.0f; // clear float drift around zero

    for (int i = 0; i < NUM_MOTORS; i++)
      motor_set(&motors[i], (sel < 0 || sel == i) ? cmd : 0.0f, duty_max);

    if (sel < 0)
      printf("all motors: cmd = %.1f\n", cmd);
    else
      printf("motor %d: cmd = %.1f\n", sel + 1, cmd);
  }
}
