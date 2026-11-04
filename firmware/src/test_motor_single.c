#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h>
#include <stdio.h>

#define M1_IN1 2   // GP2 and GP3 share PWM slice 1 (channels A and B)
#define M1_IN2 3
#define PWM_WRAP 6249   // 125 MHz / 6250 = 20 kHz

static uint slice;

void motor_init(void) {
    gpio_set_function(M1_IN1, GPIO_FUNC_PWM);
    gpio_set_function(M1_IN2, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(M1_IN1);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_enabled(slice, true);
}

// cmd: -1.0 (full reverse) to 1.0 (full forward)
void motor_set(float cmd, float duty_max) {
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    uint16_t level = (uint16_t)(fabsf(cmd) * duty_max * (PWM_WRAP + 1));
    if (cmd >= 0) {
        pwm_set_chan_level(slice, PWM_CHAN_A, level);   // IN1 = PWM
        pwm_set_chan_level(slice, PWM_CHAN_B, 0);       // IN2 = 0
    } else {
        pwm_set_chan_level(slice, PWM_CHAN_A, 0);
        pwm_set_chan_level(slice, PWM_CHAN_B, level);
    }
}

int main(void) {
    stdio_init_all();
    motor_init();
    float cmd = 0.0f;
    float duty_max = 1.0f;   // swap in 12.0f / V_batt once the ADC read exists

    while (1) {
        int c = getchar_timeout_us(0);   // non-blocking: returns immediately if no key
        if (c == 'w') cmd += 0.1f;
        else if (c == 's') cmd -= 0.1f;
        else if (c == ' ') cmd = 0.0f;
        if (c != PICO_ERROR_TIMEOUT) {
            motor_set(cmd, duty_max);
            printf("cmd = %.1f\n", cmd);
        }
    }
}
