// Bench tool for Stages 4 and 5. One binary, two modes.
//
// Console is UART0 on GP0/GP1 at 115200, NOT USB. Read it on the Pi with
//   screen /dev/serial0 115200
//
// RUN mode (default): 1-4 pick a motor, w/s step +/-10%, space stops it,
//                     x stops everything, t enters TEST mode, ? help
// TEST mode (Stage 4): 1-8 toggle one IN pin as plain GPIO so you can meter
//                      it at the driver's screw terminal, 0 drives all low,
//                      r returns to RUN mode
//
// Every mode change passes through all-pins-low first.
// duty_max is hard-coded for the 12.0 V bench supply. Swap in 12.0f/V_batt
// once the GP28 ADC read is in and the pack goes on.

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <math.h>
#include <stdio.h>

#define PWM_WRAP 6249  // 125 MHz / 6250 = 20 kHz
#define LED 25         // plain Pico only; on a Pico W this is WL_GPIO0

// ---- Pin map (from continuity check, 2026-11-08) -------------------------
// Note: IN1 is on the ODD gpio for FL, FR and RR, which means IN1 is PWM
// channel B on those three. Channels are derived per-pin below, never
// assumed, so the ordering here is free to be whatever the wiring is.
typedef struct {
    const char *name;
    uint in1;
    uint in2;
} motor_t;

static const motor_t MOTORS[] = {
    { "FL", 15, 14 },
    { "FR",  3,  2 },
    { "RL",  6,  7 },
    { "RR",  9,  8 },
};
#define N_MOTORS (sizeof(MOTORS) / sizeof(MOTORS[0]))

// Flat view of the same pins, for TEST mode. Order matches the printout.
static const uint *in_pin(int i) {
    const motor_t *m = &MOTORS[i / 2];
    return (i % 2 == 0) ? &m->in1 : &m->in2;
}
#define N_IN_PINS (N_MOTORS * 2)

// ---- Low-level ----------------------------------------------------------

// Return a pin to SIO, driven low. Value is set before the driver is
// enabled so there is no window where a stale high gets pushed out.
static void pin_idle(uint gpio) {
    gpio_init(gpio);
    gpio_put(gpio, 0);
    gpio_set_dir(gpio, GPIO_OUT);
}

static void all_idle(void) {
    for (int i = 0; i < N_IN_PINS; i++) pin_idle(*in_pin(i));
}

static void slice_prepare(uint gpio) {
    uint s = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(s, 1.0f);
    pwm_set_wrap(s, PWM_WRAP);
    pwm_set_chan_level(s, pwm_gpio_to_channel(gpio), 0);
    pwm_set_enabled(s, true);
}

// Hand one motor's two pins to the PWM block, both at 0% first.
static void motor_claim(const motor_t *m) {
    slice_prepare(m->in1);
    slice_prepare(m->in2);
    gpio_set_function(m->in1, GPIO_FUNC_PWM);
    gpio_set_function(m->in2, GPIO_FUNC_PWM);
}

static void pin_level(uint gpio, uint16_t level) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(gpio),
                       pwm_gpio_to_channel(gpio), level);
}

// cmd: -1.0 (full reverse) to 1.0 (full forward).
// Forward PWMs IN1 and holds IN2 low, whichever channel each happens to be.
static void motor_set(const motor_t *m, float cmd, float duty_max) {
    if (cmd > 1.0f) cmd = 1.0f;
    if (cmd < -1.0f) cmd = -1.0f;
    uint16_t level = (uint16_t)(fabsf(cmd) * duty_max * (PWM_WRAP + 1));
    pin_level(cmd >= 0 ? m->in1 : m->in2, level);
    pin_level(cmd >= 0 ? m->in2 : m->in1, 0);
}

// ---- Console ------------------------------------------------------------

static void print_help(void) {
    printf("\nRUN:  1-4 select motor | w/s +/-10%% | space stop | x all stop\n");
    printf("      t -> TEST mode | ? help\n");
    printf("TEST: 1-8 toggle one IN pin | 0 all low | r -> RUN mode\n\n");
}

static void print_pin_table(void) {
    printf("\nIN pins, in TEST key order:\n");
    for (int i = 0; i < N_IN_PINS; i++) {
        printf("  %d = %s IN%d  GP%u\n",
               i + 1, MOTORS[i / 2].name, (i % 2) + 1, *in_pin(i));
    }
    printf("\n");
}

int main(void) {
    all_idle();  // first action, before stdio or anything else
    stdio_init_all();

    gpio_init(LED);
    gpio_set_dir(LED, GPIO_OUT);
    gpio_put(LED, 1);

    const float duty_max = 1.0f;  // correct at 12.0 V in, for 12 V motors

    bool test_mode = false;
    int sel = -1;        // selected motor in RUN mode
    float cmd = 0.0f;
    bool pin_high[N_IN_PINS] = { false };

    printf("\nbench_motors: idle, RUN mode, no motor selected\n");
    print_help();

    while (1) {
        int c = getchar_timeout_us(0);  // non-blocking
        if (c == PICO_ERROR_TIMEOUT) continue;

        if (c == '?') { print_help(); continue; }

        if (test_mode) {
            if (c == 'r') {
                all_idle();
                for (int i = 0; i < N_IN_PINS; i++) pin_high[i] = false;
                test_mode = false;
                sel = -1;
                cmd = 0.0f;
                printf("RUN mode, all low, no motor selected\n");
            } else if (c == '0') {
                all_idle();
                for (int i = 0; i < N_IN_PINS; i++) pin_high[i] = false;
                printf("all IN pins low\n");
            } else if (c >= '1' && c <= '0' + N_IN_PINS) {
                int i = c - '1';
                pin_high[i] = !pin_high[i];
                gpio_put(*in_pin(i), pin_high[i]);
                printf("%s IN%d (GP%u) = %s\n",
                       MOTORS[i / 2].name, (i % 2) + 1, *in_pin(i),
                       pin_high[i] ? "HIGH" : "low");
            }
            continue;
        }

        // RUN mode
        if (c == 't') {
            all_idle();
            test_mode = true;
            sel = -1;
            cmd = 0.0f;
            printf("TEST mode. All pins low, plain GPIO. Meter at the "
                   "driver screw terminals.\n");
            print_pin_table();
        } else if (c == 'x') {
            all_idle();
            sel = -1;
            cmd = 0.0f;
            printf("ALL STOP, pins low\n");
        } else if (c >= '1' && c <= '0' + (int)N_MOTORS) {
            all_idle();
            sel = c - '1';
            cmd = 0.0f;
            motor_claim(&MOTORS[sel]);
            motor_set(&MOTORS[sel], 0.0f, duty_max);
            printf("selected %s (IN1 GP%u, IN2 GP%u), cmd = 0.0\n",
                   MOTORS[sel].name, MOTORS[sel].in1, MOTORS[sel].in2);
        } else if (c == 'w' || c == 's' || c == ' ') {
            if (sel < 0) { printf("no motor selected -- press 1-4\n"); continue; }
            if (c == 'w') cmd += 0.1f;
            else if (c == 's') cmd -= 0.1f;
            else cmd = 0.0f;
            if (cmd > 1.0f) cmd = 1.0f;
            if (cmd < -1.0f) cmd = -1.0f;
            motor_set(&MOTORS[sel], cmd, duty_max);
            printf("%s cmd = %.1f\n", MOTORS[sel].name, cmd);
        }
    }
}
