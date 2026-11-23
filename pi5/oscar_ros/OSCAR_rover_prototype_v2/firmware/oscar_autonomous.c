// OSCAR_V1 autonomous prototype for Ethan's Nov 16 pin map.
// Build with Ethan's diag.c + quadrature_encoder.pio.
// OUTPUTS DISABLED BY DEFAULT. Enable only after wheels-off-ground checks.
#include "oscar_hw.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef OSCAR_ENABLE_MOTOR_OUTPUT
#define OSCAR_ENABLE_MOTOR_OUTPUT 0
#endif
#define PWM_FREQ_HZ 20000u
#define CMD_TIMEOUT_MS 250u
#define TELEMETRY_PERIOD_MS 100u
#define HELLO_PERIOD_MS 500u
#define WHEEL_SPEED_LIMIT_M_S 0.50f // prototype, not measured
#define ROTATION_ARM_M 0.20f // (wheelbase+track)/2, provisional
#define MAX_VX 0.15f
#define MAX_VY 0.00f // rear-only encoder localization: no strafe
#define MAX_WZ 0.40f
#define MOTOR_V_RATED_LOCAL 12.0f

typedef struct { uint in1, in2; bool invert; } motor_t;
static const motor_t motors[4] = {
    {FL_IN1, FL_IN2, true},
    {FR_IN1, FR_IN2, false},
    {RL_IN1, RL_IN2, true}, // requires Ethan's physical validation
    {RR_IN1, RR_IN2, false}
};
static uint16_t pwm_full;
static bool armed = false;
static uint32_t last_vel_ms;
void diag_init(void);
void diag_poll(void);
int32_t diag_count(int index);
float diag_pack_v(void);

static void all_idle(void) {
    for (int i = 0; i < 4; i++) {
        gpio_init(motors[i].in1);
        gpio_put(motors[i].in1, 0);
        gpio_set_dir(motors[i].in1, GPIO_OUT);
        gpio_init(motors[i].in2);
        gpio_put(motors[i].in2, 0);
        gpio_set_dir(motors[i].in2, GPIO_OUT);
    }
}
static void pwm_init_motors(void) {
#if OSCAR_ENABLE_MOTOR_OUTPUT
    for (int i = 0; i < 4; i++) {
        uint slice = pwm_gpio_to_slice_num(motors[i].in1);
        pwm_set_clkdiv(slice, 1.0f);
        pwm_set_wrap(slice, pwm_full - 1);
        pwm_set_both_levels(slice, 0, 0);
        pwm_set_enabled(slice, true);
        gpio_set_function(motors[i].in1, GPIO_FUNC_PWM);
        gpio_set_function(motors[i].in2, GPIO_FUNC_PWM);
    }
#endif
}
static void motor_write(int index, float command) {
#if OSCAR_ENABLE_MOTOR_OUTPUT
    const motor_t *m = &motors[index];
    command = fmaxf(-1.0f, fminf(1.0f, command));
    if (m->invert) command = -command;
    float battery = diag_pack_v();
    float cap = (battery >= 9.0f && battery <= 17.5f) ? fminf(1.0f, MOTOR_V_RATED_LOCAL/battery) : 0.70f;
    uint16_t duty = (uint16_t)(fabsf(command) * cap * pwm_full + 0.5f);
    uint drive = command >= 0 ? m->in1 : m->in2;
    uint other = command >= 0 ? m->in2 : m->in1;
    if (!duty) {
        pwm_set_chan_level(pwm_gpio_to_slice_num(m->in1), pwm_gpio_to_channel(m->in1), 0);
        pwm_set_chan_level(pwm_gpio_to_slice_num(m->in2), pwm_gpio_to_channel(m->in2), 0);
    } else { // Ethan's slow-decay PWM mode
        pwm_set_chan_level(pwm_gpio_to_slice_num(drive), pwm_gpio_to_channel(drive), pwm_full);
        pwm_set_chan_level(pwm_gpio_to_slice_num(other), pwm_gpio_to_channel(other), pwm_full-duty);
    }
#else
    (void)index; (void)command;
#endif
}
static void stop(void) {
    for (int i = 0; i < 4; i++) motor_write(i, 0.0f);
}
static bool valid_velocity(float vx, float vy, float wz) {
    return isfinite(vx) && isfinite(vy) && isfinite(wz) &&
           fabsf(vx) <= MAX_VX && fabsf(vy) <= MAX_VY && fabsf(wz) <= MAX_WZ;
}
static void apply_velocity(float vx, float vy, float wz) {
    float w[4] = {vx-vy-ROTATION_ARM_M*wz,
                  vx+vy+ROTATION_ARM_M*wz,
                  vx+vy-ROTATION_ARM_M*wz,
                  vx-vy+ROTATION_ARM_M*wz};
    float peak = WHEEL_SPEED_LIMIT_M_S;
    for (int i = 0; i < 4; i++) peak = fmaxf(peak, fabsf(w[i]));
    for (int i = 0; i < 4; i++) motor_write(i, w[i]/peak);
}
static void handle_line(char *line) {
    if (!strcmp(line, "DISARM")) {
        stop(); armed = false; puts("OK DISARM"); return;
    }
    if (!strcmp(line, "STOP")) { stop(); puts("OK STOP"); return; }
    if (!strcmp(line, "ARM")) {
        stop(); armed = true; last_vel_ms = to_ms_since_boot(get_absolute_time());
        puts("OK ARM"); return;
    }
    if (!strncmp(line, "VEL ", 4)) {
        float vx, vy, wz;
        char extra;
        if (sscanf(line + 4, "%f %f %f %c", &vx, &vy, &wz, &extra) != 3 ||
            !valid_velocity(vx, vy, wz) || !armed) {
            stop(); armed = false; puts("ERR VEL"); return;
        }
        last_vel_ms = to_ms_since_boot(get_absolute_time());
        apply_velocity(vx, vy, wz);
        return;
    }
    if (!strcmp(line, "PING")) { puts("HELLO OSCAR_V1"); return; }
    stop(); armed = false; puts("ERR UNKNOWN");
}
int main(void) {
    all_idle(); // both inputs low before UART or ADC
    pwm_full = (uint16_t)(clock_get_hz(clk_sys)/PWM_FREQ_HZ);
    stdio_init_all(); // CMake target enables UART stdio and disables USB stdio
    diag_init(); // Ethan's existing PIO encoder/ADC logic
    pwm_init_motors();
    last_vel_ms = to_ms_since_boot(get_absolute_time());
    uint32_t last_enc = last_vel_ms, last_hello = last_vel_ms;
    char line[96]; size_t len = 0;
    puts("HELLO OSCAR_V1");
    while (1) {
        diag_poll();
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (armed && (uint32_t)(now - last_vel_ms) > CMD_TIMEOUT_MS) {
            stop(); armed = false; puts("ERR TIMEOUT");
        }
        if ((uint32_t)(now-last_hello) >= HELLO_PERIOD_MS) {
            last_hello = now; puts("HELLO OSCAR_V1");
        }
        if ((uint32_t)(now-last_enc) >= TELEMETRY_PERIOD_MS) {
            last_enc = now;
            printf("ENC %ld %ld\n", (long)diag_count(0), (long)diag_count(1));
        }
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) { sleep_ms(1); continue; }
        if (c == '\n' || c == '\r') {
            if (len) { line[len] = '\0'; handle_line(line); len = 0; }
        } else if (c >= 32 && c <= 126 && len + 1 < sizeof(line)) {
            line[len++] = (char)c;
        } else { len = 0; stop(); armed = false; }
    }
}
