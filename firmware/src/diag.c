// diag.c -- OSCAR rear encoders (PIO), wheel speed, pack voltage.
// Call diag_init() once after stdio_init_all(), diag_poll() every pass of the
// main loop, and route RUN-mode keys to diag_handle_key().
//   e = encoder counts, speeds, raw pin levels
//   z = zero encoder counts
//   v = toggle a 10 Hz speed stream (does NOT refresh the command timeout)
//   a = ADC raw, pin voltage, pack voltage
//   r = RR pin sampler, 20 samples 200 us apart
//
// Encoders: quadrature_encoder.pio from pico-examples on PIO0 SM0/SM1. It has
// .origin 0, so nothing may load into PIO0 ahead of it. Only the 10 ms timer
// reads the PIO; everything else uses the values it stores.
//
// Pack voltage: sampled in diag_poll (main loop, never in the IRQ) every
// 20 ms and low-passed. test_motor_single.c derives duty_max from it.
//
// Sign: enc_invert makes a positive robot-forward cmd count UP, the same
// convention as the motor invert flags.

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "oscar_hw.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"
#include <stdio.h>

#define NUM_ENC 2
static const uint enc_base[NUM_ENC] = {RL_ENC_BASE, RR_ENC_BASE};
// UNCONFIRMED after the rewire (encoders moved pins and A/B order flipped).
// Drive each rear wheel forward with 'e' and flip any that count down.
static const bool enc_invert[NUM_ENC] = {false, true};
static const char *const enc_name[NUM_ENC] = {"RL", "RR"};
#define ENC_PIO pio0

#define COUNTS_PER_MM (COUNTS_PER_REV / WHEEL_CIRC_MM) // ~9.80
#define SPEED_PERIOD_MS 10
#define SPEED_ALPHA 0.2f // ~50 ms time constant
#define STREAM_PERIOD_MS 100
#define ADC_PERIOD_MS 20
#define PACK_ALPHA 0.3f
#define RAW_TO_PIN_V (3.3f / 4095.0f)

static volatile int32_t enc_raw[NUM_ENC]; // last PIO count, timer writes
static volatile float enc_speed[NUM_ENC]; // mm/s, filtered, signed
static int32_t enc_zero[NUM_ENC];         // raw value at last 'z'
static repeating_timer_t speed_timer;

static bool stream_on = false;
static absolute_time_t next_stream, next_adc;
static float pack_v; // filtered pack voltage

static int32_t apply_sign(int i, int32_t v) { return enc_invert[i] ? -v : v; }

static bool speed_tick(repeating_timer_t *t) {
  (void)t;
  for (int i = 0; i < NUM_ENC; i++) {
    int32_t raw = quadrature_encoder_get_count(ENC_PIO, i);
    // Unsigned subtraction so a counter wrap can't hit signed overflow.
    int32_t d = (int32_t)((uint32_t)raw - (uint32_t)enc_raw[i]);
    enc_raw[i] = raw;
    float v = apply_sign(i, d) * (1000.0f / SPEED_PERIOD_MS) / COUNTS_PER_MM;
    enc_speed[i] += SPEED_ALPHA * (v - enc_speed[i]);
  }
  return true;
}

static float adc_raw_avg(void) {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++)
    sum += adc_read();
  return sum / 16.0f;
}

// ---- Public -------------------------------------------------------------

int32_t diag_count(int i) { return apply_sign(i, enc_raw[i] - enc_zero[i]); }
float diag_speed_mm_s(int i) { return enc_speed[i]; }
float diag_pack_v(void) { return pack_v; }

void diag_init(void) {
  pio_add_program(ENC_PIO, &quadrature_encoder_program); // lands at 0
  for (int i = 0; i < NUM_ENC; i++) {
    pio_sm_claim(ENC_PIO, i);
    quadrature_encoder_program_init(ENC_PIO, i, enc_base[i], 0);
  }
  // Negative period: 10 ms between callback starts, not between ends.
  add_repeating_timer_ms(-SPEED_PERIOD_MS, speed_tick, NULL, &speed_timer);

  adc_init();
  adc_gpio_init(BATT_ADC_PIN);
  adc_select_input(BATT_ADC_PIN - 26); // only ADC user, select once
  pack_v = adc_raw_avg() * RAW_TO_PIN_V * DIVIDER_RATIO; // seed the filter

  next_adc = next_stream = get_absolute_time();
}

void diag_poll(void) {
  if (time_reached(next_adc)) {
    next_adc = make_timeout_time_ms(ADC_PERIOD_MS);
    float v = adc_raw_avg() * RAW_TO_PIN_V * DIVIDER_RATIO;
    pack_v += PACK_ALPHA * (v - pack_v);
  }
  if (stream_on && time_reached(next_stream)) {
    next_stream = make_timeout_time_ms(STREAM_PERIOD_MS);
    for (int i = 0; i < NUM_ENC; i++) {
      float v = enc_speed[i];
      printf("%s %+7.1f mm/s %+6.1f rpm   ", enc_name[i], v,
             v / WHEEL_CIRC_MM * 60.0f);
    }
    printf("\n");
  }
}

bool diag_handle_key(int c) {
  switch (c) {
  case 'r': // RR pin levels, base pin first
    for (int k = 0; k < 20; k++) {
      printf("%d%d ", gpio_get(enc_base[1]), gpio_get(enc_base[1] + 1));
      sleep_us(200);
    }
    printf("\n");
    return true;
  case 'e':
    for (int i = 0; i < NUM_ENC; i++)
      printf("%s: count=%ld  speed=%+.1f mm/s  GP%u=%d GP%u=%d\n", enc_name[i],
             (long)diag_count(i), enc_speed[i], enc_base[i],
             gpio_get(enc_base[i]), enc_base[i] + 1, gpio_get(enc_base[i] + 1));
    return true;
  case 'v':
    stream_on = !stream_on;
    next_stream = get_absolute_time();
    printf("speed stream %s\n", stream_on ? "ON" : "OFF");
    return true;
  case 'a': {
    float raw = adc_raw_avg();
    printf("ADC%d raw=%.0f  pin=%.3f V  pack=%.2f V (filtered %.2f V)\n",
           BATT_ADC_PIN - 26, raw, raw * RAW_TO_PIN_V,
           raw * RAW_TO_PIN_V * DIVIDER_RATIO, pack_v);
    return true;
  }
  case 'z':
    for (int i = 0; i < NUM_ENC; i++)
      enc_zero[i] = enc_raw[i];
    printf("encoder counts zeroed\n");
    return true;
  default:
    return false;
  }
}
