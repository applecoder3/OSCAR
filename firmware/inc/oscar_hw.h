// oscar_hw.h -- OSCAR pin map and chassis constants. One source of truth for
// test_motor_single.c and diag.c. Pin map from the 2026-11-16 rewire
// (DRV8871 boards soldered to the perfboard, no sockets).
#pragma once

// UART0 console to the Pi Zero 2 W: GP0 TX, GP1 RX.
// GP4/GP5 unused. GP5 reads ~2 ohm to GND on this Pico -- never drive it.

// Motor IN pins. Every pair is (even, odd) so IN1/IN2 share one PWM slice:
// RL 2/3 = slice 1, FL 6/7 = slice 3, FR 8/9 = slice 4, RR 14/15 = slice 7.
#define FL_IN1 7
#define FL_IN2 6
#define FR_IN1 9
#define FR_IN2 8
#define RL_IN1 3
#define RL_IN2 2
#define RR_IN1 15
#define RR_IN2 14

// Rear encoders. quadrature_encoder.pio reads two adjacent pins starting at
// a base pin. On this wiring the lower pin is ENCB (C2), so the PIO's "A" is
// the motor's B. That only flips the count sign -- fix it with enc_invert in
// diag.c, never by rewiring.
#define RL_ENC_BASE 12 // ENCB GP12, ENCA GP13
#define RR_ENC_BASE 10 // ENCB GP10, ENCA GP11

// Pack sense divider on GP28 / ADC2, ratio measured at 12.08 V in.
#define BATT_ADC_PIN 28
#define DIVIDER_RATIO (12.08f / 2.12f) // ~5.70 (nominal 122/22 = 5.55)

// OSOYOO 2024005900: 11 PPR x 4 edges x 56:1. Bench-checked ~2470.
#define COUNTS_PER_REV 2464.0f
#define WHEEL_DIAM_MM 80.0f
#define WHEEL_CIRC_MM (3.14159265f * WHEEL_DIAM_MM)

#define MOTOR_V_RATED 12.0f
