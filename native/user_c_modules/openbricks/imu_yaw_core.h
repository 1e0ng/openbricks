// SPDX-License-Identifier: MIT
//
// imu_yaw_core — gyro-Z heading integration for the hard tick.
//
// The Pybricks-Prime architecture: a RAW 6-axis IMU (ICM-45686 over
// SPI — a blocking read is ~3 µs at 24 MHz, legal in the hard
// context) feeds its Z rate here every tick; this core owns bias
// estimation and integration, and the drivebase reads the resulting
// continuous body heading every millisecond — no Python in the
// loop, unlike the BNO055 path whose fused heading is pumped from
// ``done()`` polling at ~50-100 Hz.
//
// Bias handling (the whole game for integrated gyros):
//   * An EWMA of the raw rate (~20 ms horizon) is the stillness
//     reference. A sample is "still" when it sits within
//     OB_YAW_STILL_BAND_DPS of that mean AND the mean itself is
//     under OB_YAW_STILL_RATE_DPS (a robot turning slowly at a
//     constant rate must not have its motion eaten as bias).
//   * After OB_YAW_STILL_MS of continuous stillness, the bias slews
//     toward the mean — FAST until the first lock (boot
//     calibration: the robot is usually at rest when powered on),
//     SLOW afterwards (temperature drift tracking).
//   * Bias is clamped to ±OB_YAW_BIAS_MAX_DPS: a real bias beyond
//     that means a broken part, not a calibration target.
//   * Before the first lock, integration runs with bias 0 — a robot
//     that starts moving immediately gets relative heading with
//     uncorrected drift until its first stillness. Documented
//     trade; the launcher idle time before a run makes it rare.
//
// ``scale`` multiplies the incoming rate — it carries the mounting
// sign (the project is CW-positive viewed from above; a Z-up
// right-hand-rule gyro reads CCW-positive, so a top-mounted part
// typically needs -1.0) and any sensitivity trim.
//
// Context contract: pure C, no MicroPython, no allocation — hard
// tick + unix/c-unit testable.

#pragma once

#include <stdint.h>

#include "trajectory_core.h"   // ob_float_t

#define OB_YAW_MEAN_ALPHA      0.05    // EWMA per 1 kHz sample (~20 ms)
#define OB_YAW_STILL_BAND_DPS  1.0    // sample-to-mean band
#define OB_YAW_STILL_RATE_DPS  3.0    // |mean| ceiling for stillness
#define OB_YAW_STILL_MS        500.0  // hold before bias updates
#define OB_YAW_BIAS_FAST_ALPHA 0.05   // pre-first-lock slew, per sample
#define OB_YAW_BIAS_SLOW_ALPHA 0.002  // post-lock tracking, per sample
#define OB_YAW_BIAS_MAX_DPS    5.0

typedef struct {
    ob_float_t yaw_deg;       // continuous heading, CW-positive
    ob_float_t scale;         // rate multiplier incl. mounting sign
    ob_float_t bias_dps;      // learned zero-rate offset (raw frame)
    ob_float_t mean_dps;      // EWMA stillness reference (raw frame)
    ob_float_t still_ms;      // consecutive stillness
    uint8_t    bias_locked;   // first calibration achieved
} ob_yaw_t;

void ob_yaw_init(ob_yaw_t *y, ob_float_t scale);

// One sample: learn bias when still, integrate (rate - bias) * scale.
// ``dt_ms`` is the REAL elapsed time (the hard tick is nominally
// 1 ms; jitter integrates correctly by measuring).
void ob_yaw_feed(ob_yaw_t *y, ob_float_t dt_ms, ob_float_t rate_dps);

static inline ob_float_t ob_yaw_deg(const ob_yaw_t *y) {
    return y->yaw_deg;
}

// Zero the heading (bias and lock state survive — resetting the
// frame must not throw away the calibration).
void ob_yaw_reset(ob_yaw_t *y);
