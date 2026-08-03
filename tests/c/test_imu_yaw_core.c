// SPDX-License-Identifier: MIT
// Native tests for imu_yaw_core — synthetic gyro streams: bias
// learning, stillness gating, integration accuracy, sign/scale.

#include <math.h>

#include "harness.h"
#include "imu_yaw_core.h"

static ob_yaw_t y;

static void feed_ms(ob_float_t rate, int ms) {
    for (int i = 0; i < ms; i++) {
        ob_yaw_feed(&y, 1.0, rate);
    }
}

TEST(constant_rate_integrates_to_angle) {
    ob_yaw_init(&y, 1.0);
    feed_ms(90.0, 1000);           // 90 dps for 1 s
    CHECK(fabs((double)ob_yaw_deg(&y) - 90.0) < 0.5);
}

TEST(rest_bias_is_learned_and_cancelled) {
    ob_yaw_init(&y, 1.0);
    // 1.2 dps of standing bias for 3 s of rest: fast initial lock
    // should eat it. Some pre-lock drift integrates (documented);
    // after the lock the yaw must stop moving.
    feed_ms(1.2, 3000);
    CHECK(y.bias_locked == 1);
    CHECK(fabs((double)y.bias_dps - 1.2) < 0.1);
    ob_float_t before = ob_yaw_deg(&y);
    feed_ms(1.2, 2000);            // two more seconds at rest
    CHECK(fabs((double)(ob_yaw_deg(&y) - before)) < 0.1);
}

TEST(motion_does_not_corrupt_the_bias) {
    ob_yaw_init(&y, 1.0);
    feed_ms(1.0, 2000);            // learn ~1.0 dps at rest
    ob_float_t learned = y.bias_dps;
    feed_ms(46.0, 2000);           // then a 45-dps turn (46 raw)
    CHECK(fabs((double)(y.bias_dps - learned)) < 0.05);
    // Yaw advanced by (46 - ~1.0) * 2 s = ~90.
    CHECK(fabs((double)ob_yaw_deg(&y) - 90.0) < 3.0);
}

TEST(slow_constant_creep_is_not_eaten_as_bias) {
    // A robot turning at a steady 4 dps (above STILL_RATE) must not
    // have its motion calibrated away, no matter how stable.
    ob_yaw_init(&y, 1.0);
    feed_ms(4.0, 5000);
    CHECK(y.bias_locked == 0);
    CHECK(fabs((double)ob_yaw_deg(&y) - 20.0) < 0.5);
}

TEST(noisy_rest_blocks_the_stillness_clock) {
    // Alternating +-3 dps around zero: mean ~0 but samples leave the
    // band, so stillness never accumulates and bias stays unlocked.
    ob_yaw_init(&y, 1.0);
    for (int i = 0; i < 4000; i++) {
        ob_yaw_feed(&y, 1.0, (i & 1) ? 3.0 : -3.0);
    }
    CHECK(y.bias_locked == 0);
}

TEST(scale_carries_mounting_sign) {
    ob_yaw_init(&y, -1.0);         // top-mounted: CCW-positive gyro
    feed_ms(90.0, 1000);
    CHECK(fabs((double)ob_yaw_deg(&y) + 90.0) < 0.5);
}

TEST(bias_target_is_bounded_by_the_stillness_gate) {
    // Learning only engages while |mean| < STILL_RATE, so the bias
    // can converge to a target just under the gate but never chase
    // anything beyond it (no clamp needed by construction).
    ob_yaw_init(&y, 1.0);
    feed_ms(2.9, 10000);
    CHECK(fabs((double)y.bias_dps - 2.9) < 0.1);
    CHECK((double)y.bias_dps < OB_YAW_STILL_RATE_DPS);
}

TEST(dt_jitter_integrates_by_measured_time) {
    ob_yaw_init(&y, 1.0);
    // 90 dps for exactly 1 s of accumulated time, in irregular
    // slices (0.5 / 2.0 ms alternating).
    ob_float_t total = 0.0;
    int i = 0;
    while (total < 1000.0) {
        ob_float_t dt = (i & 1) ? 2.0 : 0.5;
        if (total + dt > 1000.0) {
            dt = 1000.0 - total;
        }
        ob_yaw_feed(&y, dt, 90.0);
        total += dt;
        i++;
    }
    CHECK(fabs((double)ob_yaw_deg(&y) - 90.0) < 0.5);
}

TEST(reset_zeroes_yaw_but_keeps_calibration) {
    ob_yaw_init(&y, 1.0);
    feed_ms(0.8, 3000);            // learn bias at rest
    CHECK(y.bias_locked == 1);
    feed_ms(45.8, 1000);           // turn
    ob_yaw_reset(&y);
    CHECK(fabs((double)ob_yaw_deg(&y)) < 1e-9);
    CHECK(y.bias_locked == 1);
    CHECK(fabs((double)y.bias_dps - 0.8) < 0.1);
}

TEST(zero_or_negative_dt_is_ignored) {
    ob_yaw_init(&y, 1.0);
    ob_yaw_feed(&y, 0.0, 500.0);
    ob_yaw_feed(&y, -3.0, 500.0);
    CHECK(fabs((double)ob_yaw_deg(&y)) < 1e-9);
}

int main(void) {
    RUN(constant_rate_integrates_to_angle);
    RUN(rest_bias_is_learned_and_cancelled);
    RUN(motion_does_not_corrupt_the_bias);
    RUN(slow_constant_creep_is_not_eaten_as_bias);
    RUN(noisy_rest_blocks_the_stillness_clock);
    RUN(scale_carries_mounting_sign);
    RUN(bias_target_is_bounded_by_the_stillness_gate);
    RUN(dt_jitter_integrates_by_measured_time);
    RUN(reset_zeroes_yaw_but_keeps_calibration);
    RUN(zero_or_negative_dt_is_ignored);
    return harness_exit("imu_yaw_core");
}
