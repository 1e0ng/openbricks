// SPDX-License-Identifier: MIT
// imu_yaw_core — implementation. See header for the contract.

#include "imu_yaw_core.h"

#include <string.h>


void ob_yaw_init(ob_yaw_t *y, ob_float_t scale) {
    memset(y, 0, sizeof(*y));
    y->scale = scale;
}


void ob_yaw_reset(ob_yaw_t *y) {
    y->yaw_deg = (ob_float_t)0.0;
}


void ob_yaw_feed(ob_yaw_t *y, ob_float_t dt_ms, ob_float_t rate_dps) {
    if (dt_ms <= (ob_float_t)0.0) {
        return;
    }

    // Stillness reference: short-horizon EWMA of the raw rate.
    y->mean_dps += (rate_dps - y->mean_dps)
                   * (ob_float_t)OB_YAW_MEAN_ALPHA;

    ob_float_t dev = rate_dps - y->mean_dps;
    ob_float_t mag = y->mean_dps;
    if (dev < 0) {
        dev = -dev;
    }
    if (mag < 0) {
        mag = -mag;
    }
    if (dev < (ob_float_t)OB_YAW_STILL_BAND_DPS
        && mag < (ob_float_t)OB_YAW_STILL_RATE_DPS) {
        y->still_ms += dt_ms;
    } else {
        y->still_ms = (ob_float_t)0.0;
    }

    if (y->still_ms >= (ob_float_t)OB_YAW_STILL_MS) {
        ob_float_t alpha = y->bias_locked
                           ? (ob_float_t)OB_YAW_BIAS_SLOW_ALPHA
                           : (ob_float_t)OB_YAW_BIAS_FAST_ALPHA;
        y->bias_dps += (y->mean_dps - y->bias_dps) * alpha;
        if (y->bias_dps > (ob_float_t)OB_YAW_BIAS_MAX_DPS) {
            y->bias_dps = (ob_float_t)OB_YAW_BIAS_MAX_DPS;
        } else if (y->bias_dps < -(ob_float_t)OB_YAW_BIAS_MAX_DPS) {
            y->bias_dps = -(ob_float_t)OB_YAW_BIAS_MAX_DPS;
        }
        y->bias_locked = 1;
    }

    y->yaw_deg += (rate_dps - y->bias_dps) * y->scale
                  * dt_ms / (ob_float_t)1000.0;
}
