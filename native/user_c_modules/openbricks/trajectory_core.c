// SPDX-License-Identifier: MIT
//
// Trapezoidal speed profile — portable pure-C implementation shared
// between the firmware's MicroPython ``user_c_module`` and the
// openbricks-sim CPython extension. The algorithm matches pbio's
// ``pbio/src/trajectory.c``.
//
// Profile cases:
//
//   * Trapezoidal (distance large enough to reach cruise speed):
//     [0, t_ramp):           accelerate from 0 to v_cruise
//     [t_ramp, t_ramp+t_c):  hold at v_cruise
//     [t_ramp+t_c, t_total): decelerate from v_cruise to 0
//
//   * Triangular (distance too short to reach cruise speed):
//     [0, t_peak):           accelerate from 0 to v_peak
//     [t_peak, 2*t_peak):    decelerate from v_peak to 0
//     where v_peak = sqrt(distance * accel)
//
// Negative distance flips the direction of position and velocity; the
// time/phase arithmetic operates on the magnitude.

#include <math.h>

#include "trajectory_core.h"

void ob_trajectory_init(ob_trajectory_t *t,
                        ob_float_t start,
                        ob_float_t target,
                        ob_float_t cruise,
                        ob_float_t accel) {
    t->start    = start;
    t->distance = target - start;
    t->cruise   = (cruise < 0) ? -cruise : cruise;
    t->accel    = (accel  < 0) ? -accel  : accel;

    ob_float_t D = (t->distance < 0) ? -t->distance : t->distance;
    t->direction = (t->distance < 0) ? -1.0 : 1.0;

    if (D == 0.0 || t->cruise == 0.0 || t->accel == 0.0) {
        // Degenerate — no motion.
        t->t_ramp     = 0.0;
        t->t_cruise   = 0.0;
        t->t_total    = 0.0;
        t->d_ramp     = 0.0;
        t->v_peak     = 0.0;
        t->triangular = false;
        return;
    }

    ob_float_t t_ramp_full = t->cruise / t->accel;
    ob_float_t d_ramp_full = 0.5 * t->accel * t_ramp_full * t_ramp_full;

    if (2.0 * d_ramp_full <= D) {
        t->triangular = false;
        t->t_ramp     = t_ramp_full;
        t->d_ramp     = d_ramp_full;
        t->v_peak     = t->cruise;
        t->t_cruise   = (D - 2.0 * d_ramp_full) / t->cruise;
        t->t_total    = 2.0 * t->t_ramp + t->t_cruise;
    } else {
        t->triangular = true;
        ob_float_t t_peak = ob_sqrt(D / t->accel);
        t->t_ramp   = t_peak;
        t->t_cruise = 0.0;
        t->t_total  = 2.0 * t_peak;
        t->v_peak   = t->accel * t_peak;
        t->d_ramp   = 0.5 * t->accel * t_peak * t_peak;  // = D/2
    }
}

void ob_trajectory_sample(const ob_trajectory_t *t,
                          ob_float_t t_s,
                          ob_float_t *pos_out,
                          ob_float_t *vel_out) {
    ob_float_t abs_pos;
    ob_float_t abs_vel;

    if (t_s <= 0.0) {
        abs_pos = 0.0;
        abs_vel = 0.0;
    } else if (t_s >= t->t_total) {
        abs_pos = (t->distance < 0) ? -t->distance : t->distance;
        abs_vel = 0.0;
    } else if (t_s < t->t_ramp) {
        // Accel phase: v = a*t, x = 0.5*a*t^2
        abs_vel = t->accel * t_s;
        abs_pos = 0.5 * t->accel * t_s * t_s;
    } else if (!t->triangular && t_s < t->t_ramp + t->t_cruise) {
        // Cruise phase
        abs_vel = t->v_peak;
        abs_pos = t->d_ramp + t->v_peak * (t_s - t->t_ramp);
    } else {
        // Decel phase
        ob_float_t decel_start = t->triangular
                                 ? t->t_ramp
                                 : t->t_ramp + t->t_cruise;
        ob_float_t td = t_s - decel_start;
        abs_vel = t->v_peak - t->accel * td;
        if (abs_vel < 0.0) {
            abs_vel = 0.0;
        }
        ob_float_t d_before_decel = t->triangular
                                    ? t->d_ramp
                                    : t->d_ramp + t->v_peak * t->t_cruise;
        abs_pos = d_before_decel + t->v_peak * td - 0.5 * t->accel * td * td;
    }

    *pos_out = t->start   + t->direction * abs_pos;
    *vel_out = t->direction * abs_vel;
}
