// SPDX-License-Identifier: MIT
//
// Trapezoidal speed profile — portable pure-C implementation shared
// between the firmware's MicroPython ``user_c_module`` and the
// openbricks-sim CPython extension. The algorithm matches pbio's
// ``pbio/src/trajectory.c``, including nonzero entry speed.
//
// Profiles start from ``v0`` — the speed the axis is ALREADY moving
// at when the move is armed (0 for a standing start). Without this,
// arming straight() while the wheels cruise (a line-follow loop
// handing over to a trajectory move) cliffed the commanded speed
// from cruise to ~zero in one tick, and the duty-mode FF+PI braked
// as hard as the plant allowed — far beyond settings.acceleration
// (bench 2026-08-16: "the deceleration is too much while the
// acceleration is correct").
//
// Segment shape (velocities relative to the move's direction; v0
// may be negative — moving the wrong way — or above cruise):
//
//   A: [0, tA)            v0 -> v_peak at ±accel
//   B: [tA, tA+t_cruise)  hold v_peak
//   C: [..., t_total)     v_peak -> 0 at -accel
//
// When the distance cannot even absorb stopping from v0
// (D < v0²/2a), the profile is a pure deceleration that OVERSHOOTS
// the target by the physics-mandated margin; position feedback pulls
// the residual back after the profile expires. That is the honest
// answer — the alternative was violating the acceleration limit.

#include <math.h>

#include "trajectory_core.h"

void ob_trajectory_init_v0(ob_trajectory_t *t,
                           ob_float_t start,
                           ob_float_t target,
                           ob_float_t cruise,
                           ob_float_t accel,
                           ob_float_t v0_world) {
    t->start    = start;
    t->distance = target - start;
    t->cruise   = (cruise < 0) ? -cruise : cruise;
    t->accel    = (accel  < 0) ? -accel  : accel;

    ob_float_t D = (t->distance < 0) ? -t->distance : t->distance;
    t->direction = (t->distance < 0) ? -1.0 : 1.0;
    // Entry speed relative to the move's direction: positive =
    // already moving toward the target.
    ob_float_t v0 = v0_world * t->direction;
    t->v0 = v0;

    if (D == 0.0 || t->cruise == 0.0 || t->accel == 0.0) {
        // Degenerate — no motion (same contract as always; arming a
        // zero move while in motion is the caller's residual to own).
        t->v0         = 0.0;
        t->t_entry    = 0.0;
        t->a_entry    = 0.0;
        t->d_entry    = 0.0;
        t->t_ramp     = 0.0;
        t->t_cruise   = 0.0;
        t->t_total    = 0.0;
        t->d_ramp     = 0.0;
        t->v_peak     = 0.0;
        t->triangular = false;
        return;
    }

    ob_float_t a  = t->accel;
    ob_float_t vc = t->cruise;

    // Net displacement of a monotonic ramp v0 -> v at ±a is
    // (v² - v0²) / (2a) — the algebra holds for signed v0.
    ob_float_t d_entry_trap = ((vc * vc) - (v0 * v0)) / (2.0 * a);
    if (d_entry_trap < 0.0) {
        d_entry_trap = -d_entry_trap;   // v0 > vc: entry is a decel
    }
    ob_float_t d_exit = (vc * vc) / (2.0 * a);

    if (d_entry_trap + d_exit <= D) {
        // Full trapezoid at cruise.
        t->triangular = false;
        t->v_peak     = vc;
        t->a_entry    = (v0 <= vc) ? a : -a;
        t->t_entry    = ((v0 <= vc) ? (vc - v0) : (v0 - vc)) / a;
        t->d_entry    = ((vc * vc) - (v0 * v0)) / (2.0 * a);
        t->t_cruise   = (D - d_entry_trap - d_exit) / vc;
        t->t_ramp     = vc / a;                  // exit ramp
        t->d_ramp     = d_exit;
        t->t_total    = t->t_entry + t->t_cruise + t->t_ramp;
        return;
    }

    // Cruise unreachable. Peak that fits D from v0:
    //   (vp² - v0²)/2a + vp²/2a = D  =>  vp = sqrt((2aD + v0²) / 2)
    ob_float_t vp2 = (2.0 * a * D + v0 * v0) / 2.0;
    ob_float_t vp  = ob_sqrt(vp2 > 0.0 ? vp2 : 0.0);

    if (v0 > 0.0 && vp < v0) {
        // Cannot even stop within D: pure deceleration, overshoot
        // owned by position feedback after expiry.
        t->triangular = true;
        t->v_peak     = v0;
        t->a_entry    = -a;
        t->t_entry    = v0 / a;
        t->d_entry    = (v0 * v0) / (2.0 * a);
        t->t_cruise   = 0.0;
        t->t_ramp     = 0.0;
        t->d_ramp     = 0.0;
        t->t_total    = t->t_entry;
        return;
    }

    t->triangular = true;
    t->v_peak     = vp;
    t->a_entry    = a;
    t->t_entry    = (vp - v0) / a;
    t->d_entry    = ((vp * vp) - (v0 * v0)) / (2.0 * a);
    t->t_cruise   = 0.0;
    t->t_ramp     = vp / a;                      // exit ramp
    t->d_ramp     = (vp * vp) / (2.0 * a);
    t->t_total    = t->t_entry + t->t_ramp;
}

void ob_trajectory_init(ob_trajectory_t *t,
                        ob_float_t start,
                        ob_float_t target,
                        ob_float_t cruise,
                        ob_float_t accel) {
    ob_trajectory_init_v0(t, start, target, cruise, accel, 0.0);
}

void ob_trajectory_sample(const ob_trajectory_t *t,
                          ob_float_t t_s,
                          ob_float_t *pos_out,
                          ob_float_t *vel_out) {
    ob_float_t abs_pos;
    ob_float_t abs_vel;

    if (t_s <= 0.0) {
        abs_pos = 0.0;
        abs_vel = (t->t_total > 0.0) ? t->v0 : 0.0;
    } else if (t_s >= t->t_total) {
        abs_pos = (t->distance < 0) ? -t->distance : t->distance;
        abs_vel = 0.0;
        if (t->t_cruise == 0.0 && t->t_ramp == 0.0
            && t->t_entry > 0.0) {
            // Pure-decel profile: it lands where physics allowed,
            // not exactly at the target.
            abs_pos = t->d_entry;
        }
    } else if (t_s < t->t_entry) {
        // Entry ramp: v0 toward v_peak at a_entry.
        abs_vel = t->v0 + t->a_entry * t_s;
        abs_pos = t->v0 * t_s + 0.5 * t->a_entry * t_s * t_s;
    } else if (t_s < t->t_entry + t->t_cruise) {
        abs_vel = t->v_peak;
        abs_pos = t->d_entry + t->v_peak * (t_s - t->t_entry);
    } else {
        // Exit ramp: v_peak -> 0 at -accel.
        ob_float_t td = t_s - t->t_entry - t->t_cruise;
        abs_vel = t->v_peak - t->accel * td;
        if (abs_vel < 0.0) {
            abs_vel = 0.0;
        }
        ob_float_t d_before = t->d_entry + t->v_peak * t->t_cruise;
        abs_pos = d_before + t->v_peak * td - 0.5 * t->accel * td * td;
    }

    *pos_out = t->start   + t->direction * abs_pos;
    *vel_out = t->direction * abs_vel;
}
