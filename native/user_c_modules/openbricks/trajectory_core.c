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
// Profiles end at ``v3`` (2.5.0, Pybricks Stop.NONE): 0 for a
// stopping move, or a carried speed so the NEXT command picks up
// where this one leaves off — chained maneuvers without stopping.
//
// Segment shape (velocities relative to the move's direction; v0
// may be negative — moving the wrong way — or above cruise):
//
//   A: [0, tA)            v0 -> v_peak at ±accel
//   B: [tA, tA+t_cruise)  hold v_peak
//   C: [..., t_total)     v_peak -> v3 at -accel
//
// When the distance cannot absorb slowing from v0 to v3 at the
// configured accel (D < (v0²-v3²)/2a), the deceleration is raised
// for that one move to exactly (v0²-v3²)/(2D): a pure, steeper ramp
// that lands at the end speed precisely on target — continuous from
// the speed the axis is actually doing, never overshooting, never
// stepping the feed-forward down.

#include <math.h>

#include "trajectory_core.h"

void ob_trajectory_init_v0v3(ob_trajectory_t *t,
                             ob_float_t start,
                             ob_float_t target,
                             ob_float_t cruise,
                             ob_float_t accel,
                             ob_float_t v0_world,
                             ob_float_t v3_end) {
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
    // End speed is a magnitude along the move's direction, bounded
    // by cruise (you can't carry faster than you cruise).
    ob_float_t v3 = (v3_end < 0) ? 0.0 : v3_end;
    if (v3 > t->cruise) {
        v3 = t->cruise;
    }
    t->v3 = v3;

    if (D == 0.0 || t->cruise == 0.0 || t->accel == 0.0) {
        // Degenerate — no motion (same contract as always; arming a
        // zero move while in motion is the caller's residual to own).
        t->v0         = 0.0;
        t->v3         = 0.0;
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

    // Entry speed the distance cannot absorb: slowing from v0 to v3
    // takes (v0²-v3²)/2a, so beyond that the configured accel cannot
    // land inside D. Raise the deceleration for THIS move to exactly
    // (v0²-v3²)/(2D) — the value that lands at the end speed
    // precisely on target — and keep the true entry speed. pbio
    // clamps w0 instead (bind_w0 against max(accel, decel)), which
    // steps the feed-forward down once; the steeper ramp brakes
    // harder but stays continuous from the speed the robot is
    // actually doing (user decision 2026-08-17, replacing the 2.0.1
    // clamp). With the raised a the segment math below degenerates
    // to a pure deceleration: v_peak = v0, no entry ramp, no cruise.
    if (v0 > v3) {
        ob_float_t need = (v0 * v0 - v3 * v3) / (2.0 * a);
        if (need > D) {
            a        = (v0 * v0 - v3 * v3) / (2.0 * D);
            t->accel = a;
        }
    }

    // Net displacement of a monotonic ramp v0 -> v at ±a is
    // (v² - v0²) / (2a) — the algebra holds for signed v0.
    ob_float_t d_entry_trap = ((vc * vc) - (v0 * v0)) / (2.0 * a);
    if (d_entry_trap < 0.0) {
        d_entry_trap = -d_entry_trap;   // v0 > vc: entry is a decel
    }
    ob_float_t d_exit = ((vc * vc) - (v3 * v3)) / (2.0 * a);

    if (d_entry_trap + d_exit <= D) {
        // Full trapezoid at cruise.
        t->triangular = false;
        t->v_peak     = vc;
        t->a_entry    = (v0 <= vc) ? a : -a;
        t->t_entry    = ((v0 <= vc) ? (vc - v0) : (v0 - vc)) / a;
        // Net entry displacement (vc^2 - v0^2) / (2 * a_entry) — the
        // SIGNED acceleration, so a faster-than-cruise entry (decel
        // ramp, a_entry = -a) yields the POSITIVE distance it truly
        // covers. Dividing by +a stored it negative and shifted every
        // cruise/exit sample backward by twice the ramp's length —
        // bench 2026-08-17 (flight recorder): line-follow at ~800 dps
        // handing into a cruise-643 curve ran the whole move against
        // a reference 150 wheel-deg short, then the endpoint snapped
        // forward 150.0 at expiry and the settle walked the robot in:
        // THE end-of-run twitch, misdiagnosed twice as plant lag.
        t->d_entry    = ((vc * vc) - (v0 * v0)) / (2.0 * t->a_entry);
        t->t_cruise   = (D - d_entry_trap - d_exit) / vc;
        t->t_ramp     = (vc - v3) / a;           // exit ramp
        t->d_ramp     = d_exit;
        t->t_total    = t->t_entry + t->t_cruise + t->t_ramp;
        return;
    }

    // Cruise unreachable. Peak that fits D from v0 down to v3:
    //   (vp² - v0²)/2a + (vp² - v3²)/2a = D
    //   =>  vp = sqrt((2aD + v0² + v3²) / 2)
    ob_float_t vp2 = (2.0 * a * D + v0 * v0 + v3 * v3) / 2.0;
    ob_float_t vp  = ob_sqrt(vp2 > 0.0 ? vp2 : 0.0);

    // The decel raise above guarantees vp >= v0 AND vp >= v3 here:
    // vp² - v0² = (2aD + v3² - v0²)/2 >= 0 once (v0²-v3²) <= 2aD,
    // and vp² - v3² = (2aD + v0² - v3²)/2 >= 0 whenever v0 >= v3
    // (for v0 < v3 the entry ramp supplies the difference).
    t->triangular = true;
    t->v_peak     = vp;
    t->a_entry    = a;
    t->t_entry    = (vp - v0) / a;
    t->d_entry    = ((vp * vp) - (v0 * v0)) / (2.0 * a);
    t->t_cruise   = 0.0;
    t->t_ramp     = (vp - v3) / a;               // exit ramp
    t->d_ramp     = ((vp * vp) - (v3 * v3)) / (2.0 * a);
    t->t_total    = t->t_entry + t->t_ramp;
}

void ob_trajectory_init_v0(ob_trajectory_t *t,
                           ob_float_t start,
                           ob_float_t target,
                           ob_float_t cruise,
                           ob_float_t accel,
                           ob_float_t v0_world) {
    ob_trajectory_init_v0v3(t, start, target, cruise, accel,
                            v0_world, 0.0);
}

void ob_trajectory_init(ob_trajectory_t *t,
                        ob_float_t start,
                        ob_float_t target,
                        ob_float_t cruise,
                        ob_float_t accel) {
    ob_trajectory_init_v0v3(t, start, target, cruise, accel, 0.0, 0.0);
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
        // A carrying profile keeps its end speed; extending the
        // position reference past the target is the caller's job.
        abs_vel = (t->t_total > 0.0) ? t->v3 : 0.0;
    } else if (t_s < t->t_entry) {
        // Entry ramp: v0 toward v_peak at a_entry.
        abs_vel = t->v0 + t->a_entry * t_s;
        abs_pos = t->v0 * t_s + 0.5 * t->a_entry * t_s * t_s;
    } else if (t_s < t->t_entry + t->t_cruise) {
        abs_vel = t->v_peak;
        abs_pos = t->d_entry + t->v_peak * (t_s - t->t_entry);
    } else {
        // Exit ramp: v_peak -> v3 at -accel.
        ob_float_t td = t_s - t->t_entry - t->t_cruise;
        abs_vel = t->v_peak - t->accel * td;
        if (abs_vel < t->v3) {
            abs_vel = t->v3;
        }
        ob_float_t d_before = t->d_entry + t->v_peak * t->t_cruise;
        abs_pos = d_before + t->v_peak * td - 0.5 * t->accel * td * td;
    }

    *pos_out = t->start   + t->direction * abs_pos;
    *vel_out = t->direction * abs_vel;
}
