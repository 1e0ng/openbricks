// SPDX-License-Identifier: MIT
//
// drivebase_core — algorithm body for the 2-DOF coupled drivebase.
// See ``drivebase_core.h`` for the design notes; this file is just
// the math, with no MicroPython / Python.h symbols.

// Windows / MSVC hides M_PI behind this feature macro. Must come
// before <math.h>. No-op on POSIX compilers (gcc / clang) where
// M_PI is exposed unconditionally.
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdbool.h>

#include "drivebase_core.h"

// Belt-and-suspenders: even with ``_USE_MATH_DEFINES`` set, some
// embedded toolchains (older newlib variants, occasional MinGW
// configurations) still don't expose ``M_PI``. Define it inline if
// the system header didn't.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


void ob_drivebase_init(ob_drivebase_t *db,
                       ob_servo_t *left, ob_servo_t *right,
                       ob_float_t wheel_diameter_mm,
                       ob_float_t axle_track_mm,
                       ob_float_t kp_sum,
                       ob_float_t kp_diff) {
    db->left  = left;
    db->right = right;

    db->wheel_circumference_mm = (ob_float_t)M_PI * wheel_diameter_mm;
    db->axle_track_mm          = axle_track_mm;
    db->kp_sum                 = kp_sum;
    db->kp_diff                = kp_diff;
    db->accel_dps2             = (ob_float_t)OB_DRIVEBASE_DEFAULT_ACCEL_DPS2;

    ob_trajectory_init(&db->fwd,  0.0, 0.0, 0.0, 0.0);
    db->fwd_start_ms = 0;
    db->fwd_active   = false;
    db->fwd_hold     = 0.0;

    ob_trajectory_init(&db->turn, 0.0, 0.0, 0.0, 0.0);
    db->turn_start_ms = 0;
    db->turn_active   = false;
    db->turn_hold     = 0.0;

    db->use_gyro                   = false;
    db->heading_override_wheel_deg = 0.0;
    db->done                       = true;
    db->settling                   = false;
    db->settle_start_ms            = 0;

    db->integ_sum       = 0.0;
    db->integ_diff      = 0.0;
    db->ki              = (ob_float_t)OB_DRIVEBASE_DEFAULT_KI;
    db->last_tick_ms    = 0;
    db->meas_vel_sum    = 0.0;
    db->meas_vel_diff   = 0.0;
    db->prev_sum_pos    = 0.0;
    db->prev_diff_pos   = 0.0;
    db->have_prev_pos   = false;
    db->trace_next      = 0;
    db->trace_count     = 0;
    db->trace_last_ms   = 0;
    db->landings        = 0;
    db->landing_active  = false;
    db->landing_best_err = 0.0;
    db->expiry_residual = 0.0;
    db->expiry_res_sum = 0.0;
    db->expiry_res_diff = 0.0;
    db->expiry_integ_sum_dps = 0.0;
    db->expiry_integ_diff_dps = 0.0;
    db->expiry_captured = false;
}


// Per-move controller state reset — every arm (and stop) starts a
// fresh integral and landing budget, pbio-style (their integrators
// reset at maneuver start): windup from one move must not bias the
// next, and each move gets its own landing retries.
static void db_move_state_reset(ob_drivebase_t *db) {
    db->integ_sum       = 0.0;
    db->integ_diff      = 0.0;
    db->landings        = 0;
    db->landing_active  = false;
    db->landing_best_err = 0.0;
    db->expiry_residual = 0.0;
    db->expiry_res_sum = 0.0;
    db->expiry_res_diff = 0.0;
    db->expiry_integ_sum_dps = 0.0;
    db->expiry_integ_diff_dps = 0.0;
    db->expiry_captured = false;
}


// ---------------------------------------------------------------------
// Helpers — current observer-derived sum / diff positions.

static ob_float_t db_sum_pos(const ob_drivebase_t *db) {
    return (db->left->observer.pos_hat + db->right->observer.pos_hat) /
           (ob_float_t)2.0;
}


static ob_float_t db_diff_pos_encoder(const ob_drivebase_t *db) {
    return (db->left->observer.pos_hat - db->right->observer.pos_hat) /
           (ob_float_t)2.0;
}


// ---------------------------------------------------------------------
// Move setup

void ob_drivebase_straight(ob_drivebase_t *db,
                           long now_ms,
                           ob_float_t distance_mm,
                           ob_float_t speed_mm_s,
                           bool carry) {
    // Convert mm-space → wheel-degree space.
    ob_float_t distance_deg = distance_mm /
                              db->wheel_circumference_mm * (ob_float_t)360.0;
    ob_float_t speed_dps    = (ob_float_t)fabs((double)speed_mm_s) /
                              db->wheel_circumference_mm * (ob_float_t)360.0;

    ob_float_t sum_pos  = db_sum_pos(db);
    // Move-start heading frame must match what the tick will READ.
    // In gyro mode the frame is ABSOLUTE (Pybricks-style, 1.25.0):
    // the binding accumulates a continuous body delta since
    // use_gyro-enable and the persistent target lives in
    // ``turn_hold`` in the same frame — so a straight() holds the
    // TARGET heading, and overshoot banked by a previous turn is
    // pulled back here instead of forgiven (per-move re-baselining
    // measured ~+7 deg of drift per gyro'd square on the ST-3032
    // bench). Snapshotting the ENCODER diff here would mix frames —
    // the controller then chases the encoder's lifetime accumulated
    // diff forever (the pre-1.15.2 runaway).
    ob_float_t diff_pos;
    if (db->use_gyro) {
        diff_pos = db->turn_hold;
    } else {
        diff_pos = db_diff_pos_encoder(db);
    }

    // Enter at the CURRENT commanded speed of the sum axis — a
    // straight armed while the wheels cruise (line-follow handing
    // over) blends down through the accel limit instead of cliffing
    // the command to zero (bench 2026-08-16).
    // v0 excludes the integral's share of the outgoing command —
    // the new move resets the integral, so a v0 that contains it
    // would hand the FF a bias the controller no longer supplies.
    ob_float_t v0_sum = (db->left->target_dps + db->right->target_dps)
                        / (ob_float_t)2.0
                        - db->ki * db->integ_sum;
    // carry (then="continue"): end the profile AT cruise — the tick
    // keeps the reference advancing at that speed past the target.
    ob_trajectory_init_v0v3(&db->fwd, sum_pos, sum_pos + distance_deg,
                            speed_dps, db->accel_dps2, v0_sum,
                            carry ? speed_dps : (ob_float_t)0.0);
    db->fwd_start_ms = now_ms;
    db->fwd_active   = true;

    // Hold whatever heading we have right now; feedback will defend it.
    db->turn_hold   = diff_pos;
    db->turn_active = false;

    db->done = false;
    db->settling = false;
    db_move_state_reset(db);
}


void ob_drivebase_turn(ob_drivebase_t *db,
                       long now_ms,
                       ob_float_t angle_deg,
                       ob_float_t rate_dps) {
    // Body-degrees θ → wheel-degree differential α:
    //   arc_mm    = radians(|θ|) * axle_track / 2
    //   α (deg)   = arc_mm / circumference * 360
    // A positive body turn is CW / right (Pybricks convention,
    // "positive means clockwise", adopted system-wide in 1.24.0):
    // it drives the left wheel forward and the right wheel
    // backward, so diff_pos = (L - R)/2 INCREASES with θ.
    ob_float_t arc_mm   = (ob_float_t)fabs((double)angle_deg) *
                          ((ob_float_t)M_PI / (ob_float_t)180.0) *
                          (db->axle_track_mm / (ob_float_t)2.0);
    ob_float_t wheel_deg = arc_mm / db->wheel_circumference_mm *
                           (ob_float_t)360.0;
    ob_float_t signed_delta = (angle_deg >= 0.0 ? wheel_deg : -wheel_deg);

    ob_float_t rate_arc_mm_s = (ob_float_t)fabs((double)rate_dps) *
                               ((ob_float_t)M_PI / (ob_float_t)180.0) *
                               (db->axle_track_mm / (ob_float_t)2.0);
    ob_float_t rate_wheel_dps = rate_arc_mm_s /
                                db->wheel_circumference_mm *
                                (ob_float_t)360.0;

    ob_float_t sum_pos  = db_sum_pos(db);
    // Same gyro-frame rule as ob_drivebase_straight above: in gyro
    // mode the turn trajectory runs from the PREVIOUS absolute
    // target to target+delta — a turn arriving with overshoot
    // banked simply has less real distance to cover.
    ob_float_t diff_pos;
    if (db->use_gyro) {
        diff_pos = db->turn_hold;
    } else {
        diff_pos = db_diff_pos_encoder(db);
    }

    // Same entry-speed rule for the diff axis (a turn armed while
    // the chassis still rotates from steering).
    ob_float_t v0_diff = (db->left->target_dps - db->right->target_dps)
                         / (ob_float_t)2.0
                         - db->ki * db->integ_diff;
    ob_trajectory_init_v0(&db->turn, diff_pos, diff_pos + signed_delta,
                          rate_wheel_dps,
                          db->accel_dps2, v0_diff);
    db->turn_start_ms = now_ms;
    db->turn_active   = true;

    // The FORWARD axis of a turn armed while the chassis still
    // translates (line-follow handing straight into turn()): an
    // instant position-hold is the same cliff the entry speeds
    // exist to remove — the P-term brakes at plant limit. Give it a
    // STOP trajectory instead: decelerate to rest at the accel
    // limit, landing wherever v0²/2a puts us; the expiry lock
    // anchors the hold THERE (where the robot actually stops).
    ob_float_t v0_sum = (db->left->target_dps + db->right->target_dps)
                        / (ob_float_t)2.0
                        - db->ki * db->integ_sum;
    ob_float_t v0_abs = (v0_sum < 0.0) ? -v0_sum : v0_sum;
    if (v0_abs > (ob_float_t)1.0 && db->accel_dps2 > (ob_float_t)0.0) {
        ob_float_t stop_d = v0_sum * v0_abs
                            / ((ob_float_t)2.0 * db->accel_dps2);
        ob_trajectory_init_v0(&db->fwd, sum_pos, sum_pos + stop_d,
                              v0_abs, db->accel_dps2, v0_sum);
        db->fwd_start_ms = now_ms;
        db->fwd_active   = true;
    } else {
        db->fwd_hold   = sum_pos;
        db->fwd_active = false;
    }

    db->done = false;
    db->settling = false;
    db_move_state_reset(db);
}


void ob_drivebase_curve(ob_drivebase_t *db,
                        long now_ms,
                        ob_float_t radius_mm,
                        ob_float_t angle_deg,
                        ob_float_t speed_mm_s,
                        bool carry) {
    // Forward component: the CENTRE of the robot travels
    // |radians(angle)| * radius mm, signed by the radius (Pybricks:
    // negative radius drives the arc backward).
    ob_float_t theta_abs = (ob_float_t)fabs((double)angle_deg) *
                           ((ob_float_t)M_PI / (ob_float_t)180.0);
    ob_float_t distance_mm  = radius_mm * theta_abs;
    ob_float_t distance_deg = distance_mm /
                              db->wheel_circumference_mm * (ob_float_t)360.0;
    ob_float_t speed_dps    = (ob_float_t)fabs((double)speed_mm_s) /
                              db->wheel_circumference_mm * (ob_float_t)360.0;

    // Turn component: same mapping as ob_drivebase_turn.
    ob_float_t arc_mm    = (ob_float_t)fabs((double)angle_deg) *
                           ((ob_float_t)M_PI / (ob_float_t)180.0) *
                           (db->axle_track_mm / (ob_float_t)2.0);
    ob_float_t wheel_deg = arc_mm / db->wheel_circumference_mm *
                           (ob_float_t)360.0;
    ob_float_t turn_delta = (angle_deg >= 0.0 ? wheel_deg : -wheel_deg);

    ob_float_t sum_pos = db_sum_pos(db);
    // Same gyro-frame rule as straight/turn: absolute frame when the
    // gyro drives heading, encoder diff otherwise.
    ob_float_t diff_pos;
    if (db->use_gyro) {
        diff_pos = db->turn_hold;
    } else {
        diff_pos = db_diff_pos_encoder(db);
    }

    ob_float_t fwd_abs  = (distance_deg < 0) ? -distance_deg : distance_deg;
    ob_float_t turn_abs = (turn_delta < 0) ? -turn_delta : turn_delta;

    if (turn_abs < (ob_float_t)1e-9) {
        // angle 0: zero arc length whatever the radius — complete.
        ob_drivebase_stop(db);
        db->fwd_hold  = sum_pos;
        db->turn_hold = diff_pos;
        return;
    }
    if (fwd_abs < (ob_float_t)1e-9) {
        // radius 0: a turn in place, wheels at the rim speed. The
        // forward axis gets the same stop-trajectory treatment as
        // ob_drivebase_turn when entered while translating.
        ob_trajectory_init_v0v3(&db->turn, diff_pos,
                                diff_pos + turn_delta,
                                speed_dps, db->accel_dps2,
                                (db->left->target_dps
                                 - db->right->target_dps)
                                / (ob_float_t)2.0
                                - db->ki * db->integ_diff,
                                carry ? speed_dps : (ob_float_t)0.0);
        db->turn_start_ms = now_ms;
        db->turn_active   = true;
        ob_float_t cv0 = (db->left->target_dps + db->right->target_dps)
                         / (ob_float_t)2.0
                         - db->ki * db->integ_sum;
        ob_float_t cva = (cv0 < 0.0) ? -cv0 : cv0;
        if (cva > (ob_float_t)1.0 && db->accel_dps2 > (ob_float_t)0.0) {
            ob_trajectory_init_v0(&db->fwd, sum_pos,
                                  sum_pos + cv0 * cva
                                  / ((ob_float_t)2.0 * db->accel_dps2),
                                  cva, db->accel_dps2, cv0);
            db->fwd_start_ms = now_ms;
            db->fwd_active   = true;
        } else {
            db->fwd_hold   = sum_pos;
            db->fwd_active = false;
        }
        db->done          = false;
        db->settling = false;
        db_move_state_reset(db);
        return;
    }

    // Both components live: scale the turn profile's cruise AND
    // accel by the target ratio so the two trapezoids share their
    // exact time shape — heading stays proportional to distance at
    // every instant (a true circle), ramps included.
    ob_float_t ratio      = turn_abs / fwd_abs;
    ob_float_t turn_speed = speed_dps * ratio;
    ob_float_t turn_accel = db->accel_dps2 * ratio;

    // Curve entry: each axis blends from its current speed. The
    // entry ramps can differ in length, so the arc's very start may
    // deviate from the exact circle — the price of never cliffing.
    // carry: BOTH axes end at their cruise speeds — the reference
    // continues along the same arc past the end point.
    ob_trajectory_init_v0v3(&db->fwd, sum_pos, sum_pos + distance_deg,
                            speed_dps, db->accel_dps2,
                            (db->left->target_dps + db->right->target_dps)
                            / (ob_float_t)2.0
                            - db->ki * db->integ_sum,
                            carry ? speed_dps : (ob_float_t)0.0);
    db->fwd_start_ms = now_ms;
    db->fwd_active   = true;

    ob_trajectory_init_v0v3(&db->turn, diff_pos, diff_pos + turn_delta,
                            turn_speed, turn_accel,
                            (db->left->target_dps - db->right->target_dps)
                            / (ob_float_t)2.0
                            - db->ki * db->integ_diff,
                            carry ? turn_speed : (ob_float_t)0.0);
    db->turn_start_ms = now_ms;
    db->turn_active   = true;

    db->done = false;
    db->settling = false;
    db_move_state_reset(db);
}


void ob_drivebase_stop(ob_drivebase_t *db) {
    db->fwd_active  = false;
    db->turn_active = false;
    db->done        = true;
    db->settling    = false;
    // Integrals and the landing budget die with the move — but NOT
    // the settle diagnostics: done() dispatches stop() before the
    // user's move call even returns, so wiping expiry stats here
    // made db_settle_stats() read (0.0, 0) on every standard-path
    // move (bench 2026-08-17 — the diagnostic destroyed its own
    // evidence). Stats reset only when the NEXT move arms.
    db->integ_sum      = 0.0;
    db->integ_diff     = 0.0;
    db->landing_active = false;
    db->landing_best_err = 0.0;
    // db->landings is DIAGNOSTIC state alongside expiry_residual —
    // it survives the stop dispatch (bench read (94.4, 0): the
    // landing count wiped while the residual survived) and resets
    // at the next arm.
}


// ---------------------------------------------------------------------
// Per-tick control law

// Position-integral update (2.7.x): decreasing the magnitude is
// always allowed; growth happens whenever the TRACKING error is in
// the band DEADZONE <= |err| <= (P-linear region), is rate-capped
// per tick, and the total clamps at the value whose ki-term alone
// commands ACTUATION_MAX. Deliberate deviation from pbio's
// remaining-distance band (2026-08-17, bench-measured): their
// near-target-only integral assumes a factory-calibrated
// feed-forward; ours under-delivered ~250 dps on the reference
// curve, the plant ran ~100 wheel-deg behind the reference the
// whole move, and it entered pbio's band only 0.16 s before expiry
// — integral at 51 dps of a 250 dps deficit, three landings, the
// end-of-run stutter. Gating on tracking error engages the
// integral from move start, so it cancels the FF deficit DURING
// the motion and the reference arrives with the plant on it. The
// deadzone still stops stiction hunting at rest; the band's upper
// edge (P saturation) still refuses to wind against a hard stall —
// the fault latch and watchdog own that case.
static ob_float_t db_integ_update(ob_float_t *integ,
                                  ob_float_t err,
                                  ob_float_t dt_s,
                                  ob_float_t kp,
                                  ob_float_t ki) {
    ob_float_t e = err;
    ob_float_t next = *integ + e * dt_s;
    ob_float_t mag_next = (next < 0) ? -next : next;
    ob_float_t mag_cur  = (*integ < 0) ? -*integ : *integ;
    bool decrease = mag_next < mag_cur;
    if (!decrease) {
        if (e >  (ob_float_t)OB_DRIVEBASE_INTEG_RATE_MAX_WHEEL_DEG) {
            e =  (ob_float_t)OB_DRIVEBASE_INTEG_RATE_MAX_WHEEL_DEG;
        }
        if (e < -(ob_float_t)OB_DRIVEBASE_INTEG_RATE_MAX_WHEEL_DEG) {
            e = -(ob_float_t)OB_DRIVEBASE_INTEG_RATE_MAX_WHEEL_DEG;
        }
        next     = *integ + e * dt_s;
        mag_next = (next < 0) ? -next : next;
        decrease = mag_next < mag_cur;
    }
    ob_float_t ea = (err < 0) ? -err : err;
    ob_float_t upper = (kp > 0)
        ? ((ob_float_t)OB_DRIVEBASE_ACTUATION_MAX_DPS / kp)
        : (ob_float_t)0.0;
    if ((ea >= (ob_float_t)OB_DRIVEBASE_INTEG_DEADZONE_WHEEL_DEG
         && ea <= upper) || decrease) {
        *integ = next;
    }
    if (ki > 0) {
        ob_float_t imax = (ob_float_t)OB_DRIVEBASE_ACTUATION_MAX_DPS / ki;
        if (*integ >  imax) { *integ =  imax; }
        if (*integ < -imax) { *integ = -imax; }
    }
    return *integ;
}

void ob_drivebase_tick(ob_drivebase_t *db, long now_ms) {
    // Integral timebase: real elapsed ms, clamped so a stalled tick
    // (blocked bus) cannot jump the integral.
    ob_float_t dt_s = 0.0;
    if (db->last_tick_ms != 0) {
        dt_s = (ob_float_t)(now_ms - db->last_tick_ms) / (ob_float_t)1000.0;
        if (dt_s < (ob_float_t)0.0)  { dt_s = (ob_float_t)0.0; }
        if (dt_s > (ob_float_t)0.05) { dt_s = (ob_float_t)0.05; }
    }
    db->last_tick_ms = now_ms;
    // 1. Sample fwd profile (or hold). An axis whose profile has
    //    expired is "flying" no more; a CARRYING axis (v3 > 0,
    //    then="continue") stays active past t_total with its
    //    reference advancing at the end speed — Pybricks Stop.NONE —
    //    until the next command supersedes it.
    ob_float_t fwd_target = 0.0;
    ob_float_t fwd_ff_vel = 0.0;
    bool fwd_flying = false;
    if (db->fwd_active) {
        ob_float_t elapsed = (ob_float_t)(now_ms - db->fwd_start_ms) /
                             (ob_float_t)1000.0;
        if (elapsed >= db->fwd.t_total) {
            ob_float_t abs_dist = (db->fwd.distance < 0)
                                  ? -db->fwd.distance : db->fwd.distance;
            ob_float_t end = db->fwd.start + db->fwd.direction * abs_dist;
            if (db->fwd.v3 > (ob_float_t)0.0) {
                // Carry: keep integrating past the target.
                ob_float_t over = elapsed - db->fwd.t_total;
                fwd_target = end + db->fwd.direction * db->fwd.v3 * over;
                fwd_ff_vel = db->fwd.direction * db->fwd.v3;
                db->fwd_hold = fwd_target;   // stop() anchors HERE
            } else {
                // Lock on end-point so feedback corrects any residual.
                fwd_target = end;
                fwd_ff_vel = 0.0;
                db->fwd_hold   = fwd_target;
                db->fwd_active = false;
            }
        } else {
            ob_trajectory_sample(&db->fwd, elapsed, &fwd_target, &fwd_ff_vel);
            fwd_flying = true;
        }
    } else {
        fwd_target = db->fwd_hold;
    }

    // 2. Sample turn profile (or hold) — same carry rule.
    ob_float_t turn_target = 0.0;
    ob_float_t turn_ff_vel = 0.0;
    bool turn_flying = false;
    if (db->turn_active) {
        ob_float_t elapsed = (ob_float_t)(now_ms - db->turn_start_ms) /
                             (ob_float_t)1000.0;
        if (elapsed >= db->turn.t_total) {
            ob_float_t abs_dist = (db->turn.distance < 0)
                                  ? -db->turn.distance : db->turn.distance;
            ob_float_t end = db->turn.start + db->turn.direction * abs_dist;
            if (db->turn.v3 > (ob_float_t)0.0) {
                ob_float_t over = elapsed - db->turn.t_total;
                turn_target = end + db->turn.direction * db->turn.v3 * over;
                turn_ff_vel = db->turn.direction * db->turn.v3;
                db->turn_hold = turn_target;
            } else {
                turn_target = end;
                turn_ff_vel = 0.0;
                db->turn_hold   = turn_target;
                db->turn_active = false;
            }
        } else {
            ob_trajectory_sample(&db->turn, elapsed, &turn_target, &turn_ff_vel);
            turn_flying = true;
        }
    } else {
        turn_target = db->turn_hold;
    }

    // 3. Actual sum / diff positions.
    ob_float_t sum_pos  = db_sum_pos(db);
    ob_float_t diff_pos = db->use_gyro
                          ? db->heading_override_wheel_deg
                          : db_diff_pos_encoder(db);

    // 3b. Measured axis speeds: EMA of the position derivative
    //     (~20 ms window at the 1 kHz tick — smooths the 1-count
    //     quantization step, fast enough for a 60 dps threshold).
    if (dt_s > (ob_float_t)0.0) {
        if (db->have_prev_pos) {
            // Clamp each raw sample to a physically plausible bound:
            // a heading-frame jump (gyro re-base, injected test
            // heading) is a position STEP whose derivative would
            // poison the EMA for ~100 ms.
            ob_float_t vmax = (ob_float_t)OB_DRIVEBASE_ACTUATION_MAX_DPS
                              * (ob_float_t)4.0;
            ob_float_t rs = (sum_pos - db->prev_sum_pos) / dt_s;
            ob_float_t rd = (diff_pos - db->prev_diff_pos) / dt_s;
            if (rs >  vmax) { rs =  vmax; }
            if (rs < -vmax) { rs = -vmax; }
            if (rd >  vmax) { rd =  vmax; }
            if (rd < -vmax) { rd = -vmax; }
            ob_float_t a = (ob_float_t)0.1;
            db->meas_vel_sum  += a * (rs - db->meas_vel_sum);
            db->meas_vel_diff += a * (rd - db->meas_vel_diff);
        }
        db->prev_sum_pos  = sum_pos;
        db->prev_diff_pos = diff_pos;
        db->have_prev_pos = true;
    }

    // 4. Coupled P + feedforward.
    ob_float_t sum_err  = fwd_target  - sum_pos;
    ob_float_t diff_err = turn_target - diff_pos;

    // 4b. Arrival cuts a landing short: the landing is only a settle
    //     aid, so the moment BOTH errors are inside the arrival
    //     tolerance, done latches and the landing profile is dropped
    //     — a robot that physically arrives early must not wait out
    //     the rest of a scheduled correction ramp (the old arrival
    //     latch was instant; this keeps it so under landings).
    if (db->landing_active) {
        // Errors to the move's ENDPOINTS (the holds), NOT to the
        // landing's moving reference — that reference starts at the
        // measured position, so its tracking error is ~0 by
        // construction and would latch a stuck robot instantly.
        ob_float_t se_now = db->fwd_hold  - sum_pos;
        ob_float_t de_now = db->turn_hold - diff_pos;
        se_now = (se_now < 0) ? -se_now : se_now;
        de_now = (de_now < 0) ? -de_now : de_now;
        // ...and pbio's STANDSTILL condition on MEASURED speed —
        // theirs tests the plant, not the command. A command-based
        // check deadlocks: a wound integral commands hundreds of
        // dps into an arrived robot, and with error ~0 it never
        // unwinds; an ff-based check let the same integral latch
        // done with the wheels genuinely turning.
        ob_float_t mv_sum  = (db->meas_vel_sum  < 0)
                             ? -db->meas_vel_sum  : db->meas_vel_sum;
        ob_float_t mv_diff = (db->meas_vel_diff < 0)
                             ? -db->meas_vel_diff : db->meas_vel_diff;
        if (se_now < (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG
            && de_now < (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG
            && mv_sum  < (ob_float_t)OB_DRIVEBASE_ARRIVAL_SPEED_TOL_DPS
            && mv_diff < (ob_float_t)OB_DRIVEBASE_ARRIVAL_SPEED_TOL_DPS) {
            db->fwd_active     = false;
            db->turn_active    = false;
            db->landing_active = false;
            db->done           = true;
        }
    }

    // 5. Move complete = profiles expired AND the robot has ARRIVED.
    //    Time-based done alone left the final move's settling error
    //    permanently uncorrected (bench: +4.5 body-deg banked at
    //    every gyro'd turn end; only non-final turns were rescued by
    //    the next move). Matches the classic fallback's
    //    reach-the-target semantics; callers own the stall timeout.
    if (!fwd_flying && !turn_flying) {
        db->landing_active = false;   // any landing profile has expired
        ob_float_t se = (sum_err  < 0) ? -sum_err  : sum_err;
        ob_float_t de = (diff_err < 0) ? -diff_err : diff_err;
        ob_float_t worst = (se > de) ? se : de;
        if (db->fwd_active || db->turn_active) {
            // A carrying axis is still active past its profile: the
            // reference MOVES, so there is no settle window to run.
            // pbio's Stop.NONE rule (control.c): past the nominal
            // time, done latches as soon as the MEASURED position is
            // at or past the target — no tolerance band. A lagging
            // plant latches the moment it crosses the mark (a
            // tracking-error bar deadlocked the sim, whose plant
            // trails the moving reference at cruise); a stalled one
            // never crosses, so the caller's watchdog still fires.
            (void)worst;
            bool fwd_past = true;
            if (db->fwd_active) {
                ob_float_t d = (db->fwd.distance < 0)
                               ? -db->fwd.distance : db->fwd.distance;
                ob_float_t end = db->fwd.start + db->fwd.direction * d;
                fwd_past = db->fwd.direction * (sum_pos - end)
                           >= (ob_float_t)0.0;
            }
            bool turn_past = true;
            if (db->turn_active) {
                ob_float_t d = (db->turn.distance < 0)
                               ? -db->turn.distance : db->turn.distance;
                ob_float_t end = db->turn.start + db->turn.direction * d;
                turn_past = db->turn.direction * (diff_pos - end)
                            >= (ob_float_t)0.0;
            }
            if (fwd_past && turn_past) {
                db->done = true;
            }
        } else {
            // First evaluation after this move's profiles expired:
            // record the gap the settle has to close — the bench's
            // answer to "how big is the correction, really".
            if (!db->expiry_captured) {
                db->expiry_captured = true;
                db->expiry_residual = worst;
                db->expiry_res_sum  = se;
                db->expiry_res_diff = de;
                db->expiry_integ_sum_dps  = db->ki * db->integ_sum;
                db->expiry_integ_diff_dps = db->ki * db->integ_diff;
            }
            // Landing settle (2.6.0): residual above arrival
            // tolerance gets a SHAPED mini-trajectory back to the
            // hold target instead of a raw P step — same accel
            // contract as every other motion, done latches when the
            // landing arrives. Bounded retries; the forgive/cap
            // machinery below remains the backstop for a genuinely
            // stuck robot.
            // Retry a landing only while landings make PROGRESS —
            // a stuck robot's first landing changes nothing, and
            // burning the retry budget (plus its ramp time) against
            // stiction just delays the forgive/cap verdict below.
            bool landing_progress =
                db->landings == 0
                || worst < db->landing_best_err
                   - (ob_float_t)OB_DRIVEBASE_SETTLE_PROGRESS_WHEEL_DEG;
            if (worst >= (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG
                && db->landings < OB_DRIVEBASE_MAX_LANDINGS
                && landing_progress) {
                db->landing_best_err = worst;
                db->landings++;
                ob_float_t landing_v = db->accel_dps2 > 0
                    ? (ob_float_t)OB_DRIVEBASE_LANDING_DPS
                    : (ob_float_t)0.0;
                // Entry speed = the CURRENT commanded axis speed —
                // re-basing the reference onto the measured position
                // zeroes the P-term's contribution, and starting the
                // landing from rest would step the total command
                // down by exactly that amount in one tick (bench
                // harness: 727 steps/s). Entering at the live
                // command keeps the total continuous, the same rule
                // move arming follows.
                // ...minus the integral's share: the I-term keeps
                // being ADDED to the landing's feed-forward, so a v0
                // that contains it would double-count it — measured
                // as a +ki*integ step at every landing arm.
                ob_float_t v0_sum = (db->left->target_dps
                                     + db->right->target_dps)
                                    / (ob_float_t)2.0
                                    - db->ki * db->integ_sum;
                ob_float_t v0_diff = (db->left->target_dps
                                      - db->right->target_dps)
                                     / (ob_float_t)2.0
                                     - db->ki * db->integ_diff;
                if (se >= (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG
                    && landing_v > 0) {
                    ob_trajectory_init_v0(&db->fwd, sum_pos, db->fwd_hold,
                                          landing_v, db->accel_dps2,
                                          v0_sum);
                    db->fwd_start_ms = now_ms;
                    db->fwd_active   = true;
                }
                if (de >= (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG
                    && landing_v > 0) {
                    ob_trajectory_init_v0(&db->turn, diff_pos, db->turn_hold,
                                          landing_v, db->accel_dps2,
                                          v0_diff);
                    db->turn_start_ms = now_ms;
                    db->turn_active   = true;
                }
                db->landing_active = true;
                db->settling = false;
                return;   // commands resume next tick on the landing
            }
            if (!db->settling) {
                db->settling = true;
                db->settle_start_ms = now_ms;
                db->settle_best_err = worst;
            } else if (worst < db->settle_best_err
                       - (ob_float_t)OB_DRIVEBASE_SETTLE_PROGRESS_WHEEL_DEG) {
                // Still converging: progress re-stamps the window, so
                // a healthy settle keeps its full arrival accuracy.
                db->settle_best_err = worst;
                db->settle_start_ms = now_ms;
            }
            // Arrival latches done; the cap fires only after
            // SETTLE_MS with NO progress and a residual inside the
            // forgive limit (duty-mode stiction can leave the last
            // degrees of a turn below feedback's breakaway authority
            // — bench 2026-08-14: intermittent ~1 s pause between
            // turn() and the next straight()). A forgiven residual
            // is NOT banked in gyro mode: the absolute frame carries
            // it and the next move corrects it in motion. A residual
            // beyond the forgive limit is a move that DIDN'T HAPPEN
            // — done stays false and the caller's watchdog raises
            // loudly.
            ob_float_t amv_sum  = (db->meas_vel_sum  < 0)
                                  ? -db->meas_vel_sum  : db->meas_vel_sum;
            ob_float_t amv_diff = (db->meas_vel_diff < 0)
                                  ? -db->meas_vel_diff : db->meas_vel_diff;
            bool arrived =
                se < (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG
                && de < (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG
                && amv_sum  < (ob_float_t)OB_DRIVEBASE_ARRIVAL_SPEED_TOL_DPS
                && amv_diff < (ob_float_t)OB_DRIVEBASE_ARRIVAL_SPEED_TOL_DPS;
            bool capped  = (now_ms - db->settle_start_ms)
                               >= (long)OB_DRIVEBASE_SETTLE_MS
                           && worst < (ob_float_t)
                                  OB_DRIVEBASE_SETTLE_FORGIVE_WHEEL_DEG;
            if (arrived || capped) {
                db->done = true;
            }
        }
    }
    if (db->done) {
        // Post-move hold: retire the integral SMOOTHLY (~35 ms
        // half-life). An instant zero at the done latch was itself
        // a command step; left wound instead, it pushed an arrived
        // robot at 204 dps through the inter-move hold. The bleed
        // is both: shaped on the way out, gone within a tenth of a
        // second.
        db->integ_sum  *= (ob_float_t)0.98;
        db->integ_diff *= (ob_float_t)0.98;
    }
    ob_float_t integ_sum  = db_integ_update(&db->integ_sum, sum_err,
                                            dt_s, db->kp_sum, db->ki);
    ob_float_t integ_diff = db_integ_update(&db->integ_diff, diff_err,
                                            dt_s, db->kp_diff, db->ki);

    ob_float_t fwd_cmd  = fwd_ff_vel  + db->kp_sum  * sum_err
                          + db->ki * integ_sum;
    ob_float_t diff_cmd = turn_ff_vel + db->kp_diff * diff_err
                          + db->ki * integ_diff;

    // 6. Mix into per-servo target velocities. diff_pos = (L - R)/2,
    //    so diff_cmd is (L_vel - R_vel)/2 — the rate at which left
    //    out-paces right. Positive diff_cmd: left speeds up, right
    //    slows down.
    db->left->target_dps  = fwd_cmd + diff_cmd;
    db->right->target_dps = fwd_cmd - diff_cmd;

    // 7. Flight recorder — rolling, stride-decimated, sum axis.
    if (now_ms - db->trace_last_ms >= (long)OB_DRIVEBASE_TRACE_STRIDE_MS) {
        db->trace_last_ms = now_ms;
        float *row = db->trace[db->trace_next];
        row[0] = (float)now_ms;
        row[1] = (float)fwd_target;
        row[2] = (float)sum_pos;
        row[3] = (float)fwd_ff_vel;
        row[4] = (float)fwd_cmd;
        row[5] = (float)(db->ki * integ_sum);
        db->trace_next = (db->trace_next + 1) % OB_DRIVEBASE_TRACE_N;
        if (db->trace_count < OB_DRIVEBASE_TRACE_N) {
            db->trace_count++;
        }
    }
}


bool ob_drivebase_is_done(const ob_drivebase_t *db) {
    return db->done;
}


// Body heading delta (degrees) → wheel-degree differential the
// controller expects in ``heading_override_wheel_deg``. Inverse of
// the body→wheel mapping used for turn-in-place: a positive body
// heading delta (CW / right, Pybricks convention since 1.24.0)
// corresponds to a positive diff_pos (left wheel out-paced right).
ob_float_t ob_drivebase_body_to_wheel_diff(const ob_drivebase_t *db,
                                            ob_float_t body_heading_delta_deg) {
    return body_heading_delta_deg * db->axle_track_mm * (ob_float_t)M_PI /
           db->wheel_circumference_mm;
}


// Reset the gyro-mode absolute frame: called by the bindings on the
// use_gyro ENABLE transition, so "here, now" becomes both the zero
// of the continuous measured heading and the initial target. Clears
// any encoder-frame residue from turn_hold (its value before enable
// lives in the lifetime encoder-diff frame, meaningless here).
void ob_drivebase_gyro_frame_reset(ob_drivebase_t *db) {
    db->turn_hold                  = 0.0;
    db->heading_override_wheel_deg = 0.0;
    // The diff integral was learned against the OLD frame.
    db->integ_diff                 = 0.0;
}


int ob_drivebase_trace_dump(const ob_drivebase_t *db,
                            float out[][6], int max_rows) {
    int n = db->trace_count;
    if (n > max_rows) {
        n = max_rows;
    }
    int start = (db->trace_count == OB_DRIVEBASE_TRACE_N)
                ? db->trace_next : 0;
    for (int k = 0; k < n; k++) {
        const float *row = db->trace[(start + k) % OB_DRIVEBASE_TRACE_N];
        for (int j = 0; j < 6; j++) {
            out[k][j] = row[j];
        }
    }
    return n;
}


// Settle diagnostics (2.6.0): worst-axis residual captured at the
// last move's first profile expiry, and how many shaped landings it
// took. The bench reads this to decide whether the integral gains
// need another look — measured numbers, not guesses.
void ob_drivebase_settle_stats(const ob_drivebase_t *db,
                               ob_float_t *res_sum,
                               ob_float_t *res_diff,
                               int *landings,
                               ob_float_t *integ_sum_dps,
                               ob_float_t *integ_diff_dps) {
    *res_sum        = db->expiry_res_sum;
    *res_diff       = db->expiry_res_diff;
    *landings       = (int)db->landings;
    *integ_sum_dps  = db->expiry_integ_sum_dps;
    *integ_diff_dps = db->expiry_integ_diff_dps;
}
