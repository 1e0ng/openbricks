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
                           ob_float_t speed_mm_s) {
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
    ob_float_t v0_sum = (db->left->target_dps + db->right->target_dps)
                        / (ob_float_t)2.0;
    ob_trajectory_init_v0(&db->fwd, sum_pos, sum_pos + distance_deg,
                          speed_dps, db->accel_dps2, v0_sum);
    db->fwd_start_ms = now_ms;
    db->fwd_active   = true;

    // Hold whatever heading we have right now; feedback will defend it.
    db->turn_hold   = diff_pos;
    db->turn_active = false;

    db->done = false;
    db->settling = false;
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
                         / (ob_float_t)2.0;
    ob_trajectory_init_v0(&db->turn, diff_pos, diff_pos + signed_delta,
                          rate_wheel_dps,
                          db->accel_dps2, v0_diff);
    db->turn_start_ms = now_ms;
    db->turn_active   = true;

    db->fwd_hold   = sum_pos;
    db->fwd_active = false;

    db->done = false;
    db->settling = false;
}


void ob_drivebase_curve(ob_drivebase_t *db,
                        long now_ms,
                        ob_float_t radius_mm,
                        ob_float_t angle_deg,
                        ob_float_t speed_mm_s) {
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
        // radius 0: a turn in place, wheels at the rim speed.
        ob_trajectory_init_v0(&db->turn, diff_pos, diff_pos + turn_delta,
                              speed_dps, db->accel_dps2,
                              (db->left->target_dps
                               - db->right->target_dps)
                              / (ob_float_t)2.0);
        db->turn_start_ms = now_ms;
        db->turn_active   = true;
        db->fwd_hold      = sum_pos;
        db->fwd_active    = false;
        db->done          = false;
        db->settling = false;
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
    ob_trajectory_init_v0(&db->fwd, sum_pos, sum_pos + distance_deg,
                          speed_dps, db->accel_dps2,
                          (db->left->target_dps + db->right->target_dps)
                          / (ob_float_t)2.0);
    db->fwd_start_ms = now_ms;
    db->fwd_active   = true;

    ob_trajectory_init_v0(&db->turn, diff_pos, diff_pos + turn_delta,
                          turn_speed, turn_accel,
                          (db->left->target_dps - db->right->target_dps)
                          / (ob_float_t)2.0);
    db->turn_start_ms = now_ms;
    db->turn_active   = true;

    db->done = false;
    db->settling = false;
}


void ob_drivebase_stop(ob_drivebase_t *db) {
    db->fwd_active  = false;
    db->turn_active = false;
    db->done        = true;
    db->settling    = false;
}


// ---------------------------------------------------------------------
// Per-tick control law

void ob_drivebase_tick(ob_drivebase_t *db, long now_ms) {
    // 1. Sample fwd profile (or hold).
    ob_float_t fwd_target = 0.0;
    ob_float_t fwd_ff_vel = 0.0;
    if (db->fwd_active) {
        ob_float_t elapsed = (ob_float_t)(now_ms - db->fwd_start_ms) /
                             (ob_float_t)1000.0;
        if (elapsed >= db->fwd.t_total) {
            // Lock on end-point so feedback corrects any residual.
            ob_float_t abs_dist = (db->fwd.distance < 0)
                                  ? -db->fwd.distance : db->fwd.distance;
            fwd_target = db->fwd.start + db->fwd.direction * abs_dist;
            fwd_ff_vel = 0.0;
            db->fwd_hold   = fwd_target;
            db->fwd_active = false;
        } else {
            ob_trajectory_sample(&db->fwd, elapsed, &fwd_target, &fwd_ff_vel);
        }
    } else {
        fwd_target = db->fwd_hold;
    }

    // 2. Sample turn profile (or hold).
    ob_float_t turn_target = 0.0;
    ob_float_t turn_ff_vel = 0.0;
    if (db->turn_active) {
        ob_float_t elapsed = (ob_float_t)(now_ms - db->turn_start_ms) /
                             (ob_float_t)1000.0;
        if (elapsed >= db->turn.t_total) {
            ob_float_t abs_dist = (db->turn.distance < 0)
                                  ? -db->turn.distance : db->turn.distance;
            turn_target = db->turn.start + db->turn.direction * abs_dist;
            turn_ff_vel = 0.0;
            db->turn_hold   = turn_target;
            db->turn_active = false;
        } else {
            ob_trajectory_sample(&db->turn, elapsed, &turn_target, &turn_ff_vel);
        }
    } else {
        turn_target = db->turn_hold;
    }

    // 3. Actual sum / diff positions.
    ob_float_t sum_pos  = db_sum_pos(db);
    ob_float_t diff_pos = db->use_gyro
                          ? db->heading_override_wheel_deg
                          : db_diff_pos_encoder(db);

    // 4. Coupled P + feedforward.
    ob_float_t sum_err  = fwd_target  - sum_pos;
    ob_float_t diff_err = turn_target - diff_pos;

    // 5. Move complete = profiles expired AND the robot has ARRIVED.
    //    Time-based done alone left the final move's settling error
    //    permanently uncorrected (bench: +4.5 body-deg banked at
    //    every gyro'd turn end; only non-final turns were rescued by
    //    the next move). Matches the classic fallback's
    //    reach-the-target semantics; callers own the stall timeout.
    if (!db->fwd_active && !db->turn_active) {
        ob_float_t se = (sum_err  < 0) ? -sum_err  : sum_err;
        ob_float_t de = (diff_err < 0) ? -diff_err : diff_err;
        ob_float_t worst = (se > de) ? se : de;
        if (!db->settling) {
            db->settling = true;
            db->settle_start_ms = now_ms;
            db->settle_best_err = worst;
        } else if (worst < db->settle_best_err
                   - (ob_float_t)OB_DRIVEBASE_SETTLE_PROGRESS_WHEEL_DEG) {
            // Still converging: progress re-stamps the window, so a
            // healthy settle keeps its full arrival accuracy.
            db->settle_best_err = worst;
            db->settle_start_ms = now_ms;
        }
        // Arrival latches done; the cap fires only after SETTLE_MS
        // with NO progress and a residual inside the forgive limit
        // (duty-mode stiction can leave the last degrees of a turn
        // below feedback's breakaway authority — bench 2026-08-14:
        // intermittent ~1 s pause between turn() and the next
        // straight()). A forgiven residual is NOT banked in gyro
        // mode: the absolute frame carries it and the next move
        // corrects it in motion. A residual beyond the forgive
        // limit is a move that DIDN'T HAPPEN — done stays false and
        // the caller's watchdog raises loudly.
        bool arrived = se < (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG
                       && de < (ob_float_t)OB_DRIVEBASE_DONE_TOL_WHEEL_DEG;
        bool capped  = (now_ms - db->settle_start_ms)
                           >= (long)OB_DRIVEBASE_SETTLE_MS
                       && worst < (ob_float_t)
                              OB_DRIVEBASE_SETTLE_FORGIVE_WHEEL_DEG;
        if (arrived || capped) {
            db->done = true;
        }
    }
    ob_float_t fwd_cmd  = fwd_ff_vel  + db->kp_sum  * sum_err;
    ob_float_t diff_cmd = turn_ff_vel + db->kp_diff * diff_err;

    // 6. Mix into per-servo target velocities. diff_pos = (L - R)/2,
    //    so diff_cmd is (L_vel - R_vel)/2 — the rate at which left
    //    out-paces right. Positive diff_cmd: left speeds up, right
    //    slows down.
    db->left->target_dps  = fwd_cmd + diff_cmd;
    db->right->target_dps = fwd_cmd - diff_cmd;
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
}
