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
    ob_float_t diff_pos = db_diff_pos_encoder(db);

    ob_trajectory_init(&db->fwd, sum_pos, sum_pos + distance_deg,
                       speed_dps, (ob_float_t)OB_DRIVEBASE_DEFAULT_ACCEL_DPS2);
    db->fwd_start_ms = now_ms;
    db->fwd_active   = true;

    // Hold whatever heading we have right now; feedback will defend it.
    db->turn_hold   = diff_pos;
    db->turn_active = false;

    db->done = false;
}


void ob_drivebase_turn(ob_drivebase_t *db,
                       long now_ms,
                       ob_float_t angle_deg,
                       ob_float_t rate_dps) {
    // Body-degrees θ → wheel-degree differential α:
    //   arc_mm    = radians(|θ|) * axle_track / 2
    //   α (deg)   = arc_mm / circumference * 360
    // A positive body turn (CCW / left in Pybricks convention) drives
    // the left wheel backward and the right wheel forward, so
    // diff_pos = (L - R)/2 DECREASES — flip the sign.
    ob_float_t arc_mm   = (ob_float_t)fabs((double)angle_deg) *
                          ((ob_float_t)M_PI / (ob_float_t)180.0) *
                          (db->axle_track_mm / (ob_float_t)2.0);
    ob_float_t wheel_deg = arc_mm / db->wheel_circumference_mm *
                           (ob_float_t)360.0;
    ob_float_t signed_delta = (angle_deg >= 0.0 ? -wheel_deg : wheel_deg);

    ob_float_t rate_arc_mm_s = (ob_float_t)fabs((double)rate_dps) *
                               ((ob_float_t)M_PI / (ob_float_t)180.0) *
                               (db->axle_track_mm / (ob_float_t)2.0);
    ob_float_t rate_wheel_dps = rate_arc_mm_s /
                                db->wheel_circumference_mm *
                                (ob_float_t)360.0;

    ob_float_t sum_pos  = db_sum_pos(db);
    ob_float_t diff_pos = db_diff_pos_encoder(db);

    ob_trajectory_init(&db->turn, diff_pos, diff_pos + signed_delta,
                       rate_wheel_dps,
                       (ob_float_t)OB_DRIVEBASE_DEFAULT_ACCEL_DPS2);
    db->turn_start_ms = now_ms;
    db->turn_active   = true;

    db->fwd_hold   = sum_pos;
    db->fwd_active = false;

    db->done = false;
}


void ob_drivebase_stop(ob_drivebase_t *db) {
    db->fwd_active  = false;
    db->turn_active = false;
    db->done        = true;
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

    // 3. Both profiles done → move complete.
    if (!db->fwd_active && !db->turn_active) {
        db->done = true;
    }

    // 4. Actual sum / diff positions.
    ob_float_t sum_pos  = db_sum_pos(db);
    ob_float_t diff_pos = db->use_gyro
                          ? db->heading_override_wheel_deg
                          : db_diff_pos_encoder(db);

    // 5. Coupled P + feedforward.
    ob_float_t sum_err  = fwd_target  - sum_pos;
    ob_float_t diff_err = turn_target - diff_pos;
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
// heading delta (CCW) corresponds to a negative diff_pos.
ob_float_t ob_drivebase_body_to_wheel_diff(const ob_drivebase_t *db,
                                            ob_float_t body_heading_delta_deg) {
    return -body_heading_delta_deg * db->axle_track_mm * (ob_float_t)M_PI /
           db->wheel_circumference_mm;
}
