// SPDX-License-Identifier: MIT
// st_move_core — implementation. See header for the contract.

#include "st_move_core.h"


void ob_smove_init(ob_smove_t *m) {
    m->state       = OB_SMOVE_IDLE;
    m->done        = false;
    m->start_ms    = 0;
    m->goal_counts = (ob_float_t)0.0;
    m->kp          = (ob_float_t)OB_SMOVE_DEFAULT_KP;
    m->tol_counts  = (ob_float_t)OB_SMOVE_DONE_TOL_COUNTS;
}


void ob_smove_start(ob_smove_t *m, long now_ms,
                    ob_float_t from_counts, ob_float_t delta_counts,
                    ob_float_t speed_cps, ob_float_t accel_cps2) {
    m->goal_counts = from_counts + delta_counts;
    ob_trajectory_init(&m->traj, from_counts, m->goal_counts,
                       speed_cps, accel_cps2);
    m->start_ms = now_ms;
    m->state    = OB_SMOVE_PROFILE;
    m->done     = false;
}


void ob_smove_hold_at(ob_smove_t *m, ob_float_t counts) {
    m->goal_counts = counts;
    m->state       = OB_SMOVE_HOLD;
    // A hold at the current position is already "arrived" — callers
    // use done for move completion, and a bare hold() must not read
    // as an unfinished move.
    m->done        = true;
}


void ob_smove_stop(ob_smove_t *m) {
    m->state = OB_SMOVE_IDLE;
    m->done  = false;
}


bool ob_smove_is_done(const ob_smove_t *m) {
    return m->done;
}


ob_float_t ob_smove_tick(ob_smove_t *m, long now_ms,
                         ob_float_t meas_counts) {
    if (m->state == OB_SMOVE_IDLE) {
        return (ob_float_t)0.0;
    }

    ob_float_t target = m->goal_counts;
    ob_float_t ff_vel = (ob_float_t)0.0;

    if (m->state == OB_SMOVE_PROFILE) {
        ob_float_t elapsed = (ob_float_t)(now_ms - m->start_ms)
                             / (ob_float_t)1000.0;
        if (elapsed >= m->traj.t_total) {
            // Profile expired: lock on the endpoint and let feedback
            // settle the residual — the drivebase's end-of-profile
            // rule.
            m->state = OB_SMOVE_HOLD;
        } else {
            ob_trajectory_sample(&m->traj, elapsed, &target, &ff_vel);
        }
    }

    ob_float_t err = target - meas_counts;

    if (m->state == OB_SMOVE_HOLD && !m->done) {
        ob_float_t e = (err < 0) ? -err : err;
        if (e < m->tol_counts) {
            m->done = true;     // latched: disturbance won't un-done
        }
    }

    return ff_vel + m->kp * err;
}
