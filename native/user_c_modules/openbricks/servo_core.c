// SPDX-License-Identifier: MIT
//
// Servo state machine — control law + trajectory + observer
// composition. Same numerics in firmware and sim; the I/O glue
// (reading encoder counts, writing bridge / actuator commands)
// lives in the binding shells.
//
// Control law (matches the firmware's M1 implementation):
//
//     measured_pos_deg = count × 360 / counts_per_rev
//     dt              = (now − last_time) / 1000          (s)
//     observer.update(measured_pos_deg, dt)
//     measured_dps    = observer.vel_hat
//     error           = target_dps − measured_dps
//     ff              = (target_dps / RATED_DPS) × 100
//     power           = ff + kp × error    (clamped to ±100)
//
// If ``invert`` is set, the binding negates the returned power
// before driving its bridge — that's a hardware-wiring concern,
// not part of the math, but we surface a flag here so the binding
// doesn't have to remember it separately.

#include <stdbool.h>

#include "servo_core.h"


void ob_servo_init(ob_servo_t *s,
                   int counts_per_rev,
                   ob_float_t kp,
                   bool invert) {
    s->counts_per_rev = counts_per_rev;
    s->kp             = kp;
    s->invert         = invert;

    s->target_dps     = 0.0;
    s->active         = false;

    // Zero out the embedded trajectory cleanly via init with a
    // degenerate (zero-distance) move.
    ob_trajectory_init(&s->trajectory, 0.0, 0.0, 0.0, 0.0);
    s->traj_start_ms  = 0;
    s->traj_active    = false;
    s->traj_done      = true;

    ob_observer_init(&s->observer,
                     (ob_float_t)OB_SERVO_DEFAULT_OBS_ALPHA,
                     (ob_float_t)OB_SERVO_DEFAULT_OBS_BETA);

    s->last_time_ms   = 0;
}


ob_float_t ob_servo_count_to_angle_deg(const ob_servo_t *s, long count) {
    if (s->counts_per_rev <= 0) {
        return 0.0;
    }
    return (ob_float_t)count * (ob_float_t)360.0 / (ob_float_t)s->counts_per_rev;
}


void ob_servo_baseline(ob_servo_t *s, long count, long now_ms) {
    ob_float_t pos = ob_servo_count_to_angle_deg(s, count);
    ob_observer_reset(&s->observer, pos);
    s->last_time_ms = now_ms;
}


void ob_servo_set_speed(ob_servo_t *s, ob_float_t dps) {
    s->target_dps  = dps;
    s->traj_active = false;
    s->traj_done   = true;
}


void ob_servo_run_target(ob_servo_t *s, long count, long now_ms,
                         ob_float_t delta_deg,
                         ob_float_t cruise_dps,
                         ob_float_t accel) {
    ob_float_t start  = ob_servo_count_to_angle_deg(s, count);
    ob_float_t target = start + delta_deg;

    ob_trajectory_init(&s->trajectory, start, target, cruise_dps, accel);
    s->traj_start_ms = now_ms;
    s->traj_active   = true;
    s->traj_done     = false;
    s->target_dps    = 0.0;   // first tick will sample the profile
}


ob_float_t ob_servo_tick(ob_servo_t *s, long count, long now_ms) {
    // 1. If a trajectory is active, sample it for the velocity
    //    setpoint.
    if (s->traj_active) {
        ob_float_t elapsed_s = (ob_float_t)(now_ms - s->traj_start_ms) / 1000.0;
        if (elapsed_s >= s->trajectory.t_total) {
            s->target_dps = 0.0;
            s->traj_done  = true;
        } else {
            ob_float_t pos, vel;
            ob_trajectory_sample(&s->trajectory, elapsed_s, &pos, &vel);
            s->target_dps = vel;
        }
    }

    // 2. Read encoder, feed observer with dt.
    ob_float_t measured_pos = ob_servo_count_to_angle_deg(s, count);
    ob_float_t dt_s         = (ob_float_t)(now_ms - s->last_time_ms) / 1000.0;
    s->last_time_ms = now_ms;

    if (dt_s > 0.0) {
        ob_observer_update(&s->observer, measured_pos, dt_s);
    }
    ob_float_t measured_dps = s->observer.vel_hat;

    // 3. P-control + feed-forward.
    ob_float_t error = s->target_dps - measured_dps;
    ob_float_t ff    = s->target_dps / (ob_float_t)OB_SERVO_DEFAULT_RATED_DPS *
                      (ob_float_t)OB_SERVO_POWER_CLAMP;
    ob_float_t power = ff + s->kp * error;

    // 4. Clamp.
    if (power >  OB_SERVO_POWER_CLAMP) power =  OB_SERVO_POWER_CLAMP;
    if (power < -OB_SERVO_POWER_CLAMP) power = -OB_SERVO_POWER_CLAMP;

    return power;
}


bool ob_servo_is_done(const ob_servo_t *s) {
    return s->traj_done;
}
