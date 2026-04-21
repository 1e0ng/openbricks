// SPDX-License-Identifier: MIT
//
// Public header for the native ``Servo`` type. The struct layout lives
// here (not in servo.c) so sibling C modules — drivebase.c today,
// and a hub / kinematics module later — can read the observer state
// and write setpoints directly, at the same latency budget as the
// control tick itself.

#pragma once

#include <stdbool.h>

#include "py/obj.h"

#include "observer.h"
#include "trajectory.h"

typedef struct _servo_obj_t {
    mp_obj_base_t base;

    // Hardware handles (Python objects).
    mp_obj_t in1_pin;
    mp_obj_t in2_pin;
    mp_obj_t pwm;
    mp_obj_t encoder;

    // Cached bound methods — saves mp_load_attr per tick.
    mp_obj_t in1_value;
    mp_obj_t in2_value;
    mp_obj_t pwm_duty;
    mp_obj_t encoder_count;   // bound encoder.count — called every tick
    mp_obj_t encoder_reset;   // bound encoder.reset — called from reset_angle

    // Configuration.
    mp_int_t   counts_per_rev;
    mp_float_t kp;
    bool       invert;

    // Control state. ``target_dps`` is written by the servo's own
    // ``run_speed`` / trajectory tick, *and* by the drivebase tick
    // when this servo is part of a coupled DriveBase — either way,
    // the servo's control step reads it each beat.
    mp_float_t target_dps;
    bool       active;

    // Per-motor trajectory tracking. Not used while a drivebase owns
    // this servo (the drivebase samples its own profile and writes
    // ``target_dps`` directly).
    //
    // ``traj_start_ms`` is mp_int_t (not uint32_t) so it matches
    // ``mp_hal_ticks_ms()``'s return width on 64-bit MP ports — a
    // narrower type truncated the value and produced bogus elapsed
    // times.
    trajectory_obj_t trajectory;
    mp_int_t         traj_start_ms;
    bool             traj_active;
    bool             traj_done;

    // α-β state observer.
    observer_obj_t observer;

    // Time baseline for observer updates.
    mp_int_t last_time_ms;
} servo_obj_t;
