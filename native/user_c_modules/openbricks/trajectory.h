// SPDX-License-Identifier: MIT
//
// Internal C API for the trapezoidal trajectory planner. Used by
// servo.c to sample setpoints inside the 1 kHz control tick without
// going through the Python object model.
//
// The full type definition is here (rather than hidden in the .c) so
// ``servo_obj_t`` can embed a ``trajectory_obj_t`` directly and avoid
// an allocation per trajectory.

#pragma once

#include <stdbool.h>

#include "py/obj.h"

typedef struct _trajectory_obj_t {
    mp_obj_base_t base;
    mp_float_t    start;
    mp_float_t    distance;     // signed (target - start)
    mp_float_t    cruise;       // magnitude
    mp_float_t    accel;        // magnitude
    mp_float_t    direction;    // +1 or -1
    mp_float_t    t_ramp;       // seconds in accel (and decel) phase
    mp_float_t    t_cruise;     // seconds at cruise speed (0 for triangular)
    mp_float_t    t_total;      // seconds to completion
    mp_float_t    d_ramp;       // degrees covered in one ramp phase
    mp_float_t    v_peak;       // peak speed actually reached (magnitude)
    bool          triangular;   // true if the profile never reaches cruise
} trajectory_obj_t;

void openbricks_trajectory_init(trajectory_obj_t *t,
                                 mp_float_t start,
                                 mp_float_t target,
                                 mp_float_t cruise,
                                 mp_float_t accel);

void openbricks_trajectory_sample(const trajectory_obj_t *t,
                                   mp_float_t t_s,
                                   mp_float_t *pos_out,
                                   mp_float_t *vel_out);
