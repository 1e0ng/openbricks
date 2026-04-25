// SPDX-License-Identifier: MIT
//
// MicroPython object wrapping ``ob_drivebase_t``. The 2-DOF coupled
// control law lives in ``drivebase_core.{c,h}``; this header exposes
// only the embedding struct so future C modules can peek at fields
// (e.g. for instrumentation), mirroring how ``servo.h`` exposes
// ``servo_obj_t.core``.

#pragma once

#include <stdbool.h>

#include "py/obj.h"

#include "drivebase_core.h"
#include "servo.h"


typedef struct _drivebase_obj_t {
    mp_obj_base_t base;

    // Strong refs to keep the servo Python objects alive.
    mp_obj_t left_obj;
    mp_obj_t right_obj;

    // Optional IMU (Python object) + cached bound ``heading`` method.
    // When ``core.use_gyro`` is true, the tick reads heading from the
    // IMU and converts it to a wheel-degree differential for the core.
    mp_obj_t imu;
    mp_obj_t imu_heading_fn;
    ob_float_t heading_offset_deg;   // body-degrees captured at move-start

    // Scheduler-registration tracking — separate from ``core.done`` so
    // unregistering an already-finished move doesn't double-unregister.
    bool registered;

    // The portable state machine — physical params, gains, embedded
    // trajectories + per-axis hold targets, slip/IMU heading flag.
    ob_drivebase_t core;
} drivebase_obj_t;
