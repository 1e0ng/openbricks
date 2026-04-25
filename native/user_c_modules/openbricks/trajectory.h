// SPDX-License-Identifier: MIT
//
// MicroPython object wrapping ``ob_trajectory_t``. The algorithm
// lives in ``trajectory_core.{c,h}`` so it can compile into both the
// firmware ``_openbricks_native`` module and the host-side
// ``openbricks_sim._native`` extension without drift.

#pragma once

#include <stdbool.h>

#include "py/obj.h"

#include "trajectory_core.h"

typedef struct _trajectory_obj_t {
    mp_obj_base_t   base;
    ob_trajectory_t core;
} trajectory_obj_t;

// Backward-compat wrappers around ``ob_trajectory_init`` /
// ``ob_trajectory_sample`` that accept ``trajectory_obj_t*`` and
// ``mp_float_t``. Used by ``servo.c`` and ``drivebase.c`` which embed a
// ``trajectory_obj_t`` directly and didn't need to know about the
// firmware-vs-sim split.
void openbricks_trajectory_init(trajectory_obj_t *t,
                                 mp_float_t start,
                                 mp_float_t target,
                                 mp_float_t cruise,
                                 mp_float_t accel);

void openbricks_trajectory_sample(const trajectory_obj_t *t,
                                   mp_float_t t_s,
                                   mp_float_t *pos_out,
                                   mp_float_t *vel_out);
