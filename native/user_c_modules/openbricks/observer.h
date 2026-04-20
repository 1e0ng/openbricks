// SPDX-License-Identifier: MIT
//
// Internal C API for the state observer. Used by servo.c to replace
// the naive finite-difference velocity estimate with a smoothed one
// inside the 1 kHz control tick.

#pragma once

#include "py/obj.h"

typedef struct _observer_obj_t {
    mp_obj_base_t base;
    mp_float_t    alpha;     // position correction gain  (dimensionless)
    mp_float_t    beta;      // velocity correction gain (per second)
    mp_float_t    pos_hat;   // estimated position
    mp_float_t    vel_hat;   // estimated velocity (same units as pos / second)
} observer_obj_t;

void openbricks_observer_init(observer_obj_t *o, mp_float_t alpha, mp_float_t beta);
void openbricks_observer_reset(observer_obj_t *o, mp_float_t pos);
void openbricks_observer_update(observer_obj_t *o, mp_float_t measured_pos, mp_float_t dt);
