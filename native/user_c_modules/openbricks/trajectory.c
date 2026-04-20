// SPDX-License-Identifier: MIT
//
// openbricks — trajectory.c
//
// Trapezoidal speed profile — the same shape pbio uses in
// pbio/src/trajectory.c. Given a signed angular distance, a cruise
// speed, and an acceleration, produce position and velocity samples at
// any time between profile start and completion.
//
// Profile cases:
//
//   * Trapezoidal (distance large enough to reach cruise speed):
//     [0, t_ramp):           accelerate from 0 to v_cruise
//     [t_ramp, t_ramp+t_c):  hold at v_cruise
//     [t_ramp+t_c, t_total): decelerate from v_cruise to 0
//
//   * Triangular (distance too short to reach cruise speed):
//     [0, t_peak):           accelerate from 0 to v_peak
//     [t_peak, 2*t_peak):    decelerate from v_peak to 0
//     where v_peak = sqrt(distance * accel)
//
// Negative distance flips the direction of position and velocity; the
// time/phase arithmetic operates on the magnitude.
//
// Exposed as ``_openbricks_native.TrapezoidalProfile`` so user code can
// build custom controllers. Internally, ``servo.c`` uses this type to
// sample setpoints inside the 1 kHz tick.

#include <math.h>
#include <stdbool.h>

#include "py/runtime.h"

#include "trajectory.h"

extern const mp_obj_type_t openbricks_trajectory_type;

// -----------------------------------------------------------------------
// Profile precomputation — fold all the case splits into a few scalar
// fields so ``sample`` is branch-light.

void openbricks_trajectory_init(trajectory_obj_t *t,
                                 mp_float_t start,
                                 mp_float_t target,
                                 mp_float_t cruise,
                                 mp_float_t accel) {
    t->start    = start;
    t->distance = target - start;
    t->cruise   = (cruise < 0) ? -cruise : cruise;
    t->accel    = (accel < 0)  ? -accel  : accel;

    mp_float_t D = (t->distance < 0) ? -t->distance : t->distance;
    t->direction = (t->distance < 0) ? -1.0 : 1.0;

    if (D == 0.0 || t->cruise == 0.0 || t->accel == 0.0) {
        // Degenerate — no motion.
        t->t_ramp     = 0.0;
        t->t_cruise   = 0.0;
        t->t_total    = 0.0;
        t->d_ramp     = 0.0;
        t->v_peak     = 0.0;
        t->triangular = false;
        return;
    }

    mp_float_t t_ramp_full = t->cruise / t->accel;
    mp_float_t d_ramp_full = 0.5 * t->accel * t_ramp_full * t_ramp_full;

    if (2.0 * d_ramp_full <= D) {
        t->triangular = false;
        t->t_ramp     = t_ramp_full;
        t->d_ramp     = d_ramp_full;
        t->v_peak     = t->cruise;
        t->t_cruise   = (D - 2.0 * d_ramp_full) / t->cruise;
        t->t_total    = 2.0 * t->t_ramp + t->t_cruise;
    } else {
        t->triangular = true;
        mp_float_t t_peak = sqrt(D / t->accel);
        t->t_ramp   = t_peak;
        t->t_cruise = 0.0;
        t->t_total  = 2.0 * t_peak;
        t->v_peak   = t->accel * t_peak;
        t->d_ramp   = 0.5 * t->accel * t_peak * t_peak;  // = D/2
    }
}

void openbricks_trajectory_sample(const trajectory_obj_t *t,
                                   mp_float_t t_s,
                                   mp_float_t *pos_out,
                                   mp_float_t *vel_out) {
    mp_float_t abs_pos;
    mp_float_t abs_vel;

    if (t_s <= 0.0) {
        abs_pos = 0.0;
        abs_vel = 0.0;
    } else if (t_s >= t->t_total) {
        abs_pos = (t->distance < 0) ? -t->distance : t->distance;
        abs_vel = 0.0;
    } else if (t_s < t->t_ramp) {
        // Accel phase: v = a*t, x = 0.5*a*t^2
        abs_vel = t->accel * t_s;
        abs_pos = 0.5 * t->accel * t_s * t_s;
    } else if (!t->triangular && t_s < t->t_ramp + t->t_cruise) {
        // Cruise phase
        abs_vel = t->v_peak;
        abs_pos = t->d_ramp + t->v_peak * (t_s - t->t_ramp);
    } else {
        // Decel phase
        mp_float_t decel_start = t->triangular
                                 ? t->t_ramp
                                 : t->t_ramp + t->t_cruise;
        mp_float_t td = t_s - decel_start;
        abs_vel = t->v_peak - t->accel * td;
        if (abs_vel < 0.0) {
            abs_vel = 0.0;
        }
        mp_float_t d_before_decel = t->triangular
                                    ? t->d_ramp
                                    : t->d_ramp + t->v_peak * t->t_cruise;
        abs_pos = d_before_decel + t->v_peak * td - 0.5 * t->accel * td * td;
    }

    *pos_out = t->start   + t->direction * abs_pos;
    *vel_out = t->direction * abs_vel;
}

// -----------------------------------------------------------------------
// Python-facing methods

static mp_obj_t traj_make_new(const mp_obj_type_t *type,
                              size_t n_args, size_t n_kw,
                              const mp_obj_t *all_args) {
    enum { ARG_start, ARG_target, ARG_cruise, ARG_accel };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_start,      MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_target,     MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_cruise_dps, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_accel_dps2, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed), allowed, parsed);

    trajectory_obj_t *self = mp_obj_malloc(trajectory_obj_t, type);
    openbricks_trajectory_init(
        self,
        mp_obj_get_float(parsed[ARG_start].u_obj),
        mp_obj_get_float(parsed[ARG_target].u_obj),
        mp_obj_get_float(parsed[ARG_cruise].u_obj),
        mp_obj_get_float(parsed[ARG_accel].u_obj)
    );
    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t traj_sample(mp_obj_t self_in, mp_obj_t t_in) {
    trajectory_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_float_t pos, vel;
    openbricks_trajectory_sample(self, mp_obj_get_float(t_in), &pos, &vel);
    mp_obj_t pair[2] = { mp_obj_new_float(pos), mp_obj_new_float(vel) };
    return mp_obj_new_tuple(2, pair);
}
static MP_DEFINE_CONST_FUN_OBJ_2(traj_sample_obj, traj_sample);

static mp_obj_t traj_duration(mp_obj_t self_in) {
    trajectory_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float(self->t_total);
}
static MP_DEFINE_CONST_FUN_OBJ_1(traj_duration_obj, traj_duration);

static mp_obj_t traj_is_triangular(mp_obj_t self_in) {
    trajectory_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->triangular);
}
static MP_DEFINE_CONST_FUN_OBJ_1(traj_is_triangular_obj, traj_is_triangular);

// -----------------------------------------------------------------------
// Type definition

static const mp_rom_map_elem_t traj_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_sample),         MP_ROM_PTR(&traj_sample_obj) },
    { MP_ROM_QSTR(MP_QSTR_duration),       MP_ROM_PTR(&traj_duration_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_triangular),  MP_ROM_PTR(&traj_is_triangular_obj) },
};
static MP_DEFINE_CONST_DICT(traj_locals_dict, traj_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_trajectory_type,
    MP_QSTR_TrapezoidalProfile,
    MP_TYPE_FLAG_NONE,
    make_new,    traj_make_new,
    locals_dict, &traj_locals_dict
);
