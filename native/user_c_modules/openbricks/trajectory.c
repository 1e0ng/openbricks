// SPDX-License-Identifier: MIT
//
// MicroPython bindings for the trapezoidal trajectory planner. The
// algorithm lives in ``trajectory_core.c`` so the firmware image and
// the host-side ``openbricks_sim._native`` CPython extension compile
// against identical math — no drift between sim and hardware.
//
// Exposed as ``_openbricks_native.TrapezoidalProfile``.

#include <stdbool.h>

#include "py/runtime.h"

#include "trajectory.h"

extern const mp_obj_type_t openbricks_trajectory_type;

// Servo.c embeds a trajectory inside its state; keep these wrappers
// available so the refactor is transparent to callers that previously
// used ``openbricks_trajectory_init`` / ``_sample`` directly.
void openbricks_trajectory_init(trajectory_obj_t *t,
                                 mp_float_t start,
                                 mp_float_t target,
                                 mp_float_t cruise,
                                 mp_float_t accel) {
    ob_trajectory_init(&t->core,
                       (ob_float_t)start,
                       (ob_float_t)target,
                       (ob_float_t)cruise,
                       (ob_float_t)accel);
}

void openbricks_trajectory_sample(const trajectory_obj_t *t,
                                   mp_float_t t_s,
                                   mp_float_t *pos_out,
                                   mp_float_t *vel_out) {
    ob_float_t pos, vel;
    ob_trajectory_sample(&t->core, (ob_float_t)t_s, &pos, &vel);
    *pos_out = (mp_float_t)pos;
    *vel_out = (mp_float_t)vel;
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
    ob_trajectory_init(
        &self->core,
        (ob_float_t)mp_obj_get_float(parsed[ARG_start].u_obj),
        (ob_float_t)mp_obj_get_float(parsed[ARG_target].u_obj),
        (ob_float_t)mp_obj_get_float(parsed[ARG_cruise].u_obj),
        (ob_float_t)mp_obj_get_float(parsed[ARG_accel].u_obj)
    );
    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t traj_sample(mp_obj_t self_in, mp_obj_t t_in) {
    trajectory_obj_t *self = MP_OBJ_TO_PTR(self_in);
    ob_float_t pos, vel;
    ob_trajectory_sample(&self->core,
                         (ob_float_t)mp_obj_get_float(t_in),
                         &pos, &vel);
    mp_obj_t pair[2] = {
        mp_obj_new_float((mp_float_t)pos),
        mp_obj_new_float((mp_float_t)vel),
    };
    return mp_obj_new_tuple(2, pair);
}
static MP_DEFINE_CONST_FUN_OBJ_2(traj_sample_obj, traj_sample);

static mp_obj_t traj_duration(mp_obj_t self_in) {
    trajectory_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float((mp_float_t)self->core.t_total);
}
static MP_DEFINE_CONST_FUN_OBJ_1(traj_duration_obj, traj_duration);

static mp_obj_t traj_is_triangular(mp_obj_t self_in) {
    trajectory_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->core.triangular);
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
