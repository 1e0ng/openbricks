// SPDX-License-Identifier: MIT
//
// MicroPython bindings for the α-β state observer. The algorithm
// lives in ``observer_core.c``; this file is the ``mp_obj_t`` shell.
//
// Two-state (position, velocity) α-β observer. Same role as pbio's
// model-based observer but strictly less capable — no motor model,
// just smooths the encoder readings. Documented in detail at the top
// of ``observer_core.c``.
//
// Exposed as ``_openbricks_native.Observer``.

#include "py/runtime.h"

#include "observer.h"

extern const mp_obj_type_t openbricks_observer_type;

// -----------------------------------------------------------------------
// Backward-compat C API used by ``servo.c`` (which embeds an
// ``observer_obj_t`` and was written against ``mp_float_t`` arguments).

void openbricks_observer_init(observer_obj_t *o,
                               mp_float_t alpha,
                               mp_float_t beta) {
    ob_observer_init(&o->core, (ob_float_t)alpha, (ob_float_t)beta);
}

void openbricks_observer_reset(observer_obj_t *o, mp_float_t pos) {
    ob_observer_reset(&o->core, (ob_float_t)pos);
}

void openbricks_observer_update(observer_obj_t *o,
                                 mp_float_t measured_pos,
                                 mp_float_t dt) {
    ob_observer_update(&o->core, (ob_float_t)measured_pos, (ob_float_t)dt);
}

// -----------------------------------------------------------------------
// Python-facing methods

static mp_obj_t obs_make_new(const mp_obj_type_t *type,
                              size_t n_args, size_t n_kw,
                              const mp_obj_t *all_args) {
    enum { ARG_alpha, ARG_beta };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_alpha, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_beta,  MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed), allowed, parsed);

    ob_float_t alpha = (parsed[ARG_alpha].u_obj != MP_OBJ_NULL)
                      ? (ob_float_t)mp_obj_get_float(parsed[ARG_alpha].u_obj)
                      : (ob_float_t)0.5;
    ob_float_t beta  = (parsed[ARG_beta].u_obj  != MP_OBJ_NULL)
                      ? (ob_float_t)mp_obj_get_float(parsed[ARG_beta].u_obj)
                      : (ob_float_t)0.15;

    observer_obj_t *self = mp_obj_malloc(observer_obj_t, type);
    ob_observer_init(&self->core, alpha, beta);
    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t obs_update(mp_obj_t self_in, mp_obj_t pos_in, mp_obj_t dt_in) {
    observer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    ob_observer_update(&self->core,
                       (ob_float_t)mp_obj_get_float(pos_in),
                       (ob_float_t)mp_obj_get_float(dt_in));
    mp_obj_t pair[2] = {
        mp_obj_new_float((mp_float_t)self->core.pos_hat),
        mp_obj_new_float((mp_float_t)self->core.vel_hat),
    };
    return mp_obj_new_tuple(2, pair);
}
static MP_DEFINE_CONST_FUN_OBJ_3(obs_update_obj, obs_update);

static mp_obj_t obs_reset(size_t n_args, const mp_obj_t *args) {
    observer_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    ob_float_t pos = (n_args > 1) ? (ob_float_t)mp_obj_get_float(args[1])
                                  : (ob_float_t)0.0;
    ob_observer_reset(&self->core, pos);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(obs_reset_obj, 1, 2, obs_reset);

static mp_obj_t obs_position(mp_obj_t self_in) {
    observer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float((mp_float_t)self->core.pos_hat);
}
static MP_DEFINE_CONST_FUN_OBJ_1(obs_position_obj, obs_position);

static mp_obj_t obs_velocity(mp_obj_t self_in) {
    observer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float((mp_float_t)self->core.vel_hat);
}
static MP_DEFINE_CONST_FUN_OBJ_1(obs_velocity_obj, obs_velocity);

// -----------------------------------------------------------------------
// Type definition

static const mp_rom_map_elem_t obs_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_update),   MP_ROM_PTR(&obs_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset),    MP_ROM_PTR(&obs_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_position), MP_ROM_PTR(&obs_position_obj) },
    { MP_ROM_QSTR(MP_QSTR_velocity), MP_ROM_PTR(&obs_velocity_obj) },
};
static MP_DEFINE_CONST_DICT(obs_locals_dict, obs_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_observer_type,
    MP_QSTR_Observer,
    MP_TYPE_FLAG_NONE,
    make_new,    obs_make_new,
    locals_dict, &obs_locals_dict
);
