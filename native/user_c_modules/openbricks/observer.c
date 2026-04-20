// SPDX-License-Identifier: MIT
//
// openbricks — observer.c
//
// Two-state (position, velocity) α-β state observer. Given a periodic
// stream of noisy position measurements plus the time step between
// them, produces smoothed estimates of both position and velocity.
//
// Plays the same role as pbio's observer.c, but is strictly less
// capable. pbio runs a **model-based full-state observer** — 4 states
// (position, velocity, motor current, stator flux), driven by both the
// PWM command and the encoder read, with a motor model (torque,
// inertia, friction, back-EMF). That lets it predict how the motor
// responds to voltage between encoder edges and handle disturbances
// (friction, load, battery sag) gracefully.
//
// This α-β filter is a step up from raw finite-differencing but nowhere
// near pbio's observer. It has no motor model; it only sees the
// encoder; it assumes constant velocity between samples. The reason
// it's worth landing anyway is that at 1 kHz, raw finite-differencing
// is extremely noisy — ΔN is 0 or ±1 most ticks — and the α-β form
// gives a ~60x variance reduction in velocity for very little code.
// Good enough for M2b. The full model-based observer with current /
// flux states and PWM coupling lands in a later milestone, once the
// drivebase work exposes where the remaining control-quality gap is.
//
// Algorithm (per tick):
//
//     predict:  p̂' = p̂ + v̂ · dt
//               v̂' = v̂
//     innovate: r  = y - p̂'
//     update:   p̂  = p̂' + α · r
//               v̂  = v̂' + (β / dt) · r
//
// With α = 0.5 and β = 0.15 (the defaults) the filter is close to
// critically damped: it tracks step changes in position within a few
// samples and attenuates high-frequency noise by ~10x. Users can
// override alpha/beta per motor if they have specific noise / bandwidth
// requirements.
//
// Exposed as ``_openbricks_native.Observer`` so user code and tests can
// exercise it standalone; ``servo.c`` embeds one by value and calls
// the C API in its tick body.

#include "py/runtime.h"

#include "observer.h"

extern const mp_obj_type_t openbricks_observer_type;

// -----------------------------------------------------------------------
// C API

void openbricks_observer_init(observer_obj_t *o,
                               mp_float_t alpha,
                               mp_float_t beta) {
    o->alpha   = alpha;
    o->beta    = beta;
    o->pos_hat = 0.0;
    o->vel_hat = 0.0;
}

void openbricks_observer_reset(observer_obj_t *o, mp_float_t pos) {
    o->pos_hat = pos;
    o->vel_hat = 0.0;
}

void openbricks_observer_update(observer_obj_t *o,
                                 mp_float_t measured_pos,
                                 mp_float_t dt) {
    if (dt <= 0.0) {
        return;
    }
    // Predict.
    mp_float_t pos_pred = o->pos_hat + o->vel_hat * dt;
    // Innovate.
    mp_float_t residual = measured_pos - pos_pred;
    // Update.
    o->pos_hat = pos_pred          + o->alpha * residual;
    o->vel_hat = o->vel_hat        + (o->beta / dt) * residual;
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

    mp_float_t alpha = (parsed[ARG_alpha].u_obj != MP_OBJ_NULL)
                      ? mp_obj_get_float(parsed[ARG_alpha].u_obj) : 0.5;
    mp_float_t beta  = (parsed[ARG_beta].u_obj  != MP_OBJ_NULL)
                      ? mp_obj_get_float(parsed[ARG_beta].u_obj)  : 0.15;

    observer_obj_t *self = mp_obj_malloc(observer_obj_t, type);
    openbricks_observer_init(self, alpha, beta);
    return MP_OBJ_FROM_PTR(self);
}

static mp_obj_t obs_update(mp_obj_t self_in, mp_obj_t pos_in, mp_obj_t dt_in) {
    observer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    openbricks_observer_update(self,
                                mp_obj_get_float(pos_in),
                                mp_obj_get_float(dt_in));
    mp_obj_t pair[2] = {
        mp_obj_new_float(self->pos_hat),
        mp_obj_new_float(self->vel_hat),
    };
    return mp_obj_new_tuple(2, pair);
}
static MP_DEFINE_CONST_FUN_OBJ_3(obs_update_obj, obs_update);

static mp_obj_t obs_reset(size_t n_args, const mp_obj_t *args) {
    observer_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_float_t pos = (n_args > 1) ? mp_obj_get_float(args[1]) : 0.0;
    openbricks_observer_reset(self, pos);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(obs_reset_obj, 1, 2, obs_reset);

static mp_obj_t obs_position(mp_obj_t self_in) {
    observer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float(self->pos_hat);
}
static MP_DEFINE_CONST_FUN_OBJ_1(obs_position_obj, obs_position);

static mp_obj_t obs_velocity(mp_obj_t self_in) {
    observer_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float(self->vel_hat);
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
