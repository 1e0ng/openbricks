// SPDX-License-Identifier: MIT
//
// MicroPython binding shell for the native ``DriveBase`` type. The
// 2-DOF coupled control law lives in ``drivebase_core.c`` so the
// firmware build and the host-side ``openbricks_sim._native``
// extension run identical math.
//
// What stays here:
//   - IMU heading read (cached ``imu.heading`` bound method) +
//     body→wheel-degree conversion before each tick.
//   - mp_arg_parse + mp_obj_t constructor / accessor methods.
//   - The motor_process subscription — the firmware uses the MP
//     scheduler singleton; the sim binding registers against its own
//     ``ob_motor_process_t`` independently.

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "py/runtime.h"
#include "py/mphal.h"

#include "motor_process.h"
#include "drivebase.h"
#include "servo.h"

extern const mp_obj_type_t openbricks_drivebase_type;

// ---------------------------------------------------------------------
// IMU heading helpers — ABSOLUTE frame (Pybricks-style, 1.25.0):
// baselined ONCE at the use_gyro enable transition, then accumulated
// continuously. Re-baselining per move (the old behaviour) forgave
// each turn's termination overshoot, which accumulated ~+7 deg per
// gyro'd square on the bench; in the absolute frame the persistent
// target (core ``turn_hold``) pulls overshoot back in later moves.

static void drivebase_gyro_baseline(drivebase_obj_t *self) {
    if (self->imu_heading_fn == MP_OBJ_NULL) {
        return;
    }
    self->heading_prev_deg =
        (ob_float_t)mp_obj_get_float(mp_call_function_0(self->imu_heading_fn));
    self->heading_cont_deg = 0.0;
}


// Advance and return the continuous body heading (degrees since the
// enable baseline). Per-tick deltas are wrapped into [-180, 180] so
// crossing the BNO055's ±180 boundary doesn't inject a ±360 jump,
// then accumulated — multi-turn totals stay representable.
static ob_float_t drivebase_read_body_delta(drivebase_obj_t *self) {
    ob_float_t body = (ob_float_t)mp_obj_get_float(
        mp_call_function_0(self->imu_heading_fn));
    ob_float_t delta = body - self->heading_prev_deg;
    if (delta >  (ob_float_t)180.0) delta -= (ob_float_t)360.0;
    if (delta < (ob_float_t)-180.0) delta += (ob_float_t)360.0;
    self->heading_prev_deg  = body;
    self->heading_cont_deg += delta;
    return self->heading_cont_deg;
}


// ---------------------------------------------------------------------
// Control tick — the native hot path. Reads the IMU (if active),
// stuffs the wheel-degree differential into the core's override slot,
// and calls into the shared core for the actual math.

static void drivebase_control_tick(void *ctx) {
    drivebase_obj_t *self = (drivebase_obj_t *)ctx;
    if (self->core.use_gyro && self->imu_heading_fn != MP_OBJ_NULL) {
        ob_float_t body_delta = drivebase_read_body_delta(self);
        self->core.heading_override_wheel_deg =
            ob_drivebase_body_to_wheel_diff(&self->core, body_delta);
    }
    ob_drivebase_tick(&self->core, (long)openbricks_motor_process_now_ms());
}


// ---------------------------------------------------------------------
// Scheduler register / unregister

static void drivebase_register(drivebase_obj_t *self) {
    if (self->registered) {
        return;
    }
    // (No gyro re-baseline here — the absolute frame is seeded once
    // at the use_gyro enable transition, deliberately NOT per move.)
    // Re-baseline each servo's observer so the first tick doesn't see
    // a phantom residual. The observer reads its measurement from the
    // bound encoder.count() — uniform across QuadratureEncoder /
    // PCNTEncoder / any future encoder type.
    servo_obj_t *left_servo  = MP_OBJ_TO_PTR(self->left_obj);
    servo_obj_t *right_servo = MP_OBJ_TO_PTR(self->right_obj);
    int64_t lc = (int64_t)mp_obj_get_ll(mp_call_function_0(left_servo->encoder_count));
    int64_t rc = (int64_t)mp_obj_get_ll(mp_call_function_0(right_servo->encoder_count));
    ob_float_t lp = ob_servo_count_to_angle_deg(self->core.left,  lc);
    ob_float_t rp = ob_servo_count_to_angle_deg(self->core.right, rc);
    ob_observer_reset(&self->core.left->observer,  lp);
    ob_observer_reset(&self->core.right->observer, rp);
    long now = (long)openbricks_motor_process_now_ms();
    self->core.left->last_time_ms  = now;
    self->core.right->last_time_ms = now;

    // Hand both servos into "speed" mode at zero target. The Python
    // caller is expected to have already attached the servos to the
    // motor process (e.g. via ``run_speed(0)``); we just make sure
    // their internal trajectory tracking is off so DriveBase's
    // target_dps writes aren't fought by the per-servo profile.
    self->core.left->target_dps   = 0.0;
    self->core.right->target_dps  = 0.0;
    self->core.left->traj_active  = false;
    self->core.right->traj_active = false;
    self->core.left->traj_done    = true;
    self->core.right->traj_done   = true;

    openbricks_motor_process_register_c(drivebase_control_tick, self);
    self->registered = true;
}


static void drivebase_unregister(drivebase_obj_t *self) {
    if (!self->registered) {
        return;
    }
    openbricks_motor_process_unregister_c(drivebase_control_tick, self);
    self->registered = false;
}


// ---------------------------------------------------------------------
// Python-facing methods

static mp_obj_t db_straight(mp_obj_t self_in, mp_obj_t distance_in,
                             mp_obj_t speed_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    ob_drivebase_straight(&self->core,
                          (long)openbricks_motor_process_now_ms(),
                          (ob_float_t)mp_obj_get_float(distance_in),
                          (ob_float_t)mp_obj_get_float(speed_in));
    drivebase_register(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(db_straight_obj, db_straight);


static mp_obj_t db_turn(mp_obj_t self_in, mp_obj_t angle_in,
                         mp_obj_t rate_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    ob_drivebase_turn(&self->core,
                      (long)openbricks_motor_process_now_ms(),
                      (ob_float_t)mp_obj_get_float(angle_in),
                      (ob_float_t)mp_obj_get_float(rate_in));
    drivebase_register(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(db_turn_obj, db_turn);


static mp_obj_t db_stop(mp_obj_t self_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    ob_drivebase_stop(&self->core);
    drivebase_unregister(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(db_stop_obj, db_stop);


static mp_obj_t db_use_gyro(mp_obj_t self_in, mp_obj_t enable_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    bool enable = mp_obj_is_true(enable_in);
    if (enable && self->imu_heading_fn == MP_OBJ_NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "no imu attached; construct DriveBase(imu=...) first"));
    }
    bool transitioning_on = enable && !self->core.use_gyro;
    self->core.use_gyro = enable;
    if (transitioning_on) {
        // "Here, now" becomes both measured-zero and the initial
        // target of the absolute frame.
        drivebase_gyro_baseline(self);
        ob_drivebase_gyro_frame_reset(&self->core);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(db_use_gyro_obj, db_use_gyro);


static mp_obj_t db_is_done(mp_obj_t self_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(ob_drivebase_is_done(&self->core));
}
static MP_DEFINE_CONST_FUN_OBJ_1(db_is_done_obj, db_is_done);


// Set the trajectory acceleration (wheel-degrees/s²) used by
// subsequent straight() / turn() moves. Shared by both profiles.
// An in-flight move keeps the acceleration it was armed with.
static mp_obj_t db_set_accel(mp_obj_t self_in, mp_obj_t accel_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_float_t accel = mp_obj_get_float(accel_in);
    if (!(accel > (mp_float_t)0.0)) {
        mp_raise_ValueError(MP_ERROR_TEXT(
            "acceleration must be > 0 deg/s^2"));
    }
    self->core.accel_dps2 = (ob_float_t)accel;
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(db_set_accel_obj, db_set_accel);


// ---------------------------------------------------------------------
// Constructor

static mp_obj_t db_make_new(const mp_obj_type_t *type,
                             size_t n_args, size_t n_kw,
                             const mp_obj_t *all_args) {
    enum { ARG_left, ARG_right, ARG_wheel_d, ARG_axle, ARG_imu,
           ARG_kp_sum, ARG_kp_diff };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_left,              MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_right,             MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_wheel_diameter_mm, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_axle_track_mm,     MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_imu,               MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_kp_sum,            MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_kp_diff,           MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed), allowed, parsed);

    extern const mp_obj_type_t openbricks_servo_type;
    mp_obj_t left  = parsed[ARG_left].u_obj;
    mp_obj_t right = parsed[ARG_right].u_obj;
    if (!mp_obj_is_type(left,  &openbricks_servo_type) ||
        !mp_obj_is_type(right, &openbricks_servo_type)) {
        mp_raise_TypeError(MP_ERROR_TEXT(
            "left and right must be Servo instances"));
    }

    drivebase_obj_t *self = mp_obj_malloc(drivebase_obj_t, type);
    self->left_obj  = left;
    self->right_obj = right;

    ob_float_t kp_sum  = (parsed[ARG_kp_sum].u_obj  != MP_OBJ_NULL)
        ? (ob_float_t)mp_obj_get_float(parsed[ARG_kp_sum].u_obj)
        : (ob_float_t)OB_DRIVEBASE_DEFAULT_KP_SUM;
    ob_float_t kp_diff = (parsed[ARG_kp_diff].u_obj != MP_OBJ_NULL)
        ? (ob_float_t)mp_obj_get_float(parsed[ARG_kp_diff].u_obj)
        : (ob_float_t)OB_DRIVEBASE_DEFAULT_KP_DIFF;

    ob_drivebase_init(&self->core,
                      &((servo_obj_t *)MP_OBJ_TO_PTR(left))->core,
                      &((servo_obj_t *)MP_OBJ_TO_PTR(right))->core,
                      (ob_float_t)mp_obj_get_float(parsed[ARG_wheel_d].u_obj),
                      (ob_float_t)mp_obj_get_float(parsed[ARG_axle].u_obj),
                      kp_sum,
                      kp_diff);

    self->registered         = false;
    self->heading_prev_deg = 0.0;
    self->heading_cont_deg = 0.0;

    self->imu            = parsed[ARG_imu].u_obj;
    self->imu_heading_fn = MP_OBJ_NULL;
    if (self->imu != MP_OBJ_NULL && self->imu != mp_const_none) {
        self->imu_heading_fn = mp_load_attr(self->imu, MP_QSTR_heading);
    }

    return MP_OBJ_FROM_PTR(self);
}


// ---------------------------------------------------------------------
// Type definition

static const mp_rom_map_elem_t db_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_straight), MP_ROM_PTR(&db_straight_obj) },
    { MP_ROM_QSTR(MP_QSTR_turn),     MP_ROM_PTR(&db_turn_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),     MP_ROM_PTR(&db_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_done),  MP_ROM_PTR(&db_is_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_use_gyro), MP_ROM_PTR(&db_use_gyro_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_accel), MP_ROM_PTR(&db_set_accel_obj) },
};
static MP_DEFINE_CONST_DICT(db_locals_dict, db_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_drivebase_type,
    MP_QSTR_DriveBase,
    MP_TYPE_FLAG_NONE,
    make_new,    db_make_new,
    locals_dict, &db_locals_dict
);
