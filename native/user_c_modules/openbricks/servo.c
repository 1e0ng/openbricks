// SPDX-License-Identifier: MIT
//
// MicroPython binding shell for the native ``Servo`` type. The state
// machine + control law live in ``servo_core.c`` so the firmware
// build and the host-side ``openbricks_sim._native`` extension run
// identical math.
//
// What stays here:
//   - Hardware I/O (cached encoder.count(), in1.value(), in2.value(),
//     pwm.duty()).
//   - The brake / coast bridge driving — the L298N has a specific
//     IN1=IN2=1, duty=max pattern for active brake; that's hardware-
//     wiring, not algorithm.
//   - mp_arg_parse + mp_obj_t constructor / accessor methods.
//   - The motor_process subscription (``servo_attach`` /
//     ``servo_detach``) — the firmware uses the MP scheduler
//     singleton; the sim side will register against its own
//     ``ob_motor_process_t``.
//
// Sibling C modules (drivebase.c) reach into ``self->core.observer``
// / ``self->core.target_dps`` / ``self->core.traj_active`` directly
// for zero-overhead reads in the 1 kHz tick.

#include <stdbool.h>

#include "py/runtime.h"
#include "py/mphal.h"

#include "motor_process.h"
#include "servo.h"

#define DUTY_MAX 1023               // 10-bit PWM (ESP32 default; matches L298N driver)
#define DEFAULT_ACCEL ((mp_float_t)720.0)  // deg/s^2 if run_target omits accel

extern const mp_obj_type_t openbricks_servo_type;

// -----------------------------------------------------------------------
// Hardware primitives — called from the C tick body.

static inline mp_int_t servo_read_count(servo_obj_t *self) {
    return mp_obj_get_int(mp_call_function_0(self->encoder_count));
}

static inline void servo_write_in1(servo_obj_t *self, int value) {
    mp_call_function_1(self->in1_value, MP_OBJ_NEW_SMALL_INT(value));
}

static inline void servo_write_in2(servo_obj_t *self, int value) {
    mp_call_function_1(self->in2_value, MP_OBJ_NEW_SMALL_INT(value));
}

static inline void servo_write_duty(servo_obj_t *self, int duty) {
    mp_call_function_1(self->pwm_duty, MP_OBJ_NEW_SMALL_INT(duty));
}

// Drive the H-bridge to ``power`` in the range [-100, 100]. The core
// has already clamped the magnitude; we just translate sign +
// magnitude into IN1/IN2/duty patterns. The ``invert`` flag from the
// core swaps direction for motors wired the opposite way.
static void servo_drive_power(servo_obj_t *self, mp_float_t power) {
    mp_float_t effective = self->core.invert ? -power : power;

    if (effective > 0.0) {
        servo_write_in1(self, 1);
        servo_write_in2(self, 0);
    } else if (effective < 0.0) {
        servo_write_in1(self, 0);
        servo_write_in2(self, 1);
    } else {
        servo_write_in1(self, 0);
        servo_write_in2(self, 0);
    }

    int duty = (int)(((effective < 0.0) ? -effective : effective) * DUTY_MAX / 100.0);
    if (duty > DUTY_MAX) {
        duty = DUTY_MAX;
    } else if (duty < 0) {
        duty = 0;
    }
    servo_write_duty(self, duty);
}

static void servo_brake_bridge(servo_obj_t *self) {
    // Short both terminals — active brake on the L298N.
    servo_write_in1(self, 1);
    servo_write_in2(self, 1);
    servo_write_duty(self, DUTY_MAX);
}

static void servo_coast_bridge(servo_obj_t *self) {
    servo_write_in1(self, 0);
    servo_write_in2(self, 0);
    servo_write_duty(self, 0);
}

// -----------------------------------------------------------------------
// Control tick — the native hot path. The control math sits in
// ``ob_servo_tick``; here we just read the encoder + write the bridge.

static void servo_control_tick(void *ctx) {
    servo_obj_t *self = (servo_obj_t *)ctx;
    long count = (long)servo_read_count(self);
    long now   = (long)openbricks_motor_process_now_ms();
    mp_float_t power = (mp_float_t)ob_servo_tick(&self->core, count, now);
    servo_drive_power(self, power);
}

// -----------------------------------------------------------------------
// Attach / detach from motor_process

static void servo_attach(servo_obj_t *self) {
    if (self->core.active) {
        return;
    }
    long count = (long)servo_read_count(self);
    long now   = (long)openbricks_motor_process_now_ms();
    ob_servo_baseline(&self->core, count, now);
    openbricks_motor_process_register_c(servo_control_tick, self);
    self->core.active = true;
}

static void servo_detach(servo_obj_t *self) {
    if (!self->core.active) {
        return;
    }
    openbricks_motor_process_unregister_c(servo_control_tick, self);
    self->core.active      = false;
    self->core.target_dps  = 0.0;
    self->core.traj_active = false;
    self->core.traj_done   = true;
}

// -----------------------------------------------------------------------
// Python-facing methods

static mp_obj_t servo_run_speed(mp_obj_t self_in, mp_obj_t dps_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    ob_servo_set_speed(&self->core, (ob_float_t)mp_obj_get_float(dps_in));
    servo_attach(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(servo_run_speed_obj, servo_run_speed);

// servo.run_target(delta_deg, cruise_dps, accel_dps2=DEFAULT_ACCEL)
static mp_obj_t servo_run_target(size_t n_args, const mp_obj_t *args) {
    servo_obj_t *self     = MP_OBJ_TO_PTR(args[0]);
    mp_float_t delta_deg  = mp_obj_get_float(args[1]);
    mp_float_t cruise_dps = mp_obj_get_float(args[2]);
    mp_float_t accel      = (n_args > 3) ? mp_obj_get_float(args[3]) : DEFAULT_ACCEL;

    long count = (long)servo_read_count(self);
    long now   = (long)openbricks_motor_process_now_ms();
    ob_servo_run_target(&self->core, count, now,
                        (ob_float_t)delta_deg,
                        (ob_float_t)cruise_dps,
                        (ob_float_t)accel);
    servo_attach(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(servo_run_target_obj, 3, 4, servo_run_target);

static mp_obj_t servo_is_done(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(ob_servo_is_done(&self->core));
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_is_done_obj, servo_is_done);

static mp_obj_t servo_target_dps(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float((mp_float_t)self->core.target_dps);
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_target_dps_obj, servo_target_dps);

static mp_obj_t servo_is_active(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->core.active);
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_is_active_obj, servo_is_active);

static mp_obj_t servo_run(mp_obj_t self_in, mp_obj_t power_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    servo_detach(self);
    mp_float_t power = mp_obj_get_float(power_in);
    if (power >  100.0) power =  100.0;
    if (power < -100.0) power = -100.0;
    servo_drive_power(self, power);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(servo_run_obj, servo_run);

static mp_obj_t servo_brake(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    servo_detach(self);
    servo_brake_bridge(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_brake_obj, servo_brake);

static mp_obj_t servo_coast(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    servo_detach(self);
    servo_coast_bridge(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_coast_obj, servo_coast);

static mp_obj_t servo_angle(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    long count = (long)servo_read_count(self);
    return mp_obj_new_float((mp_float_t)ob_servo_count_to_angle_deg(&self->core, count));
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_angle_obj, servo_angle);

static mp_obj_t servo_reset_angle(size_t n_args, const mp_obj_t *args) {
    servo_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_float_t angle = (n_args > 1) ? mp_obj_get_float(args[1]) : 0.0;
    mp_int_t new_count = (mp_int_t)(angle * (mp_float_t)self->core.counts_per_rev / 360.0);
    // encoder.reset(new_count) — uniform across all encoder drivers,
    // including ones that back ``_count`` with a hardware peripheral.
    mp_call_function_1(self->encoder_reset, MP_OBJ_NEW_SMALL_INT(new_count));
    // Re-baseline the observer + time baseline so the first tick after
    // a reset_angle doesn't see a huge phantom delta.
    ob_servo_baseline(&self->core, (long)new_count,
                      (long)openbricks_motor_process_now_ms());
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(servo_reset_angle_obj, 1, 2, servo_reset_angle);

// run_angle is intentionally omitted from C — it's a blocking loop
// that's easier to express in Python (sleep_ms / break on angle).
// The Python wrapper provides it, calling run_speed + brake.

// -----------------------------------------------------------------------
// Constructor

static mp_obj_t servo_make_new(const mp_obj_type_t *type,
                               size_t n_args, size_t n_kw,
                               const mp_obj_t *all_args) {
    enum { ARG_in1, ARG_in2, ARG_pwm, ARG_encoder, ARG_counts_per_rev, ARG_invert, ARG_kp };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_in1,              MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_in2,              MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_pwm,              MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_encoder,          MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_counts_per_rev,   MP_ARG_INT, {.u_int  = 1320} },
        { MP_QSTR_invert,           MP_ARG_BOOL,{.u_bool = false} },
        { MP_QSTR_kp,               MP_ARG_OBJ, {.u_obj  = MP_OBJ_NULL} },
    };
    mp_arg_val_t parsed[MP_ARRAY_SIZE(allowed)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args,
                              MP_ARRAY_SIZE(allowed), allowed, parsed);

    servo_obj_t *self = mp_obj_malloc(servo_obj_t, type);
    self->in1_pin = parsed[ARG_in1].u_obj;
    self->in2_pin = parsed[ARG_in2].u_obj;
    self->pwm     = parsed[ARG_pwm].u_obj;
    self->encoder = parsed[ARG_encoder].u_obj;

    // Cache bound methods so the tick body does pure calls.
    self->in1_value     = mp_load_attr(self->in1_pin, MP_QSTR_value);
    self->in2_value     = mp_load_attr(self->in2_pin, MP_QSTR_value);
    self->pwm_duty      = mp_load_attr(self->pwm,     MP_QSTR_duty);
    self->encoder_count = mp_load_attr(self->encoder, MP_QSTR_count);
    self->encoder_reset = mp_load_attr(self->encoder, MP_QSTR_reset);

    ob_float_t kp = (parsed[ARG_kp].u_obj != MP_OBJ_NULL)
                    ? (ob_float_t)mp_obj_get_float(parsed[ARG_kp].u_obj)
                    : (ob_float_t)OB_SERVO_DEFAULT_KP;
    ob_servo_init(&self->core,
                  (int)parsed[ARG_counts_per_rev].u_int,
                  kp,
                  parsed[ARG_invert].u_bool);
    self->core.last_time_ms = (long)openbricks_motor_process_now_ms();

    return MP_OBJ_FROM_PTR(self);
}

// -----------------------------------------------------------------------
// Type definition

static const mp_rom_map_elem_t servo_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_run_speed),   MP_ROM_PTR(&servo_run_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_run_target),  MP_ROM_PTR(&servo_run_target_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_done),     MP_ROM_PTR(&servo_is_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_target_dps),  MP_ROM_PTR(&servo_target_dps_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_active),   MP_ROM_PTR(&servo_is_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_run),         MP_ROM_PTR(&servo_run_obj) },
    { MP_ROM_QSTR(MP_QSTR_brake),       MP_ROM_PTR(&servo_brake_obj) },
    { MP_ROM_QSTR(MP_QSTR_coast),       MP_ROM_PTR(&servo_coast_obj) },
    { MP_ROM_QSTR(MP_QSTR_angle),       MP_ROM_PTR(&servo_angle_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_angle), MP_ROM_PTR(&servo_reset_angle_obj) },
};
static MP_DEFINE_CONST_DICT(servo_locals_dict, servo_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_servo_type,
    MP_QSTR_Servo,
    MP_TYPE_FLAG_NONE,
    make_new,    servo_make_new,
    locals_dict, &servo_locals_dict
);
