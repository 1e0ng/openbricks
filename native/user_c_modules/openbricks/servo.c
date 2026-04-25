// SPDX-License-Identifier: MIT
//
// openbricks — servo.c
//
// Native per-motor state machine. Subscribes its control tick to the
// motor_process fast C path, so at 1 kHz the per-motor overhead is the
// C math (~1 µs) plus a handful of cached Python method invocations on
// the H-bridge pins and the encoder object (~20 µs total).
//
// The Python JGB37Motor wrapper in openbricks/drivers/jgb37_520.py is a
// thin forwarding layer over this type. Public API matches the Python
// equivalent pinned by tests/_fakes.py::_Servo.
//
// Control law for M1 is the same P-with-feedforward used in the previous
// Python implementation. Observer + trajectory land in M2; the
// structure here (a single ``_control_step`` tick body registered with
// the scheduler) is identical to what pbio does in servo.c and is where
// those land when ported.

#include <stdbool.h>

#include "py/runtime.h"
#include "py/mphal.h"

#include "motor_process.h"
#include "servo.h"

#define DUTY_MAX    1023               // 10-bit PWM (ESP32 default; matches L298N driver)
#define POWER_CLAMP ((mp_float_t)100.0)
#define RATED_DPS   ((mp_float_t)300.0) // feed-forward normaliser; tune per gearbox
#define DEFAULT_ACCEL ((mp_float_t)720.0)  // deg/s^2 if run_target omits accel

extern const mp_obj_type_t openbricks_servo_type;

// -----------------------------------------------------------------------
// Hardware primitives — called from the C tick body.

static inline mp_int_t servo_read_count(servo_obj_t *self) {
    // Call encoder.count() — cheap compared to mp_load_attr every tick,
    // and lets encoders (like the PCNT-backed one) compute the total
    // dynamically from hardware instead of shadowing it in an attribute.
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

// Drive the H-bridge to ``power`` in the range [-100, 100]. The servo's
// ``invert`` flag swaps direction, for motors wired the opposite way.
static void servo_drive_power(servo_obj_t *self, mp_float_t power) {
    if (power > POWER_CLAMP) {
        power = POWER_CLAMP;
    } else if (power < -POWER_CLAMP) {
        power = -POWER_CLAMP;
    }
    mp_float_t effective = self->invert ? -power : power;

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
// Control tick — the native hot path. Registered with motor_process's
// C callback list; called at the configured tick rate (1 kHz default).

static void servo_control_tick(void *ctx) {
    servo_obj_t *self = (servo_obj_t *)ctx;

    // If we're tracking a trajectory, sample it to get the current
    // velocity setpoint before the P-control loop runs.
    if (self->traj_active) {
        mp_float_t elapsed_s = (mp_float_t)(openbricks_motor_process_now_ms() - self->traj_start_ms) / (mp_float_t)1000.0;
        if (elapsed_s >= self->trajectory.core.t_total) {
            self->target_dps = 0.0;
            self->traj_done  = true;
        } else {
            mp_float_t pos, vel;
            openbricks_trajectory_sample(&self->trajectory, elapsed_s, &pos, &vel);
            self->target_dps = vel;
        }
    }

    // Read encoder as position (deg) and feed the observer.
    mp_int_t count = servo_read_count(self);
    mp_float_t measured_pos = (mp_float_t)count * 360.0 / (mp_float_t)self->counts_per_rev;

    mp_int_t now   = openbricks_motor_process_now_ms();
    mp_float_t dt_s = (mp_float_t)(now - self->last_time_ms) / (mp_float_t)1000.0;
    self->last_time_ms = now;

    if (dt_s > 0.0) {
        openbricks_observer_update(&self->observer, measured_pos, dt_s);
    }
    mp_float_t measured_dps = (mp_float_t)self->observer.core.vel_hat;

    mp_float_t error = self->target_dps - measured_dps;
    mp_float_t ff    = self->target_dps / RATED_DPS * POWER_CLAMP;
    mp_float_t power = ff + self->kp * error;

    servo_drive_power(self, power);
}

// -----------------------------------------------------------------------
// Attach / detach from motor_process

static void servo_attach(servo_obj_t *self) {
    if (self->active) {
        return;
    }
    // Re-baseline the observer to the current shaft angle, with zero
    // velocity — otherwise the first tick sees a huge residual because
    // last-time-ms is stale from whenever the servo was idle.
    mp_int_t count = servo_read_count(self);
    mp_float_t pos = (mp_float_t)count * 360.0 / (mp_float_t)self->counts_per_rev;
    openbricks_observer_reset(&self->observer, pos);
    self->last_time_ms = openbricks_motor_process_now_ms();
    openbricks_motor_process_register_c(servo_control_tick, self);
    self->active = true;
}

static void servo_detach(servo_obj_t *self) {
    if (!self->active) {
        return;
    }
    openbricks_motor_process_unregister_c(servo_control_tick, self);
    self->active      = false;
    self->target_dps  = 0.0;
    self->traj_active = false;
    self->traj_done   = true;
}

// -----------------------------------------------------------------------
// Python-facing methods

static mp_obj_t servo_run_speed(mp_obj_t self_in, mp_obj_t dps_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->target_dps  = mp_obj_get_float(dps_in);
    self->traj_active = false;
    self->traj_done   = true;
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

    // The trajectory operates on positions, not deltas — build it from
    // the current shaft angle.
    mp_int_t count = servo_read_count(self);
    mp_float_t start = (mp_float_t)count * 360.0 / (mp_float_t)self->counts_per_rev;
    mp_float_t target = start + delta_deg;

    openbricks_trajectory_init(&self->trajectory, start, target, cruise_dps, accel);
    self->traj_start_ms = openbricks_motor_process_now_ms();
    self->traj_active   = true;
    self->traj_done     = false;
    self->target_dps    = 0.0;   // first tick will sample the profile
    servo_attach(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(servo_run_target_obj, 3, 4, servo_run_target);

static mp_obj_t servo_is_done(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->traj_done);
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_is_done_obj, servo_is_done);

static mp_obj_t servo_target_dps(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_float(self->target_dps);
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_target_dps_obj, servo_target_dps);

static mp_obj_t servo_is_active(mp_obj_t self_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->active);
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_is_active_obj, servo_is_active);

static mp_obj_t servo_run(mp_obj_t self_in, mp_obj_t power_in) {
    servo_obj_t *self = MP_OBJ_TO_PTR(self_in);
    servo_detach(self);
    servo_drive_power(self, mp_obj_get_float(power_in));
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
    mp_int_t count = servo_read_count(self);
    mp_float_t angle = (mp_float_t)count * 360.0 / (mp_float_t)self->counts_per_rev;
    return mp_obj_new_float(angle);
}
static MP_DEFINE_CONST_FUN_OBJ_1(servo_angle_obj, servo_angle);

static mp_obj_t servo_reset_angle(size_t n_args, const mp_obj_t *args) {
    servo_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_float_t angle = (n_args > 1) ? mp_obj_get_float(args[1]) : 0.0;
    mp_int_t new_count = (mp_int_t)(angle * (mp_float_t)self->counts_per_rev / 360.0);
    // encoder.reset(new_count) — uniform across all encoder drivers,
    // including ones that back ``_count`` with a hardware peripheral.
    mp_call_function_1(self->encoder_reset, MP_OBJ_NEW_SMALL_INT(new_count));
    // Re-baseline the observer and the time stamp, so the first tick
    // after a reset_angle() doesn't see a huge phantom delta.
    openbricks_observer_reset(&self->observer, angle);
    self->last_time_ms = openbricks_motor_process_now_ms();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(servo_reset_angle_obj, 1, 2, servo_reset_angle);

// run_angle is intentionally omitted from C — it's a blocking loop that's
// easier to express in Python (sleep_ms / break on angle). The Python
// wrapper provides it, calling run_speed + brake via this servo.

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
    self->in1_pin        = parsed[ARG_in1].u_obj;
    self->in2_pin        = parsed[ARG_in2].u_obj;
    self->pwm            = parsed[ARG_pwm].u_obj;
    self->encoder        = parsed[ARG_encoder].u_obj;
    self->counts_per_rev = parsed[ARG_counts_per_rev].u_int;
    self->invert         = parsed[ARG_invert].u_bool;
    self->kp             = (parsed[ARG_kp].u_obj != MP_OBJ_NULL)
                         ? mp_obj_get_float(parsed[ARG_kp].u_obj)
                         : 0.3;

    // Cache bound methods so the tick body does pure calls.
    self->in1_value     = mp_load_attr(self->in1_pin, MP_QSTR_value);
    self->in2_value     = mp_load_attr(self->in2_pin, MP_QSTR_value);
    self->pwm_duty      = mp_load_attr(self->pwm,     MP_QSTR_duty);
    self->encoder_count = mp_load_attr(self->encoder, MP_QSTR_count);
    self->encoder_reset = mp_load_attr(self->encoder, MP_QSTR_reset);

    self->target_dps   = 0.0;
    self->active       = false;
    self->traj_active  = false;
    self->traj_done    = true;
    self->traj_start_ms = 0;
    self->last_time_ms = (mp_int_t)openbricks_motor_process_now_ms();

    // Default observer tuning — caller can use a standalone Observer
    // type later if they want different gains per motor.
    openbricks_observer_init(&self->observer, 0.5, 0.15);

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
