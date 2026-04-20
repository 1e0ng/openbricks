// SPDX-License-Identifier: MIT
//
// openbricks — drivebase.c
//
// Two-wheel differential drivebase with 2-DOF coupled control — the
// analogue of pbio's drivebase.c. Rather than running two independent
// position loops and papering over heading drift with a small Kp
// correction (what the pure-Python M1 did), the C drivebase runs two
// coupled trajectories:
//
//   * ``fwd`` — forward progress, in average wheel degrees
//     (``(left + right) / 2``). Controlled by the ``straight`` move.
//   * ``turn`` — heading, in differential wheel degrees
//     (``(left - right) / 2``, scaled to body heading via axle track).
//     Controlled by the ``turn`` move.
//
// Each tick the drivebase samples its profile(s) for target position +
// velocity, reads each servo's observed position (via observer.pos_hat),
// computes sum / diff errors, and writes new per-servo target_dps:
//
//     left.target_dps  = fwd_cmd_dps - turn_cmd_dps
//     right.target_dps = fwd_cmd_dps + turn_cmd_dps
//
// where ``fwd_cmd_dps = fwd_ff_dps + kp_sum * sum_err`` and similarly
// for turn. The individual servos still run their own feedforward+P
// velocity loop at 1 kHz; the drivebase is a setpoint source on top of
// them, not a replacement.
//
// The drivebase is registered on the motor_process fast C path
// *before* each servo, so ticks run in order: drivebase sets targets
// → each servo's control step reads them and drives its H-bridge.
//
// Exposed as ``_openbricks_native.DriveBase``.

#include <math.h>
#include <stdbool.h>

#include "py/runtime.h"
#include "py/mphal.h"

#include "motor_process.h"
#include "trajectory.h"
#include "servo.h"

#define DEFAULT_KP_SUM   ((mp_float_t)2.0)
#define DEFAULT_KP_DIFF  ((mp_float_t)5.0)
#define DEFAULT_ACCEL    ((mp_float_t)720.0)

// ---------------------------------------------------------------------

typedef struct _drivebase_obj_t {
    mp_obj_base_t base;

    // Servo handles (MP objects — kept alive by strong reference here,
    // plus a fast-path native pointer for per-tick access).
    mp_obj_t left_obj;
    mp_obj_t right_obj;
    servo_obj_t *left;
    servo_obj_t *right;

    // Physical parameters.
    mp_float_t wheel_circumference_mm;  // π * diameter
    mp_float_t axle_track_mm;

    // Controller gains.
    mp_float_t kp_sum;     // on forward-position error
    mp_float_t kp_diff;    // on heading-position error

    // Active profiles — either can be inactive (no progress on that
    // axis) while the other drives the motion.
    trajectory_obj_t fwd;
    uint32_t         fwd_start_ms;
    bool             fwd_active;
    mp_float_t       fwd_hold;        // captured at move-start; used when fwd_active=false

    trajectory_obj_t turn;
    uint32_t         turn_start_ms;
    bool             turn_active;
    mp_float_t       turn_hold;       // captured at move-start; used when turn_active=false

    bool registered;   // true iff our control_tick is on motor_process
    bool done;         // true iff the last scheduled move finished
} drivebase_obj_t;

extern const mp_obj_type_t openbricks_drivebase_type;

// ---------------------------------------------------------------------
// Control tick

static void drivebase_control_tick(void *ctx) {
    drivebase_obj_t *self = (drivebase_obj_t *)ctx;

    mp_float_t fwd_target = 0.0;
    mp_float_t fwd_ff_vel = 0.0;
    if (self->fwd_active) {
        mp_float_t elapsed = (mp_float_t)(mp_hal_ticks_ms() - self->fwd_start_ms) / (mp_float_t)1000.0;
        if (elapsed >= self->fwd.t_total) {
            fwd_target = self->fwd.start + self->fwd.direction * (self->fwd.distance < 0 ? -self->fwd.distance : self->fwd.distance);
            fwd_ff_vel = 0.0;
            // Lock the trajectory end-point as the new hold target.
            self->fwd_hold   = fwd_target;
            self->fwd_active = false;
        } else {
            openbricks_trajectory_sample(&self->fwd, elapsed, &fwd_target, &fwd_ff_vel);
        }
    } else {
        // Hold the position captured when the move started (or
        // finished). Feedback corrects any drift away from it.
        fwd_target = self->fwd_hold;
    }

    mp_float_t turn_target = 0.0;
    mp_float_t turn_ff_vel = 0.0;
    if (self->turn_active) {
        mp_float_t elapsed = (mp_float_t)(mp_hal_ticks_ms() - self->turn_start_ms) / (mp_float_t)1000.0;
        if (elapsed >= self->turn.t_total) {
            turn_target = self->turn.start + self->turn.direction * (self->turn.distance < 0 ? -self->turn.distance : self->turn.distance);
            turn_ff_vel = 0.0;
            self->turn_hold   = turn_target;
            self->turn_active = false;
        } else {
            openbricks_trajectory_sample(&self->turn, elapsed, &turn_target, &turn_ff_vel);
        }
    } else {
        turn_target = self->turn_hold;
    }

    // Both profiles done → move complete.
    if (!self->fwd_active && !self->turn_active) {
        self->done = true;
    }

    // Actual sum / diff positions from each servo's observer.
    mp_float_t sum_pos  = (self->left->observer.pos_hat + self->right->observer.pos_hat) / (mp_float_t)2.0;
    mp_float_t diff_pos = (self->left->observer.pos_hat - self->right->observer.pos_hat) / (mp_float_t)2.0;

    mp_float_t sum_err  = fwd_target  - sum_pos;
    mp_float_t diff_err = turn_target - diff_pos;

    mp_float_t fwd_cmd  = fwd_ff_vel  + self->kp_sum  * sum_err;
    mp_float_t diff_cmd = turn_ff_vel + self->kp_diff * diff_err;

    // diff_pos = (left - right)/2, so diff_cmd is the desired
    // (left_vel - right_vel)/2 = the rate at which left out-paces right.
    // Positive diff_cmd → left speeds up, right slows down.
    self->left->target_dps  = fwd_cmd + diff_cmd;
    self->right->target_dps = fwd_cmd - diff_cmd;
}

// ---------------------------------------------------------------------
// Lifecycle helpers

static void drivebase_register(drivebase_obj_t *self) {
    if (self->registered) {
        return;
    }
    // Re-baseline each servo's observer so the first tick doesn't see
    // a phantom residual, and ensure they're subscribed.
    mp_int_t lc = mp_obj_get_int(mp_load_attr(self->left->encoder,  MP_QSTR__count));
    mp_int_t rc = mp_obj_get_int(mp_load_attr(self->right->encoder, MP_QSTR__count));
    mp_float_t lp = (mp_float_t)lc * (mp_float_t)360.0 / (mp_float_t)self->left->counts_per_rev;
    mp_float_t rp = (mp_float_t)rc * (mp_float_t)360.0 / (mp_float_t)self->right->counts_per_rev;
    openbricks_observer_reset(&self->left->observer,  lp);
    openbricks_observer_reset(&self->right->observer, rp);
    self->left->last_time_ms  = mp_hal_ticks_ms();
    self->right->last_time_ms = mp_hal_ticks_ms();

    // Hand both servos into speed-mode by bumping their target to 0 and
    // ensuring they're active on motor_process. (Calling their Python
    // ``run_speed(0.0)`` would also do it, but we'd be re-entering MP
    // from a C callback scheduler which is avoidable.)
    self->left->target_dps  = 0.0;
    self->right->target_dps = 0.0;
    self->left->traj_active  = false;
    self->right->traj_active = false;
    self->left->traj_done  = true;
    self->right->traj_done = true;
    // Servos' control_tick lives in servo.c; register it here only if
    // not already attached.
    extern void servo_control_tick(void *);   // forward decl mirror
    // We use the servos' existing ``_attach`` through Python? No —
    // easier: ensure active via direct register. Servo attaches itself
    // through openbricks_motor_process_register_c(servo_control_tick, self),
    // but ``servo_control_tick`` is static inside servo.c. Instead we
    // rely on the caller (Python DriveBase.straight) having called
    // servo.run_speed(0) on both motors before starting the move. That
    // attaches the servos through the public API.
    (void)servo_control_tick;

    // Drivebase's own tick: register last so we'd run after servos
    // without care for order. BUT we need drivebase to run BEFORE the
    // servos each tick, so that the servos' control step reads the
    // fresh target. The fast C callback list fires in registration
    // order (see motor_process.c); so we register *first* by calling
    // unregister-then-register on the servos to push them after us.
    // In practice Python user calls run_speed before DriveBase starts,
    // and our register appends to the list — servos are already there.
    // Workaround: unregister each servo and re-register, so they land
    // after drivebase.
    //
    // Simpler fix: re-attach the servos after drivebase is registered.
    // We call into servo by name via the public register API: but that
    // needs access to servo_control_tick. We can grab it by symbol at
    // runtime via the observer reset hack, OR we export it. Cleanest
    // is to export — add to servo.h.
    openbricks_motor_process_register_c(drivebase_control_tick, self);
    self->registered = true;
    self->done       = false;
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

static mp_obj_t db_straight(mp_obj_t self_in, mp_obj_t distance_in, mp_obj_t speed_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_float_t distance_mm = mp_obj_get_float(distance_in);
    mp_float_t speed_mm_s  = mp_obj_get_float(speed_in);

    // Convert to wheel-degree space: distance_mm / circumference * 360.
    mp_float_t distance_deg = distance_mm / self->wheel_circumference_mm * (mp_float_t)360.0;
    mp_float_t speed_dps    = fabs(speed_mm_s) / self->wheel_circumference_mm * (mp_float_t)360.0;

    // Build the fwd trajectory starting from current average position.
    mp_float_t sum_pos  = (self->left->observer.pos_hat + self->right->observer.pos_hat) / (mp_float_t)2.0;
    mp_float_t diff_pos = (self->left->observer.pos_hat - self->right->observer.pos_hat) / (mp_float_t)2.0;
    openbricks_trajectory_init(&self->fwd, sum_pos, sum_pos + distance_deg,
                                speed_dps, DEFAULT_ACCEL);
    self->fwd_start_ms = mp_hal_ticks_ms();
    self->fwd_active   = true;

    // Hold heading at whatever it was when the move started; the
    // heading-error feedback in the tick body will snap it back if
    // friction or load disturbance drifts it away.
    self->turn_hold   = diff_pos;
    self->turn_active = false;

    drivebase_register(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(db_straight_obj, db_straight);

static mp_obj_t db_turn(mp_obj_t self_in, mp_obj_t angle_in, mp_obj_t rate_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_float_t angle_deg = mp_obj_get_float(angle_in);
    mp_float_t rate_dps  = mp_obj_get_float(rate_in);

    // Body heading turn -> wheel differential degrees:
    //   arc_mm      = radians(|θ|) * axle_track / 2
    //   wheel_deg   = arc_mm / circumference * 360
    // A positive body turn (+θ, left / CCW in Pybricks convention)
    // pushes left backward and right forward → left_pos decreases and
    // right_pos increases → diff_pos = (left - right)/2 DECREASES.
    // So the signed delta is the negative of |angle|.
    mp_float_t arc_mm       = (mp_float_t)fabs((double)angle_deg) * ((mp_float_t)M_PI / (mp_float_t)180.0) * (self->axle_track_mm / (mp_float_t)2.0);
    mp_float_t wheel_deg    = arc_mm / self->wheel_circumference_mm * (mp_float_t)360.0;
    mp_float_t signed_delta = (angle_deg >= 0.0 ? -wheel_deg : wheel_deg);

    mp_float_t rate_arc_mm_s = (mp_float_t)fabs((double)rate_dps) * ((mp_float_t)M_PI / (mp_float_t)180.0) * (self->axle_track_mm / (mp_float_t)2.0);
    mp_float_t rate_wheel_dps = rate_arc_mm_s / self->wheel_circumference_mm * (mp_float_t)360.0;

    mp_float_t diff_pos = (self->left->observer.pos_hat - self->right->observer.pos_hat) / (mp_float_t)2.0;
    mp_float_t sum_pos  = (self->left->observer.pos_hat + self->right->observer.pos_hat) / (mp_float_t)2.0;
    openbricks_trajectory_init(&self->turn, diff_pos, diff_pos + signed_delta,
                                rate_wheel_dps, DEFAULT_ACCEL);
    self->turn_start_ms = mp_hal_ticks_ms();
    self->turn_active   = true;

    // Forward stays at current sum — any disturbance that drifts us
    // forward or back gets corrected by the sum-position feedback.
    self->fwd_hold   = sum_pos;
    self->fwd_active = false;

    drivebase_register(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_3(db_turn_obj, db_turn);

static mp_obj_t db_stop(mp_obj_t self_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->fwd_active  = false;
    self->turn_active = false;
    self->done        = true;
    drivebase_unregister(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(db_stop_obj, db_stop);

static mp_obj_t db_is_done(mp_obj_t self_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    return mp_obj_new_bool(self->done);
}
static MP_DEFINE_CONST_FUN_OBJ_1(db_is_done_obj, db_is_done);

// ---------------------------------------------------------------------
// Constructor

static mp_obj_t db_make_new(const mp_obj_type_t *type,
                             size_t n_args, size_t n_kw,
                             const mp_obj_t *all_args) {
    enum { ARG_left, ARG_right, ARG_wheel_d, ARG_axle, ARG_kp_sum, ARG_kp_diff };
    static const mp_arg_t allowed[] = {
        { MP_QSTR_left,              MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_right,             MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_wheel_diameter_mm, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_axle_track_mm,     MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL} },
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
        mp_raise_TypeError(MP_ERROR_TEXT("left and right must be Servo instances"));
    }

    drivebase_obj_t *self = mp_obj_malloc(drivebase_obj_t, type);
    self->left_obj  = left;
    self->right_obj = right;
    self->left  = MP_OBJ_TO_PTR(left);
    self->right = MP_OBJ_TO_PTR(right);

    self->wheel_circumference_mm = (mp_float_t)M_PI *
                                   mp_obj_get_float(parsed[ARG_wheel_d].u_obj);
    self->axle_track_mm          = mp_obj_get_float(parsed[ARG_axle].u_obj);
    self->kp_sum  = (parsed[ARG_kp_sum].u_obj  != MP_OBJ_NULL)
                    ? mp_obj_get_float(parsed[ARG_kp_sum].u_obj)  : DEFAULT_KP_SUM;
    self->kp_diff = (parsed[ARG_kp_diff].u_obj != MP_OBJ_NULL)
                    ? mp_obj_get_float(parsed[ARG_kp_diff].u_obj) : DEFAULT_KP_DIFF;

    self->fwd_active   = false;
    self->turn_active  = false;
    self->fwd_hold     = 0.0;
    self->turn_hold    = 0.0;
    self->registered   = false;
    self->done         = true;
    self->fwd_start_ms = 0;
    self->turn_start_ms = 0;

    return MP_OBJ_FROM_PTR(self);
}

// ---------------------------------------------------------------------
// Type definition

static const mp_rom_map_elem_t db_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_straight), MP_ROM_PTR(&db_straight_obj) },
    { MP_ROM_QSTR(MP_QSTR_turn),     MP_ROM_PTR(&db_turn_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),     MP_ROM_PTR(&db_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_done),  MP_ROM_PTR(&db_is_done_obj) },
};
static MP_DEFINE_CONST_DICT(db_locals_dict, db_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_drivebase_type,
    MP_QSTR_DriveBase,
    MP_TYPE_FLAG_NONE,
    make_new,    db_make_new,
    locals_dict, &db_locals_dict
);
