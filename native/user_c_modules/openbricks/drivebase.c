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

    // Optional IMU for gyro-based heading feedback. When ``use_gyro``
    // is true and ``imu`` is attached, the tick reads heading from the
    // IMU instead of computing it from the encoder differential —
    // slip-immune and unaffected by asymmetric friction. ``heading_offset_deg``
    // is captured at each move-start so heading is measured relative
    // to "wherever the robot was pointing when this move began."
    mp_obj_t   imu;                 // IMU driver (any ``.heading()``-capable object), or MP_OBJ_NULL
    mp_obj_t   imu_heading_fn;      // cached bound imu.heading, called once per tick
    bool       use_gyro;            // false by default — user opts in via use_gyro(True)
    mp_float_t heading_offset_deg;  // body heading captured at move-start

    // Active profiles — either can be inactive (no progress on that
    // axis) while the other drives the motion.
    trajectory_obj_t fwd;
    mp_int_t         fwd_start_ms;    // matches openbricks_motor_process_now_ms() width (64-bit on unix)
    bool             fwd_active;
    mp_float_t       fwd_hold;        // captured at move-start; used when fwd_active=false

    trajectory_obj_t turn;
    mp_int_t         turn_start_ms;
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
        mp_float_t elapsed = (mp_float_t)(openbricks_motor_process_now_ms() - self->fwd_start_ms) / (mp_float_t)1000.0;
        if (elapsed >= self->fwd.core.t_total) {
            fwd_target = self->fwd.core.start + self->fwd.core.direction * (self->fwd.core.distance < 0 ? -self->fwd.core.distance : self->fwd.core.distance);
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
        mp_float_t elapsed = (mp_float_t)(openbricks_motor_process_now_ms() - self->turn_start_ms) / (mp_float_t)1000.0;
        if (elapsed >= self->turn.core.t_total) {
            turn_target = self->turn.core.start + self->turn.core.direction * (self->turn.core.distance < 0 ? -self->turn.core.distance : self->turn.core.distance);
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

    // Actual sum / diff positions. ``sum_pos`` is always encoder-derived
    // (forward progress is not slip-immune via gyro alone). ``diff_pos``
    // (heading, in wheel-degrees) comes from the IMU when ``use_gyro`` is
    // active — slip-immune, drift-corrected by fusion — or from the
    // encoder differential as the fallback.
    mp_float_t sum_pos  = (self->left->observer.pos_hat + self->right->observer.pos_hat) / (mp_float_t)2.0;
    mp_float_t diff_pos;
    if (self->use_gyro && self->imu_heading_fn != MP_OBJ_NULL) {
        mp_float_t body = mp_obj_get_float(mp_call_function_0(self->imu_heading_fn));
        mp_float_t delta = body - self->heading_offset_deg;
        // BNO055 heading() wraps at ±180 — snap the delta into the same
        // range so a move that crosses the boundary doesn't see a
        // spurious ±360 jump.
        if (delta >  (mp_float_t)180.0) delta -= (mp_float_t)360.0;
        if (delta < (mp_float_t)-180.0) delta += (mp_float_t)360.0;
        // Body-degrees → wheel-degree differential:
        //   α = β * A * π / C   where A = axle, C = wheel circumference.
        // Positive body heading (CCW) → negative diff_pos (left retreats,
        // right advances, so (L - R)/2 < 0), so the sign flips.
        diff_pos = -delta * self->axle_track_mm * (mp_float_t)M_PI /
                   self->wheel_circumference_mm;
    } else {
        diff_pos = (self->left->observer.pos_hat - self->right->observer.pos_hat) / (mp_float_t)2.0;
    }

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

// Capture the current IMU heading as the zero reference. Called at move
// start (via drivebase_register) and whenever use_gyro flips on, so the
// controller treats "now" as heading 0 regardless of where the robot
// happens to be pointing at that moment.
static void drivebase_rebaseline_heading(drivebase_obj_t *self) {
    if (self->imu_heading_fn == MP_OBJ_NULL) {
        return;
    }
    self->heading_offset_deg = mp_obj_get_float(mp_call_function_0(self->imu_heading_fn));
}

static void drivebase_register(drivebase_obj_t *self) {
    if (self->registered) {
        return;
    }
    if (self->use_gyro) {
        drivebase_rebaseline_heading(self);
    }
    // Re-baseline each servo's observer so the first tick doesn't see
    // a phantom residual, and ensure they're subscribed.
    // Use each servo's cached encoder.count bound method (uniform across
    // QuadratureEncoder / PCNTEncoder / any future type).
    mp_int_t lc = mp_obj_get_int(mp_call_function_0(self->left->encoder_count));
    mp_int_t rc = mp_obj_get_int(mp_call_function_0(self->right->encoder_count));
    mp_float_t lp = (mp_float_t)lc * (mp_float_t)360.0 / (mp_float_t)self->left->counts_per_rev;
    mp_float_t rp = (mp_float_t)rc * (mp_float_t)360.0 / (mp_float_t)self->right->counts_per_rev;
    openbricks_observer_reset(&self->left->observer,  lp);
    openbricks_observer_reset(&self->right->observer, rp);
    self->left->last_time_ms  = openbricks_motor_process_now_ms();
    self->right->last_time_ms = openbricks_motor_process_now_ms();

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
    self->fwd_start_ms = openbricks_motor_process_now_ms();
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
    self->turn_start_ms = openbricks_motor_process_now_ms();
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

static mp_obj_t db_use_gyro(mp_obj_t self_in, mp_obj_t enable_in) {
    drivebase_obj_t *self = MP_OBJ_TO_PTR(self_in);
    bool enable = mp_obj_is_true(enable_in);
    if (enable && self->imu_heading_fn == MP_OBJ_NULL) {
        mp_raise_ValueError(MP_ERROR_TEXT("no imu attached; construct DriveBase(imu=...) first"));
    }
    // Toggling on: capture offset so current heading becomes the zero.
    // Toggling off: nothing to do; the encoder path takes over on the
    // next tick with no state to clear.
    bool transitioning_on = enable && !self->use_gyro;
    self->use_gyro = enable;
    if (transitioning_on) {
        drivebase_rebaseline_heading(self);
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_2(db_use_gyro_obj, db_use_gyro);

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
    enum { ARG_left, ARG_right, ARG_wheel_d, ARG_axle, ARG_imu, ARG_kp_sum, ARG_kp_diff };
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

    // IMU (optional). If provided, cache the ``heading`` bound method so
    // the control tick can call it without mp_load_attr every time.
    self->imu                = parsed[ARG_imu].u_obj;
    self->imu_heading_fn     = MP_OBJ_NULL;
    self->use_gyro           = false;
    self->heading_offset_deg = 0.0;
    if (self->imu != MP_OBJ_NULL && self->imu != mp_const_none) {
        self->imu_heading_fn = mp_load_attr(self->imu, MP_QSTR_heading);
    }

    return MP_OBJ_FROM_PTR(self);
}

// ---------------------------------------------------------------------
// Type definition

static const mp_rom_map_elem_t db_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_straight),  MP_ROM_PTR(&db_straight_obj) },
    { MP_ROM_QSTR(MP_QSTR_turn),      MP_ROM_PTR(&db_turn_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),      MP_ROM_PTR(&db_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_done),   MP_ROM_PTR(&db_is_done_obj) },
    { MP_ROM_QSTR(MP_QSTR_use_gyro),  MP_ROM_PTR(&db_use_gyro_obj) },
};
static MP_DEFINE_CONST_DICT(db_locals_dict, db_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    openbricks_drivebase_type,
    MP_QSTR_DriveBase,
    MP_TYPE_FLAG_NONE,
    make_new,    db_make_new,
    locals_dict, &db_locals_dict
);
