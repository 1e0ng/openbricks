/* SPDX-License-Identifier: MIT
 *
 * CPython bindings for the shared openbricks numerical cores.
 *
 * Why this exists: the firmware's ``_openbricks_native`` MicroPython
 * module and this ``openbricks_sim._native`` CPython extension wrap
 * the same algorithm source files (``native/user_c_modules/openbricks/
 * <name>_core.c``). Building the same bytes into both targets means
 * the sim's control loop is literally identical to the firmware's —
 * no drift.
 *
 * Phase B1 scope: only ``TrapezoidalProfile``. Subsequent phases add
 * Observer, Servo, DriveBase, MotorProcess as their cores are
 * extracted.
 */

#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "trajectory_core.h"
#include "observer_core.h"
#include "motor_process_core.h"
#include "servo_core.h"
#include "drivebase_core.h"


/* -------------------------------------------------------------------
 * TrapezoidalProfile
 * ------------------------------------------------------------------- */

typedef struct {
    PyObject_HEAD
    ob_trajectory_t core;
} TrajectoryObject;


static int Trajectory_init(TrajectoryObject *self, PyObject *args, PyObject *kwargs) {
    static char *kwlist[] = {"start", "target", "cruise_dps", "accel_dps2", NULL};
    double start, target, cruise, accel;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "dddd", kwlist,
                                     &start, &target, &cruise, &accel)) {
        return -1;
    }
    ob_trajectory_init(&self->core,
                       (ob_float_t)start,
                       (ob_float_t)target,
                       (ob_float_t)cruise,
                       (ob_float_t)accel);
    return 0;
}


static PyObject *Trajectory_sample(TrajectoryObject *self, PyObject *arg) {
    double t_s_py = PyFloat_AsDouble(arg);
    if (t_s_py == -1.0 && PyErr_Occurred()) {
        return NULL;
    }
    ob_float_t pos, vel;
    ob_trajectory_sample(&self->core, (ob_float_t)t_s_py, &pos, &vel);
    return Py_BuildValue("(dd)", (double)pos, (double)vel);
}


static PyObject *Trajectory_duration(TrajectoryObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble((double)self->core.t_total);
}


static PyObject *Trajectory_is_triangular(TrajectoryObject *self, PyObject *Py_UNUSED(ignored)) {
    if (self->core.triangular) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}


static PyMethodDef Trajectory_methods[] = {
    {"sample",        (PyCFunction)Trajectory_sample,        METH_O,
     "Sample the profile at time t (seconds); returns (pos, vel)."},
    {"duration",      (PyCFunction)Trajectory_duration,      METH_NOARGS,
     "Total move duration in seconds."},
    {"is_triangular", (PyCFunction)Trajectory_is_triangular, METH_NOARGS,
     "True if the profile never reaches cruise speed."},
    {NULL, NULL, 0, NULL},
};


static PyTypeObject TrajectoryType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name      = "openbricks_sim._native.TrapezoidalProfile",
    .tp_basicsize = sizeof(TrajectoryObject),
    .tp_itemsize  = 0,
    .tp_flags     = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc       = PyDoc_STR(
        "Trapezoidal speed profile — same algorithm as the firmware's "
        "``_openbricks_native.TrapezoidalProfile``. Constructed with "
        "start, target, cruise_dps, accel_dps2; sample at any t in "
        "[0, duration()]."),
    .tp_new       = PyType_GenericNew,
    .tp_init      = (initproc)Trajectory_init,
    .tp_methods   = Trajectory_methods,
};


/* -------------------------------------------------------------------
 * Observer (α-β position/velocity smoother)
 * ------------------------------------------------------------------- */

typedef struct {
    PyObject_HEAD
    ob_observer_t core;
} ObserverObject;


static int Observer_init(ObserverObject *self, PyObject *args, PyObject *kwargs) {
    static char *kwlist[] = {"alpha", "beta", NULL};
    double alpha = 0.5;
    double beta  = 0.15;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|dd", kwlist,
                                     &alpha, &beta)) {
        return -1;
    }
    ob_observer_init(&self->core, (ob_float_t)alpha, (ob_float_t)beta);
    return 0;
}


static PyObject *Observer_update(ObserverObject *self, PyObject *args) {
    double pos, dt;
    if (!PyArg_ParseTuple(args, "dd", &pos, &dt)) {
        return NULL;
    }
    ob_observer_update(&self->core, (ob_float_t)pos, (ob_float_t)dt);
    return Py_BuildValue("(dd)",
                         (double)self->core.pos_hat,
                         (double)self->core.vel_hat);
}


static PyObject *Observer_reset(ObserverObject *self, PyObject *args) {
    double pos = 0.0;
    if (!PyArg_ParseTuple(args, "|d", &pos)) {
        return NULL;
    }
    ob_observer_reset(&self->core, (ob_float_t)pos);
    Py_RETURN_NONE;
}


static PyObject *Observer_position(ObserverObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble((double)self->core.pos_hat);
}


static PyObject *Observer_velocity(ObserverObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble((double)self->core.vel_hat);
}


static PyMethodDef Observer_methods[] = {
    {"update",   (PyCFunction)Observer_update,   METH_VARARGS,
     "update(measured_pos, dt) -> (pos_hat, vel_hat). Step the observer one tick."},
    {"reset",    (PyCFunction)Observer_reset,    METH_VARARGS,
     "reset(pos=0.0). Re-anchor the position estimate; zero the velocity."},
    {"position", (PyCFunction)Observer_position, METH_NOARGS,
     "Estimated position."},
    {"velocity", (PyCFunction)Observer_velocity, METH_NOARGS,
     "Estimated velocity."},
    {NULL, NULL, 0, NULL},
};


static PyTypeObject ObserverType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name      = "openbricks_sim._native.Observer",
    .tp_basicsize = sizeof(ObserverObject),
    .tp_itemsize  = 0,
    .tp_flags     = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc       = PyDoc_STR(
        "Two-state α-β position/velocity observer — same algorithm as "
        "the firmware's ``_openbricks_native.Observer``. Construct with "
        "alpha (default 0.5) and beta (default 0.15); call ``update`` "
        "each tick with the latest measured position and the time step."),
    .tp_new       = PyType_GenericNew,
    .tp_init      = (initproc)Observer_init,
    .tp_methods   = Observer_methods,
};


/* -------------------------------------------------------------------
 * MotorProcess (C-callback registry + tick clock)
 *
 * The sim runner uses this to drive the same servo / drivebase tick
 * functions firmware does, except triggered from MuJoCo's step loop
 * instead of a hardware Timer ISR. Python callback dispatch (the
 * firmware's "slow path") isn't exposed here — sim user code runs
 * directly inside the step loop, no need for a separate Python-
 * callable list.
 * ------------------------------------------------------------------- */

typedef struct {
    PyObject_HEAD
    ob_motor_process_t core;
} MotorProcessObject;


static int MotorProcess_init(MotorProcessObject *self, PyObject *args, PyObject *kwargs) {
    static char *kwlist[] = {"period_ms", NULL};
    int period_ms = 1;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|i", kwlist, &period_ms)) {
        return -1;
    }
    ob_motor_process_init(&self->core);
    ob_motor_process_set_period_ms(&self->core, period_ms);
    return 0;
}


static PyObject *MotorProcess_tick(MotorProcessObject *self, PyObject *Py_UNUSED(ignored)) {
    /* Fire all registered C callbacks once + advance the tick clock.
     *
     * Python-side users who want their own callbacks fired as part
     * of a tick simply call them themselves alongside this method —
     * there's no equivalent of the firmware's Python-callable list
     * here because the sim's "tick driver" is already pure Python
     * (the MuJoCo step loop). */
    ob_motor_process_fire_c(&self->core);
    Py_RETURN_NONE;
}


static PyObject *MotorProcess_now_ms(MotorProcessObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyLong_FromLong((long)self->core.virtual_now_ms);
}


static PyObject *MotorProcess_period_ms(MotorProcessObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyLong_FromLong((long)self->core.period_ms);
}


static PyObject *MotorProcess_set_period_ms(MotorProcessObject *self, PyObject *arg) {
    long period_ms = PyLong_AsLong(arg);
    if (period_ms == -1 && PyErr_Occurred()) {
        return NULL;
    }
    ob_motor_process_set_period_ms(&self->core, (int)period_ms);
    Py_RETURN_NONE;
}


static PyObject *MotorProcess_count_c(MotorProcessObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyLong_FromSize_t(ob_motor_process_count_c(&self->core));
}


static PyObject *MotorProcess_reset(MotorProcessObject *self, PyObject *Py_UNUSED(ignored)) {
    ob_motor_process_reset(&self->core);
    Py_RETURN_NONE;
}


static PyMethodDef MotorProcess_methods[] = {
    {"tick",          (PyCFunction)MotorProcess_tick,          METH_NOARGS,
     "Fire every registered C callback once and advance the tick clock by period_ms."},
    {"now_ms",        (PyCFunction)MotorProcess_now_ms,        METH_NOARGS,
     "Tick-driven monotonic clock in milliseconds."},
    {"period_ms",     (PyCFunction)MotorProcess_period_ms,     METH_NOARGS,
     "Current tick period in milliseconds."},
    {"set_period_ms", (PyCFunction)MotorProcess_set_period_ms, METH_O,
     "Set the tick period in milliseconds."},
    {"count_c",       (PyCFunction)MotorProcess_count_c,       METH_NOARGS,
     "Number of C callbacks currently registered."},
    {"reset",         (PyCFunction)MotorProcess_reset,         METH_NOARGS,
     "Clear callbacks + zero the clock + reset period to default 1 ms."},
    {NULL, NULL, 0, NULL},
};


static PyTypeObject MotorProcessType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name      = "openbricks_sim._native.MotorProcess",
    .tp_basicsize = sizeof(MotorProcessObject),
    .tp_itemsize  = 0,
    .tp_flags     = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc       = PyDoc_STR(
        "Tick scheduler — same C-callback registry + virtual-clock "
        "behaviour as the firmware's ``_openbricks_native.motor_process``. "
        "Callable from the sim runner's MuJoCo step loop. The firmware's "
        "Python-callable list isn't exposed here — sim ticks run inside "
        "Python already, so callers call their own functions directly."),
    .tp_new       = PyType_GenericNew,
    .tp_init      = (initproc)MotorProcess_init,
    .tp_methods   = MotorProcess_methods,
};


/* -------------------------------------------------------------------
 * Servo (control state machine + observer + trajectory composition)
 *
 * I/O is the caller's responsibility — sim binds its read-encoder /
 * write-motor against MuJoCo joint sensors + actuators. The class
 * here is pure state machine: ``tick(count, now_ms) -> power`` runs
 * the same control law as the firmware's ``servo.c``.
 * ------------------------------------------------------------------- */

typedef struct {
    PyObject_HEAD
    ob_servo_t core;
} ServoObject;


static int Servo_init(ServoObject *self, PyObject *args, PyObject *kwargs) {
    static char *kwlist[] = {"counts_per_rev", "kp", "invert", NULL};
    int    counts_per_rev = 1320;
    double kp             = OB_SERVO_DEFAULT_KP;
    int    invert         = 0;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "|idp", kwlist,
                                     &counts_per_rev, &kp, &invert)) {
        return -1;
    }
    ob_servo_init(&self->core, counts_per_rev, (ob_float_t)kp, invert ? true : false);
    return 0;
}


static PyObject *Servo_tick(ServoObject *self, PyObject *args) {
    long count, now_ms;
    if (!PyArg_ParseTuple(args, "ll", &count, &now_ms)) {
        return NULL;
    }
    double power = (double)ob_servo_tick(&self->core, count, now_ms);
    return PyFloat_FromDouble(power);
}


static PyObject *Servo_set_speed(ServoObject *self, PyObject *arg) {
    double dps = PyFloat_AsDouble(arg);
    if (dps == -1.0 && PyErr_Occurred()) {
        return NULL;
    }
    ob_servo_set_speed(&self->core, (ob_float_t)dps);
    Py_RETURN_NONE;
}


static PyObject *Servo_run_target(ServoObject *self, PyObject *args) {
    long   count, now_ms;
    double delta_deg, cruise_dps, accel;
    if (!PyArg_ParseTuple(args, "llddd",
                          &count, &now_ms,
                          &delta_deg, &cruise_dps, &accel)) {
        return NULL;
    }
    ob_servo_run_target(&self->core, count, now_ms,
                        (ob_float_t)delta_deg,
                        (ob_float_t)cruise_dps,
                        (ob_float_t)accel);
    Py_RETURN_NONE;
}


static PyObject *Servo_baseline(ServoObject *self, PyObject *args) {
    long count, now_ms;
    if (!PyArg_ParseTuple(args, "ll", &count, &now_ms)) {
        return NULL;
    }
    ob_servo_baseline(&self->core, count, now_ms);
    Py_RETURN_NONE;
}


static PyObject *Servo_is_done(ServoObject *self, PyObject *Py_UNUSED(ignored)) {
    if (ob_servo_is_done(&self->core)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}


static PyObject *Servo_target_dps(ServoObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble((double)self->core.target_dps);
}


static PyObject *Servo_observed_dps(ServoObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble((double)self->core.observer.vel_hat);
}


static PyObject *Servo_observed_pos(ServoObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble((double)self->core.observer.pos_hat);
}


static PyObject *Servo_count_to_angle(ServoObject *self, PyObject *arg) {
    long count = PyLong_AsLong(arg);
    if (count == -1 && PyErr_Occurred()) {
        return NULL;
    }
    return PyFloat_FromDouble((double)ob_servo_count_to_angle_deg(&self->core, count));
}


static PyMethodDef Servo_methods[] = {
    {"tick",            (PyCFunction)Servo_tick,            METH_VARARGS,
     "tick(count, now_ms) -> power. One control step; returns desired power in [-100, 100]."},
    {"set_speed",       (PyCFunction)Servo_set_speed,       METH_O,
     "Set a constant velocity target (deg/s); cancels any active trajectory."},
    {"run_target",      (PyCFunction)Servo_run_target,      METH_VARARGS,
     "run_target(count, now_ms, delta_deg, cruise_dps, accel). Kick off a trapezoidal move."},
    {"baseline",        (PyCFunction)Servo_baseline,        METH_VARARGS,
     "baseline(count, now_ms). Re-anchor observer + time baseline (call on attach)."},
    {"is_done",         (PyCFunction)Servo_is_done,         METH_NOARGS,
     "True if no trajectory active or the active one has completed."},
    {"target_dps",      (PyCFunction)Servo_target_dps,      METH_NOARGS,
     "Current velocity setpoint."},
    {"observed_dps",    (PyCFunction)Servo_observed_dps,    METH_NOARGS,
     "Observer's velocity estimate."},
    {"observed_pos",    (PyCFunction)Servo_observed_pos,    METH_NOARGS,
     "Observer's position estimate (degrees)."},
    {"count_to_angle",  (PyCFunction)Servo_count_to_angle,  METH_O,
     "Convert a raw encoder count to degrees using counts_per_rev."},
    {NULL, NULL, 0, NULL},
};


static PyTypeObject ServoType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name      = "openbricks_sim._native.Servo",
    .tp_basicsize = sizeof(ServoObject),
    .tp_itemsize  = 0,
    .tp_flags     = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc       = PyDoc_STR(
        "Servo control state machine — same control law as the "
        "firmware's ``_openbricks_native.Servo``. I/O is the caller's "
        "responsibility (read encoder, call ``tick``, write the "
        "returned power to your motor). The sim runner binds these "
        "calls against MuJoCo joint sensors + actuators."),
    .tp_new       = PyType_GenericNew,
    .tp_init      = (initproc)Servo_init,
    .tp_methods   = Servo_methods,
};


/* -------------------------------------------------------------------
 * DriveBase (2-DOF coupled controller — fwd + turn trajectories,
 * sum/diff PID with feedforward, mixed into per-servo target_dps).
 *
 * Construction takes two ``Servo`` instances by reference; the
 * DriveBase keeps strong refs so they outlive it. The sim runner
 * registers ``DriveBase.tick`` on its motor_process *before* the
 * per-servo ticks so each tick fans out drivebase → servos.
 * Heading override is a single-method push: when ``use_gyro`` is on,
 * the sim runner reads the MuJoCo IMU body-yaw delta and stuffs the
 * wheel-degree differential into the core via ``set_heading_override``.
 * ------------------------------------------------------------------- */

typedef struct {
    PyObject_HEAD
    ob_drivebase_t  core;
    PyObject       *left_obj;   /* strong ref to Servo */
    PyObject       *right_obj;  /* strong ref to Servo */
} DriveBaseObject;


static int DriveBase_init(DriveBaseObject *self, PyObject *args, PyObject *kwargs) {
    static char *kwlist[] = {"left", "right",
                             "wheel_diameter_mm", "axle_track_mm",
                             "kp_sum", "kp_diff", NULL};
    PyObject *left_obj  = NULL;
    PyObject *right_obj = NULL;
    double    wheel_d;
    double    axle;
    double    kp_sum  = OB_DRIVEBASE_DEFAULT_KP_SUM;
    double    kp_diff = OB_DRIVEBASE_DEFAULT_KP_DIFF;
    if (!PyArg_ParseTupleAndKeywords(args, kwargs, "OOdd|dd", kwlist,
                                     &left_obj, &right_obj,
                                     &wheel_d, &axle,
                                     &kp_sum, &kp_diff)) {
        return -1;
    }
    if (!PyObject_TypeCheck(left_obj,  &ServoType) ||
        !PyObject_TypeCheck(right_obj, &ServoType)) {
        PyErr_SetString(PyExc_TypeError,
                        "left and right must be Servo instances");
        return -1;
    }

    Py_INCREF(left_obj);
    Py_INCREF(right_obj);
    Py_XSETREF(self->left_obj,  left_obj);
    Py_XSETREF(self->right_obj, right_obj);

    ob_drivebase_init(&self->core,
                      &((ServoObject *)left_obj)->core,
                      &((ServoObject *)right_obj)->core,
                      (ob_float_t)wheel_d,
                      (ob_float_t)axle,
                      (ob_float_t)kp_sum,
                      (ob_float_t)kp_diff);
    return 0;
}


static void DriveBase_dealloc(DriveBaseObject *self) {
    Py_CLEAR(self->left_obj);
    Py_CLEAR(self->right_obj);
    Py_TYPE(self)->tp_free((PyObject *)self);
}


static PyObject *DriveBase_straight(DriveBaseObject *self, PyObject *args) {
    long   now_ms;
    double distance_mm, speed_mm_s;
    if (!PyArg_ParseTuple(args, "ldd", &now_ms, &distance_mm, &speed_mm_s)) {
        return NULL;
    }
    ob_drivebase_straight(&self->core, now_ms,
                          (ob_float_t)distance_mm,
                          (ob_float_t)speed_mm_s);
    Py_RETURN_NONE;
}


static PyObject *DriveBase_turn(DriveBaseObject *self, PyObject *args) {
    long   now_ms;
    double angle_deg, rate_dps;
    if (!PyArg_ParseTuple(args, "ldd", &now_ms, &angle_deg, &rate_dps)) {
        return NULL;
    }
    ob_drivebase_turn(&self->core, now_ms,
                      (ob_float_t)angle_deg,
                      (ob_float_t)rate_dps);
    Py_RETURN_NONE;
}


static PyObject *DriveBase_stop(DriveBaseObject *self, PyObject *Py_UNUSED(ignored)) {
    ob_drivebase_stop(&self->core);
    Py_RETURN_NONE;
}


static PyObject *DriveBase_tick(DriveBaseObject *self, PyObject *arg) {
    long now_ms = PyLong_AsLong(arg);
    if (now_ms == -1 && PyErr_Occurred()) {
        return NULL;
    }
    ob_drivebase_tick(&self->core, now_ms);
    Py_RETURN_NONE;
}


static PyObject *DriveBase_is_done(DriveBaseObject *self, PyObject *Py_UNUSED(ignored)) {
    if (ob_drivebase_is_done(&self->core)) {
        Py_RETURN_TRUE;
    }
    Py_RETURN_FALSE;
}


static PyObject *DriveBase_set_use_gyro(DriveBaseObject *self, PyObject *arg) {
    int enable = PyObject_IsTrue(arg);
    if (enable < 0) {
        return NULL;
    }
    self->core.use_gyro = enable ? true : false;
    Py_RETURN_NONE;
}


/* set_heading_override(body_delta_deg) — converts via the core utility
 * so the binding doesn't need the axle/wheel constants on the sim side. */
static PyObject *DriveBase_set_heading_override(DriveBaseObject *self, PyObject *arg) {
    double body_delta = PyFloat_AsDouble(arg);
    if (body_delta == -1.0 && PyErr_Occurred()) {
        return NULL;
    }
    self->core.heading_override_wheel_deg =
        ob_drivebase_body_to_wheel_diff(&self->core, (ob_float_t)body_delta);
    Py_RETURN_NONE;
}


static PyObject *DriveBase_target_left_dps(DriveBaseObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble((double)self->core.left->target_dps);
}


static PyObject *DriveBase_target_right_dps(DriveBaseObject *self, PyObject *Py_UNUSED(ignored)) {
    return PyFloat_FromDouble((double)self->core.right->target_dps);
}


static PyMethodDef DriveBase_methods[] = {
    {"straight",             (PyCFunction)DriveBase_straight,             METH_VARARGS,
     "straight(now_ms, distance_mm, speed_mm_s). Kick off a straight move."},
    {"turn",                 (PyCFunction)DriveBase_turn,                 METH_VARARGS,
     "turn(now_ms, angle_deg, rate_dps). Kick off a turn-in-place."},
    {"stop",                 (PyCFunction)DriveBase_stop,                 METH_NOARGS,
     "Cancel any active move."},
    {"tick",                 (PyCFunction)DriveBase_tick,                 METH_O,
     "tick(now_ms). One control step — samples profiles, computes errors, writes per-servo target_dps."},
    {"is_done",              (PyCFunction)DriveBase_is_done,              METH_NOARGS,
     "True iff no move is active."},
    {"set_use_gyro",         (PyCFunction)DriveBase_set_use_gyro,         METH_O,
     "Toggle slip-immune heading feedback. Caller is responsible for "
     "calling ``set_heading_override`` before each tick when on."},
    {"set_heading_override", (PyCFunction)DriveBase_set_heading_override, METH_O,
     "Push the latest body-heading delta (degrees) — converted internally to wheel-degree differential."},
    {"target_left_dps",      (PyCFunction)DriveBase_target_left_dps,      METH_NOARGS,
     "Current per-tick velocity setpoint for the left servo."},
    {"target_right_dps",     (PyCFunction)DriveBase_target_right_dps,     METH_NOARGS,
     "Current per-tick velocity setpoint for the right servo."},
    {NULL, NULL, 0, NULL},
};


static PyTypeObject DriveBaseType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    .tp_name      = "openbricks_sim._native.DriveBase",
    .tp_basicsize = sizeof(DriveBaseObject),
    .tp_itemsize  = 0,
    .tp_flags     = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,
    .tp_doc       = PyDoc_STR(
        "Two-DOF coupled drivebase — same control law as the firmware's "
        "``_openbricks_native.DriveBase``. Construct with two Servo "
        "instances, wheel diameter (mm), axle track (mm), and optional "
        "kp_sum / kp_diff overrides. Sim runner calls ``tick(now_ms)`` "
        "in its motor_process step; per-servo ``target_dps`` is updated "
        "by each tick."),
    .tp_new       = PyType_GenericNew,
    .tp_init      = (initproc)DriveBase_init,
    .tp_dealloc   = (destructor)DriveBase_dealloc,
    .tp_methods   = DriveBase_methods,
};


/* -------------------------------------------------------------------
 * Module init
 * ------------------------------------------------------------------- */

static PyModuleDef openbricks_sim_native_module = {
    PyModuleDef_HEAD_INIT,
    .m_name = "openbricks_sim._native",
    .m_doc  = PyDoc_STR(
        "CPython bindings for the shared openbricks numerical cores. "
        "Identical algorithms to the firmware's MicroPython "
        "``_openbricks_native`` module — same C sources, different "
        "binding layer."),
    .m_size = -1,
};


PyMODINIT_FUNC PyInit__native(void) {
    if (PyType_Ready(&TrajectoryType) < 0) {
        return NULL;
    }
    PyObject *m = PyModule_Create(&openbricks_sim_native_module);
    if (m == NULL) {
        return NULL;
    }
    Py_INCREF(&TrajectoryType);
    if (PyModule_AddObject(m, "TrapezoidalProfile",
                           (PyObject *)&TrajectoryType) < 0) {
        Py_DECREF(&TrajectoryType);
        Py_DECREF(m);
        return NULL;
    }
    if (PyType_Ready(&ObserverType) < 0) {
        Py_DECREF(m);
        return NULL;
    }
    Py_INCREF(&ObserverType);
    if (PyModule_AddObject(m, "Observer",
                           (PyObject *)&ObserverType) < 0) {
        Py_DECREF(&ObserverType);
        Py_DECREF(m);
        return NULL;
    }
    if (PyType_Ready(&MotorProcessType) < 0) {
        Py_DECREF(m);
        return NULL;
    }
    Py_INCREF(&MotorProcessType);
    if (PyModule_AddObject(m, "MotorProcess",
                           (PyObject *)&MotorProcessType) < 0) {
        Py_DECREF(&MotorProcessType);
        Py_DECREF(m);
        return NULL;
    }
    if (PyType_Ready(&ServoType) < 0) {
        Py_DECREF(m);
        return NULL;
    }
    Py_INCREF(&ServoType);
    if (PyModule_AddObject(m, "Servo", (PyObject *)&ServoType) < 0) {
        Py_DECREF(&ServoType);
        Py_DECREF(m);
        return NULL;
    }
    if (PyType_Ready(&DriveBaseType) < 0) {
        Py_DECREF(m);
        return NULL;
    }
    Py_INCREF(&DriveBaseType);
    if (PyModule_AddObject(m, "DriveBase", (PyObject *)&DriveBaseType) < 0) {
        Py_DECREF(&DriveBaseType);
        Py_DECREF(m);
        return NULL;
    }
    return m;
}
