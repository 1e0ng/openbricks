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
    return m;
}
