// SPDX-License-Identifier: MIT
//
// Module registration for the ``_openbricks_native`` MicroPython module.
// Pulls together the types defined in sibling files (motor_process.c,
// servo.c, …) into a single user_c_module.
//
// The Python re-export layer at ``openbricks/_native.py`` imports from
// this module and presents a tidier public API to user code.

#include "py/runtime.h"

#include "motor_process.h"

// Types defined in sibling files.
extern const mp_obj_type_t openbricks_motor_process_type;
extern const mp_obj_type_t openbricks_servo_type;
extern const mp_obj_type_t openbricks_trajectory_type;
extern const mp_obj_type_t openbricks_observer_type;
extern const mp_obj_type_t openbricks_drivebase_type;
extern const mp_obj_type_t openbricks_pcnt_encoder_type;
extern const mp_obj_type_t openbricks_encoder_type;
extern const mp_obj_type_t openbricks_bno055_type;

// Inject a KeyboardInterrupt into the MAIN thread — the same mechanism
// the REPL uses for a host Ctrl-C. This sets the VM's pending exception,
// which is checked between *every* bytecode regardless of scheduler
// state, so it interrupts whatever is running in the main thread.
//
// Why this exists: the launcher's program-button STOP path used to do
// ``micropython.schedule(func)`` where ``func`` raised KeyboardInterrupt.
// Bench-confirmed that does NOT stop the program — a scheduled callback
// that raises only unwinds the callback itself (printing a traceback);
// the running program keeps going. And the scheduled-callback queue is
// gated by a lock while another scheduled callback runs, so it can't
// even fire while a button-started program executes inside
// ``_scheduled_start``. The pending-exception path has neither problem.
static mp_obj_t openbricks_request_keyboard_interrupt(void) {
    mp_sched_keyboard_interrupt();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(openbricks_request_keyboard_interrupt_obj,
                                 openbricks_request_keyboard_interrupt);

static const mp_rom_map_elem_t openbricks_native_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),           MP_ROM_QSTR(MP_QSTR__openbricks_native) },
    { MP_ROM_QSTR(MP_QSTR_request_keyboard_interrupt),
      MP_ROM_PTR(&openbricks_request_keyboard_interrupt_obj) },
    { MP_ROM_QSTR(MP_QSTR_motor_process),      MP_ROM_PTR(&motor_process_singleton) },
    { MP_ROM_QSTR(MP_QSTR_Servo),              MP_ROM_PTR(&openbricks_servo_type) },
    { MP_ROM_QSTR(MP_QSTR_TrapezoidalProfile), MP_ROM_PTR(&openbricks_trajectory_type) },
    { MP_ROM_QSTR(MP_QSTR_Observer),           MP_ROM_PTR(&openbricks_observer_type) },
    { MP_ROM_QSTR(MP_QSTR_DriveBase),          MP_ROM_PTR(&openbricks_drivebase_type) },
    { MP_ROM_QSTR(MP_QSTR_PCNTEncoder),        MP_ROM_PTR(&openbricks_pcnt_encoder_type) },
    { MP_ROM_QSTR(MP_QSTR_QuadratureEncoder),  MP_ROM_PTR(&openbricks_encoder_type) },
    { MP_ROM_QSTR(MP_QSTR_BNO055),             MP_ROM_PTR(&openbricks_bno055_type) },
};
static MP_DEFINE_CONST_DICT(openbricks_native_globals, openbricks_native_globals_table);

const mp_obj_module_t openbricks_native_cmodule = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&openbricks_native_globals,
};

MP_REGISTER_MODULE(MP_QSTR__openbricks_native, openbricks_native_cmodule);
