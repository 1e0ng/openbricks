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

static const mp_rom_map_elem_t openbricks_native_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),           MP_ROM_QSTR(MP_QSTR__openbricks_native) },
    { MP_ROM_QSTR(MP_QSTR_motor_process),      MP_ROM_PTR(&motor_process_singleton) },
    { MP_ROM_QSTR(MP_QSTR_Servo),              MP_ROM_PTR(&openbricks_servo_type) },
    { MP_ROM_QSTR(MP_QSTR_TrapezoidalProfile), MP_ROM_PTR(&openbricks_trajectory_type) },
};
static MP_DEFINE_CONST_DICT(openbricks_native_globals, openbricks_native_globals_table);

const mp_obj_module_t openbricks_native_cmodule = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&openbricks_native_globals,
};

MP_REGISTER_MODULE(MP_QSTR__openbricks_native, openbricks_native_cmodule);
