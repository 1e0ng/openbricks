// SPDX-License-Identifier: MIT
//
// ``exec_mpy(path, globals)`` — load a host-cross-compiled ``.mpy``
// file and execute its top level under the caller-supplied globals
// dict.
//
// The launcher's source path runs programs as
// ``exec(code, {"__name__": "__main__"})``. A ``.mpy`` cannot go
// through ``exec`` (it is persistent bytecode, not source), and the
// import machinery would run it with ``__name__ == "program"`` —
// silently breaking the ``if __name__ == "__main__":`` idiom. This
// binding keeps exact exec parity: the caller owns the globals dict,
// so ``__name__`` is whatever the launcher sets.
//
// Mirrors ``do_execute_proto_fun`` in py/builtinimport.c: save the
// current globals/locals, install the supplied dict, nlr-protect the
// restore so an exception inside the program (including the stop
// button's KeyboardInterrupt) unwinds with the REPL's context intact.
//
// Deliberately NO feature-flag guard: if a port config ever drops
// MICROPY_PERSISTENT_CODE_LOAD this must fail the build, not compile
// into a stub (see pcnt_encoder.c for how silent stubs burned us).

#include "py/persistentcode.h"
#include "py/runtime.h"

static mp_obj_t openbricks_exec_mpy(mp_obj_t path_in, mp_obj_t globals_in) {
    if (!mp_obj_is_type(globals_in, &mp_type_dict)) {
        mp_raise_TypeError(MP_ERROR_TEXT("globals must be a dict"));
    }
    // The context's constants table is filled in by the loader; the
    // module object only exists to carry the globals dict.
    mp_module_context_t *context = m_new_obj(mp_module_context_t);
    context->module.base.type = &mp_type_module;
    context->module.globals = MP_OBJ_TO_PTR(globals_in);

    // Raises OSError on a missing file and ValueError("incompatible
    // .mpy file") on a format-version mismatch — both surface to the
    // run log / console exactly like source-program errors.
    mp_compiled_module_t cm;
    cm.context = context;
    mp_raw_code_load_file(qstr_from_str(mp_obj_str_get_str(path_in)), &cm);

    nlr_jump_callback_node_globals_locals_t ctx;
    ctx.globals = mp_globals_get();
    ctx.locals = mp_locals_get();
    mp_globals_set(context->module.globals);
    mp_locals_set(context->module.globals);
    nlr_push_jump_callback(&ctx.callback,
                           mp_globals_locals_set_from_nlr_jump_callback);

    mp_obj_t module_fun = mp_make_function_from_proto_fun(cm.rc, context, NULL);
    mp_call_function_0(module_fun);

    nlr_pop_jump_callback(true);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(openbricks_exec_mpy_obj, openbricks_exec_mpy);
