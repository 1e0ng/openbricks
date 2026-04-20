# SPDX-License-Identifier: MIT
"""
Public re-export of the ``_openbricks_native`` C module.

On firmware, ``_openbricks_native`` is a built-in module registered by
``native/user_c_modules/openbricks/motor_process.c``. On desktop
CPython, it's the Python fake installed by ``tests/_fakes.py``. Either
way, user code imports from here — the concrete backend is an
implementation detail.

    from openbricks._native import motor_process
"""

from _openbricks_native import (  # noqa: F401
    motor_process,
    Servo,
    TrapezoidalProfile,
    Observer,
    DriveBase,
)

