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
    PCNTEncoder,
    QuadratureEncoder,
    BNO055,
)

# The serial bus is firmware-only: the sim's _openbricks_native
# provides virtual motors — there is no wire — and deliberately omits
# st_bus (first push of this re-export broke the whole host suite by
# assuming it). Consumers select the native-bus path via attribute
# presence (hasattr), so absence here is a meaningful signal, not an
# error to paper over.
try:
    from _openbricks_native import st_bus  # noqa: F401
except ImportError:
    pass

# Same attribute-presence contract for the raw-IMU heading source.
try:
    from _openbricks_native import icm45686  # noqa: F401
except ImportError:
    pass
