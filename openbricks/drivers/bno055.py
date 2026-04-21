# SPDX-License-Identifier: MIT
"""
BNO055 — re-export of the native ``BNO055`` C type.

The implementation lives in ``native/user_c_modules/openbricks/bno055.c``
so the drivebase tick can read ``imu.heading()`` every ms (when
``use_gyro=True``) without a Python frame on the hot path. This module
exists so existing user imports stay the same:

    from openbricks.drivers.bno055 import BNO055
"""

from openbricks._native import BNO055  # noqa: F401
