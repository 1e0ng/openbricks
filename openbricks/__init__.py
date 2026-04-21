# SPDX-License-Identifier: MIT
"""openbricks — Pybricks-style robotics for open hardware on MicroPython."""

__version__ = "0.7.0"

# Re-export the most useful things for ergonomic imports.
from openbricks.interfaces import Motor, Servo, IMU, ColorSensor  # noqa: F401


# ---- BLE toggle auto-boot ----
#
# On firmware, importing any part of the ``openbricks`` package is enough
# to get the BLE toggle + LED feedback running — users don't have to
# import ``openbricks.hub`` specifically. We detect the board via
# ``sys.implementation._machine`` (which MicroPython fills from the
# MICROPY_HW_MCU_NAME macro our board configs set) and construct the
# matching hub, whose ``__init__`` does the rest.
#
# Silent no-op off-firmware: tests running under unix MicroPython hit
# the ``except Exception`` below. That's intentional — we don't want
# ``import openbricks.anything`` to crash when bluetooth/esp32 modules
# are unavailable.
#
# Exposed as ``openbricks.board`` for users who want to reach into the
# auto-constructed hub (its ``.led`` / ``.button`` / ``.bluetooth_toggle``).


board = None


def _auto_boot():
    """Construct the matching board hub if we recognise the platform."""
    global board
    try:
        import sys
        mach = getattr(sys.implementation, "_machine", "") or ""
        if "ESP32-S3" in mach or "ESP32S3" in mach:
            from openbricks.hub import ESP32S3DevkitHub as _HubCls
        elif "ESP32" in mach:
            from openbricks.hub import ESP32DevkitHub as _HubCls
        else:
            return
        board = _HubCls()
    except Exception:
        # Any failure (no BLE module, no machine.Pin, test mode without
        # the relevant fake, etc.) silently disables auto-boot — callers
        # can still construct a hub manually.
        pass


_auto_boot()
del _auto_boot
