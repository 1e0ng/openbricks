# SPDX-License-Identifier: MIT
"""openbricks — Pybricks-style robotics for open hardware on MicroPython."""

__version__ = "1.2.2"

# Re-export the most useful things for ergonomic imports.
from openbricks.interfaces import Motor, Servo, IMU, ColorSensor  # noqa: F401


# ---- hub name ----
#
# A short string that identifies this specific hub — the same role as
# ``pybricks-dev run -n <name>``. Used as the BLE advertising name so
# future code-transfer tooling can target one hub on a bench full of
# them.
#
# Written **at flash time** (not build time) so a single firmware image
# is reused across every hub. See ``scripts/flash_firmware.sh``, which
# stashes the name in NVS under namespace ``openbricks``, key
# ``hub_name``. Reading from NVS on each access lets tests and future
# in-field rename flows see updates without a reboot.
#
# If the flash step was skipped (or the NVS partition was wiped),
# ``HUB_NAME`` is ``None``. Non-BLE code paths don't care. Activating
# BLE with ``HUB_NAME is None`` raises — we refuse to advertise under
# a shared default, because two hubs with the same name can't be
# individually addressed.

_HUB_NAME_NVS_NAMESPACE = "openbricks"
_HUB_NAME_NVS_KEY       = "hub_name"
_HUB_NAME_MAX_BYTES     = 32


def _read_hub_name():
    """Return the flashed hub name, or ``None`` if unset.

    Reads freshly from NVS on every call. ``esp32`` is imported lazily
    so desktop tests that don't install the ``_fakes_ble`` fakes don't
    blow up when importing ``openbricks`` for unrelated reasons.
    """
    try:
        import esp32
    except ImportError:
        return None
    try:
        nvs = esp32.NVS(_HUB_NAME_NVS_NAMESPACE)
        buf = bytearray(_HUB_NAME_MAX_BYTES)
        n = nvs.get_blob(_HUB_NAME_NVS_KEY, buf)
        return bytes(buf[:n]).decode()
    except OSError:
        return None


def __getattr__(name):
    # PEP 562: resolved on each attribute miss. CPython still falls
    # through to submodule auto-import when a parent package defines
    # __getattr__, but MicroPython does not — so a bare
    # ``from openbricks import bluetooth`` would fail here with
    # AttributeError. Handle submodule names explicitly by importing
    # them on demand; HUB_NAME stays freshly NVS-backed.
    if name == "HUB_NAME":
        return _read_hub_name()
    import sys
    fullname = "openbricks." + name
    mod = sys.modules.get(fullname)
    if mod is not None:
        return mod
    try:
        __import__(fullname)
    except ImportError:
        raise AttributeError("module 'openbricks' has no attribute %r" % name)
    return sys.modules[fullname]
