# SPDX-License-Identifier: MIT
"""
Default ``main.py`` frozen into the openbricks firmware image.

Runs on every boot unless the user has written their own ``/main.py``
into the VFS (which takes priority over the frozen copy). The default
wires the Pybricks-style workflow: bring BLE up, instantiate the
platform's hub for the BLE long-press toggle, and hand off to the
button-gated launcher so user code from ``openbricks-dev download``
only runs when the user actually presses the hub button.

If you want a different boot sequence — e.g. a calibration routine
that runs unconditionally before the launcher, or BLE-off-by-default —
write your own ``/main.py`` via ``mpremote cp`` or the REPL.
"""

from openbricks import bluetooth

# BLE + NUS REPL up first so ``openbricks-dev run`` / ``download`` /
# ``stop`` are always reachable, even if the rest of main.py crashes.
bluetooth.apply_persisted_state()

# Try to wire the platform hub so the BLE-toggle long-press works.
# Missing pins (wrong board) or a crashed import shouldn't brick boot,
# so we fall through to the launcher if anything goes wrong.
try:
    from openbricks.hub import ESP32S3DevkitHub as _Hub
    _hub = _Hub()
except Exception:
    try:
        from openbricks.hub import ESP32DevkitHub as _Hub
        _hub = _Hub()
    except Exception:
        _hub = None

# Block forever. Short-press the hub button to run /program.py (written
# by ``openbricks-dev download``); second short-press interrupts.
from openbricks import launcher
launcher.run()
