# SPDX-License-Identifier: MIT
"""
Default ``main.py`` frozen into the openbricks firmware image.

Runs on every boot unless the user has written their own ``/main.py``
into the VFS (which takes priority over the frozen copy). The default
wires the Pybricks-style workflow:

* BLE + NUS REPL up first so the openbricks-dev tool can always reach
  the hub (via ``bluetooth.apply_persisted_state()``).
* Platform hub constructed to wire the BLE-toggle **short-press**
  button on GPIO 5 (flashes the LED blue/yellow for on/off feedback
  on RGB-capable hubs).
* The program-button watcher on GPIO 4 (short-press = run/stop) is
  started via ``launcher.run()``, which also blocks here forever.

If you want a different boot sequence — e.g. a calibration routine
that runs unconditionally before the launcher, or BLE-off-by-default —
write your own ``/main.py`` via ``openbricks-dev download --path /main.py``
or ``mpremote cp``.
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
