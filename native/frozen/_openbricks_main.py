# SPDX-License-Identifier: MIT
"""
Default openbricks app, frozen into the firmware image.

Imported (and immediately executed) by frozen ``boot.py`` on every
boot. Renamed from ``main.py`` to ``_openbricks_main.py`` in 1.2.3
so upstream ``ports/esp32/main.c``'s post-boot
``pyexec_file_if_exists("main.py")`` doesn't find a frozen module
to re-run after our app exits — see the long note in
``native/frozen/boot.py`` for why that re-run was harmful.

Pybricks-style workflow:

* BLE + NUS REPL up first so the openbricks-dev tool can always reach
  the hub (via ``bluetooth.apply_persisted_state()``).
* If BLE didn't activate (e.g. fresh chip with no hub name in NVS),
  drop straight to the REPL — entering ``launcher.run()`` would
  block ``mpremote`` from writing the hub name during
  ``openbricks flash``, leaving the chip permanently un-named.
* Platform hub constructed to wire the BLE-toggle **short-press**
  button on GPIO 5 (flashes the LED blue/yellow for on/off feedback
  on RGB-capable hubs).
* The program-button watcher on GPIO 4 (short-press = run/stop) is
  started via ``launcher.run()``, which also blocks here forever.

Defensive structure (since 1.0.5): the entire body is wrapped in a
``try`` / ``except``. If any step crashes — bad NVS state, BLE init
failure on a particular chip — the traceback prints and the module
returns. MicroPython then drops the user into the REPL, where they
can fix things instead of seeing a silent boot. This matters because
the ESP32-S3 USB-Serial-JTAG transport can lose output during the
first second after reset (the host hasn't fully connected yet); a
raised exception during BLE init at top of file produced no visible
traceback in 1.0.0 → 1.0.3 and looked like the chip was bricked.

If you want a different boot sequence — e.g. a calibration routine
that runs unconditionally before the launcher, or BLE-off-by-default —
write your own ``/main.py`` via ``openbricks upload --path /main.py``
or ``mpremote cp``. Frozen ``boot.py`` prefers a VFS ``/main.py``
when present.
"""

import sys


def _main():
    from openbricks import bluetooth, ble_repl

    # BLE + NUS REPL up first so ``openbricks run`` / ``upload`` /
    # ``stop`` are always reachable, even if the rest of main fails.
    # ``apply_persisted_state`` is itself defensive about a missing
    # hub name — a freshly-flashed chip just skips BLE with a one-line
    # warning instead of raising.
    bluetooth.apply_persisted_state()

    # Fresh-chip path: no hub name in NVS → BLE not activated. Don't
    # enter the blocking launcher loop — the next step in the
    # ``openbricks flash`` flow is ``mpremote ... exec`` to write the
    # hub name, and that needs the chip's REPL reachable. Returning
    # here drops us into the friendly REPL.
    if not ble_repl.is_running():
        print("openbricks: BLE inactive — dropping to REPL for setup.")
        return

    # Try to wire the platform hub so the BLE-toggle long-press works.
    # Missing pins (wrong board) or a crashed import shouldn't brick
    # boot; fall through to the launcher if anything goes wrong.
    try:
        from openbricks.hub import ESP32S3DevkitHub as _Hub
        _hub = _Hub()
    except Exception:
        try:
            from openbricks.hub import ESP32DevkitHub as _Hub
            _hub = _Hub()
        except Exception:
            _hub = None

    # Block forever. Short-press the hub button to run /program.py
    # (written by ``openbricks upload``); second short-press interrupts.
    from openbricks import launcher
    launcher.run()


try:
    _main()
except Exception as _exc:
    # Print the traceback to stdout so it reaches whichever console
    # the user has connected (UART0 or USB-Serial-JTAG, both wired
    # by mphalport). Then return — MicroPython drops to the REPL,
    # which is far more useful than a silent boot.
    sys.print_exception(_exc)
    print("openbricks: app failed; dropping to REPL.")
