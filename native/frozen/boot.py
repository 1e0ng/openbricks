# SPDX-License-Identifier: MIT
"""
Frozen ``boot.py`` that runs our default ``main`` module unconditionally.

Why this file exists: upstream ``ports/esp32/main.c`` only invokes
``pyexec_file_if_exists("main.py")`` when ``pyexec_mode_kind ==
PYEXEC_MODE_FRIENDLY_REPL`` — the global is preserved across
``soft_reset`` (``mp_init`` does not reset it) and ``mpremote``'s
exec/connect cycles can leave it stuck in ``PYEXEC_MODE_RAW_REPL``.
End result: after any ``mpremote ... exec ...`` call, the next soft
reset comes up to the bare REPL with our ``main.py`` silently
skipped. BLE never advertises, the launcher button-watcher never
runs, ``openbricks run`` can't reach the hub.

``boot.py`` runs through the *unconditional* path
(``pyexec_file_if_exists("boot.py")`` — no mode gate), so importing
our ``main`` module from here brings up BLE + launcher regardless of
how the chip was reset. Once ``main._main()`` enters
``launcher.run()`` (a ``while True:`` loop), control never returns
to upstream ``main.c``, so the trailing ``pyexec_file_if_exists(
"main.py")`` call below it is never reached and we don't double-run.

If ``launcher.run`` is interrupted (e.g. host Ctrl-C through a live
BLE REPL), the import returns, ``boot.py`` exits, and upstream's
``main.c`` drops to the friendly REPL — which is what the user
wants in that situation.
"""

try:
    import main
except KeyboardInterrupt:
    # User interrupted launcher.run() — fall through to REPL instead
    # of re-executing main via upstream main.c's main.py path.
    pass
