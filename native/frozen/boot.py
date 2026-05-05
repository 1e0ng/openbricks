# SPDX-License-Identifier: MIT
"""
Frozen ``boot.py`` — runs the openbricks default app unconditionally.

Why this file exists: upstream ``ports/esp32/main.c`` only invokes
``pyexec_file_if_exists("main.py")`` when ``pyexec_mode_kind ==
PYEXEC_MODE_FRIENDLY_REPL``. The global is preserved across
``soft_reset`` (``mp_init`` does not reset it) and ``mpremote``'s
exec/connect cycles can leave it stuck in ``PYEXEC_MODE_RAW_REPL``.
End result: after any ``mpremote ... exec ...`` call, the next soft
reset comes up to a bare REPL with our app silently skipped — BLE
never advertises, the launcher button-watcher never runs.

``boot.py`` runs through the *unconditional* path
(``pyexec_file_if_exists("boot.py")``, no mode gate), so importing
our app from here brings up BLE + launcher regardless of how the
chip was reset.

Why the app file is renamed (``_openbricks_main.py`` rather than
``main.py``): if our app exited cleanly (e.g. host Ctrl-C through a
live BLE REPL interrupting ``launcher.run``), upstream ``main.c``
would then run ``pyexec_file_if_exists("main.py")`` itself and
re-execute the same frozen module — re-init BLE, re-grab Timer 0,
print "openbricks: idle" again. By naming the frozen module
``_openbricks_main`` it doesn't match upstream's filename lookup,
so after our app returns control falls cleanly through to the
friendly REPL.

User override: a ``/main.py`` uploaded to the VFS still wins —
``boot.py`` checks for it first and runs that instead of the
default. (And upstream's post-boot ``main.py`` lookup also finds
the VFS file, but by then our boot.py path has already started it.)
"""

import sys


def _run_user_main():
    """If the user uploaded ``/main.py`` to VFS, run it and return True.
    Returns False when no user file is present."""
    try:
        f = open("main.py")
    except OSError:
        return False
    try:
        code = f.read()
    finally:
        f.close()
    try:
        exec(compile(code, "main.py", "exec"), {"__name__": "__main__", "__file__": "/main.py"})
    except KeyboardInterrupt:
        pass
    except Exception as e:
        sys.print_exception(e)
        print("openbricks: /main.py failed; dropping to REPL.")
    return True


def _run_default():
    try:
        import _openbricks_main  # frozen — see native/frozen/_openbricks_main.py
    except KeyboardInterrupt:
        pass
    # Other exceptions are caught inside _openbricks_main itself.


if not _run_user_main():
    _run_default()
