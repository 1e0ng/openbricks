# SPDX-License-Identifier: MIT
# Runs on every boot, before any user boot.py / main.py.
#
# Triggers openbricks/__init__.py's _auto_boot, which detects the board,
# constructs the matching hub, restores the persisted BLE state, and
# paints the onboard LED. With this in place the LED reflects BLE
# state even when the user writes an empty main.py (or no main.py at
# all — the REPL also gets the correct colour).
#
# Wrapped in try/except so a breakage in openbricks can't soft-brick
# the REPL: users can still drop into REPL, diagnose, and repair.

try:
    import openbricks  # noqa: F401
except Exception as _e:
    # Print for diagnostics but keep the boot chain going.
    try:
        print("openbricks_boot: import failed:", _e)
    except Exception:
        pass
