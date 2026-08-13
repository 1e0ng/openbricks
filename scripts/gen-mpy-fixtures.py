#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Regenerate the checked-in ``.mpy`` test fixtures.

The fixtures under ``tests/fixtures/`` are compiled with the SAME
pinned mpy-cross the CLI depends on (tools/openbricks/pyproject.toml),
so the unix-MicroPython test job exercises exactly the bytes a user's
``openbricks run`` would stage. Rerun this after bumping either the
mpy-cross pin or the MicroPython submodule:

    python3 -m pip install "mpy-cross == <the pyproject pin>"
    python3 scripts/gen-mpy-fixtures.py

A stale fixture fails loudly — the firmware loader raises
``ValueError: incompatible .mpy file`` in tests — so drift cannot
pass silently.
"""

import os
import subprocess
import sys

FIXTURES = os.path.join(os.path.dirname(__file__), "..", "tests", "fixtures")


def main():
    import mpy_cross
    for name in sorted(os.listdir(FIXTURES)):
        if not name.endswith(".py"):
            continue
        src = os.path.join(FIXTURES, name)
        out = os.path.join(FIXTURES, name[:-3] + ".mpy")
        subprocess.run(
            [mpy_cross.mpy_cross, "-o", out, "-s", name, src],
            check=True)
        print("compiled", name, "->", os.path.basename(out),
              os.path.getsize(out), "bytes")


if __name__ == "__main__":
    sys.exit(main())
