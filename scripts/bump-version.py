#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Bump the project version.

One command, one edit, every ``__version__`` constant stays in sync:

    scripts/bump-version.py 1.0.0

Writes the new value to ``VERSION`` (the canonical source) and
regenerates the ``__version__ = "..."`` line in both packages'
``__init__.py``:

* ``openbricks/__init__.py``            — frozen into the firmware
* ``openbricks_dev/__init__.py``        — exposed by the host CLI

The ``VERSION`` file is what ``scripts/check-version.py`` compares
against in CI, so drift between any of the three locations fails the
build before a release can go out with a stale version string.
"""

import re
import sys
from pathlib import Path


_VERSION_RE     = re.compile(r"^\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?$")
_INIT_LINE_RE   = re.compile(r'^__version__\s*=\s*"[^"]*"$', re.M)

_INIT_FILES = (
    "openbricks/__init__.py",
    "tools/openbricks-dev/openbricks_dev/__init__.py",
)


def _update_init(path, new_version):
    text = path.read_text()
    new_text, n = _INIT_LINE_RE.subn(
        '__version__ = "{}"'.format(new_version), text, count=1)
    if n == 0:
        raise RuntimeError(
            "no ``__version__ = \"...\"`` line in {}".format(path))
    if new_text != text:
        path.write_text(new_text)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("usage: scripts/bump-version.py X.Y.Z", file=sys.stderr)
        return 2
    new_version = argv[0]
    if not _VERSION_RE.match(new_version):
        print("error: invalid version {!r} (expected X.Y.Z)".format(new_version),
              file=sys.stderr)
        return 2

    root = Path(__file__).resolve().parent.parent
    (root / "VERSION").write_text(new_version + "\n")
    for rel in _INIT_FILES:
        _update_init(root / rel, new_version)

    print("bumped to {}".format(new_version))
    return 0


if __name__ == "__main__":
    sys.exit(main())
