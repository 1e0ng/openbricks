#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Assert the two per-package ``VERSION`` files match their
``__init__.py`` counterparts. Run from CI to catch drift before a
release goes out with a stale version string.

* Root ``VERSION`` ⇄ ``openbricks/__init__.py::__version__`` (firmware)
* ``tools/openbricks-dev/VERSION`` ⇄ ``openbricks_dev/__init__.py`` (CLI)

If anything drifts:

    scripts/bump-version.py --firmware <version>
    scripts/bump-version.py --openbricks-dev <version>

resynchronises whichever side is off.
"""

import re
import sys
from pathlib import Path


_INIT_VERSION_RE = re.compile(r'^__version__\s*=\s*"([^"]*)"$', re.M)

_PAIRS = [
    ("firmware",       "VERSION",                         "openbricks/__init__.py"),
    ("openbricks-dev", "tools/openbricks-dev/VERSION",    "tools/openbricks-dev/openbricks_dev/__init__.py"),
]


def _read_init_version(path):
    m = _INIT_VERSION_RE.search(path.read_text())
    if not m:
        raise RuntimeError("no ``__version__`` in {}".format(path))
    return m.group(1)


def main():
    root = Path(__file__).resolve().parent.parent
    failures = []
    for label, version_path, init_path in _PAIRS:
        canonical = (root / version_path).read_text().strip()
        actual    = _read_init_version(root / init_path)
        if actual != canonical:
            failures.append(
                "  {}: {} is {!r}, {} says {!r}".format(
                    label, version_path, canonical, init_path, actual))
        else:
            print("{}: OK ({})".format(label, canonical))
    if failures:
        print("\nversion drift detected:", file=sys.stderr)
        for f in failures:
            print(f, file=sys.stderr)
        print("\nrun: scripts/bump-version.py --firmware <v> --openbricks-dev <v>",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
