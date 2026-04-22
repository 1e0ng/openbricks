#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Assert ``VERSION`` matches the ``__version__`` constant in every
package ``__init__.py``. Run from CI to catch drift before a release
goes out with a stale version string.

Intentionally small — no deps. If the version strings drift:

    scripts/bump-version.py <VERSION-contents>

brings everything back in sync.
"""

import re
import sys
from pathlib import Path


_INIT_VERSION_RE = re.compile(r'^__version__\s*=\s*"([^"]*)"$', re.M)

_INIT_FILES = (
    "openbricks/__init__.py",
    "tools/openbricks-dev/openbricks_dev/__init__.py",
)


def _read_init_version(path):
    m = _INIT_VERSION_RE.search(path.read_text())
    if not m:
        raise RuntimeError("no ``__version__`` in {}".format(path))
    return m.group(1)


def main():
    root = Path(__file__).resolve().parent.parent
    canonical = (root / "VERSION").read_text().strip()
    failures = []
    for rel in _INIT_FILES:
        actual = _read_init_version(root / rel)
        if actual != canonical:
            failures.append("  {}: {!r} != VERSION ({!r})".format(
                rel, actual, canonical))
    if failures:
        print("version drift detected:", file=sys.stderr)
        for f in failures:
            print(f, file=sys.stderr)
        print("run: scripts/bump-version.py {}".format(canonical),
              file=sys.stderr)
        return 1
    print("version OK: {}".format(canonical))
    return 0


if __name__ == "__main__":
    sys.exit(main())
