#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Bump THE version — firmware and host tooling share one number.

Since 1.15.0 the firmware and the host tooling (the unified
``openbricks`` package shipping the CLI and the MuJoCo sim) are
versioned in LOCKSTEP: one number, one bump, two tags. The wheel
doesn't bundle the firmware package, so the sharing happens here —
this script writes the same version literal into both packages'
``__init__.py`` (each remains its artifact's single source of truth;
``pyproject.toml`` reads it back via ``attr = "<pkg>.__version__"``).

Usage:

    scripts/bump-version.py 1.15.0

Then, after the PR merges, cut BOTH releases:

    git tag v<version>       # firmware GitHub Release (.bin files)
    git tag cli/v<version>   # PyPI publish (CLI + sim wheels)
    git push origin v<version> cli/v<version>

History: the two tracks were versioned independently through
firmware 1.14.0 / openbricks 0.14.0 (the old ``--firmware`` /
``--openbricks`` flags), so PyPI versions jump from 0.14.0 straight
to 1.15.0. Older still: the host package was published as
``openbricks-dev`` (tags frozen), and ``openbricks/v*`` was the PyPI
namespace up to 0.10.24 — the namespace is ``cli/v*``.
"""

import argparse
import re
import sys
from pathlib import Path


_VERSION_RE   = re.compile(r"^\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?$")
_INIT_LINE_RE = re.compile(r'^__version__\s*=\s*"[^"]*"$', re.M)

# Both files carry the same number; tests/test_version_lockstep.py
# (firmware suite) and tools/openbricks/tests/test_release_tags.py
# pin them equal.
_INIT_FILES = [
    "openbricks/__init__.py",
    "tools/openbricks/openbricks_dev/__init__.py",
]


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
    ap = argparse.ArgumentParser(
        description="Bump the shared firmware + openbricks version.")
    ap.add_argument("version", metavar="X.Y.Z",
                    help="New version, written to both packages.")
    args = ap.parse_args(argv)

    if not _VERSION_RE.match(args.version):
        print("error: invalid version {!r} (expected X.Y.Z)".format(
            args.version), file=sys.stderr)
        return 2

    root = Path(__file__).resolve().parent.parent
    for rel in _INIT_FILES:
        _update_init(root / rel, args.version)
    print("bumped firmware + openbricks to {}".format(args.version))
    print("  tag with: git tag v{v} cli/v{v} && "
          "git push origin v{v} cli/v{v}".format(v=args.version))
    return 0


if __name__ == "__main__":
    sys.exit(main())
