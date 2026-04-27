#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Bump the firmware and/or host-tooling version.

Firmware and the host tooling (the unified ``openbricks`` package
that ships both the CLI and the MuJoCo sim) are versioned
independently — firmware changes are big and slow, host changes
are small and fast, so coupling their version numbers held each
hostage to the other. Each package's ``__init__.py`` carries its
own ``__version__`` literal, which is the single source of truth —
``pyproject.toml`` reads it back via ``attr = "<pkg>.__version__"``
and the import-time constant is what users see at runtime.

Usage:

    scripts/bump-version.py --firmware 0.9.3
    scripts/bump-version.py --openbricks 0.10.0
    scripts/bump-version.py --firmware 0.9.3 --openbricks 0.10.0

Either flag may be omitted; a version is only bumped when its flag
is present.

Tags use separate namespaces:

  * firmware:   ``git tag v<version>``
  * openbricks: ``git tag openbricks/v<version>``

The ``openbricks`` package was previously published as
``openbricks-dev`` (host CLI) and ``openbricks-sim`` (sim) before the
unification PR. Old tags ``openbricks-dev/v*`` are frozen — the new
namespace is ``openbricks/v*``.
"""

import argparse
import re
import sys
from pathlib import Path


_VERSION_RE   = re.compile(r"^\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?$")
_INIT_LINE_RE = re.compile(r'^__version__\s*=\s*"[^"]*"$', re.M)


_FIRMWARE = {
    "label":    "firmware",
    "init":     "openbricks/__init__.py",
    "tag_hint": "git tag v{version}",
}
_HOST = {
    "label":    "openbricks",
    "init":     "tools/openbricks/openbricks_dev/__init__.py",
    "tag_hint": "git tag openbricks/v{version}",
}


def _update_init(path, new_version):
    text = path.read_text()
    new_text, n = _INIT_LINE_RE.subn(
        '__version__ = "{}"'.format(new_version), text, count=1)
    if n == 0:
        raise RuntimeError(
            "no ``__version__ = \"...\"`` line in {}".format(path))
    if new_text != text:
        path.write_text(new_text)


def _bump(root, component, new_version):
    if not _VERSION_RE.match(new_version):
        print("error: invalid version {!r} for {} (expected X.Y.Z)".format(
            new_version, component["label"]), file=sys.stderr)
        return 2
    _update_init(root / component["init"], new_version)
    print("bumped {} to {}".format(component["label"], new_version))
    print("  tag with: " + component["tag_hint"].format(version=new_version))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Bump firmware and/or openbricks (host) versions.")
    ap.add_argument("--firmware", metavar="X.Y.Z",
                    help="New firmware version (tag: v<version>).")
    ap.add_argument("--openbricks", metavar="X.Y.Z",
                    help="New openbricks (host) version "
                         "(tag: openbricks/v<version>).")
    args = ap.parse_args(argv)

    if not args.firmware and not args.openbricks:
        ap.error("pass --firmware and/or --openbricks")

    root = Path(__file__).resolve().parent.parent
    if args.firmware:
        rc = _bump(root, _FIRMWARE, args.firmware)
        if rc:
            return rc
    if args.openbricks:
        rc = _bump(root, _HOST, args.openbricks)
        if rc:
            return rc
    return 0


if __name__ == "__main__":
    sys.exit(main())
