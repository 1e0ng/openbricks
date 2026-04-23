#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""
Bump the firmware and/or openbricks-dev version.

The firmware (``openbricks/``) and the host CLI
(``tools/openbricks-dev/``) move independently — firmware changes are
big and slow, CLI changes are small and fast, so coupling their
version numbers held one hostage to the other. Each has its own
``VERSION`` file now:

* Root ``VERSION``                        — firmware version.
  Mirrored in ``openbricks/__init__.py::__version__``.
  Released via ``git tag v<version>``.

* ``tools/openbricks-dev/VERSION``        — openbricks-dev version.
  Mirrored in ``openbricks_dev/__init__.py::__version__``.
  Released via ``git tag openbricks-dev/v<version>``.

Usage:

    scripts/bump-version.py --firmware 0.9.3
    scripts/bump-version.py --openbricks-dev 0.10.0
    scripts/bump-version.py --firmware 0.9.3 --openbricks-dev 0.10.0

Either flag may be omitted; a version is only bumped when its flag is
present.
"""

import argparse
import re
import sys
from pathlib import Path


_VERSION_RE   = re.compile(r"^\d+\.\d+\.\d+(?:[-+][0-9A-Za-z.-]+)?$")
_INIT_LINE_RE = re.compile(r'^__version__\s*=\s*"[^"]*"$', re.M)


_FIRMWARE = {
    "label":     "firmware",
    "version":   "VERSION",
    "init":      "openbricks/__init__.py",
    "tag_hint":  "git tag v{version}",
}
_CLI = {
    "label":     "openbricks-dev",
    "version":   "tools/openbricks-dev/VERSION",
    "init":      "tools/openbricks-dev/openbricks_dev/__init__.py",
    "tag_hint":  "git tag openbricks-dev/v{version}",
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
    (root / component["version"]).write_text(new_version + "\n")
    _update_init(root / component["init"], new_version)
    print("bumped {} to {}".format(component["label"], new_version))
    print("  tag with: " + component["tag_hint"].format(version=new_version))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Bump firmware and/or openbricks-dev versions.")
    ap.add_argument("--firmware", metavar="X.Y.Z",
                    help="New firmware version (tag: v<version>).")
    ap.add_argument("--openbricks-dev", metavar="X.Y.Z", dest="openbricks_dev",
                    help="New openbricks-dev version (tag: openbricks-dev/v<version>).")
    args = ap.parse_args(argv)

    if not args.firmware and not args.openbricks_dev:
        ap.error("pass --firmware and/or --openbricks-dev")

    root = Path(__file__).resolve().parent.parent
    if args.firmware:
        rc = _bump(root, _FIRMWARE, args.firmware)
        if rc:
            return rc
    if args.openbricks_dev:
        rc = _bump(root, _CLI, args.openbricks_dev)
        if rc:
            return rc
    return 0


if __name__ == "__main__":
    sys.exit(main())
