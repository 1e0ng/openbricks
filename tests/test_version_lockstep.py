# SPDX-License-Identifier: MIT
"""Firmware and host tooling share ONE version (lockstep since
1.15.0). The wheel doesn't bundle the firmware package, so the value
is duplicated into both ``__init__.py`` files by
``scripts/bump-version.py`` — this test pins the duplicates equal so
a manual edit of one can't silently desynchronise the release pair.

Runs under CPython and unix MicroPython; plain string scanning."""

import tests._fakes  # noqa: F401

import unittest


_here = __file__
_idx = _here.rfind("/")
_ROOT = (_here[:_idx] if _idx >= 0 else ".") + "/.."

_FILES = [
    _ROOT + "/openbricks/__init__.py",
    _ROOT + "/tools/openbricks/openbricks_dev/__init__.py",
]


def _read_version(path):
    with open(path) as f:
        for line in f:
            if line.startswith("__version__"):
                return line.split('"')[1]
    return None


class VersionLockstepTests(unittest.TestCase):
    def test_firmware_and_cli_versions_match(self):
        firmware = _read_version(_FILES[0])
        cli = _read_version(_FILES[1])
        self.assertTrue(firmware is not None and cli is not None)
        self.assertEqual(
            firmware, cli,
            "firmware %s != openbricks %s — bump both together via "
            "scripts/bump-version.py X.Y.Z" % (firmware, cli))


if __name__ == "__main__":
    unittest.main()
