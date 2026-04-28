# SPDX-License-Identifier: MIT
"""Regression test for the cibuildwheel config that drives PyPI wheel
publishing.

The failure mode this guards against: someone deletes or trims the
``[tool.cibuildwheel]`` section in ``pyproject.toml`` thinking it's
unused (it isn't — the CI publish job reads it implicitly via
``pypa/cibuildwheel``, with no other config file). Without the
config, cibuildwheel falls back to its defaults: PyPy gets built
(slow, no users), aarch64 + i686 attempt cross-compile, and the
post-build smoke import is skipped — a broken wheel can land on
PyPI before anyone notices.

We assert the contract this repo's publish workflow depends on:
  * The CPython 3.9-3.13 build line.
  * Skips for PyPy / musllinux / 32-bit.
  * The smoke ``test-command`` imports a real attribute of
    ``openbricks_sim._native``.
  * Linux uses ``manylinux_2_28`` (PyPI-accepted; the 0.10.2
    failure was specifically about wheels not being manylinux-tagged).
  * macOS builds universal2.

Skipped if neither ``tomllib`` (3.11+) nor ``tomli`` (the 3.9-3.10
backport) is importable.
"""

import unittest
from pathlib import Path


_PKG_ROOT = Path(__file__).resolve().parent.parent
_PYPROJECT = _PKG_ROOT / "pyproject.toml"


def _load_pyproject():
    try:
        import tomllib
    except ImportError:
        try:
            import tomli as tomllib
        except ImportError:
            raise unittest.SkipTest(
                "neither tomllib (py311+) nor tomli is available")
    return tomllib.loads(_PYPROJECT.read_text())


class CibuildwheelConfigTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.cfg = _load_pyproject().get("tool", {}).get("cibuildwheel")

    def test_section_exists(self):
        self.assertIsNotNone(
            self.cfg,
            "[tool.cibuildwheel] missing from pyproject.toml — the "
            "publish workflow's wheel build relies on it.")

    def test_builds_cpython_39_through_313(self):
        build_line = self.cfg.get("build", "")
        for py in ("cp39-*", "cp310-*", "cp311-*", "cp312-*", "cp313-*"):
            self.assertIn(
                py, build_line,
                "build line missing %s — wheels for that interpreter "
                "won't be produced." % py)

    def test_skips_pypy_musllinux_and_32bit(self):
        skip = self.cfg.get("skip", [])
        # Each skip pattern is fail-safe: if cibuildwheel ever changes
        # its defaults, these stay explicit.
        self.assertIn("pp*", skip)
        self.assertIn("*-musllinux*", skip)
        self.assertIn("*-manylinux_i686", skip)
        self.assertIn("*-win32", skip)

    def test_smoke_import_targets_a_real_attribute(self):
        # The test-command imports openbricks_sim._native and asserts
        # ``TrapezoidalProfile`` is defined. If we ever rename or
        # remove that class, both the wheel smoke test and this test
        # need to update — keeping them in sync prevents a silent
        # regression where the smoke test imports nothing real.
        from openbricks_sim import _native
        self.assertTrue(
            hasattr(_native, "TrapezoidalProfile"),
            "openbricks_sim._native.TrapezoidalProfile is gone — update "
            "[tool.cibuildwheel].test-command in pyproject.toml to "
            "import the new pillar attribute.")
        cmd = self.cfg.get("test-command", "")
        self.assertIn("TrapezoidalProfile", cmd)
        self.assertIn("openbricks_sim", cmd)

    def test_linux_uses_manylinux_2_28(self):
        # PyPI rejected the 0.10.0 / 0.10.1 wheels because they were
        # tagged ``linux_x86_64`` instead of ``manylinux*``. Pinning
        # the manylinux image here is the contract that prevents a
        # regression to that failure mode.
        linux_cfg = self.cfg.get("linux", {})
        self.assertEqual(
            linux_cfg.get("manylinux-x86_64-image"), "manylinux_2_28")

    def test_macos_builds_universal2(self):
        # One wheel that runs on Intel + Apple Silicon. If this
        # regresses to the cibuildwheel default (host-arch only),
        # users on the *other* arch fall back to the sdist + local
        # compile.
        macos_cfg = self.cfg.get("macos", {})
        self.assertEqual(macos_cfg.get("archs"), ["universal2"])


if __name__ == "__main__":
    unittest.main()
