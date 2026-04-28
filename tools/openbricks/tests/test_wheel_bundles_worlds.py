# SPDX-License-Identifier: MIT
"""Regression test for the 0.10.3-0.10.5 missing-worlds bug.

The failure mode: ``[tool.setuptools.packages.find]`` only catches
``*.py`` files inside the package directory; ``MANIFEST.in``'s
``recursive-include worlds *.xml`` only seeds the *sdist*, not the
wheel. With the worlds living at ``tools/openbricks/worlds/``
(sibling of the package), every wheel published 0.10.3 → 0.10.5
shipped without world XMLs. End users hit
``WorldLoadError: world file not found: wro-2026-elementary``
even though ``openbricks sim --help`` listed the alias.

The 0.10.6 fix moves ``worlds/`` into the package and adds a
``package-data`` directive. This test runs ``python -m build
--wheel``, opens the produced .whl as a zip, and asserts every
shipped world has its ``world.xml`` inside ``openbricks_sim/worlds/``.

Skipped if ``build`` isn't installed (which would also block
``test_sdist_build.py`` — same gating).
"""

import os
import subprocess
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path


_PKG_ROOT = Path(__file__).resolve().parent.parent


_REQUIRED_WORLDS = [
    "wro_2026_elementary_robot_rockstars",
    "wro_2026_junior_heritage_heroes",
    "wro_2026_senior_mosaic_masters",
    "practice_zones",
    "practice_walls",
]


def _build_wheel_into(tmpdir):
    try:
        subprocess.run(
            [sys.executable, "-m", "build", "--wheel", "--outdir", tmpdir],
            cwd=str(_PKG_ROOT),
            check=True,
            capture_output=True,
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"})
    except FileNotFoundError:
        raise unittest.SkipTest("python interpreter unavailable")
    except subprocess.CalledProcessError as e:
        raise AssertionError(
            "python -m build --wheel failed:\n" +
            e.stderr.decode("utf-8", "replace"))
    wheels = sorted(Path(tmpdir).glob("openbricks-*.whl"))
    if not wheels:
        raise AssertionError(
            "no openbricks-*.whl produced; got: %s" %
            list(Path(tmpdir).iterdir()))
    return wheels[-1]


class WheelBundlesWorldsTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        try:
            import build  # noqa: F401
        except ImportError:
            raise unittest.SkipTest(
                "the ``build`` package is required for this test; "
                "install via ``pip install build`` (already in the "
                "[dev] extras of ``openbricks``)")

    def test_wheel_contains_world_xml_for_each_alias(self):
        with tempfile.TemporaryDirectory() as tmp:
            wheel = _build_wheel_into(tmp)
            with zipfile.ZipFile(wheel) as zf:
                names = zf.namelist()

        missing = []
        for name in _REQUIRED_WORLDS:
            wanted = "openbricks_sim/worlds/" + name + "/world.xml"
            if wanted not in names:
                missing.append(name)
        self.assertEqual(
            missing, [],
            "wheel is missing world.xml for: %s\n"
            "Check ``[tool.setuptools.package-data]`` in pyproject.toml — "
            "without it, ``packages.find`` only ships ``*.py`` files. "
            "Wheel contents under openbricks_sim/worlds/: %s" % (
                missing,
                sorted(n for n in names
                       if n.startswith("openbricks_sim/worlds/"))))

    def test_wheel_contains_mat_png_for_wro_worlds(self):
        # Only the WRO worlds have ``mat.png`` (the practice worlds
        # use solid-rgba slabs, no texture). If the textured WRO
        # worlds lose their PNG, the colour sensor's Phase E1 sampling
        # falls back to material rgba — the texture-pattern bug
        # we just fixed in PR #84 returns silently.
        wro_with_textures = [
            "wro_2026_elementary_robot_rockstars",
            "wro_2026_junior_heritage_heroes",
            "wro_2026_senior_mosaic_masters",
        ]
        with tempfile.TemporaryDirectory() as tmp:
            wheel = _build_wheel_into(tmp)
            with zipfile.ZipFile(wheel) as zf:
                names = zf.namelist()
        missing = []
        for name in wro_with_textures:
            wanted = "openbricks_sim/worlds/" + name + "/mat.png"
            if wanted not in names:
                missing.append(name)
        self.assertEqual(
            missing, [],
            "wheel is missing mat.png for WRO worlds: %s — "
            "Phase E1 colour-sensor texture sampling silently degrades "
            "to flat material rgba without these." % missing)


if __name__ == "__main__":
    unittest.main()
