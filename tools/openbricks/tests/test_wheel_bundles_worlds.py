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

    def test_wheel_contains_the_firmware_package(self):
        # 3.6.0: the sim runs the FIRMWARE'S driver code (its shim
        # subclasses openbricks.drivers.*, user scripts import them),
        # so the wheel must carry the ``openbricks`` package that
        # lives at the repo root. Wheels up to 3.5.0 didn't, and a
        # pipx install could not ``sim run`` any hub-style script
        # (``No module named 'openbricks'``). Pin the modules the shim
        # itself imports plus the ones every bench script does.
        with tempfile.TemporaryDirectory() as tmp:
            wheel = _build_wheel_into(tmp)
            with zipfile.ZipFile(wheel) as zf:
                names = zf.namelist()
        for wanted in ("openbricks/__init__.py",
                       "openbricks/parameters.py",
                       "openbricks/drivers/__init__.py",
                       "openbricks/drivers/qtr.py",
                       "openbricks/drivers/st3032.py",
                       "openbricks/drivers/tcs34725.py",
                       "openbricks/robotics/drivebase.py"):
            self.assertIn(wanted, names,
                          "wheel lacks %s — setup.py::_sync_firmware / "
                          "packages.find include" % wanted)
        # ...and no bytecode caches from the checkout.
        self.assertFalse([n for n in names if "__pycache__" in n
                          and n.startswith("openbricks/")])

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

    def test_wheel_contains_ldr_props(self):
        # F2 (#98 onwards) replaced inline single-box approximations
        # with ``<lego_prop ldr="props/*.ldr"/>`` placeholders that
        # ``world.py`` expands at load time by reading the ``.ldr``
        # file off disk. Wheels 0.10.7-0.10.10 shipped without these
        # files (package-data only listed *.xml / *.png / *.md) so
        # ``pip install openbricks && openbricks sim --world wro-2026-*``
        # raised ``WorldLoadError: lego_prop ... references missing
        # .ldr file ...`` for every prop. Pin every WRO world has at
        # least one ``.ldr`` in the wheel.
        wro_worlds = [
            "wro_2026_elementary_robot_rockstars",
            "wro_2026_junior_heritage_heroes",
            "wro_2026_senior_mosaic_masters",
        ]
        with tempfile.TemporaryDirectory() as tmp:
            wheel = _build_wheel_into(tmp)
            with zipfile.ZipFile(wheel) as zf:
                names = zf.namelist()
        empty = []
        for name in wro_worlds:
            prefix = "openbricks_sim/worlds/" + name + "/props/"
            ldrs = [n for n in names
                    if n.startswith(prefix) and n.endswith(".ldr")]
            if not ldrs:
                empty.append(name)
        self.assertEqual(
            empty, [],
            "wheel is missing ALL props/*.ldr for WRO worlds: %s — "
            "without these, world.py's lego_prop expansion raises "
            "WorldLoadError on every prop. Check "
            "``[tool.setuptools.package-data]`` includes "
            "``worlds/*/props/*.ldr``." % empty)

    def test_wheel_contains_senior_mosaic_frame_stl(self):
        # The Senior world references ``mosaic_frame.stl`` via a
        # ``<mesh file="mosaic_frame.stl"/>`` declaration. STL paths
        # are resolved by MuJoCo at ``MjModel.from_xml_string`` time
        # against the working directory, so the file must ship with
        # the wheel. Without ``worlds/*/*.stl`` in package-data, the
        # Senior world fails to compile with ``Error opening file
        # 'mosaic_frame.stl'``.
        with tempfile.TemporaryDirectory() as tmp:
            wheel = _build_wheel_into(tmp)
            with zipfile.ZipFile(wheel) as zf:
                names = zf.namelist()
        wanted = ("openbricks_sim/worlds/wro_2026_senior_mosaic_masters/"
                  "mosaic_frame.stl")
        self.assertIn(
            wanted, names,
            "wheel is missing %s — the Senior world's <mesh> "
            "declaration won't resolve at load time." % wanted)

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


    def test_wheel_contains_the_offline_docs_bundle(self):
        # ``openbricks docs`` opens the Sphinx site out of
        # openbricks_dev/_docs/offline-docs.zip (1.63.0 — before that,
        # markdown guide pages). Same ship-the-command-forget-the-data
        # bug class as the missing worlds (0.10.3-0.10.5) and missing
        # .ldr props (0.10.7-0.10.10): without the package-data stanza
        # the command exists but shows nothing on an installed wheel.
        with tempfile.TemporaryDirectory() as tmp:
            wheel = _build_wheel_into(tmp)
            with zipfile.ZipFile(wheel) as zf:
                names = zf.namelist()
                bundle = "openbricks_dev/_docs/offline-docs.zip"
                self.assertIn(
                    bundle, names,
                    "wheel is missing the offline docs bundle — check "
                    "setup.py::_sync_docs, MANIFEST.in, and the "
                    "openbricks_dev package-data stanza.")
                # And the bundle inside the wheel is the real site,
                # not an empty or truncated file: the pages the CLI's
                # topic map points at must exist.
                import io
                with zipfile.ZipFile(io.BytesIO(zf.read(bundle))) as dz:
                    pages = set(dz.namelist())
        for page in ("index.html", "install.html", "api/robotics.html"):
            self.assertIn(page, pages,
                          "offline-docs.zip in the wheel lacks %s" % page)


class MatTextureSizeBudgetTests(unittest.TestCase):
    """The PyPI project hit its 10 GB storage limit (2026-07-19)
    because every release duplicated ~17 MB of lossless mat textures
    into 16 wheels. The mats are 75 dpi + 256-colour quantized now;
    this guard fails a regen that silently reinflates them."""

    _BUDGET_BYTES = 2_500_000   # per mat; current mats are 0.8-1.4 MB

    def test_each_mat_is_within_budget(self):
        import glob
        mats = glob.glob(os.path.join(
            os.path.dirname(__file__), "..",
            "openbricks_sim", "worlds", "*", "mat.png"))
        self.assertTrue(mats, "no mat.png found — path drift?")
        for m in mats:
            sz = os.path.getsize(m)
            self.assertLessEqual(
                sz, self._BUDGET_BYTES,
                "%s is %.1f MB — regen without the quantize step? "
                "See scripts/regen-wro-mat-textures.sh header"
                % (m, sz / 1e6))


if __name__ == "__main__":
    unittest.main()
