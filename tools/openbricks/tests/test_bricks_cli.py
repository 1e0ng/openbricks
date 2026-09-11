# SPDX-License-Identifier: MIT
"""``openbricks bricks fetch`` / ``convert``."""
import io
import json
import os
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from unittest import mock

from openbricks_dev import cli

try:
    import numpy  # noqa: F401  - the converter's dependency
    HAVE_NUMPY = True
except ImportError:                      # pragma: no cover
    HAVE_NUMPY = False

from tests.ldraw_fixture import write_mini_library


class FetchTests(unittest.TestCase):
    def test_fetch_calls_the_library_downloader(self):
        with mock.patch("openbricks_sim.bricks.fetch_library", return_value="/x/ldraw") as fetch, \
                mock.patch("openbricks_sim.bricks.library_present", return_value=True):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = cli.main(["bricks", "fetch", "--dest", "/x/ldraw", "--force"])
        self.assertEqual(rc, 0)
        fetch.assert_called_once_with(dest="/x/ldraw", force=True, progress=print)
        self.assertIn("openbricks bricks convert", out.getvalue())

    def test_fetch_reports_failure(self):
        with mock.patch("openbricks_sim.bricks.fetch_library", return_value="/x/ldraw"), \
                mock.patch("openbricks_sim.bricks.library_present", return_value=False):
            with redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["bricks", "fetch"]), 1)

    def test_help_parses(self):
        for argv in (["bricks", "--help"], ["bricks", "fetch", "--help"], ["bricks", "convert", "--help"]):
            with self.assertRaises(SystemExit) as cm, redirect_stdout(io.StringIO()):
                cli.main(argv)
            self.assertEqual(cm.exception.code, 0, argv)


@unittest.skipIf(not HAVE_NUMPY, "numpy (the [sim] extra) is required")
class ConvertTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = write_mini_library(os.path.join(self.tmp.name, "ldraw"))

    def tearDown(self):
        self.tmp.cleanup()

    def test_convert_writes_a_bundle_file(self):
        out = os.path.join(self.tmp.name, "more.json")
        err = io.StringIO()
        with redirect_stderr(err):
            rc = cli.main(["bricks", "convert", "9999", "7777", "0000", "--ldraw", self.root, "--out", out])
        self.assertEqual(rc, 0)
        with open(out) as fh:
            bundle = json.load(fh)
        self.assertEqual(sorted(bundle["parts"]), ["7777", "9999"])
        self.assertEqual(bundle["missing"], ["0000"])
        self.assertIn("not in the library: 0000", err.getvalue())
        self.assertIn("--bricks " + out, err.getvalue())

    def test_convert_to_stdout_with_weights(self):
        weights = os.path.join(self.tmp.name, "w.json")
        with open(weights, "w") as fh:
            json.dump({"9999": {"g": 0.5376}}, fh)
        out = io.StringIO()
        with redirect_stdout(out), redirect_stderr(io.StringIO()):
            rc = cli.main(["bricks", "convert", "9999", "--ldraw", self.root, "--weights", weights])
        self.assertEqual(rc, 0)
        bundle = json.loads(out.getvalue())
        self.assertEqual(bundle["parts"]["9999"]["source"], "vendor")
        self.assertAlmostEqual(bundle["parts"]["9999"]["density_g_cm3"], 1.05, places=2)

    def test_convert_without_a_library_points_at_fetch(self):
        err = io.StringIO()
        with redirect_stderr(err):
            rc = cli.main(["bricks", "convert", "9999", "--ldraw", os.path.join(self.tmp.name, "nowhere")])
        self.assertEqual(rc, 1)
        self.assertIn("openbricks bricks fetch", err.getvalue())

    def test_convert_uses_the_cache_dir_by_default(self):
        with mock.patch.dict(os.environ, {"OPENBRICKS_LDRAW_DIR": self.root}):
            out = io.StringIO()
            with redirect_stdout(out), redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(["bricks", "convert", "9999"]), 0)
        self.assertIn("9999", json.loads(out.getvalue())["parts"])

    def test_nothing_converted_is_a_failure(self):
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(cli.main(["bricks", "convert", "0000", "--ldraw", self.root]), 1)


class BricksErrorPathTests(unittest.TestCase):
    def test_unknown_action_is_rejected(self):
        import types
        from openbricks_dev import bricks
        err = io.StringIO()
        with redirect_stderr(err):
            self.assertEqual(bricks.run(types.SimpleNamespace(bricks_command="nope")), 2)
        self.assertIn("unknown bricks action", err.getvalue())

    def test_convert_without_numpy_points_at_the_extra(self):
        import sys
        err = io.StringIO()
        with mock.patch.dict(sys.modules, {"openbricks_sim.bricks.ldraw": None}), redirect_stderr(err):
            rc = cli.main(["bricks", "convert", "9999"])
        self.assertEqual(rc, 1)
        self.assertIn("pip install openbricks[sim]", err.getvalue())
