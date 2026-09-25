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
                mock.patch("openbricks_sim.bricks.library_complete", return_value=True):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = cli.main(["bricks", "fetch", "--dest", "/x/ldraw", "--force"])
        self.assertEqual(rc, 0)
        fetch.assert_called_once_with(dest="/x/ldraw", force=True, progress=print)
        self.assertIn("openbricks bricks convert", out.getvalue())

    def test_fetch_reports_failure(self):
        with mock.patch("openbricks_sim.bricks.fetch_library", return_value="/x/ldraw"), \
                mock.patch("openbricks_sim.bricks.library_complete", return_value=False):
            with redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["bricks", "fetch"]), 1)

    def test_fetch_by_number_keeps_the_parts_under_the_data_dir(self):
        from openbricks_sim.bricks import fetch

        calls = []

        def fake(number, root=None, say=None, colors=True, force=False):
            calls.append((number, root, colors, force))
            say("fetched parts/%s.dat" % number)
            if number == "0000":
                raise fetch.NotInLibrary(number, ["u"])
            if number == "5000":
                raise fetch.FetchError("u5", "HTTP 500")
            return {"format": "openbricks-brick-bundle/1", "files": 1, "parts": {number: {"name": "Part " + number}}}
        with tempfile.TemporaryDirectory() as tmp, \
                mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": tmp}), \
                mock.patch.object(fetch, "fetch_part", fake):
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                rc = cli.main(["bricks", "fetch", "2458", "3005"])
            self.assertEqual(rc, 0, err.getvalue())
            for number in ("2458", "3005"):
                with open(os.path.join(tmp, "bricks", number + ".json")) as fh:
                    self.assertEqual(json.load(fh)["parts"][number]["name"], "Part " + number)
            self.assertIn("fetched parts/2458.dat", out.getvalue())
            self.assertIn("openbricks sim", out.getvalue())
            self.assertEqual(calls[-1], ("3005", None, True, False))
            with redirect_stdout(io.StringIO()), redirect_stderr(err):
                self.assertEqual(cli.main(["bricks", "fetch", "0000", "3005", "--no-colors"]), 1)
                self.assertEqual(cli.main(["bricks", "fetch", "5000"]), 1)
            self.assertIn("no part 0000", err.getvalue())
            self.assertIn("HTTP 500", err.getvalue())
            self.assertTrue(os.path.exists(os.path.join(tmp, "bricks", "3005.json")), "the good one still lands")
            # --force and --dest reach the fetcher: the part's files and the tables again, into that cache
            with redirect_stdout(io.StringIO()), redirect_stderr(err):
                self.assertEqual(cli.main(["bricks", "fetch", "2458", "--force", "--dest", "/x/ldraw"]), 0)
            self.assertEqual(calls[-1], ("2458", "/x/ldraw", True, True))
            # a number the library ships is refused: the shipped record would win anyway
            err = io.StringIO()
            with redirect_stdout(io.StringIO()), redirect_stderr(err):
                self.assertEqual(cli.main(["bricks", "fetch", "3001", "3005"]), 1)
            self.assertIn("3001 is in the library already (Brick  2 x  4)", err.getvalue())
            self.assertEqual(calls[-1][0], "3005", "the others are still fetched")
            # a file that cannot be written is an error line, and the next number is still tried
            err = io.StringIO()
            with open(os.path.join(tmp, "blocked"), "w") as fh:
                fh.write("x")
            with mock.patch.dict(os.environ, {"OPENBRICKS_DATA_DIR": os.path.join(tmp, "blocked")}), \
                    redirect_stdout(io.StringIO()), redirect_stderr(err):
                self.assertEqual(cli.main(["bricks", "fetch", "2458", "3005"]), 1)
            self.assertIn("error:", err.getvalue())
            self.assertIn("blocked", err.getvalue())
            self.assertNotIn("Traceback", err.getvalue())
            self.assertEqual([c[0] for c in calls[-2:]], ["2458", "3005"])

    def test_fetch_by_number_without_numpy_points_at_the_extra(self):
        import builtins
        real_import = builtins.__import__

        def no_numpy(name, *a, **k):
            if name.startswith("openbricks_sim.bricks.ldraw"):
                raise ImportError("no numpy")
            return real_import(name, *a, **k)
        with mock.patch.object(builtins, "__import__", no_numpy):
            err = io.StringIO()
            with redirect_stderr(err), redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["bricks", "fetch", "2458"]), 1)
        self.assertIn("pip install openbricks[sim]", err.getvalue())

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
