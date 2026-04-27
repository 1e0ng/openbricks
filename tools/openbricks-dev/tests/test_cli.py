# SPDX-License-Identifier: MIT
"""Tests for the top-level argparse + dispatch in ``openbricks_dev.cli``."""

import io
import sys
import unittest
from unittest.mock import patch

from openbricks_dev import cli


class BuildParserTests(unittest.TestCase):
    """Ensure required args are required and optional ones default correctly."""

    def setUp(self):
        self.parser = cli._build_parser()

    def _parse(self, argv):
        return self.parser.parse_args(argv)

    # ---- flash ----

    def test_flash_requires_name_port_firmware(self):
        # Each of the three required args, missing in turn, should exit.
        base = ["flash", "--name", "A", "--port", "P", "--firmware", "F"]
        for missing in ("--name", "--port", "--firmware"):
            idx = base.index(missing)
            truncated = base[:idx] + base[idx + 2:]
            with self.assertRaises(SystemExit):
                # Suppress argparse's error text so the test output stays
                # clean — we only care that it exits non-zero.
                with patch("sys.stderr", new_callable=io.StringIO):
                    self._parse(truncated)

    def test_flash_defaults(self):
        args = self._parse([
            "flash", "--name", "RobotA", "--port", "/dev/ttyUSB0",
            "--firmware", "firmware.bin",
        ])
        self.assertEqual(args.chip, "auto")
        self.assertEqual(args.baud, "460800")
        self.assertFalse(args.skip_erase)

    def test_flash_overrides(self):
        args = self._parse([
            "flash", "--name", "RobotA", "--port", "COM5",
            "--firmware", "fw.bin", "--chip", "esp32s3",
            "--baud", "921600", "--skip-erase",
        ])
        self.assertEqual(args.chip, "esp32s3")
        self.assertEqual(args.baud, "921600")
        self.assertTrue(args.skip_erase)

    # ---- run ----

    def test_run_requires_name_and_script(self):
        for missing in (["run", "script.py"],                         # no -n
                        ["run", "-n", "A"]):                          # no script
            with self.assertRaises(SystemExit):
                with patch("sys.stderr", new_callable=io.StringIO):
                    self._parse(missing)

    def test_run_defaults(self):
        args = self._parse(["run", "-n", "RobotA", "myscript.py"])
        self.assertEqual(args.name, "RobotA")
        self.assertEqual(args.script, "myscript.py")
        self.assertEqual(args.scan_timeout, 5.0)

    def test_run_accepts_scan_timeout(self):
        args = self._parse(["run", "-n", "A", "s.py", "--scan-timeout", "2"])
        self.assertEqual(args.scan_timeout, 2.0)

    # ---- upload ----

    def test_upload_requires_name_and_script(self):
        for missing in (["upload", "s.py"],
                        ["upload", "-n", "A"]):
            with self.assertRaises(SystemExit):
                with patch("sys.stderr", new_callable=io.StringIO):
                    self._parse(missing)

    def test_upload_defaults(self):
        args = self._parse(["upload", "-n", "RobotA", "s.py"])
        self.assertEqual(args.name, "RobotA")
        self.assertEqual(args.script, "s.py")
        # Default target is /program.py — the path the firmware's
        # frozen launcher reads on button press.
        self.assertEqual(args.path, "/program.py")
        self.assertEqual(args.scan_timeout, 5.0)

    def test_upload_accepts_path_override(self):
        args = self._parse([
            "upload", "-n", "A", "s.py", "--path", "/main.py",
        ])
        self.assertEqual(args.path, "/main.py")

    # ---- stop ----

    def test_stop_requires_name(self):
        with self.assertRaises(SystemExit):
            with patch("sys.stderr", new_callable=io.StringIO):
                self._parse(["stop"])

    def test_stop_defaults(self):
        args = self._parse(["stop", "-n", "RobotA"])
        self.assertEqual(args.name, "RobotA")
        self.assertEqual(args.scan_timeout, 5.0)

    # ---- list ----

    def test_list_defaults(self):
        args = self._parse(["list"])
        self.assertEqual(args.timeout, 5.0)
        self.assertFalse(args.all)

    def test_list_accepts_timeout_and_all(self):
        args = self._parse(["list", "--timeout", "2.5", "--all"])
        self.assertEqual(args.timeout, 2.5)
        self.assertTrue(args.all)

    # ---- no subcommand ----

    def test_missing_subcommand_exits(self):
        with self.assertRaises(SystemExit):
            with patch("sys.stderr", new_callable=io.StringIO):
                self._parse([])


class MainDispatchTests(unittest.TestCase):
    """``cli.main`` should route to the right subcommand module."""

    def test_flash_routes_to_flash_run(self):
        with patch("openbricks_dev.flash.run", return_value=0) as flash_run:
            rc = cli.main([
                "flash", "--name", "A", "--port", "P", "--firmware", "F",
            ])
        self.assertEqual(rc, 0)
        flash_run.assert_called_once()
        args = flash_run.call_args[0][0]
        self.assertEqual(args.name, "A")

    def test_list_routes_to_scan_run(self):
        with patch("openbricks_dev.scan.run", return_value=0) as scan_run:
            rc = cli.main(["list", "--timeout", "1"])
        self.assertEqual(rc, 0)
        scan_run.assert_called_once()

    def test_run_routes_to_run_module(self):
        with patch("openbricks_dev.run.run", return_value=0) as run_run:
            rc = cli.main(["run", "-n", "A", "script.py"])
        self.assertEqual(rc, 0)
        run_run.assert_called_once()

    def test_stop_routes_to_stop_module(self):
        with patch("openbricks_dev.stop.run", return_value=0) as stop_run:
            rc = cli.main(["stop", "-n", "A"])
        self.assertEqual(rc, 0)
        stop_run.assert_called_once()

    def test_upload_routes_to_upload_module(self):
        with patch("openbricks_dev.upload.run", return_value=0) as ul_run:
            rc = cli.main(["upload", "-n", "A", "s.py"])
        self.assertEqual(rc, 0)
        ul_run.assert_called_once()

    def test_exception_from_subcommand_becomes_rc_1(self):
        def _boom(args):
            raise RuntimeError("boom")
        with patch("openbricks_dev.flash.run", side_effect=_boom):
            with patch("sys.stderr", new_callable=io.StringIO) as err:
                rc = cli.main([
                    "flash", "--name", "A", "--port", "P", "--firmware", "F",
                ])
        self.assertEqual(rc, 1)
        self.assertIn("boom", err.getvalue())

    def test_keyboard_interrupt_becomes_rc_130(self):
        def _cancel(args):
            raise KeyboardInterrupt()
        with patch("openbricks_dev.scan.run", side_effect=_cancel):
            with patch("sys.stderr", new_callable=io.StringIO):
                rc = cli.main(["list"])
        self.assertEqual(rc, 130)


class SimPassthroughTests(unittest.TestCase):
    """``openbricks sim …`` short-circuits argparse and forwards the
    remaining argv to ``openbricks_sim.cli.main``."""

    def test_sim_dispatch_calls_sim_cli(self):
        # Inject a fake openbricks_sim module that records its argv.
        import sys, types
        fake = types.ModuleType("openbricks_sim")
        fake_cli = types.ModuleType("openbricks_sim.cli")
        recorded = {}
        def _fake_main(argv=None):
            recorded["argv"] = argv
            return 0
        fake_cli.main = _fake_main
        fake.cli = fake_cli
        prev_pkg = sys.modules.get("openbricks_sim")
        prev_cli = sys.modules.get("openbricks_sim.cli")
        sys.modules["openbricks_sim"] = fake
        sys.modules["openbricks_sim.cli"] = fake_cli
        try:
            rc = cli.main(["sim", "preview", "--world", "empty"])
        finally:
            if prev_pkg is None:
                sys.modules.pop("openbricks_sim", None)
            else:
                sys.modules["openbricks_sim"] = prev_pkg
            if prev_cli is None:
                sys.modules.pop("openbricks_sim.cli", None)
            else:
                sys.modules["openbricks_sim.cli"] = prev_cli

        self.assertEqual(rc, 0)
        self.assertEqual(recorded["argv"],
                         ["preview", "--world", "empty"])

    def test_sim_dispatch_without_openbricks_sim_prints_hint(self):
        # Make ``import openbricks_sim.cli`` fail.
        import sys
        prev_pkg = sys.modules.get("openbricks_sim")
        prev_cli = sys.modules.get("openbricks_sim.cli")
        # Sentinel that raises ImportError on import attempts.
        sys.modules["openbricks_sim"] = None
        sys.modules.pop("openbricks_sim.cli", None)
        try:
            with patch("sys.stderr", new_callable=io.StringIO) as err:
                rc = cli.main(["sim", "run", "foo.py"])
            self.assertEqual(rc, 1)
            self.assertIn("openbricks-sim", err.getvalue())
            self.assertIn("pip install", err.getvalue())
        finally:
            if prev_pkg is None:
                sys.modules.pop("openbricks_sim", None)
            else:
                sys.modules["openbricks_sim"] = prev_pkg
            if prev_cli is not None:
                sys.modules["openbricks_sim.cli"] = prev_cli

    def test_sim_appears_in_top_level_help(self):
        # `--help` should list the sim subcommand alongside the
        # native ones.
        with patch("sys.stdout", new_callable=io.StringIO) as out:
            with self.assertRaises(SystemExit):
                cli.main(["--help"])
        self.assertIn("sim", out.getvalue())


if __name__ == "__main__":
    unittest.main()
