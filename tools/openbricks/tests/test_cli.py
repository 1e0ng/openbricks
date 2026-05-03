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

    def test_version_flag_prints_and_exits(self):
        # ``openbricks --version`` exits 0 and writes the version to
        # stdout. Cheap "is the install live + which version" check
        # users expect from any CLI tool.
        from openbricks_dev import __version__
        out = io.StringIO()
        with self.assertRaises(SystemExit) as ctx:
            with patch("sys.stdout", new=out):
                self._parse(["--version"])
        self.assertEqual(ctx.exception.code, 0)
        self.assertIn(__version__, out.getvalue())

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


if __name__ == "__main__":
    unittest.main()
