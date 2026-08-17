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

    def test_flash_requires_only_name(self):
        # 1.22.0: --port and --firmware are optional (auto-detected /
        # auto-downloaded); --name remains the one required argument.
        with self.assertRaises(SystemExit):
            with patch("sys.stderr", new_callable=io.StringIO):
                self._parse(["flash", "--port", "P", "--firmware", "F"])
        args = self._parse(["flash", "--name", "A"])
        self.assertIsNone(args.port)
        self.assertIsNone(args.firmware)

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

    def test_run_requires_name(self):
        # ``-n NAME`` is still required at the argparse layer.
        # SCRIPT is optional now (mutually exclusive with -c). The
        # "must pass one of script/-c" check happens at runtime;
        # see test_neither_script_nor_command_raises in test_run.py.
        with self.assertRaises(SystemExit):
            with patch("sys.stderr", new_callable=io.StringIO):
                self._parse(["run", "script.py"])  # no -n

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
        # No --path means the DEFAULT program flow: compile on the
        # host, stage /program.mpy (or source for old firmware) —
        # decided in-session, so the parser carries None, not a path.
        self.assertIsNone(args.path)
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

    def test_cancelled_error_maps_to_aborted_130(self):
        # The run command converts Ctrl-C into task cancellation (a
        # raw KeyboardInterrupt inside bleak teardown hard-crashes
        # the interpreter under a notification flood) — the CLI must
        # treat a propagated cancellation exactly like Ctrl-C.
        import asyncio
        with patch("openbricks_dev.run.run",
                   side_effect=asyncio.CancelledError()):
            rc = cli.main(["run", "-n", "A", "script.py"])
        self.assertEqual(rc, 130)

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




class DispatchTests(unittest.TestCase):
    """main() routes each subcommand to its module's run(args) and
    maps typed errors / Ctrl-C to exit codes."""

    def _patch_run(self, module_name, fn):
        import importlib
        mod = importlib.import_module("openbricks_dev." + module_name)
        orig = mod.run
        mod.run = fn
        self.addCleanup(setattr, mod, "run", orig)

    def test_each_subcommand_dispatches(self):
        from openbricks_dev import cli
        seen = []
        cases = [
            ("flash", ["flash", "--name", "X", "--port", "/p",
                       "--firmware", "f.bin"]),
            ("scan", ["list"]),
            ("run", ["run", "-n", "X", "s.py"]),
            ("upload", ["upload", "-n", "X", "s.py"]),
            ("stop", ["stop", "-n", "X"]),
            ("log", ["log", "-n", "X"]),
            ("servo_id", ["servo-id", "-p", "/p", "3"]),
            ("pasteprobe", ["paste-probe", "-n", "X"]),
            ("docs", ["docs"]),
        ]
        for mod_name, argv in cases:
            seen.clear()
            self._patch_run(mod_name, lambda args, m=mod_name:
                            (seen.append(m), 0)[1])
            rc = cli.main(argv)
            self.assertEqual(rc, 0, argv)
            self.assertEqual(seen, [mod_name])

    def test_typed_error_prints_and_returns_1(self):
        import io, sys
        from openbricks_dev import cli
        from openbricks_dev.scan import ScanError

        def _boom(args):
            raise ScanError("no adapter")
        self._patch_run("scan", _boom)
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            rc = cli.main(["list"])
        finally:
            sys.stderr = orig
        self.assertEqual(rc, 1)
        self.assertIn("error: no adapter", err.getvalue())

    def test_keyboard_interrupt_returns_130(self):
        import io, sys
        from openbricks_dev import cli

        def _boom(args):
            raise KeyboardInterrupt()
        self._patch_run("stop", _boom)
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            rc = cli.main(["stop", "-n", "X"])
        finally:
            sys.stderr = orig
        self.assertEqual(rc, 130)
        self.assertIn("aborted", err.getvalue())


class SimDispatchTests(unittest.TestCase):
    def test_sim_forwards_remaining_argv(self):
        import sys, types
        from openbricks_dev import cli
        calls = []
        fake = types.ModuleType("openbricks_sim.cli")
        fake.main = lambda argv: (calls.append(list(argv)), 7)[1]
        had = "openbricks_sim.cli" in sys.modules
        prev = sys.modules.get("openbricks_sim.cli")
        sys.modules["openbricks_sim.cli"] = fake
        try:
            rc = cli.main(["sim", "preview", "--fast"])
        finally:
            if had:
                sys.modules["openbricks_sim.cli"] = prev
            else:
                del sys.modules["openbricks_sim.cli"]
        self.assertEqual(rc, 7)
        self.assertEqual(calls, [["preview", "--fast"]])

    def test_sim_missing_extra_prints_hint(self):
        import io, sys
        from openbricks_dev import cli
        had = "openbricks_sim.cli" in sys.modules
        prev = sys.modules.get("openbricks_sim.cli")
        sys.modules["openbricks_sim.cli"] = None   # import -> ImportError
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            rc = cli.main(["sim", "run"])
        finally:
            sys.stderr = orig
            if had:
                sys.modules["openbricks_sim.cli"] = prev
            else:
                del sys.modules["openbricks_sim.cli"]
        self.assertEqual(rc, 1)
        self.assertIn("pip install openbricks[sim]", err.getvalue())


class MainModuleTests(unittest.TestCase):
    def test_python_dash_m_reaches_argparse(self):
        # ``python -m openbricks_dev --help`` executes __main__.py ->
        # cli.main() -> argparse --help (SystemExit 0).
        import runpy, sys
        argv = sys.argv
        sys.argv = ["openbricks", "--help"]
        try:
            with self.assertRaises(SystemExit) as ctx:
                runpy.run_module("openbricks_dev.__main__",
                                 run_name="__main__")
        finally:
            sys.argv = argv
        self.assertEqual(ctx.exception.code, 0)


if __name__ == "__main__":
    unittest.main()
