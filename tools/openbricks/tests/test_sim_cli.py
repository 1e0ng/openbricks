# SPDX-License-Identifier: MIT
"""Tests for the openbricks-sim CLI argparse + dispatch."""

import io
import unittest
from unittest.mock import patch

from openbricks_sim import cli


class ParserTests(unittest.TestCase):
    def setUp(self):
        self.parser = cli._build_parser()

    def test_preview_defaults(self):
        args = self.parser.parse_args(["preview"])
        self.assertEqual(args.command, "preview")
        self.assertEqual(args.world, "empty")
        self.assertEqual(args.x, 0.0)
        self.assertEqual(args.y, 0.0)
        self.assertFalse(args.headless)
        self.assertEqual(args.duration, 2.0)

    def test_preview_accepts_overrides(self):
        args = self.parser.parse_args([
            "preview", "--world", "wro-2026-elementary",
            "--x", "1.0", "--y", "-0.42",
            "--headless", "--duration", "0.5",
        ])
        self.assertEqual(args.world, "wro-2026-elementary")
        self.assertEqual(args.x, 1.0)
        self.assertEqual(args.y, -0.42)
        self.assertTrue(args.headless)
        self.assertEqual(args.duration, 0.5)

    def test_missing_subcommand_exits(self):
        with self.assertRaises(SystemExit):
            with patch("sys.stderr", new_callable=io.StringIO):
                self.parser.parse_args([])


class ResolveWorldTests(unittest.TestCase):
    def test_empty_alias_returns_none(self):
        self.assertIsNone(cli._resolve_world("empty"))

    def test_wro_alias_points_to_shipped_file(self):
        path = cli._resolve_world("wro-2026-elementary")
        self.assertIsNotNone(path)
        self.assertTrue(path.endswith("world.xml"))

    def test_unknown_arg_returned_as_path(self):
        # A non-alias argument is treated as a path for downstream
        # error reporting to handle.
        self.assertEqual(cli._resolve_world("/some/explicit/path.xml"),
                         "/some/explicit/path.xml")


class MainDispatchTests(unittest.TestCase):
    def test_headless_preview_empty_world(self):
        # Exercise the full happy path: parse → dispatch →
        # standalone chassis MJCF → 0.1 s of physics.
        with patch("sys.stdout", new_callable=io.StringIO) as out:
            rc = cli.main([
                "preview", "--headless", "--duration", "0.1", "--world", "empty",
            ])
        self.assertEqual(rc, 0)
        self.assertIn("headless preview", out.getvalue())


class RelaunchUnderMjpythonTests(unittest.TestCase):
    """Pin the macOS-only mjpython auto-relaunch path. Without it,
    ``openbricks sim preview --world wro-2026-elementary`` on macOS
    crashes with ``RuntimeError: launch_passive requires that the
    Python script be run under mjpython on macOS`` and a Python
    stack trace — useless to a user trying to drive a sim.

    The helper ``execv``s mjpython with the same arguments. We mock
    ``os.execv`` here so the test process doesn't actually disappear,
    and we inspect what command would have been issued."""

    def test_noop_on_linux(self):
        with patch("openbricks_sim.cli.sys") as mock_sys, \
             patch("openbricks_sim.cli.os.execv") as mock_execv:
            mock_sys.platform = "linux"
            cli._relaunch_under_mjpython_if_needed()
            mock_execv.assert_not_called()

    def test_noop_when_already_under_mjpython(self):
        with patch("openbricks_sim.cli.sys") as mock_sys, \
             patch("openbricks_sim.cli.os.execv") as mock_execv:
            mock_sys.platform = "darwin"
            mock_sys.executable = "/path/to/venv/bin/mjpython"
            cli._relaunch_under_mjpython_if_needed()
            mock_execv.assert_not_called()

    def test_noop_when_mjpython_missing_from_venv(self):
        # Stripped MuJoCo wheel without the bundled mjpython binary
        # — let the upstream RuntimeError fire instead of pretending
        # to relaunch.
        with patch("openbricks_sim.cli.sys") as mock_sys, \
             patch("openbricks_sim.cli.os.execv") as mock_execv, \
             patch("openbricks_sim.cli.Path") as mock_path:
            mock_sys.platform = "darwin"
            mock_sys.executable = "/path/to/venv/bin/python"
            # Make ``Path(sys.executable).parent / "mjpython"``
            # report ``is_file() → False``.
            fake_python_path = mock_path.return_value
            fake_python_path.name = "python"
            fake_mjpython = type(fake_python_path).__truediv__.return_value
            fake_mjpython.is_file.return_value = False
            mock_path.return_value.parent.__truediv__.return_value = fake_mjpython
            cli._relaunch_under_mjpython_if_needed()
            mock_execv.assert_not_called()

    def test_relaunches_with_sim_subcommand_stripped(self):
        # ``openbricks sim preview --world X`` arrives as
        # ``sys.argv = [".../bin/openbricks", "sim", "preview",
        # "--world", "X"]`` — under mjpython we want to forward
        # only ``["preview", "--world", "X"]`` (the leading "sim"
        # is the openbricks-CLI dispatch keyword, not part of the
        # sim CLI's own argv).
        from pathlib import Path
        recorded = {}

        def fake_execv(path, argv):
            recorded["path"] = path
            recorded["argv"] = argv

        with patch("openbricks_sim.cli.sys") as mock_sys, \
             patch("openbricks_sim.cli.os.execv", side_effect=fake_execv), \
             patch("openbricks_sim.cli.Path") as mock_path_cls:
            mock_sys.platform = "darwin"
            mock_sys.executable = "/venv/bin/python"
            mock_sys.argv = ["/usr/bin/openbricks", "sim", "preview",
                             "--world", "wro-2026-elementary"]

            # Stub ``Path`` so ``Path(sys.executable).name == "python"``
            # and ``Path(sys.executable).parent / "mjpython"`` resolves
            # to a mjpython path that ``is_file()``.
            mjpython_path = "/venv/bin/mjpython"

            def path_factory(arg):
                # Return a small stand-in object that supports the
                # operations the helper performs.
                class _StubPath:
                    def __init__(self, p):
                        self._p = p
                        self.name = Path(p).name
                    @property
                    def parent(self):
                        return _StubPath(str(Path(self._p).parent))
                    def __truediv__(self, other):
                        return _StubPath(str(Path(self._p) / other))
                    def is_file(self):
                        return self._p == mjpython_path
                    def __str__(self):
                        return self._p
                return _StubPath(arg)

            mock_path_cls.side_effect = path_factory
            cli._relaunch_under_mjpython_if_needed()

        self.assertEqual(recorded["path"], mjpython_path)
        self.assertEqual(
            recorded["argv"],
            [mjpython_path, "-m", "openbricks_sim",
             "preview", "--world", "wro-2026-elementary"],
            "expected the leading ``sim`` to be stripped before "
            "re-executing under mjpython")


class RunSubcommandTests(unittest.TestCase):
    def setUp(self):
        self.parser = cli._build_parser()

    def test_parser_run_required_args(self):
        args = self.parser.parse_args(["run", "/tmp/script.py"])
        self.assertEqual(args.command, "run")
        self.assertEqual(args.script, "/tmp/script.py")
        self.assertEqual(args.world, "empty")
        self.assertFalse(args.viewer)

    def test_parser_run_overrides(self):
        args = self.parser.parse_args([
            "run", "main.py",
            "--world", "wro-2026-junior",
            "--x", "1.5", "--y", "-0.2",
            "--viewer",
        ])
        self.assertEqual(args.script, "main.py")
        self.assertEqual(args.world, "wro-2026-junior")
        self.assertEqual(args.x, 1.5)
        self.assertEqual(args.y, -0.2)
        self.assertTrue(args.viewer)

    def test_run_executes_user_script(self):
        # Write a tiny script that drives a straight + asserts pose
        # changed; if the SimRobot wiring is broken, the script
        # raises and main() propagates the exception.
        import tempfile, textwrap, os
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".py", delete=False) as f:
            f.write(textwrap.dedent("""
                drivebase.straight(distance_mm=100.0, speed_mm_s=80.0)
                robot.run_for(0.5)
                x, y, yaw = robot.chassis_pose()
                assert x > 10.0, "expected +X movement, got x=%r" % x
            """))
            path = f.name
        try:
            rc = cli.main(["run", path, "--world", "empty"])
            self.assertEqual(rc, 0)
        finally:
            os.unlink(path)

    def test_run_propagates_user_script_systemexit_zero(self):
        # User code calling sys.exit(0) shouldn't make us crash.
        import tempfile, os
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".py", delete=False) as f:
            f.write("import sys\nsys.exit(0)\n")
            path = f.name
        try:
            rc = cli.main(["run", path, "--world", "empty"])
            self.assertEqual(rc, 0)
        finally:
            os.unlink(path)

    def test_run_with_viewer_calls_run_viewer_after_script(self):
        # ``--viewer`` should cause cmd_run to launch the viewer once
        # the user script returns. Patch ``SimRobot.run_viewer`` so
        # we don't actually open a window in CI.
        import tempfile, os
        from openbricks_sim.robot import SimRobot
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".py", delete=False) as f:
            f.write("robot.run_for(0.01)\n")
            path = f.name
        try:
            with patch.object(SimRobot, "run_viewer", autospec=True) as mock:
                rc = cli.main(["run", path, "--world", "empty", "--viewer"])
            self.assertEqual(rc, 0)
            mock.assert_called_once()
        finally:
            os.unlink(path)

    def test_run_no_shim_skips_shim_install(self):
        # With --no-shim, ``import machine`` should still fail in the
        # user script (shim not installed).
        import tempfile, os
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".py", delete=False) as f:
            f.write(
                "try:\n"
                "    import machine\n"
                "    raise SystemExit(1)\n"
                "except ImportError:\n"
                "    pass\n")
            path = f.name
        try:
            rc = cli.main(["run", path, "--world", "empty", "--no-shim"])
            self.assertEqual(rc, 0)
        finally:
            os.unlink(path)

    def test_run_with_shim_makes_machine_importable(self):
        # Default (no --no-shim) — ``import machine`` must succeed.
        import tempfile, os
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".py", delete=False) as f:
            f.write(
                "import machine\n"
                "p = machine.Pin(0)\n"
                "p.value(1)\n")
            path = f.name
        try:
            rc = cli.main(["run", path, "--world", "empty"])
            self.assertEqual(rc, 0)
        finally:
            os.unlink(path)

    def test_run_with_viewer_holds_after_systemexit(self):
        # User script calls sys.exit(); --viewer should still drop
        # the user into the viewer rather than dying immediately.
        import tempfile, os
        from openbricks_sim.robot import SimRobot
        with tempfile.NamedTemporaryFile(
                mode="w", suffix=".py", delete=False) as f:
            f.write("import sys\nsys.exit(0)\n")
            path = f.name
        try:
            with patch.object(SimRobot, "run_viewer", autospec=True) as mock:
                rc = cli.main(["run", path, "--world", "empty", "--viewer"])
            self.assertEqual(rc, 0)
            mock.assert_called_once()
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
