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
