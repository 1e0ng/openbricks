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
        # Pose flags default to None so a chassis file's pos_x /
        # pos_y / yaw_deg survive unless the flag is given; the spec
        # they resolve to is still the origin, facing +X.
        self.assertIsNone(args.x)
        self.assertIsNone(args.y)
        self.assertIsNone(args.yaw)
        self.assertIsNone(args.chassis)
        spec = cli._chassis_spec(args)
        self.assertEqual((spec.pos_x, spec.pos_y, spec.yaw_deg),
                         (0.0, 0.0, 0.0))
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

    def test_chassis_file_and_pose_flags(self):
        # A robot description file sets the geometry AND may carry
        # the spawn pose; a pose flag given on the command line wins
        # over the file's value for that field only.
        import json, os, tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "car.json")
            with open(path, "w") as f:
                json.dump({"wheel_radius": 0.0432, "axle_length": 0.135,
                           "line_sensor_x": 0.09,
                           "pos_x": -0.55, "pos_y": -0.15, "yaw_deg": 90},
                          f)
            args = self.parser.parse_args(
                ["run", "main.py", "--chassis", path, "--y", "0.25"])
            spec = cli._chassis_spec(args)
        self.assertEqual(spec.wheel_radius, 0.0432)
        self.assertEqual(spec.axle_length, 0.135)
        self.assertEqual(spec.line_sensor_x, 0.09)
        self.assertEqual((spec.pos_x, spec.pos_y, spec.yaw_deg),
                         (-0.55, 0.25, 90))
        self.assertEqual(spec.body_length, 0.140)   # untouched default

    def test_chassis_file_unknown_field_is_named(self):
        import os, tempfile
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "car.json")
            with open(path, "w") as f:
                f.write('{"wheel_diameter": 0.0864}')
            args = self.parser.parse_args(["preview", "--chassis", path])
            with self.assertRaises(SystemExit) as ctx:
                cli._chassis_spec(args)
        self.assertIn("wheel_diameter", str(ctx.exception))
        self.assertIn("wheel_radius", str(ctx.exception))

    def test_yaw_flag_parses(self):
        args = self.parser.parse_args(["run", "main.py", "--yaw", "-90"])
        self.assertEqual(args.yaw, -90.0)
        self.assertEqual(cli._chassis_spec(args).yaw_deg, -90.0)

    def test_missing_subcommand_exits(self):
        with self.assertRaises(SystemExit):
            with patch("sys.stderr", new_callable=io.StringIO):
                self.parser.parse_args([])

    def test_version_flag_prints_and_exits(self):
        # ``openbricks-sim --version`` exits 0 and writes the version
        # to stdout. Pinned because new users expect this and it's a
        # cheap "is the install live" sanity check.
        from openbricks_dev import __version__
        out = io.StringIO()
        with self.assertRaises(SystemExit) as ctx:
            with patch("sys.stdout", new=out):
                self.parser.parse_args(["--version"])
        self.assertEqual(ctx.exception.code, 0)
        self.assertIn(__version__, out.getvalue())


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


class RunGuardTests(unittest.TestCase):
    """``--trace`` records the path; ``--max-sim-s`` ends a runaway
    script with exit status 3."""

    def test_trace_records_the_pose_and_max_sim_s_stops_a_runaway(self):
        import os, tempfile
        script = (
            "import time\n"
            "from openbricks.drivers.st3032 import ST3032Motor\n"
            "from openbricks.robotics import DriveBase\n"
            "l = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)\n"
            "r = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41)\n"
            "db = DriveBase(l, r, wheel_diameter_mm=86.4, axle_track_mm=135)\n"
            "db.drive(100, 0)\n"
            "while True:\n"
            "    time.sleep_ms(10)\n"
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "runaway.py")
            with open(path, "w") as f:
                f.write(script)
            trace = os.path.join(tmp, "trace.csv")
            with patch("sys.stderr", new_callable=io.StringIO) as err:
                rc = cli.main(["run", path, "--world", "empty",
                               "--trace", trace, "--max-sim-s", "1.5"])
            self.assertEqual(rc, 3)
            self.assertIn("--max-sim-s 1.5 reached", err.getvalue())
            with open(trace) as f:
                rows = f.read().splitlines()
        self.assertEqual(rows[0], "t_ms,x_mm,y_mm,yaw_deg")
        # One row per 50 ms of the 1.5 s budget (+/- one tick), and
        # the drive(100, 0) shows as advancing x.
        self.assertTrue(28 <= len(rows) - 1 <= 32, len(rows))
        last = rows[-1].split(",")
        self.assertEqual(len(last), 4)
        self.assertGreater(float(last[1]), 60.0, rows[-1])

    def test_parser_defaults_off(self):
        args = cli._build_parser().parse_args(["run", "x.py"])
        self.assertIsNone(args.trace)
        self.assertIsNone(args.max_sim_s)


class ShippedExamplesTests(unittest.TestCase):
    """Smoke-test the ``examples/`` scripts that demonstrate the
    sim. They're user-facing tutorials; if one starts crashing,
    a docs-link will point new users at broken code."""

    def test_wro_elementary_walkthrough_runs(self):
        # The walkthrough teleport-free version: queries the model
        # for randomized note positions, prints a route plan. Run
        # against the real Elementary world + a fixed seed, and
        # assert the CLI exits 0.
        import os
        from pathlib import Path
        example = (Path(__file__).resolve().parent.parent
                   / "examples" / "wro_elementary_walkthrough.py")
        self.assertTrue(example.is_file(),
                        "shipped example missing: %s" % example)
        rc = cli.main([
            "run", str(example),
            "--world", "wro-2026-elementary",
            "--seed", "42",
            "--no-shim",
        ])
        self.assertEqual(rc, 0)


if __name__ == "__main__":
    unittest.main()
