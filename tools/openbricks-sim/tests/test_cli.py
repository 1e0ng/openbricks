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


if __name__ == "__main__":
    unittest.main()
