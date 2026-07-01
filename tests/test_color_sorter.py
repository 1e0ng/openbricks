# SPDX-License-Identifier: MIT
"""Tests for the color-sorter demo's pure nearest-match logic
(examples/color_sorter.py)."""

import sys
import unittest

# The example lives in examples/, a sibling of the openbricks package
# rather than an importable subpackage. Both test runners (pytest and
# tests/run.py) execute with the repo root as the working directory,
# so a relative "examples" entry resolves under both CPython and the
# unix MicroPython port. color_sorter has no module-level hardware
# imports, so importing it needs no fakes.
if "examples" not in sys.path:
    sys.path.insert(0, "examples")

import color_sorter  # noqa: E402


class TestNearestColor(unittest.TestCase):
    def test_exact_palette_hit_returns_that_name(self):
        for name, rgb in color_sorter.PALETTE:
            self.assertEqual(color_sorter.nearest_color(rgb), name)

    def test_noisy_red_still_reads_red(self):
        self.assertEqual(color_sorter.nearest_color((210, 60, 55)), "red")

    def test_bright_neutral_reads_white_not_yellow(self):
        # Bright and roughly neutral is closer to white than to the
        # warm-tinted yellow reference.
        self.assertEqual(color_sorter.nearest_color((235, 235, 230)), "white")

    def test_tie_resolves_to_earlier_entry(self):
        # (5,0,0) is equidistant from (0,0,0) and (10,0,0); the earlier
        # entry wins.
        palette = (("a", (0, 0, 0)), ("b", (10, 0, 0)))
        self.assertEqual(color_sorter.nearest_color((5, 0, 0), palette), "a")

    def test_custom_palette_is_respected(self):
        palette = (("dark", (0, 0, 0)), ("bright", (255, 255, 255)))
        self.assertEqual(
            color_sorter.nearest_color((200, 200, 200), palette), "bright")


if __name__ == "__main__":
    unittest.main()
