# SPDX-License-Identifier: MIT
"""Tests for the color-array demo's pure classify() logic
(examples/color_array.py)."""

import sys
import unittest

# Same import arrangement as tests/test_color_sorter.py: the example
# lives in examples/, resolved via a relative sys.path entry that works
# under both pytest and tests/run.py (both run from the repo root).
# color_array has no module-level hardware imports.
if "examples" not in sys.path:
    sys.path.insert(0, "examples")

import color_array  # noqa: E402

classify = color_array.classify


class TestClassify(unittest.TestCase):
    # --- the six colours, with typical clear-normalised readings ---

    def test_red(self):
        self.assertEqual(classify(30, (200, 45, 40)), "red")

    def test_green(self):
        self.assertEqual(classify(30, (55, 175, 70)), "green")

    def test_blue(self):
        self.assertEqual(classify(30, (45, 80, 190)), "blue")

    def test_yellow(self):
        # Red AND green both strong — the g/r ratio splits this from red.
        self.assertEqual(classify(40, (215, 200, 55)), "yellow")

    def test_white(self):
        self.assertEqual(classify(60, (240, 240, 230)), "white")

    def test_black(self):
        self.assertEqual(classify(5, (30, 30, 30)), "black")

    # --- ambient participates, not just rgb ---

    def test_dark_reading_is_black_regardless_of_hue(self):
        # Below the darkness floor the normalised hue is noise —
        # even a red-looking rgb must not classify as red.
        self.assertEqual(classify(4, (200, 45, 40)), "black")

    def test_dim_neutral_is_black_bright_neutral_is_white(self):
        grey = (150, 150, 145)
        self.assertEqual(classify(20, grey), "black")
        self.assertEqual(classify(50, grey), "white")

    # --- boundaries ---

    def test_yellow_red_boundary(self):
        # g exactly at the ratio threshold counts as yellow (>=);
        # just below is red.
        r = 200
        g_at = int(r * color_array.YELLOW_G_OVER_R)
        self.assertEqual(classify(40, (r, g_at, 20)), "yellow")
        self.assertEqual(classify(40, (r, g_at - 30, 20)), "red")

    def test_spread_boundary_neutral_vs_chromatic(self):
        # Spread just under NEUTRAL_SPREAD is achromatic; at/over is
        # judged by hue (blue-tinted here, so the chromatic side is
        # unambiguous).
        base = 150
        under = base + color_array.NEUTRAL_SPREAD - 1
        over  = base + color_array.NEUTRAL_SPREAD
        self.assertEqual(classify(60, (base, base, under)), "white")
        self.assertEqual(classify(60, (base, base, over)), "blue")

    def test_green_vs_blue_dominance(self):
        # Blue wins ties (checked first among chromatics).
        self.assertEqual(classify(30, (40, 100, 100)), "blue")
        self.assertEqual(classify(30, (40, 101, 100)), "green")


if __name__ == "__main__":
    unittest.main()
