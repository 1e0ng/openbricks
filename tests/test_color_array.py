# SPDX-License-Identifier: MIT
"""Tests for the color-array demo's pure line_position logic
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

line_position = color_array.line_position


class TestLinePosition(unittest.TestCase):
    def test_line_under_left_sensor(self):
        # Left dark, others at/above floor -> full left.
        self.assertEqual(line_position([10, 90, 90], floor=60), -1.0)

    def test_line_under_right_sensor(self):
        self.assertEqual(line_position([90, 90, 10], floor=60), 1.0)

    def test_line_centered(self):
        self.assertEqual(line_position([90, 10, 90], floor=60), 0.0)

    def test_symmetric_readings_balance_to_center(self):
        # Two equally-dark outer sensors cancel out.
        self.assertEqual(line_position([10, 90, 10], floor=60), 0.0)

    def test_line_between_two_sensors(self):
        # Line half under mid, half under right: centroid sits between
        # their positions (0 and +1), weighted by equal darkness.
        pos = line_position([90, 30, 30], floor=60)
        self.assertEqual(pos, 0.5)

    def test_darker_sensor_pulls_harder(self):
        # Right much darker than mid -> position closer to +1 than 0.5.
        pos = line_position([90, 50, 10], floor=60)
        self.assertGreater(pos, 0.5)
        self.assertLess(pos, 1.0)

    def test_no_line_returns_none(self):
        self.assertIsNone(line_position([90, 90, 90], floor=60))

    def test_reading_at_floor_contributes_nothing(self):
        # Exactly-at-floor is "floor", not "line".
        self.assertIsNone(line_position([60, 60, 60], floor=60))

    def test_two_sensor_array(self):
        self.assertEqual(line_position([10, 90], floor=60), -1.0)
        self.assertEqual(line_position([90, 10], floor=60), 1.0)

    def test_five_sensor_array_positions_spread_evenly(self):
        # Only sensor index 1 of 5 is dark -> its position is -0.5.
        self.assertEqual(
            line_position([90, 10, 90, 90, 90], floor=60), -0.5)

    def test_single_sensor_raises(self):
        with self.assertRaises(ValueError):
            line_position([10], floor=60)


if __name__ == "__main__":
    unittest.main()
