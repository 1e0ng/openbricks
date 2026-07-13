# SPDX-License-Identifier: MIT
"""Unit tests for the line-follow example's control law.

The example wires real hardware at module level (the user-preferred
example style), so it can't be imported here. Instead the pure
control-law block is extracted by its markers and exec'd — the same
plain string scanning test_example_pins uses (works under both
CPython and unix MicroPython; no ``ast``, no ``re``).
"""

import tests._fakes  # noqa: F401

import unittest


_here = __file__
_idx = _here.rfind("/")
_EXAMPLE = (_here[:_idx] if _idx >= 0 else ".") + "/../examples/line_follow.py"

_BEGIN = "# --- control law"
_END = "# --- end control law ---"


def _load_control_law():
    with open(_EXAMPLE) as f:
        src = f.read()
    start = src.index(_BEGIN)
    end = src.index(_END)
    ns = {}
    exec(src[start:end], ns)
    return ns


class ControlLawTests(unittest.TestCase):
    def setUp(self):
        self.ns = _load_control_law()
        self.speeds = self.ns["_wheel_speeds"]
        self.cruise = self.ns["CRUISE_DPS"]
        self.turn = self.ns["TURN_DPS"]

    def test_centred_drives_straight(self):
        self.assertEqual(self.speeds(False, False),
                         (self.cruise, self.cruise))

    def test_line_under_left_sensor_slows_left_wheel(self):
        self.assertEqual(self.speeds(True, False),
                         (self.turn, self.cruise))

    def test_line_under_right_sensor_slows_right_wheel(self):
        self.assertEqual(self.speeds(False, True),
                         (self.cruise, self.turn))

    def test_both_dark_means_intersection_stop(self):
        self.assertIsNone(self.speeds(True, True))

    def test_speeds_respect_bench_cap(self):
        # Bench convention: run_speed <= 120 dps in example/test code.
        self.assertTrue(0 < self.turn < self.cruise <= 120)

    def test_line_ambient_threshold_is_sane(self):
        # Must sit between "matte black line" and "typical mat".
        thr = self.ns["LINE_AMBIENT"]
        self.assertTrue(0 < thr < 30, thr)


if __name__ == "__main__":
    unittest.main()
