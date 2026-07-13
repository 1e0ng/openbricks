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
        self.gain = self.ns["GAIN"]
        self.thr = self.ns["LINE_AMBIENT"]

    def test_centred_drives_straight(self):
        # Equal readings (any brightness above the line threshold):
        # zero error, both wheels at cruise.
        self.assertEqual(self.speeds(90, 90), (self.cruise, self.cruise))
        self.assertEqual(self.speeds(40, 40), (self.cruise, self.cruise))

    def test_drift_right_slows_left_wheel_proportionally(self):
        # Line sliding under the LEFT sensor drops its reading. Error
        # small enough that neither wheel hits the clamp, so the pure
        # proportional math is visible.
        left, right = self.speeds(80, 90)
        self.assertEqual(left, int(self.cruise + self.gain * (80 - 90)))
        self.assertEqual(right, int(self.cruise - self.gain * (80 - 90)))
        self.assertTrue(left < self.cruise < right)

    def test_drift_left_mirrors(self):
        l1, r1 = self.speeds(80, 90)
        l2, r2 = self.speeds(90, 80)
        self.assertEqual((l1, r1), (r2, l2))

    def test_small_error_gives_small_correction(self):
        big = self.speeds(60, 90)
        small = self.speeds(85, 90)
        self.assertTrue(
            abs(small[0] - small[1]) < abs(big[0] - big[1]),
            "correction must scale with the error")

    def test_large_error_clamps_to_pivot_not_reverse(self):
        # Massive difference: inner wheel clamps at 0 (pivot), never
        # negative, and the outer wheel never exceeds the bench cap.
        left, right = self.speeds(2, 95)   # left nearly on the line
        self.assertEqual(left, 0)
        self.assertTrue(right <= 120)

    def test_both_dark_means_intersection_stop(self):
        self.assertIsNone(self.speeds(self.thr - 1, self.thr - 1))

    def test_one_dark_is_steering_not_stop(self):
        # Only BOTH sensors below threshold is an intersection; one
        # dark sensor is just a big steering error.
        # assertTrue, not assertIsNotNone: MP's unittest formats the
        # failure message with '%' and a tuple value splats into it.
        self.assertTrue(self.speeds(self.thr - 1, 90) is not None)

    def test_speeds_respect_bench_cap(self):
        for la in range(0, 101, 10):
            for ra in range(0, 101, 10):
                speeds = self.speeds(la, ra)
                if speeds is None:
                    continue
                self.assertTrue(0 <= speeds[0] <= 120, speeds)
                self.assertTrue(0 <= speeds[1] <= 120, speeds)

    def test_line_ambient_threshold_is_sane(self):
        self.assertTrue(0 < self.thr < 30, self.thr)


if __name__ == "__main__":
    unittest.main()
