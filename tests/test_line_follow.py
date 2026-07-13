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
        self.max_dps = self.ns["MAX_DPS"]

    def test_centred_drives_straight(self):
        self.assertEqual(self.speeds(90, 90), (self.cruise, self.cruise))

    def test_drift_steers_proportionally_and_mirrored(self):
        l1, r1 = self.speeds(80, 90)
        self.assertEqual(l1, int(self.cruise + self.gain * (80 - 90)))
        self.assertEqual(r1, int(self.cruise - self.gain * (80 - 90)))
        self.assertTrue(l1 < self.cruise < r1)
        l2, r2 = self.speeds(90, 80)
        self.assertEqual((l1, r1), (r2, l2))

    def test_small_error_gives_small_correction(self):
        big = self.speeds(self.thr + 5, 90)
        small = self.speeds(85, 90)
        self.assertTrue(
            abs(small[0] - small[1]) < abs(big[0] - big[1]),
            "correction must scale with the error")

    def test_both_dark_stops(self):
        self.assertIsNone(self.speeds(self.thr - 1, self.thr - 1))

    def test_left_branch_is_ignored_not_followed(self):
        # A branch sweeping under ONE sensor must NOT steer — hold
        # course straight until it passes. (Steering toward it is how
        # the robot peels off onto the branch.)
        self.assertEqual(self.speeds(self.thr - 1, 90),
                         (self.cruise, self.cruise))

    def test_right_branch_is_ignored_not_followed(self):
        self.assertEqual(self.speeds(90, self.thr - 1),
                         (self.cruise, self.cruise))

    def test_branch_boundary_is_exactly_the_threshold(self):
        # AT the threshold the reading is mat, not line: steering.
        speeds = self.speeds(self.thr, 90)
        self.assertTrue(speeds[0] != speeds[1],
                        "threshold reading must steer, not ignore")

    def test_clamp_never_reverses_never_exceeds_cap(self):
        for la in range(0, 101, 10):
            for ra in range(0, 101, 10):
                speeds = self.speeds(la, ra)
                if speeds is None:
                    continue
                self.assertTrue(0 <= speeds[0] <= self.max_dps, speeds)
                self.assertTrue(0 <= speeds[1] <= self.max_dps, speeds)

    def test_clamp_cap_leaves_steering_headroom(self):
        # The cap must sit above cruise + max correction, or big
        # corrections pin both wheels at the cap and steering dies
        # (the CRUISE_DPS=200 vs cap=120 incident).
        self.assertTrue(
            self.max_dps >= self.cruise + self.gain * 100,
            "MAX_DPS=%s leaves no headroom over CRUISE_DPS=%s"
            % (self.max_dps, self.cruise))

    def test_debug_off_for_real_runs(self):
        # The per-tick calibration print stretches the control tick;
        # it must ship disabled.
        self.assertFalse(self.ns["DEBUG"])


if __name__ == "__main__":
    unittest.main()
