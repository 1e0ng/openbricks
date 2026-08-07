# SPDX-License-Identifier: MIT
"""The QTR line-follow control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_line_follow.py``: the
example wires hardware at module level, so the pure control-law
block is pulled out by its markers. What is pinned here is the
CONTRACT the bench relies on: sign conventions, the immediate
whole-array-AND-flag stop, always-steer-on-the-left-edge,
real-dt derivative scaling, hold-on-lost-line, and the clamp.
"""

import tests._fakes  # noqa: F401

import unittest


def _load():
    with open("examples/qtr_line_follow.py") as f:
        src = f.read()
    begin = "# --- control law"
    end = "# --- end control law ---"
    if begin not in src or end not in src:
        raise AssertionError(
            "control-law markers not found in examples/"
            "qtr_line_follow.py — they are load-bearing here")
    ns = {}
    exec(src[src.index(begin):src.index(end)], ns)
    return ns


class _Reading:
    """``QTRReading`` stand-in: the law consumes exactly these two
    methods of the snapshot, nothing else."""

    def __init__(self, left=None, all_dark=False):
        self._left = left
        self._all = all_dark

    def left_edge_position(self):
        return self._left

    def all_dark(self):
        return self._all


class QTRLawTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ns = _load()

    def _tick(self, left, branch=False, all_dark=False, prev=None,
              dt=0.01):
        reading = _Reading(left=left, all_dark=all_dark)
        return self.ns["_pd_wheel_speeds"](reading, branch, prev, dt)

    def _cruise(self):
        c = self.ns["CRUISE_DPS"]
        return (c, c)

    def test_centred_line_drives_straight(self):
        speeds, _ = self._tick(0.0)
        self.assertEqual(speeds, self._cruise())

    def test_line_right_steers_right(self):
        # +position = line right of centre -> left wheel faster.
        (l, r), _ = self._tick(+10.0)
        self.assertTrue(l > r, (l, r))
        (l, r), _ = self._tick(-10.0)
        self.assertTrue(l < r, (l, r))

    def test_steering_ignores_the_branch_flag(self):
        # The flag only gates the stop: with or without it, the same
        # leftmost cluster gives the same speeds.
        with_flag, _ = self._tick(+10.0, branch=True)
        without, _ = self._tick(+10.0, branch=False)
        self.assertEqual(with_flag, without)

    def test_derivative_damps_a_closing_error(self):
        # Error shrinking fast: KD subtracts from the P steering.
        (l_p, r_p), _ = self._tick(+10.0)              # no history
        (l_d, r_d), _ = self._tick(+10.0, prev=20.0)   # was worse
        self.assertTrue(l_d - r_d < l_p - r_p,
                        ((l_p, r_p), (l_d, r_d)))

    def test_derivative_uses_the_measured_dt(self):
        # The same error change over twice the time is HALF the
        # rate: the slow tick must damp less. This is what pins
        # "dt is measured, not assumed" — with a hardcoded dt both
        # calls would return identical speeds.
        (l_fast, r_fast), _ = self._tick(+10.0, prev=20.0, dt=0.01)
        (l_slow, r_slow), _ = self._tick(+10.0, prev=20.0, dt=0.02)
        self.assertTrue(l_slow - r_slow > l_fast - r_fast,
                        ((l_fast, r_fast), (l_slow, r_slow)))

    def test_zero_dt_skips_the_derivative(self):
        # First loop iteration can measure dt == 0; pure P, no
        # ZeroDivisionError.
        (l, r), _ = self._tick(+10.0, prev=20.0, dt=0)
        (l_p, r_p), _ = self._tick(+10.0)
        self.assertEqual((l, r), (l_p, r_p))

    def test_intersection_stops_immediately(self):
        # Whole array dark AND flag dark in one snapshot: stop NOW —
        # no debounce, no streak.
        speeds, _ = self._tick(+2.0, branch=True, all_dark=True)
        self.assertIsNone(speeds)

    def test_branch_alone_never_stops(self):
        # assertTrue, not assertIsNotNone: MP's unittest %-formats
        # the default message and a TUPLE value spreads its args.
        speeds, _ = self._tick(0.0, branch=True, all_dark=False)
        self.assertTrue(speeds is not None)

    def test_all_dark_alone_never_stops(self):
        speeds, _ = self._tick(0.0, branch=False, all_dark=True)
        self.assertTrue(speeds is not None)

    def test_lost_line_holds_the_previous_correction(self):
        # Nothing dark: keep steering as before — no recovery mode.
        (l, r), err = self._tick(None, prev=+10.0)
        self.assertTrue(l > r, (l, r))
        self.assertEqual(err, 10.0)
        (l, r), err = self._tick(None, prev=-10.0)
        self.assertTrue(l < r, (l, r))

    def test_lost_line_with_no_history_drives_straight(self):
        speeds, err = self._tick(None)
        self.assertEqual(speeds, self._cruise())
        self.assertEqual(err, 0.0)

    def test_lost_line_during_branch_holds_too(self):
        (l, r), err = self._tick(None, branch=True, prev=+10.0)
        self.assertTrue(l > r, (l, r))
        self.assertEqual(err, 10.0)

    def test_clamp_never_reverses_a_wheel(self):
        (l, r), _ = self._tick(+1000.0)
        self.assertEqual(r, 0)                    # clamped at zero
        self.assertEqual(l, self.ns["MAX_DPS"])

    def test_returned_error_threads_the_state(self):
        _, err = self._tick(+7.5)
        self.assertEqual(err, 7.5)


if __name__ == "__main__":
    unittest.main()
