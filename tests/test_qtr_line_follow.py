# SPDX-License-Identifier: MIT
"""The QTR line-follow control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_line_follow.py``: the
example wires hardware at module level, so the pure control-law
block is pulled out by its markers. What is pinned here is the
CONTRACT the bench relies on: sign conventions, the immediate
whole-array-AND-flag stop, always-steer-on-the-left-edge, and the
clamp. Stateless pure P since 1.69.2 — no derivative, no dt, no
lost-line handling (nothing dark raises, visibly).
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

    def _tick(self, left, branch=False, all_dark=False):
        reading = _Reading(left=left, all_dark=all_dark)
        return self.ns["_p_wheel_speeds"](reading, branch)

    def _cruise(self):
        c = self.ns["CRUISE_DPS"]
        return (c, c)

    def test_centred_line_drives_straight(self):
        self.assertEqual(self._tick(0.0), self._cruise())

    def test_line_right_steers_right(self):
        # +position = line right of centre -> left wheel faster.
        l, r = self._tick(+10.0)
        self.assertTrue(l > r, (l, r))
        l, r = self._tick(-10.0)
        self.assertTrue(l < r, (l, r))

    def test_steering_ignores_the_branch_flag(self):
        # The flag only gates the stop: with or without it, the same
        # left edge gives the same speeds.
        self.assertEqual(self._tick(+10.0, branch=True),
                         self._tick(+10.0, branch=False))

    def test_intersection_stops_immediately(self):
        # Whole array dark AND flag dark in one snapshot: stop NOW —
        # no debounce, no streak.
        self.assertIsNone(self._tick(+2.0, branch=True, all_dark=True))

    def test_branch_alone_never_stops(self):
        # assertTrue, not assertIsNotNone: MP's unittest %-formats
        # the default message and a TUPLE value spreads its args.
        self.assertTrue(self._tick(0.0, branch=True) is not None)

    def test_all_dark_alone_never_stops(self):
        self.assertTrue(self._tick(0.0, all_dark=True) is not None)

    def test_lost_line_raises_visibly(self):
        # Deliberately unhandled: nothing dark means the rig or the
        # track is wrong, and a loud TypeError beats driving blind.
        try:
            self._tick(None)
            self.fail("expected TypeError")
        except TypeError:
            pass

    def test_clamp_never_reverses_a_wheel(self):
        l, r = self._tick(+1000.0)
        self.assertEqual(r, 0)                    # clamped at zero
        self.assertEqual(l, self.ns["MAX_DPS"])


if __name__ == "__main__":
    unittest.main()
