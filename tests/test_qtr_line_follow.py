# SPDX-License-Identifier: MIT
"""The QTR line-follow control laws, arithmetic only.

Same extract-and-exec trick as ``tests/test_line_follow.py``: the
examples wire hardware at module level, so the pure control-law
block is pulled out by its markers. TWO mirrored examples share one
contract (the P law is symmetric between edge disciplines): sign
conventions, the immediate all-dark-AND-flag stop, and the clamp —
plus a per-file pin that each steers on ITS OWN edge (left file =
left edge, right file = right edge).
"""

import tests._fakes  # noqa: F401

import unittest


def _load(path):
    with open(path) as f:
        src = f.read()
    begin = "# --- control law"
    end = "# --- end control law ---"
    if begin not in src or end not in src:
        raise AssertionError(
            "control-law markers not found in %s — they are "
            "load-bearing here" % path)
    ns = {}
    exec(src[src.index(begin):src.index(end)], ns)
    return ns


class _Reading:
    """``QTRReading`` stand-in: the laws consume exactly these three
    methods. Distinct left/right edge values let a test prove which
    edge a law actually steers on."""

    def __init__(self, left=None, right=None, all_dark=False):
        self._left = left
        self._right = right
        self._all = all_dark

    def left_edge_position(self):
        return self._left

    def right_edge_position(self):
        return self._right

    def all_dark(self):
        return self._all


class _LawContract:
    """The shared battery — run against each example's extracted law.
    ``_tick`` feeds the SAME value to both edge methods, so the
    assertions hold for either discipline."""

    EXAMPLE = None          # subclasses set the path

    @classmethod
    def setUpClass(cls):
        cls.ns = _load(cls.EXAMPLE)

    def _tick(self, edge, branch=False, all_dark=False):
        reading = _Reading(left=edge, right=edge, all_dark=all_dark)
        return self.ns["get_wheel_speeds"](reading, branch)

    def _cruise(self):
        c = self.ns["CRUISE_DPS"]
        return (c, c)

    def test_centred_edge_drives_straight(self):
        self.assertEqual(self._tick(0.0), self._cruise())

    def test_edge_right_steers_right(self):
        # +edge = boundary right of centre -> left wheel faster.
        l, r = self._tick(+10.0)
        self.assertTrue(l > r, (l, r))
        l, r = self._tick(-10.0)
        self.assertTrue(l < r, (l, r))

    def test_steering_ignores_the_branch_flag(self):
        self.assertEqual(self._tick(+10.0, branch=True),
                         self._tick(+10.0, branch=False))

    def test_intersection_stops_immediately(self):
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


class LeftFollowerTests(_LawContract, unittest.TestCase):
    EXAMPLE = "examples/qtr_line_follow_left.py"

    def test_steers_on_the_left_edge_only(self):
        # Distinct edge values: only the LEFT edge may move the
        # wheels for this discipline.
        reading = _Reading(left=+10.0, right=-99.0)
        l, r = self.ns["get_wheel_speeds"](reading, False)
        self.assertTrue(l > r, (l, r))


class RightFollowerTests(_LawContract, unittest.TestCase):
    EXAMPLE = "examples/qtr_line_follow_right.py"

    def test_steers_on_the_right_edge_only(self):
        reading = _Reading(left=-99.0, right=+10.0)
        l, r = self.ns["get_wheel_speeds"](reading, False)
        self.assertTrue(l > r, (l, r))


if __name__ == "__main__":
    unittest.main()
