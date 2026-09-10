# SPDX-License-Identifier: MIT
"""The QTR line-follow control laws, arithmetic only.

Same extract-and-exec trick as ``tests/test_line_follow.py``: the
examples wire hardware at module level, so the pure control-law
block is pulled out by its markers. Three example files carry
three laws built from element readings: the right and left edge
followers steer on ONE element's ``ambient()`` (half grey is the
setpoint) and watch the mat-side elements beyond the edge for a
branch; the centre follower steers on ``position()`` and keeps its
own last-seen side for when the line leaves the window. The whole
window dark ends every run.
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


class _Element:
    def __init__(self, ambient):
        self._ambient = ambient

    def ambient(self):
        return self._ambient

    def dark(self):
        return self._ambient < 50


class _Reading:
    """``QTRReading`` stand-in: ten elements, each with a Pybricks
    scale ``ambient()`` (0 black .. 100 white), ``all_dark()`` and
    the centroid ``position()`` the centre law consumes."""

    def __init__(self, ambients=None, dark_flags=(), all_dark=False,
                 position=None):
        if ambients is None:
            ambients = [100] * 10
        for i in dark_flags:
            ambients[i] = 0
        if all_dark:
            ambients = [0] * 10
        self.elements = [_Element(a) for a in ambients]
        self._position = position

    def all_dark(self):
        for e in self.elements:
            if not e.dark():
                return False
        return True

    def position(self):
        return self._position


class _Common:
    """What every follower shares: the intersection stop, the clamp
    and the marker block loading."""

    EXAMPLE = None

    @classmethod
    def setUpClass(cls):
        cls.ns = _load(cls.EXAMPLE)

    def _cruise(self):
        c = self.ns["CRUISE_DPS"]
        return (c, c)

    def test_no_mode_api(self):
        with open(self.EXAMPLE) as f:
            src = f.read()
        for gone in ("LineMode", "set_mode", "edge_error", "last_side()"):
            self.assertFalse(gone in src, gone)

    def test_full_window_dark_stops_immediately(self):
        r = _Reading(all_dark=True, position=0.0)
        self.assertIsNone(self.ns["get_wheel_speeds"](r))

    def test_branch_seen_takes_only_the_reading(self):
        self.assertFalse(self.ns["branch_seen"](_Reading(position=0.0)))


class _EdgeContract(_Common):
    """The two edge followers: mirror images around EDGE_INDEX."""

    EDGE_INDEX = None
    FAR_SIDE = ()           # the branch watch indices
    NEAR_SIDE = ()          # dark here is the line itself, not a branch

    def _tick(self, ambient):
        amb = [100] * 10
        amb[self.EDGE_INDEX] = ambient
        return self.ns["get_wheel_speeds"](_Reading(amb))

    def test_edge_index_constant(self):
        self.assertEqual(self.ns["EDGE_INDEX"], self.EDGE_INDEX)

    def test_half_grey_drives_straight(self):
        self.assertEqual(self._tick(50), self._cruise())

    def test_clamp_never_reverses_a_wheel(self):
        for ambient in (0, 100):
            l, r = self._tick(ambient)
            self.assertTrue(l >= 0 and r >= 0, (l, r))
            self.assertTrue(l <= self.ns["MAX_DPS"], l)
            self.assertTrue(r <= self.ns["MAX_DPS"], r)

    def test_rail_saturates_one_wheel_at_zero(self):
        # KP * 50 well past the cruise headroom: the inside wheel
        # stops, the outside wheel pins at MAX_DPS.
        stalled = sorted(self._tick(0) + self._tick(100))
        self.assertEqual(stalled[0], 0)
        self.assertEqual(stalled[-1], self.ns["MAX_DPS"])

    def test_branch_watch_is_the_far_side(self):
        branch_seen = self.ns["branch_seen"]
        for i in self.FAR_SIDE:
            self.assertTrue(branch_seen(_Reading(dark_flags=(i,))), i)
        for i in self.NEAR_SIDE:
            self.assertFalse(branch_seen(_Reading(dark_flags=(i,))), i)
        self.assertFalse(branch_seen(_Reading(dark_flags=(self.EDGE_INDEX,))))
        self.assertFalse(branch_seen(_Reading()))


class RightFollowerTests(_EdgeContract, unittest.TestCase):
    EXAMPLE = "examples/qtr_line_follow_right.py"
    EDGE_INDEX = 7
    FAR_SIDE = (8, 9)
    NEAR_SIDE = (0, 1, 2, 3, 4, 5, 6)

    def test_darker_edge_element_steers_right(self):
        # Robot drifting LEFT puts more line under element 7: the
        # left wheel speeds up, the right wheel slows, turning right.
        l, r = self.ns["get_wheel_speeds"](
            _Reading([100] * 7 + [40, 100, 100]))
        self.assertTrue(l > r, (l, r))
        l, r = self.ns["get_wheel_speeds"](
            _Reading([100] * 7 + [60, 100, 100]))
        self.assertTrue(l < r, (l, r))

    def test_steer_is_proportional_to_the_edge_element(self):
        kp = self.ns["KP"]
        c = self.ns["CRUISE_DPS"]
        self.assertEqual(self._tick(40), (int(c + kp * 10), int(c - kp * 10)))


class LeftFollowerTests(_EdgeContract, unittest.TestCase):
    EXAMPLE = "examples/qtr_line_follow_left.py"
    EDGE_INDEX = 2
    FAR_SIDE = (0, 1)
    NEAR_SIDE = (3, 4, 5, 6, 7, 8, 9)

    def test_darker_edge_element_steers_left(self):
        # Robot drifting RIGHT puts more line under element 2: the
        # right wheel speeds up, the left wheel slows, turning left.
        l, r = self.ns["get_wheel_speeds"](
            _Reading([100, 100, 40] + [100] * 7))
        self.assertTrue(l < r, (l, r))
        l, r = self.ns["get_wheel_speeds"](
            _Reading([100, 100, 60] + [100] * 7))
        self.assertTrue(l > r, (l, r))

    def test_steer_is_proportional_to_the_edge_element(self):
        kp = self.ns["KP"]
        c = self.ns["CRUISE_DPS"]
        self.assertEqual(self._tick(40), (int(c - kp * 10), int(c + kp * 10)))


class CenterFollowerTests(_Common, unittest.TestCase):
    EXAMPLE = "examples/qtr_line_follow_center.py"

    def _tick(self, position, dark_flags=(4, 5)):
        return self.ns["get_wheel_speeds"](
            _Reading(dark_flags=dark_flags, position=position))

    def test_centred_line_drives_straight(self):
        self.assertEqual(self._tick(0.0), self._cruise())

    def test_line_right_of_centre_steers_right(self):
        l, r = self._tick(+8.0)
        self.assertTrue(l > r, (l, r))
        l, r = self._tick(-8.0)
        self.assertTrue(l < r, (l, r))

    def test_steer_is_proportional_to_position(self):
        kp = self.ns["KP_MM"]
        c = self.ns["CRUISE_DPS"]
        self.assertEqual(self._tick(4.0), (int(c + kp * 4), int(c - kp * 4)))

    def test_clamp_never_reverses_a_wheel(self):
        l, r = self._tick(+1000.0)
        self.assertEqual((l, r), (self.ns["MAX_DPS"], 0))
        l, r = self._tick(-1000.0)
        self.assertEqual((l, r), (0, self.ns["MAX_DPS"]))

    def test_lost_line_turns_toward_the_last_seen_side(self):
        # Seen on the right, then gone: keep turning right.
        self._tick(+12.0)
        l, r = self._tick(None, dark_flags=())
        self.assertTrue(l > r, (l, r))
        # Seen on the left, then gone: keep turning left.
        self._tick(-12.0)
        l, r = self._tick(None, dark_flags=())
        self.assertTrue(l < r, (l, r))
        # A centred sighting does not flip the remembered side.
        self._tick(0.0)
        l, r = self._tick(None, dark_flags=())
        self.assertTrue(l < r, (l, r))

    def test_lost_line_rails_the_steer(self):
        self._tick(+12.0)
        l, r = self._tick(None, dark_flags=())
        self.assertEqual((l, r), (self.ns["MAX_DPS"], 0))

    def test_branch_watch_is_both_outer_bands(self):
        branch_seen = self.ns["branch_seen"]
        n = self.ns["FLAG_COUNT"]
        for i in tuple(range(n)) + tuple(range(10 - n, 10)):
            self.assertTrue(branch_seen(_Reading(dark_flags=(i,))), i)
        for i in range(n, 10 - n):
            self.assertFalse(branch_seen(_Reading(dark_flags=(i,))), i)
        self.assertFalse(branch_seen(_Reading()))


if __name__ == "__main__":
    unittest.main()
