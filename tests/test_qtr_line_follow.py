# SPDX-License-Identifier: MIT
"""The QTR line-follow control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_line_follow.py``: the
example wires hardware at module level, so the pure control-law
block is pulled out by its markers. ONE law, TWO switchable modes:
"left" holds the line's LEFT edge at LEFT_SETPOINT_MM (channel 12),
"right" holds the RIGHT edge at RIGHT_SETPOINT_MM (channel 4). The
whole window dark ends the run; the far-side FLAG_COUNT elements
are the branch watch.
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


class _Element:
    def __init__(self, dark):
        self._dark = dark

    def dark(self):
        return self._dark


class _Reading:
    """``QTRReading`` stand-in: the law consumes the two edge
    methods, ``all_dark()``, and ``elements`` (the branch watch)."""

    def __init__(self, left=None, right=None, all_dark=False,
                 dark_flags=()):
        self._left = left
        self._right = right
        self._all = all_dark
        # 10 elements; the indices in dark_flags read dark.
        self.elements = [_Element(i in dark_flags) for i in range(10)]

    def left_edge_position(self):
        return self._left

    def right_edge_position(self):
        return self._right

    def all_dark(self):
        return self._all


class QTRLawTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ns = _load()

    def _tick(self, edge, mode="left", all_dark=False):
        # The mode's setpoint is added so ``edge`` is the ERROR the
        # test reasons about: edge=0 means "on the setpoint".
        sp = (self.ns["LEFT_SETPOINT_MM"] if mode == "left"
              else self.ns["RIGHT_SETPOINT_MM"])
        val = None if edge is None else edge + sp
        reading = _Reading(left=val, right=val, all_dark=all_dark)
        return self.ns["get_wheel_speeds"](reading, mode)

    def _cruise(self):
        c = self.ns["CRUISE_DPS"]
        return (c, c)

    def test_on_setpoint_drives_straight_both_modes(self):
        self.assertEqual(self._tick(0.0, "left"), self._cruise())
        self.assertEqual(self._tick(0.0, "right"), self._cruise())

    def test_edge_right_of_setpoint_steers_right_both_modes(self):
        for mode in ("left", "right"):
            l, r = self._tick(+10.0, mode)
            self.assertTrue(l > r, (mode, l, r))
            l, r = self._tick(-10.0, mode)
            self.assertTrue(l < r, (mode, l, r))

    def test_left_mode_steers_on_the_left_edge_only(self):
        sp = self.ns["LEFT_SETPOINT_MM"]
        reading = _Reading(left=sp + 10.0, right=-99.0)
        l, r = self.ns["get_wheel_speeds"](reading, "left")
        self.assertTrue(l > r, (l, r))

    def test_right_mode_steers_on_the_right_edge_only(self):
        sp = self.ns["RIGHT_SETPOINT_MM"]
        reading = _Reading(left=-99.0, right=sp + 10.0)
        l, r = self.ns["get_wheel_speeds"](reading, "right")
        self.assertTrue(l > r, (l, r))

    def test_setpoints_are_the_named_channels(self):
        # Channel 12 sits at -16 mm and channel 4 at +16 mm in the
        # centred 56 mm window — the modes' whole meaning.
        self.assertEqual(self.ns["LEFT_SETPOINT_MM"], -16.0)
        self.assertEqual(self.ns["RIGHT_SETPOINT_MM"], 16.0)

    def test_full_window_dark_stops_immediately(self):
        self.assertIsNone(self._tick(0.0, "left", all_dark=True))
        self.assertIsNone(self._tick(0.0, "right", all_dark=True))

    def test_unknown_mode_raises(self):
        reading = _Reading(left=0.0, right=0.0)
        try:
            self.ns["get_wheel_speeds"](reading, "center")
            self.fail("expected ValueError")
        except ValueError as e:
            self.assertTrue("left" in str(e) and "right" in str(e), e)

    def test_lost_line_raises_visibly(self):
        # Deliberately unhandled: nothing dark means the rig or the
        # track is wrong, and a loud TypeError beats driving blind.
        try:
            self._tick(None, "left")
            self.fail("expected TypeError")
        except TypeError:
            pass

    def test_clamp_never_reverses_a_wheel(self):
        l, r = self._tick(+1000.0, "left")
        self.assertEqual(r, 0)                    # clamped at zero
        self.assertEqual(l, self.ns["MAX_DPS"])

    def test_branch_watch_is_the_far_side(self):
        branch_seen = self.ns["branch_seen"]
        n_flags = self.ns["FLAG_COUNT"]
        # Left mode watches the RIGHTMOST elements...
        r = _Reading(dark_flags=(9,))
        self.assertTrue(branch_seen(r, "left"))
        self.assertFalse(branch_seen(r, "right"))
        # ...right mode the LEFTMOST.
        r = _Reading(dark_flags=(0,))
        self.assertTrue(branch_seen(r, "right"))
        self.assertFalse(branch_seen(r, "left"))
        # An element just inside the watch band counts; one outside
        # does not.
        r = _Reading(dark_flags=(10 - n_flags,))
        self.assertTrue(branch_seen(r, "left"))
        r = _Reading(dark_flags=(10 - n_flags - 1,))
        self.assertFalse(branch_seen(r, "left"))
        # No dark anywhere: no branch.
        self.assertFalse(branch_seen(_Reading(), "left"))


if __name__ == "__main__":
    unittest.main()
