# SPDX-License-Identifier: MIT
"""The QTR line-follow control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_line_follow.py``: the
examples wire hardware at module level, so the pure control-law
block is pulled out by its markers. THREE example files carry ONE
IDENTICAL law; the mode geometry (setpoints, positions, pins)
lives in the firmware's QTRLineSensor, so the law consumes only
``edge_error()``. The whole window dark ends the run; the far-side
FLAG_COUNT elements are the branch watch.
"""

import tests._fakes  # noqa: F401

import unittest
from openbricks.parameters import LineMode


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
    def __init__(self, dark):
        self._dark = dark

    def dark(self):
        return self._dark

    def ambient(self):
        # The Pybricks scale the branch watch reads: dark elements
        # sit near 0, mat elements near 100.
        return 0 if self._dark else 100


class _Reading:
    """``QTRReading`` stand-in: the law consumes ``edge_error()``
    and ``elements`` (whole-window darkness and the branch watch
    both read per-element ``ambient()``)."""

    def __init__(self, error=None, all_dark=False, dark_flags=()):
        self._error = error
        if all_dark:
            dark_flags = tuple(range(10))
        # 10 elements; the indices in dark_flags read dark.
        self.elements = [_Element(i in dark_flags) for i in range(10)]

    def edge_error(self):
        return self._error


class _LawContract:
    """The shared battery — the law body must be identical in both
    example files, so every assertion runs against each."""

    EXAMPLE = None          # subclasses set the path
    FILE_MODE = None        # the file's own MODE constant

    @classmethod
    def setUpClass(cls):
        cls.ns = _load(cls.EXAMPLE)

    def _tick(self, error, all_dark=False):
        reading = _Reading(error=error, all_dark=all_dark)
        return self.ns["get_wheel_speeds"](reading)

    def _cruise(self):
        c = self.ns["CRUISE_DPS"]
        return (c, c)

    def test_zero_error_drives_straight(self):
        self.assertEqual(self._tick(0.0), self._cruise())

    def test_error_sign_steers_toward_the_setpoint(self):
        l, r = self._tick(+10.0)
        self.assertTrue(l > r, (l, r))
        l, r = self._tick(-10.0)
        self.assertTrue(l < r, (l, r))

    def test_full_window_dark_stops_immediately(self):
        self.assertIsNone(self._tick(0.0, all_dark=True))

    def test_lost_line_rails_the_steer(self):
        # The driver's edge_error rails at +/-50 when the line is
        # gone, so the law turns hard back toward it instead of
        # raising — pin that a railed error saturates cleanly.
        l, r = self._tick(+50.0)
        self.assertEqual((l, r), (self.ns["MAX_DPS"], 0))

    def test_clamp_never_reverses_a_wheel(self):
        l, r = self._tick(+1000.0)
        self.assertEqual(r, 0)                    # clamped at zero
        self.assertEqual(l, self.ns["MAX_DPS"])

    def test_branch_watch_is_the_far_side(self):
        branch_seen = self.ns["branch_seen"]
        n_flags = self.ns["FLAG_COUNT"]
        # Left mode watches the RIGHTMOST elements...
        r = _Reading(dark_flags=(9,))
        self.assertTrue(branch_seen(r, LineMode.LEFT))
        self.assertFalse(branch_seen(r, LineMode.RIGHT))
        # ...right mode the LEFTMOST.
        r = _Reading(dark_flags=(0,))
        self.assertTrue(branch_seen(r, LineMode.RIGHT))
        self.assertFalse(branch_seen(r, LineMode.LEFT))
        # An element just inside the watch band counts; one outside
        # does not.
        r = _Reading(dark_flags=(10 - n_flags,))
        self.assertTrue(branch_seen(r, LineMode.LEFT))
        r = _Reading(dark_flags=(10 - n_flags - 1,))
        self.assertFalse(branch_seen(r, LineMode.LEFT))
        # No dark anywhere: no branch.
        self.assertFalse(branch_seen(_Reading(), LineMode.LEFT))
        # Center mode holds the line mid-window, so a branch can
        # appear on EITHER outer band.
        self.assertTrue(branch_seen(_Reading(dark_flags=(0,)), LineMode.CENTER))
        self.assertTrue(branch_seen(_Reading(dark_flags=(9,)), LineMode.CENTER))
        self.assertFalse(branch_seen(_Reading(dark_flags=(5,)), LineMode.CENTER))
        self.assertFalse(branch_seen(_Reading(), LineMode.CENTER))


    def test_file_mode_constant(self):
        # Each example ships pinned to its own discipline; the law
        # itself stays switchable per call.
        self.assertEqual(self.ns["MODE"], self.FILE_MODE)


class LeftFollowerTests(_LawContract, unittest.TestCase):
    EXAMPLE = "examples/qtr_line_follow_left.py"
    FILE_MODE = LineMode.LEFT


class RightFollowerTests(_LawContract, unittest.TestCase):
    EXAMPLE = "examples/qtr_line_follow_right.py"
    FILE_MODE = LineMode.RIGHT


class CenterFollowerTests(_LawContract, unittest.TestCase):
    EXAMPLE = "examples/qtr_line_follow_center.py"
    FILE_MODE = LineMode.CENTER


class LawIsIdenticalAcrossFilesTests(unittest.TestCase):
    """Three files, one law: the block between the markers must be
    byte-identical apart from the MODE constant."""

    FILES = ("examples/qtr_line_follow_left.py",
             "examples/qtr_line_follow_right.py",
             "examples/qtr_line_follow_center.py")

    def test_identical_law_bodies(self):
        bodies = []
        for path in self.FILES:
            with open(path) as f:
                src = f.read()
            body = src[src.index("# --- control law"):
                       src.index("# --- end control law ---")]
            lines = [ln for ln in body.split("\n")
                     if not ln.startswith("MODE = ")]
            bodies.append("\n".join(lines))
        for path, body in zip(self.FILES, bodies):
            self.assertEqual(body, bodies[0],
                             "%s carries a different law" % path)


if __name__ == "__main__":
    unittest.main()
