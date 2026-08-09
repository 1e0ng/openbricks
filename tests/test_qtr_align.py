# SPDX-License-Identifier: MIT
"""The QTR square-up-on-the-edge control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_qtr_line_follow.py``:
the example wires hardware at module level, so the pure control-law
block is pulled out by its markers. Each half of the ten-element
window is one virtual corner sensor. Two passes: seek creeps each
wheel forward until its half reaches the line; edge then servos
each wheel proportionally (the follower's KP discipline) until its
half's mean ambient sits at 50 — the elements straddling the
black/white boundary.
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
    def __init__(self, dark=False, ambient=100):
        self._dark = dark
        self._ambient = ambient

    def dark(self):
        return self._dark

    def ambient(self):
        return self._ambient


class _Reading:
    """``QTRReading`` stand-in. ``dark_flags`` drives the seek law;
    ``left_ambient`` / ``right_ambient`` give every element of that
    half the same ambient for the edge law (per-element values via
    ``ambients``)."""

    def __init__(self, dark_flags=(), left_ambient=100,
                 right_ambient=100, ambients=None):
        if ambients is None:
            ambients = [left_ambient] * 5 + [right_ambient] * 5
        self.elements = [
            _Element(dark=(i in dark_flags),
                     ambient=(0 if i in dark_flags else ambients[i]))
            for i in range(10)]


class SeekLawTests(unittest.TestCase):
    """Seek stops a wheel when its half reads SOLIDLY dark — mean
    ambient below the 30 target via ``side_ambient(half, 30)`` — so
    a single grazed element can't fake an arrival."""

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/qtr_align.py")

    def _seek(self, dark_flags=(), ambients=None):
        return self.ns["seek_wheel_speeds"](
            _Reading(dark_flags=dark_flags, ambients=ambients))

    def test_no_line_creeps_both_wheels_forward(self):
        dps = self.ns["SEEK_DPS"]
        self.assertEqual(self._seek(), (dps, dps))

    def test_solidly_dark_left_half_stops_only_the_left_wheel(self):
        dps = self.ns["SEEK_DPS"]
        self.assertEqual(self._seek(dark_flags=(0, 1, 2, 3, 4)),
                         (0, dps))

    def test_solidly_dark_right_half_stops_only_the_right_wheel(self):
        dps = self.ns["SEEK_DPS"]
        self.assertEqual(self._seek(dark_flags=(5, 6, 7, 8, 9)),
                         (dps, 0))

    def test_both_halves_dark_ends_the_phase(self):
        self.assertIsNone(self._seek(dark_flags=tuple(range(10))))

    def test_a_single_dark_element_is_not_an_arrival(self):
        # Mean of one dark + four mat elements is 80 — far above the
        # 30 target; the old any-element rule stopped here and left
        # the chassis crooked on a grazing touch.
        dps = self.ns["SEEK_DPS"]
        self.assertEqual(self._seek(dark_flags=(0,)), (dps, dps))

    def test_target_30_is_the_boundary(self):
        dps = self.ns["SEEK_DPS"]
        # Left half mean exactly 30: not there yet (error 0, needs
        # < 0)...
        self.assertEqual(
            self._seek(ambients=[0, 0, 0, 50, 100] + [100] * 5),
            (dps, dps))
        # ...one count darker: arrived.
        self.assertEqual(
            self._seek(ambients=[0, 0, 0, 45, 100] + [100] * 5),
            (0, dps))


class EdgeLawTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/qtr_align.py")

    def _edge(self, left=100, right=100, ambients=None):
        return self.ns["edge_wheel_speeds"](_Reading(
            left_ambient=left, right_ambient=right, ambients=ambients))

    def test_deep_on_the_line_backs_both_wheels(self):
        full = int(self.ns["KP"] * -50)
        self.assertEqual(self._edge(left=0, right=0), (full, full))

    def test_off_the_line_drives_both_wheels_forward(self):
        full = int(self.ns["KP"] * 50)
        self.assertEqual(self._edge(left=100, right=100),
                         (full, full))

    def test_each_side_servos_independently(self):
        kp = self.ns["KP"]
        self.assertEqual(self._edge(left=0, right=50),
                         (int(kp * -50), 0))
        self.assertEqual(self._edge(left=50, right=100),
                         (0, int(kp * 50)))

    def test_both_sides_at_the_boundary_end_the_phase(self):
        self.assertIsNone(self._edge(left=50, right=50))

    def test_speed_is_proportional_to_the_error(self):
        near = self._edge(left=30, right=50)[0]
        far = self._edge(left=0, right=50)[0]
        self.assertTrue(abs(near) < abs(far), (near, far))
        self.assertEqual(near, int(self.ns["KP"] * -20))

    def test_tolerance_band_counts_as_aligned(self):
        tol = self.ns["EDGE_TOLERANCE"]
        self.assertIsNone(self._edge(left=50 - tol, right=50 + tol))

    def test_just_outside_the_band_still_moves(self):
        tol = self.ns["EDGE_TOLERANCE"]
        kp = self.ns["KP"]
        self.assertEqual(
            self._edge(left=50 - tol - 1, right=50 + tol + 1),
            (int(kp * -(tol + 1)), int(kp * (tol + 1))))

    def test_side_ambient_is_the_mean_of_the_half(self):
        # Half at (0, 25, 50, 75, 100): mean 50 — aligned even
        # though no single element reads 50.
        self.assertIsNone(self._edge(
            ambients=[0, 25, 50, 75, 100] * 2))


if __name__ == "__main__":
    unittest.main()
