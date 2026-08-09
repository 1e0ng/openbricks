# SPDX-License-Identifier: MIT
"""The QTR square-up-on-the-edge control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_qtr_line_follow.py``:
the example wires hardware at module level, so the pure control-law
block is pulled out by its markers. Each half of the ten-element
window is one virtual corner sensor, and BOTH passes run the same
proportional servo (``edge_dps(half, target)``) on the half's mean
ambient — seek targets 30 (well onto the line), edge targets 50
(straddling the boundary)."""

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


class _Reading:
    """``QTRReading`` stand-in: both laws consume only per-element
    ``ambient()``. Uniform per-half values via ``left``/``right``,
    per-element via ``ambients``."""

    def __init__(self, left=100, right=100, ambients=None):
        if ambients is None:
            ambients = [left] * 5 + [right] * 5
        self.elements = [_Element(a) for a in ambients]


class _ServoLawContract:
    """The shared battery: seek and edge are the SAME servo with
    different targets, so every assertion runs against each."""

    LAW = None          # subclasses set the namespace key
    TARGET = None       # and the pass's ambient target

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/qtr_align.py")

    def _tick(self, left=100, right=100, ambients=None):
        return self.ns[self.LAW](_Reading(
            left=left, right=right, ambients=ambients))

    def _dps(self, ambient):
        return int(self.ns["KP"] * (ambient - self.TARGET))

    def test_far_white_drives_forward_proportionally(self):
        expect = self._dps(100)
        self.assertTrue(expect > 0)
        self.assertEqual(self._tick(left=100, right=100),
                         (expect, expect))

    def test_deep_dark_backs_up_proportionally(self):
        expect = self._dps(0)
        self.assertTrue(expect < 0)
        self.assertEqual(self._tick(left=0, right=0),
                         (expect, expect))

    def test_each_side_servos_independently(self):
        t = self.TARGET
        self.assertEqual(self._tick(left=0, right=t),
                         (self._dps(0), 0))
        self.assertEqual(self._tick(left=t, right=100),
                         (0, self._dps(100)))

    def test_both_sides_at_target_end_the_phase(self):
        t = self.TARGET
        self.assertIsNone(self._tick(left=t, right=t))

    def test_tolerance_band_counts_as_arrived(self):
        t = self.TARGET
        tol = self.ns["EDGE_TOLERANCE"]
        self.assertIsNone(self._tick(left=t - tol, right=t + tol))

    def test_just_outside_the_band_still_moves(self):
        t = self.TARGET
        tol = self.ns["EDGE_TOLERANCE"]
        self.assertEqual(
            self._tick(left=t - tol - 1, right=t + tol + 1),
            (self._dps(t - tol - 1), self._dps(t + tol + 1)))

    def test_speed_is_proportional_to_the_error(self):
        t = self.TARGET
        near = self._tick(left=t + 10, right=t)[0]
        far = self._tick(left=100, right=t)[0]
        self.assertTrue(abs(near) < abs(far), (near, far))

    def test_side_reads_the_mean_of_its_half(self):
        # Half at (t-20, t-10, t, t+10, t+20): mean t — arrived even
        # though no single element sits at the target.
        t = self.TARGET
        spread = [t - 20, t - 10, t, t + 10, t + 20]
        self.assertIsNone(self._tick(ambients=spread * 2))


class SeekLawTests(_ServoLawContract, unittest.TestCase):
    LAW = "seek_wheel_speeds"
    TARGET = 30


class EdgeLawTests(_ServoLawContract, unittest.TestCase):
    LAW = "edge_wheel_speeds"
    TARGET = 50


class PhaseHandoffTests(unittest.TestCase):
    """Seek parks both halves near ambient 30 (onto the line); the
    edge pass picks up from exactly there and backs off toward 50."""

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/qtr_align.py")

    def test_seek_end_state_starts_the_edge_pass_backwards(self):
        r = _Reading(left=30, right=30)
        self.assertIsNone(self.ns["seek_wheel_speeds"](r))
        edge = self.ns["edge_wheel_speeds"](r)
        # MP's assertIsNotNone %-formats tuples and dies; be plain.
        self.assertTrue(edge is not None)
        self.assertTrue(edge[0] < 0 and edge[1] < 0, edge)


if __name__ == "__main__":
    unittest.main()
