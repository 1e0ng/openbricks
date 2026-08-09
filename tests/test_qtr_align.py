# SPDX-License-Identifier: MIT
"""The QTR square-up-on-the-edge control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_qtr_line_follow.py``:
the example wires hardware at module level, so the pure control-law
block is pulled out by its markers. Each half of the ten-element
window is one virtual corner sensor; each wheel runs one
proportional servo (``edge_dps(half, 50)``) on its half's mean
ambient until both straddle the black/white boundary."""

import tests._fakes  # noqa: F401

import unittest


TARGET = 50


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
    """``QTRReading`` stand-in: the law consumes only per-element
    ``ambient()``. Uniform per-half values via ``left``/``right``,
    per-element via ``ambients``."""

    def __init__(self, left=100, right=100, ambients=None):
        if ambients is None:
            ambients = [left] * 5 + [right] * 5
        self.elements = [_Element(a) for a in ambients]


class EdgeLawTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/qtr_align.py")

    def _tick(self, left=100, right=100, ambients=None):
        return self.ns["edge_wheel_speeds"](_Reading(
            left=left, right=right, ambients=ambients))

    def _dps(self, ambient):
        return int(self.ns["KP"] * (ambient - TARGET))

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
        self.assertEqual(self._tick(left=0, right=TARGET),
                         (self._dps(0), 0))
        self.assertEqual(self._tick(left=TARGET, right=100),
                         (0, self._dps(100)))

    def test_both_sides_at_target_end_the_run(self):
        self.assertIsNone(self._tick(left=TARGET, right=TARGET))

    def test_tolerance_band_counts_as_arrived(self):
        tol = self.ns["EDGE_TOLERANCE"]
        self.assertIsNone(
            self._tick(left=TARGET - tol, right=TARGET + tol))

    def test_just_outside_the_band_still_moves(self):
        tol = self.ns["EDGE_TOLERANCE"]
        self.assertEqual(
            self._tick(left=TARGET - tol - 1, right=TARGET + tol + 1),
            (self._dps(TARGET - tol - 1), self._dps(TARGET + tol + 1)))

    def test_speed_is_proportional_to_the_error(self):
        near = self._tick(left=TARGET + 10, right=TARGET)[0]
        far = self._tick(left=100, right=TARGET)[0]
        self.assertTrue(abs(near) < abs(far), (near, far))

    def test_side_reads_the_mean_of_its_half(self):
        # Half at (30, 40, 50, 60, 70): mean 50 — arrived even
        # though no single element sits at the target.
        spread = [TARGET - 20, TARGET - 10, TARGET,
                  TARGET + 10, TARGET + 20]
        self.assertIsNone(self._tick(ambients=spread * 2))

    def test_the_law_is_the_only_wheel_speeds_function(self):
        # 1.83.5 removed seek_wheel_speeds: one servo, one loop.
        self.assertTrue("seek_wheel_speeds" not in self.ns)


if __name__ == "__main__":
    unittest.main()
