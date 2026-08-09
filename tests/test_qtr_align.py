# SPDX-License-Identifier: MIT
"""The QTR square-up control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_qtr_line_follow.py``:
the example wires hardware at module level, so the pure control-law
block is pulled out by its markers. Each half of the ten-element
window is one virtual corner sensor; a wheel creeps until its half
reaches the line, and both halves dark ends the move.
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
    def __init__(self, dark):
        self._dark = dark

    def dark(self):
        return self._dark


class _Reading:
    """``QTRReading`` stand-in: the align law consumes only
    ``elements`` — no mode, no edge geometry."""

    def __init__(self, dark_flags=()):
        self.elements = [_Element(i in dark_flags) for i in range(10)]


class AlignLawTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/qtr_align.py")

    def _tick(self, dark_flags=()):
        return self.ns["get_wheel_speeds"](_Reading(dark_flags))

    def test_no_line_creeps_both_wheels_forward(self):
        dps = self.ns["ALIGN_DPS"]
        self.assertEqual(self._tick(), (dps, dps))

    def test_left_half_on_line_stops_only_the_left_wheel(self):
        dps = self.ns["ALIGN_DPS"]
        self.assertEqual(self._tick(dark_flags=(0,)), (0, dps))

    def test_right_half_on_line_stops_only_the_right_wheel(self):
        dps = self.ns["ALIGN_DPS"]
        self.assertEqual(self._tick(dark_flags=(9,)), (dps, 0))

    def test_both_halves_dark_ends_the_move(self):
        self.assertIsNone(self._tick(dark_flags=(0, 9)))

    def test_half_boundary_is_five_and_five(self):
        n = self.ns["SIDE_COUNT"]
        dps = self.ns["ALIGN_DPS"]
        # The last element of the left half...
        self.assertEqual(self._tick(dark_flags=(n - 1,)), (0, dps))
        # ...and the first element of the right half.
        self.assertEqual(self._tick(dark_flags=(n,)), (dps, 0))

    def test_any_single_dark_element_claims_its_half(self):
        dps = self.ns["ALIGN_DPS"]
        for i in range(10):
            l, r = self._tick(dark_flags=(i,))
            if i < self.ns["SIDE_COUNT"]:
                self.assertEqual((l, r), (0, dps), i)
            else:
                self.assertEqual((l, r), (dps, 0), i)


if __name__ == "__main__":
    unittest.main()
