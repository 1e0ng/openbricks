# SPDX-License-Identifier: MIT
"""The QTR square-up-on-the-edge control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_qtr_line_follow.py``:
the example wires hardware at module level, so the pure control-law
block is pulled out by its markers. Each half of the ten-element
window is one virtual corner sensor. Two passes: seek creeps each
wheel forward until its half reaches the line; edge backs each
wheel off until its half turns white again, parking the bar on the
line's near edge.
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
    """``QTRReading`` stand-in: the align laws consume only
    ``elements`` — no mode, no edge geometry."""

    def __init__(self, dark_flags=()):
        self.elements = [_Element(i in dark_flags) for i in range(10)]


class AlignLawTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/qtr_align.py")

    def _seek(self, dark_flags=()):
        return self.ns["seek_speeds"](_Reading(dark_flags))

    def _edge(self, dark_flags=()):
        return self.ns["edge_speeds"](_Reading(dark_flags))

    def test_seek_no_line_creeps_both_wheels_forward(self):
        dps = self.ns["SEEK_DPS"]
        self.assertEqual(self._seek(), (dps, dps))

    def test_seek_left_half_on_line_stops_only_the_left_wheel(self):
        dps = self.ns["SEEK_DPS"]
        self.assertEqual(self._seek(dark_flags=(0,)), (0, dps))

    def test_seek_right_half_on_line_stops_only_the_right_wheel(self):
        dps = self.ns["SEEK_DPS"]
        self.assertEqual(self._seek(dark_flags=(9,)), (dps, 0))

    def test_seek_both_halves_dark_ends_the_phase(self):
        self.assertIsNone(self._seek(dark_flags=(0, 9)))

    def test_seek_half_boundary_is_five_and_five(self):
        n = self.ns["SIDE_COUNT"]
        dps = self.ns["SEEK_DPS"]
        # The last element of the left half...
        self.assertEqual(self._seek(dark_flags=(n - 1,)), (0, dps))
        # ...and the first element of the right half.
        self.assertEqual(self._seek(dark_flags=(n,)), (dps, 0))

    def test_seek_any_single_dark_element_claims_its_half(self):
        dps = self.ns["SEEK_DPS"]
        for i in range(10):
            l, r = self._seek(dark_flags=(i,))
            if i < self.ns["SIDE_COUNT"]:
                self.assertEqual((l, r), (0, dps), i)
            else:
                self.assertEqual((l, r), (dps, 0), i)

    def test_edge_both_halves_dark_backs_both_wheels(self):
        dps = self.ns["BACK_DPS"]
        self.assertEqual(self._edge(dark_flags=(0, 9)), (-dps, -dps))

    def test_edge_backs_only_the_wheel_whose_half_is_still_dark(self):
        dps = self.ns["BACK_DPS"]
        self.assertEqual(self._edge(dark_flags=(0,)), (-dps, 0))
        self.assertEqual(self._edge(dark_flags=(9,)), (0, -dps))

    def test_edge_both_halves_white_ends_the_phase(self):
        self.assertIsNone(self._edge())

    def test_edge_never_drives_forward(self):
        for flags in ((), (0,), (9,), (0, 9), (4,), (5,)):
            speeds = self._edge(dark_flags=flags)
            if speeds is None:
                continue
            for dps in speeds:
                self.assertTrue(dps <= 0, (flags, speeds))

    def test_phase_handoff_is_consistent(self):
        # Exactly the state that ends seek (both halves dark) is a
        # working start state for edge, and the state that ends
        # edge is not a finished seek — the passes always run in
        # the seek -> edge order.
        self.assertIsNone(self._seek(dark_flags=(0, 9)))
        self.assertEqual(
            self._edge(dark_flags=(0, 9)),
            (-self.ns["BACK_DPS"], -self.ns["BACK_DPS"]))
        self.assertIsNotNone(self._seek())


if __name__ == "__main__":
    unittest.main()
