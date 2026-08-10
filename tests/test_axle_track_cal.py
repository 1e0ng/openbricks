# SPDX-License-Identifier: MIT
"""The axle-track calibration law, arithmetic only.

Same extract-and-exec trick as ``tests/test_qtr_align.py``: the
example wires hardware at module level, so the pure law is pulled
out by its markers. The correction DIRECTION is the load-bearing
assertion — docs/measuring.md shipped it inverted for weeks
(overshoot was said to mean a LARGER track; physics says smaller:
actual rotation = commanded x configured/real)."""

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


class AxleTrackLawTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/icm45686_axle_track.py")

    def _corrected(self, track, commanded, measured):
        return self.ns["corrected_track"](track, commanded, measured)

    def test_exact_rotation_changes_nothing(self):
        self.assertAlmostEqual(self._corrected(136.0, 3600, 3600.0), 136.0)

    def test_overshoot_means_a_smaller_track(self):
        # Rotated PAST the mark: each wheel-degree produced more body
        # rotation than the math assumed — the real track is smaller.
        got = self._corrected(138.0, 3600, 3618.0)
        self.assertTrue(got < 138.0, got)
        self.assertAlmostEqual(got, 137.31, places=2)

    def test_undershoot_means_a_larger_track(self):
        # The bench shape: encoder squares stopping at ~356 deg per
        # 360 commanded imply the real track is larger than 136.
        got = self._corrected(136.0, 360, 356.4)
        self.assertTrue(got > 136.0, got)
        self.assertAlmostEqual(got, 137.37, places=2)

    def test_correction_is_proportional(self):
        # Twice the measured error, twice the correction (to first
        # order) — pins the formula shape, not just its direction.
        small = self._corrected(100.0, 3600, 3636.0)
        large = self._corrected(100.0, 3600, 3672.0)
        self.assertAlmostEqual((100.0 - large), 2 * (100.0 - small),
                               places=1)

    def test_turns_amplify_nothing_in_the_formula(self):
        # The ratio is all that matters: 10 turns with 18 deg error
        # equals 1 turn with 1.8 deg error.
        self.assertAlmostEqual(
            self._corrected(136.0, 3600, 3618.0),
            self._corrected(136.0, 360, 361.8), places=6)


if __name__ == "__main__":
    unittest.main()
