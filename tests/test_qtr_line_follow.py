# SPDX-License-Identifier: MIT
"""The QTR line-follow control law, arithmetic only.

Same extract-and-exec trick as ``tests/test_line_follow.py``: the
example wires hardware at module level, so the pure control-law
block is pulled out by its markers. What is pinned here is the
CONTRACT the bench relies on: sign conventions, intersection
priority, recovery direction, and the clamp.
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


class QTRLawTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ns = _load()

    def _tick(self, pos, side=0, dark=1, state=None, dt=0.01):
        return self.ns["_pd_wheel_speeds"](pos, side, dark, state, dt)

    def test_centred_line_drives_straight(self):
        speeds, _ = self._tick(0.0)
        self.assertEqual(speeds, (self.ns["CRUISE_DPS"],
                                  self.ns["CRUISE_DPS"]))

    def test_line_right_steers_right(self):
        # +position = line right of centre -> left wheel faster.
        (l, r), _ = self._tick(+10.0)
        self.assertTrue(l > r, (l, r))
        (l, r), _ = self._tick(-10.0)
        self.assertTrue(l < r, (l, r))

    def test_derivative_damps_a_closing_error(self):
        # Error shrinking fast: KD subtracts from the P steering.
        (l_p, r_p), _ = self._tick(+10.0)                # no history
        (l_d, r_d), _ = self._tick(+10.0, state=20.0)    # was worse
        self.assertTrue(l_d - r_d < l_p - r_p,
                        ((l_p, r_p), (l_d, r_d)))

    def test_intersection_wins_over_steering(self):
        speeds, _ = self._tick(+30.0,
                               dark=self.ns["INTERSECTION_COUNT"])
        self.assertIsNone(speeds)

    def test_lost_line_recovers_toward_last_side(self):
        (l, r), _ = self._tick(None, side=+1)     # escaped right
        self.assertTrue(l > r, (l, r))
        (l, r), _ = self._tick(None, side=-1)
        self.assertTrue(l < r, (l, r))

    def test_clamp_never_reverses_a_wheel(self):
        (l, r), _ = self._tick(+1000.0)
        self.assertEqual(r, 0)                    # clamped at zero
        self.assertEqual(l, self.ns["MAX_DPS"])

    def test_state_threads_the_error(self):
        _, state = self._tick(+7.5)
        self.assertEqual(state, 7.5)


if __name__ == "__main__":
    unittest.main()
