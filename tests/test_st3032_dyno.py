# SPDX-License-Identifier: MIT
"""The ST-3032 dyno example's summary arithmetic, hardware-free.

Extract-and-exec by markers, like ``tests/test_imu_warmup_probe.py``:
the example wires two servos at module level, so only the pure
summary block is pulled in. A synthetic DC-motor line must come back
with its own no-load speed and stall torque; a sweep that never
loaded the servo, or loaded it the wrong way, must refuse loudly."""

import tests._fakes  # noqa: F401

import unittest


def _load(path):
    with open(path) as f:
        src = f.read()
    begin = "# --- dyno summary"
    end = "# --- end dyno summary ---"
    if begin not in src or end not in src:
        raise AssertionError(
            "dyno-summary markers not found in %s — they are "
            "load-bearing here" % path)
    ns = {}
    exec(src[src.index(begin):src.index(end)], ns)
    return ns


class DynoSummaryTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/st3032_dyno.py")

    def test_torque_is_kt_times_current_above_no_load(self):
        torque = self.ns["torque_mnm"]
        self.assertAlmostEqual(torque(0.6, 0.1, 618.0), 309.0, delta=1e-9)
        self.assertAlmostEqual(torque(0.1, 0.1, 618.0), 0.0, delta=1e-9)

    def test_torque_never_goes_negative_below_no_load(self):
        # Sampling noise can read under the no-load baseline; that
        # is zero torque, not a negative one pulling the fit around.
        self.assertEqual(self.ns["torque_mnm"](0.08, 0.1, 618.0), 0.0)

    def test_fit_recovers_a_synthetic_motor_line(self):
        # speed = 300 - 0.5 * torque: no-load 300 dps, stall 600 mNm.
        points = [(t, 300.0 - 0.5 * t) for t in (0, 50, 100, 150, 200)]
        no_load, stall, slope = self.ns["fit_torque_speed"](points)
        self.assertAlmostEqual(no_load, 300.0, delta=1e-9)
        self.assertAlmostEqual(stall, 600.0, delta=1e-9)
        self.assertAlmostEqual(slope, -0.5, delta=1e-9)

    def test_fit_averages_noise_instead_of_chasing_it(self):
        points = [(0, 302.0), (100, 249.0), (200, 201.0), (300, 148.0)]
        no_load, stall, slope = self.ns["fit_torque_speed"](points)
        self.assertTrue(295.0 < no_load < 305.0, no_load)
        self.assertTrue(580.0 < stall < 620.0, stall)

    def test_uncoupled_sweep_is_refused(self):
        # Every step reads the same torque: the load never reached
        # the servo under test.
        points = [(0.0, 300.0), (0.0, 299.0), (0.0, 301.0)]
        try:
            self.ns["fit_torque_speed"](points)
            self.fail("expected ValueError for a flat torque sweep")
        except ValueError as e:
            self.assertTrue("coupled" in str(e), e)

    def test_helping_load_is_refused(self):
        points = [(0, 300.0), (100, 320.0), (200, 340.0)]
        try:
            self.ns["fit_torque_speed"](points)
            self.fail("expected ValueError for a rising line")
        except ValueError as e:
            self.assertTrue("flip" in str(e), e)

    def test_play_is_the_gap_between_the_two_rest_means(self):
        play = self.ns["play_deg"]
        self.assertAlmostEqual(play([10.4, 10.6], [9.4, 9.6]), 1.0,
                               delta=1e-9)
        self.assertAlmostEqual(play([9.5], [10.5]), 1.0, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
