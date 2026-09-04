# SPDX-License-Identifier: MIT
"""The ST-3032 soak test's summary arithmetic, hardware-free.

Extract-and-exec by markers (``tests/test_imu_warmup_probe.py``
pattern). The drift summary must report first-to-last change, the
peak temperature and the lowest supply voltage seen, and the play
measurement must be direction-agnostic."""

import tests._fakes  # noqa: F401

import unittest


def _load(path):
    with open(path) as f:
        src = f.read()
    begin = "# --- soak summary"
    end = "# --- end soak summary ---"
    if begin not in src or end not in src:
        raise AssertionError(
            "soak-summary markers not found in %s — they are "
            "load-bearing here" % path)
    ns = {}
    exec(src[src.index(begin):src.index(end)], ns)
    return ns


class SoakSummaryTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/st3032_soak_test.py")

    def test_play_is_direction_agnostic(self):
        play = self.ns["play_deg"]
        self.assertAlmostEqual(play(19.6, 20.4), 0.8, delta=1e-9)
        self.assertAlmostEqual(play(20.4, 19.6), 0.8, delta=1e-9)

    def test_summary_reports_first_to_last_drift(self):
        rows = [
            (0.5, 12.3, 31.0, 0.210, 0.60),
            (10.0, 12.1, 39.0, 0.230, 0.65),
            (20.0, 11.9, 44.0, 0.250, 0.75),
        ]
        s = self.ns["soak_summary"](rows)
        self.assertAlmostEqual(s["temp_rise"], 13.0, delta=1e-9)
        self.assertAlmostEqual(s["temp_peak"], 44.0, delta=1e-9)
        self.assertAlmostEqual(s["current_change"], 0.040, delta=1e-9)
        self.assertAlmostEqual(s["play_change"], 0.15, delta=1e-9)
        self.assertAlmostEqual(s["voltage_min"], 11.9, delta=1e-9)

    def test_peak_and_minimum_are_not_the_endpoints(self):
        # A servo that warmed, then cooled as the battery sagged: the
        # summary must not mistake "last" for "worst".
        rows = [
            (0.5, 12.4, 30.0, 0.20, 0.50),
            (5.0, 11.2, 52.0, 0.30, 0.55),
            (10.0, 12.0, 40.0, 0.22, 0.55),
        ]
        s = self.ns["soak_summary"](rows)
        self.assertAlmostEqual(s["temp_peak"], 52.0, delta=1e-9)
        self.assertAlmostEqual(s["voltage_min"], 11.2, delta=1e-9)
        self.assertAlmostEqual(s["temp_rise"], 10.0, delta=1e-9)

    def test_single_report_is_a_zero_drift(self):
        s = self.ns["soak_summary"]([(0.5, 12.0, 30.0, 0.2, 0.5)])
        self.assertEqual(s["temp_rise"], 0.0)
        self.assertEqual(s["play_change"], 0.0)
        self.assertEqual(s["temp_peak"], 30.0)


if __name__ == "__main__":
    unittest.main()
