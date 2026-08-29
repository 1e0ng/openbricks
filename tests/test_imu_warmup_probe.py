# SPDX-License-Identifier: MIT
"""The IMU warmup probe's settle summary, arithmetic only.

Same extract-and-exec trick as ``tests/test_qtr_align.py``: the
example wires hardware at module level, so the pure summary block is
pulled out by its markers. A decaying gyro-z trace must surface its
transient as the first-second mean minus the last-second mean; a
flat trace must surface none."""

import tests._fakes  # noqa: F401

import unittest


def _load(path):
    with open(path) as f:
        src = f.read()
    begin = "# --- settle summary"
    end = "# --- end settle summary ---"
    if begin not in src or end not in src:
        raise AssertionError(
            "settle-summary markers not found in %s — they are "
            "load-bearing here" % path)
    ns = {}
    exec(src[src.index(begin):src.index(end)], ns)
    return ns


def _trace(dps_by_second, step_ms=100):
    """(t_ms, dps) samples at ``step_ms``, one constant rate per
    listed second."""
    samples = []
    for sec, dps in enumerate(dps_by_second):
        for t in range(sec * 1000, (sec + 1) * 1000, step_ms):
            samples.append((t, dps))
    return samples


class SettleSummaryTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.ns = _load("examples/imu_warmup_probe.py")

    def _settle(self, samples):
        return self.ns["gyro_z_settle"](samples)

    def test_decaying_trace_reports_the_transient(self):
        early, late = self._settle(_trace([2.0, 1.0, 0.5, 0.1]))
        self.assertAlmostEqual(early, 2.0, delta=1e-9)
        self.assertAlmostEqual(late, 0.1, delta=1e-9)

    def test_flat_trace_reports_no_settle(self):
        early, late = self._settle(_trace([0.3, 0.3, 0.3, 0.3]))
        self.assertAlmostEqual(early, late, delta=1e-9)

    def test_middle_of_the_trace_is_ignored(self):
        early, late = self._settle(_trace([1.5, 99.0, -99.0, 0.2]))
        self.assertAlmostEqual(early, 1.5, delta=1e-9)
        self.assertAlmostEqual(late, 0.2, delta=1e-9)

    def test_windows_are_time_based_not_count_based(self):
        samples = _trace([4.0, 0.0], step_ms=100)
        sparse = [(t, dps) for t, dps in samples if t % 500 == 0]
        early, late = self._settle(sparse)
        self.assertAlmostEqual(early, 4.0, delta=1e-9)
        self.assertAlmostEqual(late, 0.0, delta=1e-9)

    def test_negative_offsets_keep_their_sign(self):
        early, late = self._settle(_trace([-1.2, -0.1]))
        self.assertAlmostEqual(early, -1.2, delta=1e-9)
        self.assertAlmostEqual(late, -0.1, delta=1e-9)


if __name__ == "__main__":
    unittest.main()
