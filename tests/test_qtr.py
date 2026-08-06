# SPDX-License-Identifier: MIT
"""Tests for the QTR/QTRX reflectance-array driver.

Runs under both CPython and unix MicroPython against the fake
``machine.ADC`` (scripted per-pin readings). The centroid sign
convention is load-bearing for the follower: pins are passed LEFT to
RIGHT, and a positive position means the line is RIGHT of centre.
"""

import tests._fakes  # noqa: F401

import unittest

from machine import ADC
from openbricks.drivers.qtr import QTRArray


_PINS = (1, 2, 3, 4, 5, 6, 7, 8, 9)
_MAT, _LINE = 2000, 60000


def _script(dark_pins=(), mat=_MAT, line=_LINE):
    """All pins read 'mat' except ``dark_pins``, which read 'line'."""
    ADC.reads = {p: (line if p in dark_pins else mat) for p in _PINS}


def _swing():
    """A per-pin oscillating reading. Each pin needs its OWN flip
    state: one closure shared by an even number of pins hands every
    element a fixed phase (parity of 8k+i is constant in k) and
    nothing ever swings."""
    state = [False]

    def read():
        state[0] = not state[0]
        return _LINE if state[0] else _MAT
    return read


def _calibrated(**kwargs):
    """Array whose every element has seen both extremes."""
    ADC.reads = {p: _swing() for p in _PINS}
    qtr = QTRArray(pins=_PINS, pitch_mm=8.0, **kwargs)
    qtr.calibrate(duration_ms=100, poll_ms=5)
    return qtr


class ConstructionTests(unittest.TestCase):
    def tearDown(self):
        ADC.reads = {}

    def test_needs_two_elements(self):
        with self.assertRaises(ValueError):
            QTRArray(pins=(1,))

    def test_reading_before_calibration_raises(self):
        # An uncalibrated centroid is a plausible-looking wrong
        # number — the failure mode must be loud, not subtly bad
        # steering.
        _script()
        qtr = QTRArray(pins=_PINS)
        try:
            qtr.read()
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("calibrate" in str(e), e)

    def test_flat_channel_is_named(self):
        # Element 4 (pin 5) never changes: unwired, or the sweep
        # missed it. Its normalized reading would be amplified noise.
        ADC.reads = {p: _swing() for p in _PINS}
        ADC.reads[5] = _MAT                     # flat
        qtr = QTRArray(pins=_PINS)
        try:
            qtr.calibrate(duration_ms=100)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            msg = str(e)
        self.assertTrue("element(s) 4" in msg, msg)


class ReadingTests(unittest.TestCase):
    def setUp(self):
        self.qtr = _calibrated()

    def tearDown(self):
        ADC.reads = {}

    def test_read_normalizes_to_full_scale(self):
        _script(dark_pins=(5,))
        r = self.qtr.read()
        self.assertEqual(r[4], 1000)            # the dark element
        self.assertTrue(all(v == 0 for i, v in enumerate(r) if i != 4),
                        r)

    def test_centred_line_reads_zero_mm(self):
        _script(dark_pins=(5,))                 # pin 5 = middle of 9
        self.assertEqual(self.qtr.position(), 0.0)

    def test_right_of_centre_is_positive(self):
        _script(dark_pins=(8,))                 # 3 elements right
        pos = self.qtr.position()
        self.assertEqual(pos, 3 * 8.0)          # pitch_mm=8
        _script(dark_pins=(2,))
        self.assertEqual(self.qtr.position(), -3 * 8.0)

    def test_straddling_line_interpolates(self):
        # Two adjacent elements equally dark: centroid lands between
        # them — the continuous error a 2-sensor rig can't produce.
        _script(dark_pins=(5, 6))
        self.assertEqual(self.qtr.position(), 4.0)   # midpoint, 8/2

    def test_lost_line_returns_none_and_remembers_the_side(self):
        _script(dark_pins=(9,))                 # rightmost, then gone
        self.assertTrue(self.qtr.position() > 0)
        _script(dark_pins=())                   # all mat
        self.assertIsNone(self.qtr.position())
        self.assertEqual(self.qtr.last_side(), 1)
        _script(dark_pins=(1,))                 # reappears far left
        self.assertTrue(self.qtr.position() < 0)
        _script(dark_pins=())
        self.assertEqual(self.qtr.last_side(), -1)

    def test_dark_count_is_the_intersection_signal(self):
        _script(dark_pins=(3, 4, 5, 6, 7, 8, 9))    # stop bar
        self.assertEqual(self.qtr.dark_count(), 7)
        _script(dark_pins=(5,))
        self.assertEqual(self.qtr.dark_count(), 1)

    def test_emitter_ctrl_pin_is_driven(self):
        ADC.reads = {p: _swing() for p in _PINS}
        qtr = QTRArray(pins=_PINS, ctrl=42)
        self.assertEqual(qtr._ctrl.value(), 1)  # emitters on at boot
        qtr.emitters(False)
        self.assertEqual(qtr._ctrl.value(), 0)
        qtr.emitters(True)
        self.assertEqual(qtr._ctrl.value(), 1)


if __name__ == "__main__":
    unittest.main()
