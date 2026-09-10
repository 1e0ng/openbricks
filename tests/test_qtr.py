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
from openbricks import pins as _pins
from openbricks.drivers.qtr import (
    QTRArray, QTRChannel, QTRElement, QTRLineSensor)


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
    def setUp(self):
        # The pin-claim registry is process-global; earlier suites in
        # the same run (hub, L298N fixtures) own low GPIOs under
        # other roles.
        _pins._claims_reset()

    def tearDown(self):
        ADC.reads = {}
        _pins._claims_reset()

    def test_needs_at_least_one_element(self):
        with self.assertRaises(ValueError):
            QTRArray(pins=())

    def test_non_adc_pins_are_refused_by_name(self):
        # pins.check knows the GPIO map, not analog capability, and
        # machine.ADC fails with a bare ValueError — after the
        # harness is soldered (bench 2026-08-06: five channels
        # landed on GPIO 38-42, which have no ADC on the S3).
        real = _pins._detect_chip
        _pins._detect_chip = lambda: "esp32s3"
        try:
            try:
                QTRArray(pins=(38, 39))
                self.fail("expected ValueError")
            except ValueError as e:
                self.assertTrue("no ADC" in str(e), e)
                self.assertTrue("38" in str(e), e)
            try:
                QTRArray(pins=(18, 2))
                self.fail("expected ValueError")
            except ValueError as e:
                self.assertTrue("ADC2" in str(e), e)
                self.assertTrue("radio" in str(e), e)
            _pins._detect_chip = lambda: "esp32"
            try:
                QTRArray(pins=(4, 5))
                self.fail("expected ValueError")
            except ValueError as e:
                self.assertTrue("32-39" in str(e), e)
        finally:
            _pins._detect_chip = real

    def test_single_element_has_no_position(self):
        # One element is a detector flag, not a line: its "centroid"
        # would always read centre. QTRChannel is the API for it.
        flip = [False]

        def swing():
            flip[0] = not flip[0]
            return _LINE if flip[0] else _MAT
        ADC.reads = {9: swing}
        ch = QTRChannel(pin=9)
        with self.assertRaises(RuntimeError):
            ch.position()
        with self.assertRaises(RuntimeError):
            ch.left_edge_position()
        with self.assertRaises(RuntimeError):
            ch.right_edge_position()
        ch.calibrate(duration_ms=100)
        ADC.reads = {9: _LINE}
        self.assertEqual(ch.value(), 1000)
        self.assertTrue(ch.dark())
        ADC.reads = {9: _MAT}
        self.assertFalse(ch.dark())

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


class CalibrationPersistenceTests(unittest.TestCase):
    """One sweep (qtr_calibrate.py) serves every later run: the
    calibration saves to the hub filesystem and loads with loud
    failures for missing / corrupt / wrong-wiring files."""

    _PATH = "qtr_test.cal"

    def setUp(self):
        _pins._claims_reset()

    def tearDown(self):
        ADC.reads = {}
        _pins._claims_reset()
        try:
            import os
            os.remove(self._PATH)
        except OSError:
            pass

    def test_round_trip_restores_readings(self):
        qtr = _calibrated()
        qtr.save_calibration(self._PATH)
        _pins._claims_reset()
        fresh = QTRArray(pins=_PINS, pitch_mm=8.0)
        fresh.load_calibration(self._PATH)
        _script(dark_pins=(5,))
        self.assertEqual(fresh.read()[4].value, 1000)
        self.assertEqual(fresh.position(), 0.0)

    def test_missing_file_names_the_calibrate_script(self):
        _script()
        qtr = QTRArray(pins=_PINS)
        try:
            qtr.load_calibration(self._PATH)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("qtr_calibrate.py" in str(e), e)

    def test_corrupt_file_is_named(self):
        with open(self._PATH, "w") as f:
            f.write("not json{")
        _script()
        qtr = QTRArray(pins=_PINS)
        try:
            qtr.load_calibration(self._PATH)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("corrupt" in str(e), e)

    def test_wrong_wiring_is_refused(self):
        # Per-element min/max does not transfer across wiring: a
        # calibration recorded for other pins silently mis-scales
        # every reading — refuse it by name.
        qtr = _calibrated()
        qtr.save_calibration(self._PATH)
        _pins._claims_reset()
        ADC.reads = {p: _MAT for p in (1, 2, 3)}
        other = QTRArray(pins=(1, 2, 3))
        try:
            other.load_calibration(self._PATH)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("wired to" in str(e), e)

    def test_saving_uncalibrated_raises(self):
        _script()
        qtr = QTRArray(pins=_PINS)
        with self.assertRaises(RuntimeError):
            qtr.save_calibration(self._PATH)


class ElementAmbientTests(unittest.TestCase):
    """``ambient()`` is the Pybricks scale: 0 black .. 100 white,
    the inverse of the calibrated 0 (mat) .. 1000 (line) value."""

    def test_full_scale_endpoints(self):
        self.assertEqual(QTRElement(1000, 300).ambient(), 0)
        self.assertEqual(QTRElement(0, 300).ambient(), 100)

    def test_midpoint_and_direction(self):
        self.assertEqual(QTRElement(500, 300).ambient(), 50)
        darker = QTRElement(800, 300)
        lighter = QTRElement(200, 300)
        self.assertTrue(darker.ambient() < lighter.ambient())

    def test_integer_percent(self):
        self.assertEqual(QTRElement(995, 300).ambient(), 0)
        self.assertEqual(QTRElement(5, 300).ambient(), 99)
        self.assertTrue(isinstance(QTRElement(437, 300).ambient(), int))

    def test_live_reading_carries_ambient(self):
        _pins._claims_reset()
        try:
            qtr = _calibrated()
            _script(dark_pins=(5,))
            r = qtr.read()
            self.assertEqual(r[4].ambient(), 0)
            self.assertEqual(r[0].ambient(), 100)
        finally:
            ADC.reads = {}
            _pins._claims_reset()


class ReadingTests(unittest.TestCase):
    def setUp(self):
        _pins._claims_reset()
        self.qtr = _calibrated()

    def tearDown(self):
        ADC.reads = {}
        _pins._claims_reset()

    def test_read_normalizes_to_full_scale(self):
        _script(dark_pins=(5,))
        r = self.qtr.read()
        self.assertEqual(r[4].value, 1000)      # the dark element
        self.assertTrue(all(e.value == 0
                            for i, e in enumerate(r) if i != 4), r)

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

    def test_lost_line_returns_none(self):
        # All mat: the line is outside the window. The driver says
        # so with None; which side it left through is the program's
        # to remember (the centre follower keeps the last sign).
        _script(dark_pins=(9,))                 # rightmost, then gone
        self.assertTrue(self.qtr.position() > 0)
        _script(dark_pins=())
        self.assertIsNone(self.qtr.position())
        self.assertIsNone(self.qtr.read().position())
        _script(dark_pins=(1,))                 # reappears far left
        self.assertTrue(self.qtr.position() < 0)

    def test_cluster_positions_split_a_fork(self):
        # Two dark clusters (a branch): the global centroid lands
        # between them — steering into the gap — while the leftmost/
        # rightmost clusters are each line's own centre.
        _script(dark_pins=(2, 8))
        self.assertEqual(self.qtr.position(), 0.0)      # the gap
        self.assertEqual(self.qtr.leftmost_position(), -3 * 8.0)
        self.assertEqual(self.qtr.rightmost_position(), +3 * 8.0)

    def test_cluster_positions_match_on_a_single_line(self):
        _script(dark_pins=(5, 6))
        self.assertEqual(self.qtr.leftmost_position(),
                         self.qtr.position())
        self.assertEqual(self.qtr.rightmost_position(),
                         self.qtr.position())

    def test_cluster_positions_none_when_nothing_is_dark(self):
        _script(dark_pins=())
        self.assertIsNone(self.qtr.leftmost_position())
        self.assertIsNone(self.qtr.rightmost_position())

    def test_left_edge_interpolates_the_boundary(self):
        # Pins 5,6,7 dark (values 1000), pin 4 white (0): the
        # white→black crossing sits where the interpolated value
        # passes dark_threshold=300 — 30% of the way from element
        # index 3 (x=-8) toward index 4. Line width doesn't move it.
        _script(dark_pins=(5, 6, 7))
        edge = self.qtr.left_edge_position()
        self.assertAlmostEqual(edge, -8.0 + 0.3 * 8.0, places=6)

    def test_left_edge_is_left_of_the_cluster_centre(self):
        _script(dark_pins=(5, 6, 7))
        self.assertTrue(self.qtr.left_edge_position()
                        < self.qtr.leftmost_position())

    def test_left_edge_uses_partial_brightness(self):
        # A half-covered element shifts the crossing: cal 200 on the
        # white side, cal 800 on the dark side → the threshold (300)
        # is 1/6 of the way across the pitch. Raw values are chosen
        # to normalize exactly (span 58000).
        ADC.reads = {p: _MAT for p in _PINS}
        ADC.reads[5] = _MAT + 200 * (_LINE - _MAT) // 1000
        ADC.reads[6] = _MAT + 800 * (_LINE - _MAT) // 1000
        edge = self.qtr.left_edge_position()
        self.assertAlmostEqual(edge, 0.0 + (100 / 600) * 8.0,
                               places=6)

    def test_left_edge_none_when_nothing_is_dark(self):
        _script(dark_pins=())
        self.assertIsNone(self.qtr.left_edge_position())

    def test_left_edge_saturates_off_array(self):
        # Leftmost element dark: the true edge is beyond the array;
        # the estimate sits half a pitch past it so the error keeps
        # sign and magnitude.
        _script(dark_pins=(1, 2))
        self.assertEqual(self.qtr.left_edge_position(),
                         -4 * 8.0 - 4.0)

    def test_left_edge_ignores_a_right_branch(self):
        # Two clusters: the left line's edge must not move when a
        # branch appears under the right side.
        _script(dark_pins=(2,))
        alone = self.qtr.left_edge_position()
        _script(dark_pins=(2, 8))
        self.assertEqual(self.qtr.left_edge_position(), alone)

    def test_left_edge_rides_the_reading_snapshot(self):
        _script(dark_pins=(5, 6))
        reading = self.qtr.read()
        self.assertEqual(reading.left_edge_position(),
                         self.qtr.left_edge_position(reading))

    def test_right_edge_mirrors_the_interpolation(self):
        # Pins 3,4,5 dark, pin 6 white: the black→white crossing
        # sits 30% of the way from element index 5 (x=+8) back
        # toward index 4 — the exact mirror of the left-edge case.
        _script(dark_pins=(3, 4, 5))
        edge = self.qtr.right_edge_position()
        self.assertAlmostEqual(edge, 8.0 - 0.3 * 8.0, places=6)

    def test_right_edge_is_right_of_the_cluster_centre(self):
        _script(dark_pins=(3, 4, 5))
        self.assertTrue(self.qtr.right_edge_position()
                        > self.qtr.rightmost_position())

    def test_right_edge_none_when_nothing_is_dark(self):
        _script(dark_pins=())
        self.assertIsNone(self.qtr.right_edge_position())

    def test_right_edge_saturates_off_array(self):
        # Rightmost element dark: the true edge is beyond the array;
        # half a pitch past it keeps the error's sign and magnitude.
        _script(dark_pins=(8, 9))
        self.assertEqual(self.qtr.right_edge_position(),
                         4 * 8.0 + 4.0)

    def test_right_edge_ignores_a_left_branch(self):
        # Two clusters: the right line's edge must not move when a
        # branch appears under the left side.
        _script(dark_pins=(8,))
        alone = self.qtr.right_edge_position()
        _script(dark_pins=(2, 8))
        self.assertEqual(self.qtr.right_edge_position(), alone)

    def test_right_edge_rides_the_reading_snapshot(self):
        _script(dark_pins=(5, 6))
        reading = self.qtr.read()
        self.assertEqual(reading.right_edge_position(),
                         self.qtr.right_edge_position(reading))

    def test_reading_is_the_user_facing_snapshot(self):
        # THE call-site contract, verbatim from the user:
        #   reading = qtr.read()
        #   reading.max(); reading.position()
        #   reading[0].dark(); reading[1].white(); reading[-1].dark()
        _script(dark_pins=(5, 6))
        reading = self.qtr.read()
        self.assertEqual(reading.max(), 1000)
        self.assertEqual(reading.position(), 4.0)
        self.assertFalse(reading[0].dark())
        self.assertTrue(reading[1].white())
        self.assertFalse(reading[-1].dark())
        self.assertTrue(reading[4].dark())      # element 4 = pin 5
        # List-like: len, iterate, negative index; elements are NOT
        # int subclasses (MicroPython cannot reflect-compare int
        # against one — max(reading) would raise on the hub), so
        # numeric code uses .value / .values() / .max().
        self.assertEqual(len(reading), 9)
        self.assertEqual([e.dark() for e in reading],
                         [False, False, False, False, True, True,
                          False, False, False])
        self.assertEqual(reading.values()[4], 1000)
        self.assertEqual(reading.dark_count(), 2)
        self.assertFalse(reading.all_dark())
        _script(dark_pins=_PINS)                # everything on line
        self.assertTrue(self.qtr.read().all_dark())
        self.assertEqual(reading.leftmost_position(),
                         reading.rightmost_position())
        self.assertTrue("dark" in repr(reading[4]), repr(reading[4]))

    def test_channel_white_mirrors_dark(self):
        # GPIO 10: the one ADC1 pin setUp's nine-pin array left free.
        ADC.reads = {10: _swing()}
        ch = QTRChannel(pin=10)
        ch.calibrate(duration_ms=100)
        ADC.reads = {10: _LINE}
        self.assertTrue(ch.dark()); self.assertFalse(ch.white())
        ADC.reads = {10: _MAT}
        self.assertTrue(ch.white()); self.assertFalse(ch.dark())

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


class PositionsMmTests(unittest.TestCase):
    """positions_mm — non-uniform element spacing. The bench case: the
    10-pin skip pattern (QTRX ch 15,13,12,11,9,7,5,4,3,1 at spacings
    8/4/4/8/8/8/4/4/8 mm) spans a 56 mm window; the geometry must
    come from the ACTUAL coordinates, not a pitch."""

    PINS = (1, 2, 3, 4, 5)
    POS = (-12.0, -4.0, 0.0, 4.0, 12.0)

    def setUp(self):
        _pins._claims_reset()
        ADC.reads = {p: _swing() for p in self.PINS}
        self.qtr = QTRArray(pins=self.PINS, positions_mm=self.POS)
        self.qtr.calibrate(duration_ms=100, poll_ms=5)

    def tearDown(self):
        ADC.reads = {}
        _pins._claims_reset()

    def _script(self, dark_pins):
        ADC.reads = {p: (_LINE if p in dark_pins else _MAT)
                     for p in self.PINS}

    def test_single_dark_element_reads_its_own_coordinate(self):
        self._script((1,))
        self.assertEqual(self.qtr.position(), -12.0)
        self._script((5,))
        self.assertEqual(self.qtr.position(), 12.0)
        self._script((3,))
        self.assertEqual(self.qtr.position(), 0.0)

    def test_left_edge_interpolates_the_local_gap(self):
        # First dark at x=-4 with its white neighbour at x=-12: the
        # threshold crossing sits 30% into an 8 mm gap...
        self._script((2, 3, 4, 5))
        self.assertAlmostEqual(self.qtr.left_edge_position(),
                               -12.0 + 0.3 * 8.0, places=6)
        # ...but 30% into a 4 mm gap one element over. A uniform
        # pitch would get one of these wrong.
        self._script((3, 4, 5))
        self.assertAlmostEqual(self.qtr.left_edge_position(),
                               -4.0 + 0.3 * 4.0, places=6)

    def test_right_edge_interpolates_the_local_gap(self):
        self._script((1, 2, 3, 4))
        self.assertAlmostEqual(self.qtr.right_edge_position(),
                               12.0 - 0.3 * 8.0, places=6)
        self._script((1, 2, 3))
        self.assertAlmostEqual(self.qtr.right_edge_position(),
                               4.0 - 0.3 * 4.0, places=6)

    def test_off_array_edges_saturate_by_mean_spacing(self):
        # Mean spacing = 24/4 = 6 -> half = 3 beyond the end element.
        self._script((1, 2))
        self.assertEqual(self.qtr.left_edge_position(), -15.0)
        self._script((4, 5))
        self.assertEqual(self.qtr.right_edge_position(), 15.0)

    def test_positions_must_match_pins(self):
        _pins._claims_reset()
        try:
            QTRArray(pins=(1, 2, 3), positions_mm=(0.0, 4.0))
            self.fail("expected ValueError")
        except ValueError as e:
            self.assertTrue("2 entries for 3 pins" in str(e), e)

    def test_positions_must_increase(self):
        _pins._claims_reset()
        try:
            QTRArray(pins=(1, 2, 3), positions_mm=(0.0, 4.0, 4.0))
            self.fail("expected ValueError")
        except ValueError as e:
            self.assertTrue("strictly increasing" in str(e), e)

    def test_uniform_default_is_unchanged(self):
        _pins._claims_reset()
        ADC.reads = {p: _swing() for p in self.PINS}
        q = QTRArray(pins=self.PINS, pitch_mm=4.0)
        self.assertEqual(q._x_mm, [-8.0, -4.0, 0.0, 4.0, 8.0])


class PositionsAccessorTests(unittest.TestCase):
    """``positions_mm`` — the element x coordinates, left to right, so
    programs and docs name elements by where they sit instead of
    carrying the numbers."""

    def setUp(self):
        _pins._claims_reset()

    def tearDown(self):
        ADC.reads = {}
        _pins._claims_reset()

    def test_uniform_array_reports_its_derived_coordinates(self):
        ADC.reads = {p: _MAT for p in _PINS}
        qtr = QTRArray(pins=_PINS, pitch_mm=8.0)
        self.assertEqual(qtr.positions_mm,
                         (-32.0, -24.0, -16.0, -8.0, 0.0,
                          8.0, 16.0, 24.0, 32.0))
        self.assertTrue(isinstance(qtr.positions_mm, tuple))

    def test_explicit_positions_come_back_verbatim(self):
        ADC.reads = {p: _MAT for p in (1, 2, 3)}
        qtr = QTRArray(pins=(1, 2, 3), positions_mm=(-12, -4, 12))
        self.assertEqual(qtr.positions_mm, (-12.0, -4.0, 12.0))

    def test_accessor_is_a_snapshot(self):
        ADC.reads = {p: _MAT for p in (1, 2)}
        qtr = QTRArray(pins=(1, 2), pitch_mm=8.0)
        first = qtr.positions_mm
        self.assertEqual(first, qtr.positions_mm)
        self.assertEqual(qtr.position, qtr.position)   # no rebinding

    def test_channel_has_one_coordinate(self):
        ADC.reads = {9: _MAT}
        self.assertEqual(QTRChannel(pin=9).positions_mm, (0.0,))


class LineSensorTests(unittest.TestCase):
    """The firmware-configured QTRLineSensor: both layouts' pins and
    geometry live in the driver so user code never carries the
    numbers. Steering is the program's job from the element
    readings (docs/hardware.md); the driver only owns geometry."""

    def setUp(self):
        _pins._claims_reset()

    def tearDown(self):
        ADC.reads = {}
        _pins._claims_reset()

    def _line_sensor(self, **kwargs):
        ADC.reads = {p: _swing() for p in QTRLineSensor.PINS}
        qtr = QTRLineSensor(**kwargs)
        qtr.calibrate(duration_ms=100, poll_ms=5)
        return qtr

    def _script_pins(self, pins, dark_pins):
        ADC.reads = {p: (_LINE if p in dark_pins else _MAT)
                     for p in pins}

    def test_default_is_the_ten_channel_bench_window(self):
        qtr = self._line_sensor()
        self.assertEqual(qtr._pins, (1, 2, 3, 4, 5, 6, 7, 8, 9, 10))
        self.assertEqual(qtr.positions_mm,
                         (-28.0, -20.0, -16.0, -12.0, -4.0,
                          4.0, 12.0, 16.0, 20.0, 28.0))
        self.assertEqual(QTRLineSensor.PINS, qtr._pins)
        self.assertEqual(QTRLineSensor.POSITIONS_MM, qtr.positions_mm)
        self.assertEqual(len(qtr.read()), 10)

    def test_channels_10_is_the_default_spelled_out(self):
        qtr = self._line_sensor(channels=10)
        self.assertEqual(qtr._pins, QTRLineSensor.PINS)
        self.assertEqual(qtr.positions_mm, QTRLineSensor.POSITIONS_MM)

    def test_eight_channel_layout_leaves_gpio_9_and_10(self):
        # Every other QTRX channel (1,3,5,...,15) on GPIO 1..8: the
        # same 56 mm window at a uniform 8 mm pitch, with the last
        # two ADC1 pins free for a second array.
        ADC.reads = {p: _swing() for p in QTRLineSensor.PINS_8}
        qtr = QTRLineSensor(channels=8)
        qtr.calibrate(duration_ms=100, poll_ms=5)
        self.assertEqual(qtr._pins, (1, 2, 3, 4, 5, 6, 7, 8))
        self.assertEqual(qtr.positions_mm,
                         (-28.0, -20.0, -12.0, -4.0,
                          4.0, 12.0, 20.0, 28.0))
        self.assertEqual(QTRLineSensor.PINS_8, qtr._pins)
        self.assertEqual(QTRLineSensor.POSITIONS_MM_8, qtr.positions_mm)
        self.assertEqual(len(qtr.read()), 8)
        self.assertFalse(9 in qtr._pins)
        self.assertFalse(10 in qtr._pins)
        # The edge elements named in the docs: right edge under
        # index 5 (+12 mm), left under index 2 (-12 mm), two
        # mat-side elements beyond each for the branch watch.
        self.assertEqual(qtr.positions_mm[5], 12.0)
        self.assertEqual(qtr.positions_mm[2], -12.0)
        self.assertEqual(len(qtr.positions_mm[6:]), 2)
        self.assertEqual(len(qtr.positions_mm[:2]), 2)

    def test_ten_channel_edge_elements_named_in_the_docs(self):
        qtr = self._line_sensor()
        self.assertEqual(qtr.positions_mm[7], 16.0)
        self.assertEqual(qtr.positions_mm[2], -16.0)
        self.assertEqual(len(qtr.positions_mm[8:]), 2)
        self.assertEqual(len(qtr.positions_mm[:2]), 2)

    def test_eight_channel_geometry_reads_like_the_driver(self):
        ADC.reads = {p: _swing() for p in QTRLineSensor.PINS_8}
        qtr = QTRLineSensor(channels=8)
        qtr.calibrate(duration_ms=100, poll_ms=5)
        self._script_pins(QTRLineSensor.PINS_8, (6,))     # index 5
        self.assertEqual(qtr.position(), 12.0)
        self._script_pins(QTRLineSensor.PINS_8, (4, 5))   # -4, +4
        self.assertEqual(qtr.position(), 0.0)
        self._script_pins(QTRLineSensor.PINS_8, (1,))
        self.assertEqual(qtr.position(), -28.0)

    def test_channels_must_be_8_or_10(self):
        for bad in (0, 7, 9, 12, "8"):
            _pins._claims_reset()
            try:
                QTRLineSensor(channels=bad)
                self.fail("expected ValueError for %r" % (bad,))
            except ValueError as e:
                self.assertTrue("8" in str(e) and "10" in str(e), e)
                self.assertTrue(repr(bad) in str(e), e)
        # A refused construction claims nothing.
        ADC.reads = {p: _MAT for p in QTRLineSensor.PINS}
        QTRLineSensor()

    def test_right_edge_error_is_the_documented_pattern(self):
        # The docs' right-edge law on the ten-channel window:
        # 50 - r[7].ambient(). Element 7 (GPIO 8, +16 mm) fully
        # dark -> the robot drifted LEFT of the edge -> +50 (steer
        # right); fully white -> -50; midway -> 0.
        qtr = self._line_sensor()
        self._script_pins(QTRLineSensor.PINS, (8,))
        self.assertEqual(50 - qtr.read()[7].ambient(), 50)
        self._script_pins(QTRLineSensor.PINS, ())
        self.assertEqual(50 - qtr.read()[7].ambient(), -50)
        ADC.reads[8] = (_MAT + _LINE) // 2
        self.assertEqual(50 - qtr.read()[7].ambient(), 0)

    def test_left_edge_error_is_the_mirror(self):
        qtr = self._line_sensor()
        self._script_pins(QTRLineSensor.PINS, (3,))
        self.assertEqual(qtr.read()[2].ambient() - 50, -50)
        self._script_pins(QTRLineSensor.PINS, ())
        self.assertEqual(qtr.read()[2].ambient() - 50, 50)

    def test_mode_api_is_gone(self):
        qtr = self._line_sensor()
        for name in ("set_mode", "mode", "edge_error", "last_side",
                     "LEFT_SETPOINT_MM", "RIGHT_SETPOINT_MM",
                     "CENTER_SETPOINT_MM"):
            self.assertFalse(hasattr(qtr, name), name)
        self.assertFalse(hasattr(qtr.read(), "edge_error"))


class TwoArrayTests(unittest.TestCase):
    """A second array on the pins the front leaves free: disjoint
    arrays coexist, each with its own calibration file; a shared pin
    is refused at construction naming both arrays."""

    FRONT_CAL = "qtr_front_test.cal"
    REAR_CAL = "qtr_rear_test.cal"

    def setUp(self):
        _pins._claims_reset()

    def tearDown(self):
        ADC.reads = {}
        _pins._claims_reset()
        import os
        for path in (self.FRONT_CAL, self.REAR_CAL):
            try:
                os.remove(path)
            except OSError:
                pass

    def _script(self, dark_pins):
        ADC.reads = {p: (_LINE if p in dark_pins else _MAT)
                     for p in range(1, 11)}

    def test_eight_channel_front_and_rear_pair_coexist(self):
        ADC.reads = {p: _swing() for p in range(1, 11)}
        front = QTRLineSensor(channels=8)
        rear = QTRArray(pins=(9, 10), pitch_mm=8.0)
        front.calibrate(duration_ms=100, poll_ms=5)
        rear.calibrate(duration_ms=100, poll_ms=5)
        self.assertEqual(rear.positions_mm, (-4.0, 4.0))
        self._script((9, 10))
        self.assertTrue(rear.read().all_dark())
        self.assertFalse(front.read().all_dark())
        self.assertEqual(front.read().dark_count(), 0)
        self._script((6,))
        self.assertEqual(front.read().position(), 12.0)
        self.assertIsNone(rear.read().position())

    def test_two_probes_on_the_free_pins(self):
        ADC.reads = {p: _swing() for p in range(1, 11)}
        QTRLineSensor(channels=8)
        a = QTRChannel(pin=9)
        b = QTRChannel(pin=10)
        a.calibrate(duration_ms=100, poll_ms=5)
        b.calibrate(duration_ms=100, poll_ms=5)
        self._script((10,))
        self.assertFalse(a.dark())
        self.assertTrue(b.dark())

    def test_shared_pin_is_refused_naming_both_arrays(self):
        # The default ten-channel window owns GPIO 9 and 10: a rear
        # array on them collides, and the error names the pin, the
        # array asking, and the array holding it.
        ADC.reads = {p: _MAT for p in range(1, 11)}
        QTRLineSensor()
        try:
            QTRArray(pins=(9, 10), pitch_mm=8.0)
            self.fail("expected ReservedPinError")
        except _pins.ReservedPinError as e:
            msg = str(e)
        self.assertTrue("GPIO 9" in msg, msg)
        self.assertTrue("QTR array on GPIO 9,10" in msg, msg)
        self.assertTrue("QTR array on GPIO 1,2,3,4,5,6,7,8,9,10" in msg,
                        msg)
        with self.assertRaises(ValueError):
            QTRChannel(pin=10)

    def test_refused_construction_claims_nothing(self):
        # A rear array refused on its SECOND pin must not leave its
        # first pin claimed: the retry with the right pins works.
        ADC.reads = {p: _MAT for p in range(1, 11)}
        QTRLineSensor(channels=8)
        with self.assertRaises(ValueError):
            QTRArray(pins=(9, 8), pitch_mm=8.0)
        QTRArray(pins=(9, 10), pitch_mm=8.0)

    def test_same_wiring_may_be_rebuilt(self):
        # Re-constructing the same array (a program's second Hub-
        # style re-check of its own pins) is not a collision.
        ADC.reads = {p: _MAT for p in range(1, 11)}
        QTRLineSensor()
        QTRLineSensor()

    def test_each_array_keeps_its_own_calibration_file(self):
        ADC.reads = {p: _swing() for p in range(1, 11)}
        front = QTRLineSensor(channels=8)
        rear = QTRArray(pins=(9, 10), pitch_mm=8.0)
        front.calibrate(duration_ms=100, poll_ms=5)
        rear.calibrate(duration_ms=100, poll_ms=5)
        front.save_calibration(self.FRONT_CAL)
        rear.save_calibration(self.REAR_CAL)
        _pins._claims_reset()
        front2 = QTRLineSensor(channels=8)
        rear2 = QTRArray(pins=(9, 10), pitch_mm=8.0)
        front2.load_calibration(self.FRONT_CAL)
        rear2.load_calibration(self.REAR_CAL)
        self._script((6, 10))
        self.assertEqual(front2.read().position(), 12.0)
        self.assertEqual(rear2.read().position(), 4.0)
        # The files carry their wiring: swapping them is refused.
        try:
            rear2.load_calibration(self.FRONT_CAL)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("wired to (9, 10)" in str(e), e)
        with self.assertRaises(RuntimeError):
            front2.load_calibration(self.REAR_CAL)


if __name__ == "__main__":
    unittest.main()
