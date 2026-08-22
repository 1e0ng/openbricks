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
from openbricks.drivers.qtr import QTRArray, QTRElement
from openbricks.parameters import LineMode


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
        from openbricks.drivers.qtr import QTRChannel
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
        from openbricks.drivers.qtr import QTRChannel
        ADC.reads = {9: _swing()}
        ch = QTRChannel(pin=9)
        ch.calibrate(duration_ms=100)
        ADC.reads = {9: _LINE}
        self.assertTrue(ch.dark()); self.assertFalse(ch.white())
        ADC.reads = {9: _MAT}
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


class ModeAndLineSensorTests(unittest.TestCase):
    """set_mode / edge_error and the firmware-configured
    QTRLineSensor: the geometry (pins, positions, setpoints) lives
    in the driver so user code never carries the numbers."""

    def setUp(self):
        _pins._claims_reset()

    def tearDown(self):
        ADC.reads = {}
        _pins._claims_reset()

    def _base_array(self):
        ADC.reads = {p: _swing() for p in _PINS}
        qtr = QTRArray(pins=_PINS, pitch_mm=8.0)
        qtr.calibrate(duration_ms=100, poll_ms=5)
        return qtr

    def test_set_mode_validates(self):
        qtr = self._base_array()
        # A string — even the right word — is not a mode: the
        # error names the members and calls the string out.
        for bad in ("middle", "left"):
            try:
                qtr.set_mode(bad)
                self.fail("expected TypeError")
            except TypeError as e:
                self.assertTrue("LineMode.LEFT" in str(e)
                                and "LineMode.RIGHT" in str(e)
                                and "LineMode.CENTER" in str(e), e)
                self.assertTrue(repr(bad) in str(e), e)
        self.assertTrue(qtr.mode() is None)
        for mode in LineMode.members():
            qtr.set_mode(mode)
            self.assertTrue(qtr.mode() is mode)

    def test_edge_error_before_set_mode_raises_with_remedy(self):
        qtr = self._base_array()
        _script(dark_pins=(5,))
        try:
            qtr.edge_error()
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("set_mode" in str(e), e)

    def test_base_array_setpoints_default_to_centre(self):
        # Setpoint 0.0 on the 9-pin rig picks the middle element
        # (pin 5, index 4) for both modes; fully dark there means
        # ambient 0 -> full-scale error, mode-signed.
        qtr = self._base_array()
        _script(dark_pins=(5,))
        qtr.set_mode(LineMode.LEFT)
        self.assertEqual(qtr.edge_error(), -50)
        qtr.set_mode(LineMode.RIGHT)
        self.assertEqual(qtr.edge_error(), 50)

    def test_edge_error_rails_toward_the_line_when_all_white(self):
        # Nothing dark: the setpoint element reads ambient 100 and
        # the error rails at the sign that steers back toward the
        # line's side (right of the edge in left mode, left of it
        # in right mode).
        qtr = self._base_array()
        _script(dark_pins=())
        qtr.set_mode(LineMode.LEFT)
        self.assertEqual(qtr.edge_error(), 50)
        qtr.set_mode(LineMode.RIGHT)
        self.assertEqual(qtr.edge_error(), -50)

    def _line_sensor(self):
        from openbricks.drivers.qtr import QTRLineSensor
        ADC.reads = {p: _swing() for p in QTRLineSensor.PINS}
        qtr = QTRLineSensor()
        qtr.calibrate(duration_ms=100, poll_ms=5)
        return qtr

    def _script10(self, dark_pins):
        from openbricks.drivers.qtr import QTRLineSensor
        ADC.reads = {p: (_LINE if p in dark_pins else _MAT)
                     for p in QTRLineSensor.PINS}

    def test_line_sensor_geometry_is_the_bench_window(self):
        from openbricks.drivers.qtr import QTRLineSensor
        qtr = self._line_sensor()
        self.assertEqual(qtr._pins, (1, 2, 3, 4, 5, 6, 7, 8, 9, 10))
        self.assertEqual(qtr._x_mm,
                         [-28.0, -20.0, -16.0, -12.0, -4.0,
                          4.0, 12.0, 16.0, 20.0, 28.0])
        # Setpoints DERIVE from the positions table: channel 12 is
        # window index 2, channel 4 is index 7.
        self.assertEqual(QTRLineSensor.LEFT_SETPOINT_MM,
                         QTRLineSensor.POSITIONS_MM[2])
        self.assertEqual(QTRLineSensor.RIGHT_SETPOINT_MM,
                         QTRLineSensor.POSITIONS_MM[7])
        self.assertEqual(QTRLineSensor.LEFT_SETPOINT_MM, -16.0)
        self.assertEqual(QTRLineSensor.RIGHT_SETPOINT_MM, 16.0)
        # The setpoint ELEMENTS resolve to those indices too.
        self.assertEqual(qtr._left_idx, 2)
        self.assertEqual(qtr._right_idx, 7)

    def test_line_sensor_error_is_the_setpoint_elements_ambient(self):
        # Left mode watches GPIO 3 (window index 2): fully dark
        # there -> ambient 0 -> error -50; a raw reading midway
        # between the calibration extremes -> ambient 50 -> error 0
        # (the element straddles the edge).
        qtr = self._line_sensor()
        qtr.set_mode(LineMode.LEFT)
        self._script10((3,))
        self.assertEqual(qtr.edge_error(), -50)
        self._script10(())
        ADC.reads[3] = (_MAT + _LINE) // 2
        self.assertEqual(qtr.edge_error(), 0)

    def test_line_sensor_switches_modes_mid_run(self):
        # Same snapshot, both disciplines: dark under both setpoint
        # elements (GPIO 3 and GPIO 8) reads as "onto the line" in
        # left mode and "onto the line" from the other side in
        # right mode — opposite signs, no reconstruction.
        qtr = self._line_sensor()
        self._script10((3, 8))
        reading = qtr.read()
        qtr.set_mode(LineMode.LEFT)
        self.assertEqual(qtr.edge_error(reading), -50)
        qtr.set_mode(LineMode.RIGHT)
        self.assertEqual(qtr.edge_error(reading), 50)

    # ---- center mode: all ten elements ----

    def test_center_error_is_zero_on_a_centred_line(self):
        qtr = self._line_sensor()
        qtr.set_mode(LineMode.CENTER)
        self._script10((5, 6))          # x = -4 and +4: centroid 0
        self.assertEqual(qtr.edge_error(), 0.0)

    def test_center_error_is_proportional_across_the_window(self):
        # One dark element at a time, left to right: the error must
        # rise monotonically through zero and reach the rails at the
        # outermost elements (x = -28 / +28 mm -> -50 / +50).
        from openbricks.drivers.qtr import QTRLineSensor
        qtr = self._line_sensor()
        qtr.set_mode(LineMode.CENTER)
        errs = []
        for pin in QTRLineSensor.PINS:
            self._script10((pin,))
            errs.append(qtr.edge_error())
        for a, b in zip(errs, errs[1:]):
            self.assertTrue(b > a, errs)
        self.assertEqual(errs[0], -50.0)
        self.assertEqual(errs[-1], 50.0)
        # The left-five scenario: right edge over the left half of
        # the window is a distinct, graded error — not a rail.
        self.assertTrue(-50.0 < errs[3] < errs[4] < 0.0, errs)
        # Scale: element at +16 mm (GPIO 8) reads 50 * 16 / 28.
        self.assertAlmostEqual(errs[7], 50.0 * 16.0 / 28.0, places=6)

    def test_center_error_sign_matches_position(self):
        qtr = self._line_sensor()
        qtr.set_mode(LineMode.CENTER)
        self._script10((9,))            # +20 mm: line right -> +
        self.assertTrue(qtr.edge_error() > 0)
        self.assertTrue(qtr.position() > 0)
        self._script10((2,))            # -20 mm: line left -> -
        self.assertTrue(qtr.edge_error() < 0)

    def test_center_error_rails_toward_the_escape_side(self):
        qtr = self._line_sensor()
        qtr.set_mode(LineMode.CENTER)
        self._script10((10,))           # seen far right...
        qtr.edge_error()
        self._script10(())              # ...then gone
        self.assertEqual(qtr.edge_error(), 50.0)
        self._script10((1,))            # seen far left, then gone
        qtr.edge_error()
        self._script10(())
        self.assertEqual(qtr.edge_error(), -50.0)

    def test_center_error_raises_when_line_never_seen(self):
        qtr = self._line_sensor()
        qtr.set_mode(LineMode.CENTER)
        self._script10(())
        try:
            qtr.edge_error()
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("never been seen" in str(e), e)

    def test_center_mode_on_the_base_array_uses_its_own_span(self):
        # 9 uniform 8 mm pins: half-span 32 mm; the end pins rail.
        qtr = self._base_array()
        qtr.set_mode(LineMode.CENTER)
        _script(dark_pins=(1,))
        self.assertEqual(qtr.edge_error(), -50.0)
        _script(dark_pins=(9,))
        self.assertEqual(qtr.edge_error(), 50.0)
        _script(dark_pins=(5,))
        self.assertEqual(qtr.edge_error(), 0.0)

    def test_center_error_switches_mid_run_on_one_snapshot(self):
        qtr = self._line_sensor()
        self._script10((3, 4, 5, 6, 7, 8))   # -16 .. +16: centred
        reading = qtr.read()
        qtr.set_mode(LineMode.CENTER)
        self.assertEqual(qtr.edge_error(reading), 0.0)
        qtr.set_mode(LineMode.LEFT)
        self.assertEqual(qtr.edge_error(reading), -50)

    def test_reading_edge_error_delegates(self):
        qtr = self._line_sensor()
        qtr.set_mode(LineMode.LEFT)
        self._script10((4, 5, 6))
        reading = qtr.read()
        self.assertEqual(reading.edge_error(),
                         qtr.edge_error(reading))


if __name__ == "__main__":
    unittest.main()
