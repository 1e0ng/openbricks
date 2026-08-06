# SPDX-License-Identifier: MIT
"""
Pololu QTR / QTRX reflectance sensor arrays (analog outputs).

A row of IR emitter/phototransistor pairs a few millimetres above the
mat: dark line = high reading, light mat = low. Unlike a pair of
colour sensors, the array gives a CONTINUOUS line position — a
weighted centroid across all elements — so a follower steers on a
real analog error instead of edge-crossings.

Wiring (QTRX-HD-15A on ESP32-S3): the analog outputs go to ADC1 pins
(GPIO 1..10 — ADC2 fights the radios). Any subset of the array's
channels works; pass the pins left-to-right as mounted, and set
``pitch_mm`` to the spacing of the channels you actually wired
(4 mm for adjacent QTRX-HD channels, 8 mm if every other one).
CTRL (emitter enable) may be tied high or given a pin.

Readings are ratiometric to whatever height and mat you mounted over,
so the array MUST be calibrated once per session: sweep it across the
line while ``calibrate()`` runs. Reading before calibration raises —
an uncalibrated centroid is a plausible-looking wrong number.

Example (bench: channels 15..9 left-to-right on ADC1; GPIO 4/6
belong to the stop button and servo bus)::

    from openbricks.drivers.qtr import QTRArray, QTRChannel

    line   = QTRArray(pins=(1, 2, 3, 7, 8, 9, 10), pitch_mm=4.0)
    branch = QTRChannel(pin=5)        # array channel 1, far right
    line.calibrate(duration_ms=3000)  # sweep across the line now
    branch.calibrate(duration_ms=3000)
    while True:
        pos = line.position()         # mm, +right of centre, or None
"""

import time

from openbricks import pins as _pins


class QTRArray:
    """Analog QTR/QTRX reflectance array on ESP32 ADC pins.

    Args:
        pins: ADC-capable GPIO numbers, LEFT to RIGHT as mounted.
        pitch_mm: physical spacing between the wired channels.
        ctrl: optional emitter-control GPIO (QTRX CTRL). Driven high
            at construction (emitters on). ``None`` = tied high.
        dark_threshold: calibrated value (0..1000) above which an
            element counts as "over the line" for ``position()`` /
            ``dark_count()``.
    """

    _FULL_SCALE = 1000

    def __init__(self, pins, pitch_mm=4.0, ctrl=None,
                 dark_threshold=300):
        if len(pins) < 1:
            raise ValueError("at least one element required")
        for p in pins:
            _pins.check(p, "QTR analog input", output=False)
            self._check_adc_capable(p)
        self._pitch = float(pitch_mm)
        self._threshold = int(dark_threshold)
        self._adcs = [self._make_adc(p) for p in pins]
        self._ctrl = None
        if ctrl is not None:
            from machine import Pin
            self._ctrl = Pin(ctrl, Pin.OUT, value=1)   # emitters on
        n = len(self._adcs)
        # Element x-positions in mm, centre of the wired span at 0.
        self._x_mm = [(i - (n - 1) / 2.0) * self._pitch
                      for i in range(n)]
        self._cal_min = None
        self._cal_max = None
        # Which side the line last left through (+1 right, -1 left):
        # when every element reads mat, the line is OUTSIDE the span
        # and this is the only information left. Follower logic uses
        # it to steer back instead of guessing.
        self._last_side = 0

    @staticmethod
    def _check_adc_capable(pin):
        """Refuse pins with no usable ADC, BY NAME, at construction.

        ``pins.check`` validates the board's GPIO map but not analog
        capability, and ``machine.ADC`` fails with a bare ValueError —
        after the user already soldered the harness (bench 2026-08-06:
        five channels landed on GPIO 38-42, which have no ADC at all
        on the S3).
        """
        chip = _pins._detect_chip()
        if chip == "esp32s3":
            if not 1 <= pin <= 10:
                if 11 <= pin <= 20:
                    raise ValueError(
                        "GPIO %d is ADC2 on the ESP32-S3 — unusable "
                        "for the QTR array: ADC2 is shared with the "
                        "radio (BLE is up during `openbricks run`) "
                        "and unreliable by errata. Use ADC1, GPIO "
                        "1-10." % pin)
                raise ValueError(
                    "GPIO %d has no ADC on the ESP32-S3 — analog "
                    "inputs must be ADC1, GPIO 1-10." % pin)
        elif chip == "esp32":
            if not 32 <= pin <= 39:
                raise ValueError(
                    "GPIO %d is not an ADC1 pin on the classic ESP32 "
                    "— use GPIO 32-39 (ADC2 is shared with the "
                    "radio)." % pin)

    @staticmethod
    def _make_adc(pin):
        from machine import ADC, Pin
        adc = ADC(Pin(pin))
        # Full 0..~3.3 V range; the QTRX output swings to its supply.
        # Ports without attenuation control (unix) simply skip it.
        try:
            adc.atten(ADC.ATTN_11DB)
        except AttributeError:
            pass
        return adc

    def _read_u16(self):
        return [adc.read_u16() for adc in self._adcs]

    # ---- calibration -------------------------------------------------

    def calibrate(self, duration_ms=3000, poll_ms=5):
        """Learn each element's mat/line extremes.

        Sweep the array across the line while this runs (rotate the
        robot, or slide it by hand). Extends any previous calibration
        rather than replacing it, so repeated calls refine.
        """
        if self._cal_min is None:
            self._cal_min = [65535] * len(self._adcs)
            self._cal_max = [0] * len(self._adcs)
        deadline = time.ticks_add(time.ticks_ms(), int(duration_ms))
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            for i, v in enumerate(self._read_u16()):
                if v < self._cal_min[i]:
                    self._cal_min[i] = v
                if v > self._cal_max[i]:
                    self._cal_max[i] = v
            time.sleep_ms(poll_ms)
        self._check_calibration()

    def _check_calibration(self):
        if self._cal_min is None:
            raise RuntimeError(
                "QTR array is not calibrated — call calibrate() while "
                "sweeping the array across the line (an uncalibrated "
                "centroid is a plausible-looking wrong number)")
        # An element whose span never opened up saw only mat (or only
        # line, or is unwired): its normalized reading would be noise
        # amplified to full scale. Name it.
        flat = [i for i in range(len(self._adcs))
                if self._cal_max[i] - self._cal_min[i] < 1024]
        if flat:
            raise RuntimeError(
                "QTR calibration saw no line/mat contrast on "
                "element(s) %s (left=0) — that channel is unwired, "
                "or the sweep never carried it across the line"
                % ",".join(str(i) for i in flat))

    # ---- reading -----------------------------------------------------

    def read(self):
        """Calibrated readings, one per element, 0 (mat) .. 1000
        (line), left to right."""
        self._check_calibration()
        out = []
        for i, v in enumerate(self._read_u16()):
            span = self._cal_max[i] - self._cal_min[i]
            n = (v - self._cal_min[i]) * self._FULL_SCALE // span
            if n < 0:
                n = 0
            elif n > self._FULL_SCALE:
                n = self._FULL_SCALE
            out.append(n)
        return out

    def dark_count(self, readings=None):
        """How many elements are over the line — the intersection /
        stop-bar signal (a full-width bar darkens most of the array,
        a branch stub only one side)."""
        if readings is None:
            readings = self.read()
        return sum(1 for r in readings if r >= self._threshold)

    def position(self, readings=None):
        """Line centre in mm relative to the array centre; positive =
        line is to the RIGHT. ``None`` when no element sees the line —
        use :meth:`last_side` to know which way it escaped.
        """
        if len(self._adcs) < 2:
            raise RuntimeError(
                "position() needs at least 2 elements — a single "
                "detector channel has no centroid (use QTRChannel"
                ".dark() for a flag)")
        readings = self.read() if readings is None else readings
        weight_sum = 0
        moment = 0.0
        seen = False
        for r, x in zip(readings, self._x_mm):
            if r >= self._threshold:
                seen = True
            weight_sum += r
            moment += r * x
        if not seen:
            return None
        pos = moment / weight_sum
        # Remember the escape side while the line is still visible:
        # strictly by sign, so a centred line keeps the previous
        # memory instead of flapping.
        if pos > self._pitch / 2:
            self._last_side = 1
        elif pos < -self._pitch / 2:
            self._last_side = -1
        return pos

    def last_side(self):
        """+1 if the line was last seen right of centre, -1 left,
        0 if it has never been off-centre. The recovery hint for a
        follower that lost the line entirely."""
        return self._last_side

    def emitters(self, on):
        """Drive the CTRL pin (no-op when CTRL is tied high)."""
        if self._ctrl is not None:
            self._ctrl.value(1 if on else 0)


class QTRChannel(QTRArray):
    """One reflectance element with the array's calibrate/read
    contract — for a DETECTOR channel wired apart from the line
    cluster (a branch / marker flag).

    Kept out of :class:`QTRArray` on purpose: a flag element folded
    into the steering centroid would yank the position toward every
    marker it passes. Steer on the array; DECIDE on this.

    Example (bench: QTRX channel 1, far right, on GPIO 9)::

        branch = QTRChannel(pin=9)
        branch.calibrate(duration_ms=3000)   # same sweep as the array
        if branch.dark():
            ...   # marker under the flag channel
    """

    def __init__(self, pin, dark_threshold=300):
        QTRArray.__init__(self, pins=(pin,),
                          dark_threshold=dark_threshold)

    def value(self):
        """Calibrated reading, 0 (mat) .. 1000 (marker/line)."""
        return self.read()[0]

    def dark(self):
        """True when the element is over a marker/line."""
        return self.value() >= self._threshold
