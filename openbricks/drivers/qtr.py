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

Example (bench: channels 15..9 left-to-right on ADC1; the stop
button and servo bus live on GPIO 39 and 14/41, off the bank)::

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
from openbricks import parameters
from openbricks.parameters import LineMode

_FULL_SCALE = 1000


class QTRElement:
    """One calibrated element reading, as returned by
    :meth:`QTRArray.read`.

    ``value`` is the 0 (mat) .. 1000 (line) int; ``dark()`` /
    ``white()`` answer per element exactly like
    :meth:`QTRChannel.dark`. Deliberately NOT an int subclass:
    MicroPython cannot reflect-compare ``int`` against one, so
    ``max()`` over such readings raises on the hub while passing on
    the desktop — numeric code uses ``.value`` instead
    (``max(e.value for e in readings)``)."""

    def __init__(self, value, threshold):
        self.value = value
        self._threshold = threshold

    def dark(self):
        """True when this element is over the line."""
        return self.value >= self._threshold

    def white(self):
        """True when this element is over the mat."""
        return self.value < self._threshold

    def ambient(self):
        """Reflected brightness as 0 (black) .. 100 (white) — the
        Pybricks scale, inverted from ``value``."""
        return (_FULL_SCALE - self.value) * 100 // _FULL_SCALE

    def __repr__(self):
        return "QTRElement(%d, %s)" % (
            self.value, "dark" if self.dark() else "white")


class QTRReading:
    """One snapshot of the whole array, as returned by
    :meth:`QTRArray.read`.

    Behaves like the list of :class:`QTRElement` it holds (index,
    iterate, ``len``) and carries every aggregate view of exactly
    this snapshot — so a control tick reads once and derives
    everything from the same coherent sample::

        r = qtr.read()
        r[0].dark()             # per element
        r.position()            # global centroid, mm
        r.left_edge_position()  # the line's left edge, mm
        r.right_edge_position() # the line's right edge, mm
        r.edge_error()          # setpoint element's ambient vs 50
        r.leftmost_position()   # fork clusters
        r.rightmost_position()
        r.dark_count()          # how many elements on the line
        r.all_dark()            # whole array on the line
        r.max()                 # brightest value (phantom gate)
    """

    def __init__(self, elements, array):
        self.elements = elements
        self._array = array

    def __len__(self):
        return len(self.elements)

    def __getitem__(self, i):
        return self.elements[i]

    def __iter__(self):
        return iter(self.elements)

    def values(self):
        """The plain 0..1000 ints, left to right."""
        return [e.value for e in self.elements]

    def max(self):
        """The brightest calibrated value in this snapshot — the
        follower's off-mat phantom gate."""
        best = 0
        for e in self.elements:
            if e.value > best:
                best = e.value
        return best

    def dark_count(self):
        return self._array.dark_count(self)

    def all_dark(self):
        """Every element over the line — with the branch flag dark
        too, the whole crossing is under the robot (the follower's
        stop condition)."""
        for e in self.elements:
            if not e.dark():
                return False
        return True

    def position(self):
        return self._array.position(self)

    def left_edge_position(self):
        return self._array.left_edge_position(self)

    def right_edge_position(self):
        return self._array.right_edge_position(self)

    def edge_error(self):
        return self._array.edge_error(self)

    def leftmost_position(self):
        return self._array.leftmost_position(self)

    def rightmost_position(self):
        return self._array.rightmost_position(self)

    def __repr__(self):
        return "QTRReading(%s)" % ",".join(
            str(e.value) for e in self.elements)


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


    def __init__(self, pins, pitch_mm=4.0, ctrl=None,
                 dark_threshold=300, positions_mm=None):
        """``positions_mm`` (optional): explicit per-element x
        coordinates in mm, left to right, strictly increasing — for
        wirings whose channels are NOT evenly spaced (e.g. the skip
        pattern QTRX ch 15,13,12,11,9,7,5,4,3,1: spacings
        8/4/4/8/8/8/4/4/8 mm span a 56 mm window on ten pins). The
        origin is wherever the caller puts it; centring the tuple on
        0 keeps ``position()`` symmetric. When given, ``pitch_mm``
        only seeds the secondary uses (the last-side hysteresis band
        and the off-array edge saturation) via the MEAN spacing; all
        real geometry comes from the positions."""
        if len(pins) < 1:
            raise ValueError("at least one element required")
        for p in pins:
            _pins.check(p, "QTR analog input", output=False)
            self._check_adc_capable(p)
        self._pins = tuple(int(p) for p in pins)
        self._threshold = int(dark_threshold)
        self._adcs = [self._make_adc(p) for p in pins]
        self._ctrl = None
        if ctrl is not None:
            from machine import Pin
            self._ctrl = Pin(ctrl, Pin.OUT, value=1)   # emitters on
        n = len(self._adcs)
        if positions_mm is not None:
            if len(positions_mm) != n:
                raise ValueError(
                    "positions_mm has %d entries for %d pins"
                    % (len(positions_mm), n))
            xs = [float(x) for x in positions_mm]
            for i in range(1, n):
                if xs[i] <= xs[i - 1]:
                    raise ValueError(
                        "positions_mm must be strictly increasing "
                        "left to right, got %r" % (positions_mm,))
            self._x_mm = xs
            self._pitch = ((xs[-1] - xs[0]) / (n - 1) if n > 1
                           else float(pitch_mm))
        else:
            self._pitch = float(pitch_mm)
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
        # Edge-following discipline, selected via set_mode(), and
        # the element each mode holds on the edge: the one nearest
        # its setpoint x.
        self._mode = None
        self._left_idx = self._nearest_index(self.LEFT_SETPOINT_MM)
        self._right_idx = self._nearest_index(self.RIGHT_SETPOINT_MM)
        # Center mode scales the centroid so the farthest element
        # from the setpoint reads +/-50.
        self._half_span = max(abs(x - self.CENTER_SETPOINT_MM)
                              for x in self._x_mm) or 1.0

    def _nearest_index(self, x_mm):
        best = 0
        for i in range(1, len(self._x_mm)):
            if abs(self._x_mm[i] - x_mm) < abs(self._x_mm[best] - x_mm):
                best = i
        return best

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

    def save_calibration(self, path):
        """Persist the calibration to a file on the hub filesystem, so
        one sweep (``examples/qtr_calibrate.py``) serves every later
        run. The wiring is stored with it: a calibration is per-
        element min/max, so loading it onto different pins would
        silently mis-scale every reading."""
        self._check_calibration()
        import json
        with open(path, "w") as f:
            json.dump({"pins": list(self._pins),
                       "min": self._cal_min,
                       "max": self._cal_max}, f)

    def load_calibration(self, path):
        """Load a calibration saved by :meth:`save_calibration`.

        Raises with the remedy when the file is missing (run the
        calibrate script), corrupt, or was recorded for DIFFERENT
        wiring. Calibration is height- and mat-dependent — resweep
        after remounting the array or changing mats."""
        import json
        try:
            with open(path) as f:
                data = json.load(f)
        except OSError:
            raise RuntimeError(
                "no saved QTR calibration at %s — run "
                "examples/qtr_calibrate.py once (sweep the array "
                "across the line while it runs)" % path)
        except ValueError:
            raise RuntimeError(
                "saved QTR calibration at %s is corrupt — delete it "
                "and re-run examples/qtr_calibrate.py" % path)
        if tuple(data.get("pins", ())) != self._pins:
            raise RuntimeError(
                "saved QTR calibration at %s was recorded for pins %s "
                "but this array is wired to %s — per-element min/max "
                "does not transfer across wiring; re-run "
                "examples/qtr_calibrate.py"
                % (path, tuple(data.get("pins", ())), self._pins))
        self._cal_min = [int(v) for v in data["min"]]
        self._cal_max = [int(v) for v in data["max"]]
        self._check_calibration()

    # ---- reading -----------------------------------------------------

    def read(self):
        """One calibrated snapshot: a :class:`QTRReading` holding a
        :class:`QTRElement` per array element, left to right —
        ``r[i].value`` 0 (mat) .. 1000 (line), ``r[i].dark()`` /
        ``r[i].white()``, and the aggregate views (``r.position()``,
        ``r.dark_count()``, ...) computed from exactly this
        sample."""
        self._check_calibration()
        out = []
        for i, v in enumerate(self._read_u16()):
            span = self._cal_max[i] - self._cal_min[i]
            n = (v - self._cal_min[i]) * _FULL_SCALE // span
            if n < 0:
                n = 0
            elif n > _FULL_SCALE:
                n = _FULL_SCALE
            out.append(QTRElement(n, self._threshold))
        return QTRReading(out, self)

    def dark_count(self, readings=None):
        """How many elements are over the line — the intersection /
        stop-bar signal (a full-width bar darkens most of the array,
        a branch stub only one side)."""
        if readings is None:
            readings = self.read()
        return sum(1 for r in readings if r.dark())

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
            if r.dark():
                seen = True
            weight_sum += r.value
            moment += r.value * x
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

    def _cluster_position(self, readings, rightmost):
        """Centroid of the leftmost (or rightmost) contiguous dark
        cluster, with one sub-threshold neighbour each side joining
        the weighting — the same between-element interpolation
        position() has. ``None`` when nothing is dark."""
        n = len(readings)
        order = range(n - 1, -1, -1) if rightmost else range(n)
        first = None
        for i in order:
            if readings[i].dark():
                first = i
                break
        if first is None:
            return None
        last = first
        step = -1 if rightmost else 1
        while 0 <= last + step < n and readings[last + step].dark():
            last += step
        i0, i1 = (last, first) if rightmost else (first, last)
        lo = i0 - 1 if i0 > 0 else 0
        hi = i1 + 1 if i1 + 1 < n else i1
        weight_sum = 0
        moment = 0.0
        for i in range(lo, hi + 1):
            weight_sum += readings[i].value
            moment += readings[i].value * self._x_mm[i]
        return moment / weight_sum

    def left_edge_position(self, readings=None):
        """Millimetre position of the line's LEFT edge — the
        white→black boundary on the left side of the leftmost dark
        cluster, in the same frame as :meth:`position` (positive =
        right of the array centre). ``None`` when nothing is dark.

        The crossing is interpolated between the last white element
        and the first dark one: the point where the calibrated value
        passes ``dark_threshold``. An edge follower keeps THIS at 0,
        so the robot straddles the boundary — half the array over
        mat, half over line — instead of centring on the line
        itself. That also makes line width irrelevant to steering.

        When the leftmost element is itself dark the true edge is
        off-array to the left; the estimate saturates half a pitch
        beyond that element, so the error keeps its sign and
        magnitude instead of vanishing.
        """
        if len(self._adcs) < 2:
            raise RuntimeError(
                "left_edge_position() needs at least 2 elements — a "
                "single detector channel has no edge to interpolate "
                "(use QTRChannel.dark() for a flag)")
        readings = self.read() if readings is None else readings
        first = None
        for i, r in enumerate(readings):
            if r.dark():
                first = i
                break
        if first is None:
            return None
        if first == 0:
            return self._x_mm[0] - self._pitch / 2
        v0 = readings[first - 1].value
        v1 = readings[first].value
        frac = (self._threshold - v0) / (v1 - v0)
        # Interpolate over the ACTUAL spacing between these two
        # elements — the pitch on a uniform array, the local gap on
        # a positions_mm one.
        span = self._x_mm[first] - self._x_mm[first - 1]
        return self._x_mm[first - 1] + frac * span

    def right_edge_position(self, readings=None):
        """Mirror of :meth:`left_edge_position`: the line's RIGHT
        edge — the black→white boundary on the right side of the
        rightmost dark cluster, in the same mm frame. ``None`` when
        nothing is dark; a dark rightmost element saturates half a
        pitch off-array to the right.

        A right-edge follower keeps THIS at 0 — the mirror-image
        track discipline of the left-edge follower, same sign
        convention (the P law is symmetric between the two)."""
        if len(self._adcs) < 2:
            raise RuntimeError(
                "right_edge_position() needs at least 2 elements — a "
                "single detector channel has no edge to interpolate "
                "(use QTRChannel.dark() for a flag)")
        readings = self.read() if readings is None else readings
        n = len(self._adcs)
        last = None
        for i in range(n - 1, -1, -1):
            if readings[i].dark():
                last = i
                break
        if last is None:
            return None
        if last == n - 1:
            return self._x_mm[n - 1] + self._pitch / 2
        v0 = readings[last + 1].value
        v1 = readings[last].value
        frac = (self._threshold - v0) / (v1 - v0)
        span = self._x_mm[last + 1] - self._x_mm[last]
        return self._x_mm[last + 1] - frac * span

    def leftmost_position(self, readings=None):
        """Centroid of the LEFTMOST contiguous dark cluster, in mm
        (same frame as :meth:`position`), or ``None`` when nothing is
        dark.

        At a branch the array sees TWO dark regions and the global
        centroid lands between them — steering into the gap. The
        leftmost cluster is the left line's own centre: a follower
        that must take the left fork steers on this while its branch
        flag says a second line is present. Computed the same way
        every tick, so switching between the two is jump-free."""
        readings = self.read() if readings is None else readings
        return self._cluster_position(readings, rightmost=False)

    def rightmost_position(self, readings=None):
        """Mirror of :meth:`leftmost_position`: the RIGHTMOST
        contiguous dark cluster's centre — the right fork, for a
        route policy that takes it."""
        readings = self.read() if readings is None else readings
        return self._cluster_position(readings, rightmost=True)

    # Edge-following setpoints, mm in the array frame — where each
    # discipline HOLDS its boundary. 0 on a plain array (edge under
    # the array centre); rig classes like :class:`QTRLineSensor`
    # override with their geometry so user code never carries the
    # numbers.
    LEFT_SETPOINT_MM = 0.0
    RIGHT_SETPOINT_MM = 0.0
    CENTER_SETPOINT_MM = 0.0

    def set_mode(self, mode):
        """Select the line-following discipline, a
        :class:`openbricks.parameters.LineMode`: ``LEFT`` holds the
        line's LEFT edge at ``LEFT_SETPOINT_MM``, ``RIGHT`` the RIGHT
        edge at ``RIGHT_SETPOINT_MM``, ``CENTER`` the line's CENTRE
        (the weighted centroid over every element) at
        ``CENTER_SETPOINT_MM``. Takes effect on the next
        :meth:`edge_error` — call it again any time to switch
        disciplines mid-run."""
        parameters.check(LineMode, mode, "mode")
        self._mode = mode

    def mode(self):
        """The selected discipline, or ``None`` before set_mode."""
        return self._mode

    def edge_error(self, readings=None):
        """Signed steering error for the discipline selected with
        :meth:`set_mode`, range -50 .. +50, positive = steer right
        (the :meth:`position` sign convention) in every mode.

        ``LineMode.LEFT`` / ``RIGHT``: how far the mode's setpoint
        element sits from the black/white boundary, as its ambient
        (0 black .. 100 white) referenced to 50 — zero exactly when
        that element straddles the edge. One element, so the error
        is proportional only within about a pitch of the setpoint
        and rails at +/-50 beyond it.

        ``LineMode.CENTER``: the line's centroid over ALL elements
        (:meth:`position`) relative to ``CENTER_SETPOINT_MM``,
        scaled so +/-50 is the far end of the window — proportional
        across the whole span. With no element dark the line is
        outside the window; the error rails toward the side it
        left through (:meth:`last_side`), and raises if the line
        has never been seen (there is nothing to steer toward)."""
        if readings is None:
            readings = self.read()
        if self._mode == LineMode.LEFT:
            return readings[self._left_idx].ambient() - 50
        if self._mode == LineMode.RIGHT:
            return 50 - readings[self._right_idx].ambient()
        if self._mode == LineMode.CENTER:
            pos = self.position(readings)
            if pos is None:
                if self._last_side == 0:
                    raise RuntimeError(
                        "center mode: no element sees the line and "
                        "it has never been seen — start with the "
                        "line inside the window")
                return 50.0 * self._last_side
            err = 50.0 * (pos - self.CENTER_SETPOINT_MM) / self._half_span
            return max(-50.0, min(50.0, err))
        raise RuntimeError(
            "no line-following mode selected — call "
            "set_mode(LineMode.LEFT / RIGHT / CENTER) first")

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
        return self.read()[0].value

    def dark(self):
        """True when the element is over a marker/line."""
        return self.value() >= self._threshold

    def white(self):
        """True when the element is over the mat."""
        return not self.dark()


class QTRLineSensor(QTRArray):
    """THE bench line sensor: one QTRX-HD-15A window of ten skip-
    pattern channels, pre-wired geometry included — construct it,
    pick a discipline, follow::

        qtr = QTRLineSensor()
        qtr.set_mode(LineMode.LEFT)            # or "right", any time
        error = qtr.read().edge_error()

    Wiring (detailed table in docs/hardware.md): QTRX channels
    1, 3, 4, 5, 7, 9, 11, 12, 13, 15 left-to-right onto GPIO 1..10
    in order — a 56 mm window at spacings 8/4/4/8/8/8/4/4/8 mm
    (the pattern is a palindrome, so board orientation only changes
    the channel LABELS, never the geometry). ``"left"`` mode holds
    the line's LEFT edge under channel 4 (-16 mm); ``"right"``
    holds the RIGHT edge under channel 12 (+16 mm) — both derived
    from the positions table, not repeated by hand. Different
    wiring: use :class:`QTRArray` directly with your own
    pins/positions/setpoints.
    """

    PINS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
    POSITIONS_MM = (-28.0, -20.0, -16.0, -12.0, -4.0,
                    4.0, 12.0, 16.0, 20.0, 28.0)
    # Channel 4 is index 2 of the window, channel 12 is index 7.
    LEFT_SETPOINT_MM = POSITIONS_MM[2]
    RIGHT_SETPOINT_MM = POSITIONS_MM[7]

    def __init__(self, dark_threshold=300):
        QTRArray.__init__(self, pins=self.PINS,
                          positions_mm=self.POSITIONS_MM,
                          dark_threshold=dark_threshold)
