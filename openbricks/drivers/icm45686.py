# SPDX-License-Identifier: MIT
"""
TDK ICM-45686 raw 6-axis IMU — the hard-tick heading source.

Unlike the BNO055 (fused heading over I2C, pumped from Python at
~50-100 Hz during moves), this part is read INSIDE the 1 kHz hard
tick over SPI (~13 µs per burst): gyro-Z integrates into a
continuous heading in C (``imu_yaw_core`` — stationarity-gated bias
learning, Pybricks-Prime architecture), and a gyro ``DriveBase``
consumes it every millisecond with no Python in the loop.

Wiring: 4 free GPIOs to the breakout's SPI pins (SCLK/MOSI(SDI)/
MISO(SDO)/CS) — the I2C bus, mux and color sensors are untouched.
INT/FSYNC/CLKIN stay unwired: the hard tick polls, nothing
interrupts.

Example::

    from openbricks.drivers.icm45686 import ICM45686
    from openbricks.drivers.st3032 import ST3032Motor
    from openbricks.robotics import DriveBase

    imu = ICM45686(sck=12, mosi=13, miso=11, cs=17)
    left  = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41, invert=True)
    right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
    db = DriveBase(left, right, wheel_diameter_mm=88,
                   axle_track_mm=138, imu=imu)
    db.use_gyro(True)      # heading now corrects at 1 kHz in C

Calibration: gyro bias learns automatically during stillness (fast
at boot rest, slow tracking after). ``save_calibration()`` persists
the learned bias to NVS; the next construction seeds from it (the
pbio trick), so boot-and-immediately-run starts corrected.
"""

from openbricks import pins


_NVS_NAMESPACE = "openbricks"
_NVS_KEY = "imu_bias_udps"      # micro-dps, stored as int


class ICM45686:
    # DriveBase's serial engine checks this marker: heading comes
    # from the hard-tick integrator (db_gyro_source(1)), so the
    # Python gyro pump is skipped entirely.
    _hard_heading_source = True

    def __init__(self, sck, mosi, miso, cs, hz=8_000_000, mode=3,
                 scale=-1.0):
        """
        Args:
            sck, mosi, miso, cs: SPI pins to the breakout.
            hz: SPI clock (8 MHz default; the part takes 24).
            mode: SPI mode (3 default — verify against your breakout
                if ``who_am_i mismatch`` is raised).
            scale: gyro-Z multiplier into the CW-positive body frame.
                -1.0 for a top-mounted part (Z up, right-hand rule);
                +1.0 if mounted upside down. Fine-trim after a
                360-degree spin test, like the axle track.
        """
        for pin, what in ((sck, "ICM-45686 SCK"), (mosi, "ICM-45686 MOSI"),
                          (cs, "ICM-45686 CS")):
            pins.check(pin, what)
        pins.check(miso, "ICM-45686 MISO", output=False)
        from openbricks import _native
        icm = getattr(_native, "icm45686", None)
        if icm is None:
            raise OSError("icm45686 native module missing on this build")
        icm.config(sck=sck, mosi=mosi, miso=miso, cs=cs,
                   hz=hz, mode=mode, scale=scale)
        self._icm = icm
        self._mp = _native.motor_process
        self._load_calibration()

    def heading(self):
        """Continuous body heading in degrees, CW-positive — the
        hard-tick integrator's value. Same contract as
        ``BNO055.heading()`` except unwrapped (multi-turn) rather
        than [-180, 180); ``DriveBase`` accepts both."""
        return self._mp.hard_yaw_deg()

    def reset_heading(self):
        """Zero the heading frame (calibration is kept).

        Refused (``OSError`` from the yaw binding) while a DriveBase
        steers by this gyro — Pybricks parity: "Can't reset heading
        while gyro in use". The controller's measured frame would
        shift under its held target and the next move veers chasing
        the difference (bench 2026-08-13: straight() after turn(-90)
        + reset_heading() pivoted left). Use ``db.reset()`` — it
        re-bases the controller and the heading together — or
        ``use_gyro(False)`` first.
        """
        self._mp.hard_yaw_reset()

    def gyro(self):
        """(x, y, z) rates in dps from the last hard-tick sample."""
        s = self._icm.read()
        return (s[3], s[4], s[5])

    def acceleration(self):
        """(x, y, z) in g from the last hard-tick sample."""
        s = self._icm.read()
        return (s[0], s[1], s[2])

    def calibrated(self):
        """True once the bias estimator has locked (robot has been
        still for ~0.5 s since boot, or a saved calibration was
        loaded)."""
        _bias, locked, _still = self._mp.hard_yaw_state()
        return bool(locked)

    def save_calibration(self):
        """Persist the learned gyro bias to NVS so the next boot
        starts corrected instead of waiting for stillness."""
        bias, locked, _still = self._mp.hard_yaw_state()
        if not locked:
            raise OSError("no calibration to save yet (never still)")
        try:
            from esp32 import NVS
        except ImportError:
            raise OSError("NVS unavailable on this build")
        nvs = NVS(_NVS_NAMESPACE)
        nvs.set_i32(_NVS_KEY, int(bias * 1_000_000))
        nvs.commit()

    def _load_calibration(self):
        try:
            from esp32 import NVS
            nvs = NVS(_NVS_NAMESPACE)
            udps = nvs.get_i32(_NVS_KEY)
        except (ImportError, OSError):
            return          # no saved calibration: live learning only
        self._mp.hard_yaw_seed_bias(udps / 1_000_000)

    def stats(self):
        """(reads_ok, read_errors, configured) — hard-tick health."""
        return self._icm.stats()
