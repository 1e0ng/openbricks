# SPDX-License-Identifier: MIT
"""
NativeDriveBase — the serial-bus drivebase on the hard-tick controller.

The 2-DOF coupled controller runs entirely in C on the esp_timer hard
tick (see ``st_bus``): ~220 Hz odometry per wheel, speed setpoints in
sync-write packets, immune to anything Python does — a 981 ms Python
stall that freezes the classic DriveBase's control loop does not
perturb this one. Floor-verified: closed square with 0.3 % odometry
closure at bench speeds.

Why a separate class instead of routing ``DriveBase`` transparently:
``ST3032Motor`` objects open ``machine.UART`` in their constructor,
and the native path's IDF driver on the same pins would double-own
the peripheral (a documented conflict). This class owns the bus
end-to-end — construct it with ids and pins, NOT motor objects, and
do not construct serial-bus Motor objects for the same UART in the
same boot. Motor-layer nativization (making ``ST3032Motor`` itself
slot-backed so classic ``DriveBase`` routes transparently) is a
planned follow-up.

Gyro: pass an ``imu`` and call ``use_gyro(True)`` — the wait loop
inside ``straight()`` / ``turn()`` reads the IMU at ~50-100 Hz and
feeds the heading to the C controller (``db_set_heading``). The
outer loop lives in the wait loop on purpose: correction matters
exactly while a move is in flight, and this avoids burning a
hardware timer (all four are spoken for).

Example::

    from openbricks.drivers.bno055 import BNO055
    from openbricks.drivers.tca9548a import TCA9548A
    from openbricks.robotics import NativeDriveBase
    from machine import I2C, Pin

    mux = TCA9548A(I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000))
    imu = BNO055(i2c=mux[3], address=0x29)
    db = NativeDriveBase(left_id=2, right_id=1, invert_left=True,
                         wheel_diameter_mm=88, axle_track_mm=136,
                         imu=imu)
    db.use_gyro(True)
    db.straight(300)
    db.turn(90)
"""

import math
import time

from openbricks import estop


_STEPS_PER_DEG = 4096 / 360.0
_LEFT_SLOT, _RIGHT_SLOT = 0, 1


def _bus():
    """The native bus module, or an informative error. A seam so the
    test suite can exercise the full class against a recording fake
    (the real module's firmware backend only exists on the hub)."""
    from openbricks import _native
    sb = getattr(_native, "st_bus", None)
    if sb is None or not hasattr(sb, "attach_uart"):
        raise RuntimeError(
            "NativeDriveBase needs the firmware native bus (st_bus with "
            "attach_uart) — it is not available on this build. Use "
            "DriveBase with ST3032Motor objects instead.")
    return sb


class NativeDriveBase:
    def __init__(self, left_id, right_id, wheel_diameter_mm,
                 axle_track_mm, imu=None,
                 invert_left=False, invert_right=False,
                 uart_id=1, tx=14, rx=6, baud=1_000_000,
                 accel_dps2=400.0):
        self._sb = _bus()
        self._wheel_circumference = math.pi * wheel_diameter_mm
        self._axle_track = float(axle_track_mm)
        self._imu = imu
        self._use_gyro = False
        # Continuous-heading frame, identical contract to the classic
        # fallback path (absolute target frame, Pybricks-style —
        # overshoot is corrected by the NEXT move, not accumulated).
        self._gyro_cont = 0.0
        self._gyro_prev = None
        self._straight_speed_dps = 200
        self._turn_rate_dps = 150

        from openbricks._native import motor_process
        motor_process.hard_tick_selftest()      # dispatcher on (idempotent)
        # Same-boot re-construction: a previous run's slots and
        # drivebase survive in the C singletons (openbricks run keeps
        # the interpreter alive between scripts), and servo_attach
        # rejects an in-use slot — so a second run of the same script
        # failed with "slot attach failed" until a power-cycle. Tear
        # down our own claims first; detach of an unclaimed slot is
        # silent, so a fresh boot pays nothing.
        self._sb.db_disable()
        self._sb.servo_detach(_LEFT_SLOT)
        self._sb.servo_detach(_RIGHT_SLOT)
        if not self._sb.attach_uart(uart_id, baud, tx, rx):
            raise RuntimeError("attach_uart(%d) failed" % uart_id)
        acc = int(accel_dps2 * _STEPS_PER_DEG / 100.0)
        acc = 0 if acc < 0 else (254 if acc > 254 else acc)
        if not self._sb.servo_attach(_LEFT_SLOT, left_id,
                                     bool(invert_left), acc):
            raise RuntimeError("left servo slot attach failed")
        if not self._sb.servo_attach(_RIGHT_SLOT, right_id,
                                     bool(invert_right), acc):
            raise RuntimeError("right servo slot attach failed")
        self._sb.db_config(_LEFT_SLOT, _RIGHT_SLOT,
                           float(wheel_diameter_mm), float(axle_track_mm),
                           float(accel_dps2))

    # -- configuration ---------------------------------------------------

    def settings(self, straight_speed=None, turn_rate=None):
        """Cruise parameters in WHEEL-deg/s — the same units as the
        classic ``DriveBase.settings`` for drop-in parity."""
        if straight_speed is not None:
            self._straight_speed_dps = straight_speed
        if turn_rate is not None:
            self._turn_rate_dps = turn_rate

    def use_gyro(self, enable):
        """Heading feedback from the IMU instead of the wheel
        differential. Requires ``imu`` at construction."""
        enable = bool(enable)
        if enable and self._imu is None:
            raise ValueError("use_gyro(True) needs an imu")
        if enable and not self._use_gyro:
            # Fresh absolute frame: current heading is zero/target.
            self._gyro_cont = 0.0
            self._gyro_prev = self._imu.heading()
        self._use_gyro = enable
        self._sb.db_use_gyro(enable)

    # -- moves -----------------------------------------------------------

    def straight(self, distance_mm):
        estop.check()
        mm_s = self._straight_speed_dps * self._wheel_circumference / 360.0
        self._sb.db_straight(float(distance_mm), float(mm_s))
        self._wait()

    def turn(self, angle_deg):
        """Body degrees, CW-positive (Pybricks convention)."""
        estop.check()
        # turn_rate is WHEEL-deg/s (settings parity with the classic
        # DriveBase); the C API takes BODY deg/s. One body-degree of
        # turn-in-place is pi*axle/360 mm of arc per wheel, and one
        # wheel-degree is circumference/360 mm — so:
        body_dps = (self._turn_rate_dps * self._wheel_circumference
                    / (math.pi * self._axle_track))
        self._sb.db_turn(float(angle_deg), float(body_dps))
        self._wait()

    def stop(self):
        self._sb.db_stop()

    def done(self):
        return bool(self._sb.db_done())

    # -- internals -------------------------------------------------------

    def _gyro_pump(self):
        """One outer-loop iteration: read the IMU, unwrap across the
        +/-180 boundary into the continuous frame, feed the C
        controller. ~50-100 Hz from the wait loop."""
        h = self._imu.heading()
        d = h - self._gyro_prev
        if d > 180.0:
            d -= 360.0
        elif d < -180.0:
            d += 360.0
        self._gyro_cont += d
        self._gyro_prev = h
        self._sb.db_set_heading(self._gyro_cont)

    _SETTLE_TIMEOUT_MS = 8000

    def _wait(self):
        deadline = time.ticks_ms() + self._SETTLE_TIMEOUT_MS
        while not self._sb.db_done():
            estop.check()
            if self._use_gyro:
                self._gyro_pump()
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                # done now requires ARRIVAL, not just profile expiry
                # (the +4.5-deg banked-overshoot fix) — so a wheel
                # that physically can't reach the target must raise,
                # same contract as the classic fallback.
                self._sb.db_stop()
                raise RuntimeError(
                    "NativeDriveBase move did not reach target within "
                    "%d ms — wheel stalled, blocked, or gyro frame "
                    "diverged" % self._SETTLE_TIMEOUT_MS)
            time.sleep_ms(10)
