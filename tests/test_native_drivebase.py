# SPDX-License-Identifier: MIT
"""Tests for ``openbricks.robotics.NativeDriveBase`` — the Python
face of the hard-tick serial-bus drivebase.

The class is exercised against a RECORDING FAKE bus injected through
``openbricks._native`` (the ``_bus()`` seam): every unit conversion,
the gyro outer loop's continuous-heading unwrap, and the estop gate
are pinned without hardware. The C layer underneath has its own
closed-loop suite (test_st_drivebase); these tests own the boundary.

Runs under BOTH CPython (fakes) and unix MicroPython.
"""

import tests._fakes  # noqa: F401

import math
import sys
import unittest

# CPython has no _openbricks_native at all (the C module is MP-only);
# install a minimal stand-in so the re-export shim loads. On unix MP
# the real module exists and this block is skipped.
try:
    import _openbricks_native  # noqa: F401
except ImportError:
    class _Stub:
        pass

    class _StubMotorProcess:
        # The stub outlives this module (sys.modules is process-wide
        # and other test modules import openbricks.launcher/estop in
        # the same run), so the seams THEY call must exist and no-op.
        @staticmethod
        def reset():
            pass

        @staticmethod
        def stop():
            pass

        @staticmethod
        def hard_tick_selftest():
            return True

    _stub_mod = type(sys)("_openbricks_native")
    _stub_mod.motor_process = _StubMotorProcess()
    for _name in ("Servo", "TrapezoidalProfile",
                  "Observer", "DriveBase", "PCNTEncoder",
                  "QuadratureEncoder", "BNO055"):
        setattr(_stub_mod, _name, _Stub())
    sys.modules["_openbricks_native"] = _stub_mod

from openbricks import _native, estop
from openbricks.robotics.native_drivebase import NativeDriveBase


class _FakeBus:
    """Records every call; db_done alternates so _wait exits fast."""

    def __init__(self):
        self.calls = []
        self.done_after = 2
        self.headings = []
        self.attached = set()

    def attach_uart(self, *a):
        self.calls.append(("attach_uart",) + a)
        return True

    def db_disable(self):
        self.calls.append(("db_disable",))

    def servo_detach(self, slot):
        self.calls.append(("servo_detach", slot))
        self.attached.discard(slot)

    def servo_attach(self, *a):
        # Slot-claim model: attach fails on an in-use slot, exactly
        # like the C layer — what the re-construction test rides on.
        self.calls.append(("servo_attach",) + a)
        if a[0] in self.attached:
            return False
        self.attached.add(a[0])
        return True

    def db_config(self, *a):
        self.calls.append(("db_config",) + a)

    def db_straight(self, mm, mm_s):
        self.calls.append(("db_straight", mm, mm_s))
        self._left = self.done_after

    def db_turn(self, deg, dps):
        self.calls.append(("db_turn", deg, dps))
        self._left = self.done_after

    def db_stop(self):
        self.calls.append(("db_stop",))

    def db_done(self):
        left = getattr(self, "_left", 0)
        self._left = left - 1
        return left <= 0

    def db_use_gyro(self, on):
        self.calls.append(("db_use_gyro", on))

    def db_set_heading(self, deg):
        self.headings.append(deg)


class _FakeMP:
    @staticmethod
    def hard_tick_selftest():
        return True


class _FakeIMU:
    def __init__(self):
        self.h = 0.0

    def heading(self):
        return self.h


class _Base(unittest.TestCase):
    def setUp(self):
        self.bus = _FakeBus()
        self._had = hasattr(_native, "st_bus")
        self._old_sb = getattr(_native, "st_bus", None)
        self._old_mp = _native.motor_process
        _native.st_bus = self.bus
        _native.motor_process = _FakeMP()
        estop.clear()

    def tearDown(self):
        _native.motor_process = self._old_mp
        if self._had and self._old_sb is not None:
            _native.st_bus = self._old_sb
        else:
            try:
                del _native.st_bus
            except AttributeError:
                pass

    def _db(self, **kw):
        args = dict(left_id=2, right_id=1, wheel_diameter_mm=88,
                    axle_track_mm=136, invert_left=True)
        args.update(kw)
        return NativeDriveBase(**args)


class ConstructionTests(_Base):
    def test_bench_construction_wires_the_stack(self):
        self._db()
        names = [c[0] for c in self.bus.calls]
        self.assertEqual(
            names, ["db_disable", "servo_detach", "servo_detach",
                    "attach_uart", "servo_attach", "servo_attach",
                    "db_config"])
        # Bench defaults: UART1 @1M on 14/6; left slot0 id2 inverted.
        by = {}
        for c in self.bus.calls:
            by.setdefault(c[0], []).append(c)
        self.assertEqual(by["attach_uart"][0],
                         ("attach_uart", 1, 1_000_000, 14, 6))
        self.assertEqual(by["servo_attach"][0][1:4], (0, 2, True))
        self.assertEqual(by["servo_attach"][1][1:4], (1, 1, False))

    def test_goal_acc_encoding_matches_the_driver_formula(self):
        # st3215.py::_encode_goal_acc — steps/100 units, clamped 254.
        self._db(accel_dps2=400.0)
        att = [c for c in self.bus.calls if c[0] == "servo_attach"]
        self.assertEqual(att[0][4], int(400.0 * (4096 / 360.0) / 100.0))
        self.bus.calls = []
        self._db(accel_dps2=99999.0)
        att = [c for c in self.bus.calls if c[0] == "servo_attach"]
        self.assertEqual(att[0][4], 254)

    def test_reconstruction_in_the_same_boot_succeeds(self):
        # The bench regression (bit twice in one day): a second
        # ``openbricks run`` of the same script found the previous
        # run's slots still claimed and died with "slot attach
        # failed" until a power-cycle. Construction now tears down
        # its own claims first.
        self._db()
        self._db()          # must not raise
        # And attach genuinely happened twice (not skipped).
        attaches = [c for c in self.bus.calls if c[0] == "servo_attach"]
        self.assertEqual(len(attaches), 4)

    def test_missing_native_bus_raises_informatively(self):
        del _native.st_bus
        with self.assertRaises(RuntimeError):
            self._db()


class UnitConversionTests(_Base):
    def test_straight_converts_wheel_dps_to_mm_s(self):
        db = self._db()
        db.settings(straight_speed=200)
        db.straight(150)
        call = [c for c in self.bus.calls if c[0] == "db_straight"][0]
        self.assertEqual(call[1], 150.0)
        expect = 200 * (math.pi * 88) / 360.0     # 153.6 mm/s
        self.assertTrue(abs(call[2] - expect) < 0.01, call)

    def test_turn_converts_wheel_dps_to_body_dps(self):
        db = self._db()
        db.settings(turn_rate=150)
        db.turn(90)
        call = [c for c in self.bus.calls if c[0] == "db_turn"][0]
        self.assertEqual(call[1], 90.0)
        expect = 150 * (math.pi * 88) / (math.pi * 136)  # wheel->body
        self.assertTrue(abs(call[2] - expect) < 0.01, call)


class GyroOuterLoopTests(_Base):
    def test_wait_loop_feeds_continuous_heading(self):
        imu = _FakeIMU()
        db = self._db(imu=imu)
        db.use_gyro(True)
        imu.h = 10.0
        self.bus.done_after = 3
        db.straight(100)
        self.assertTrue(len(self.bus.headings) >= 1)
        self.assertTrue(abs(self.bus.headings[-1] - 10.0) < 0.01)

    def test_heading_unwraps_across_the_180_boundary(self):
        imu = _FakeIMU()
        imu.h = 179.0
        db = self._db(imu=imu)
        db.use_gyro(True)                 # primes at 179
        imu.h = -179.0                    # +2 deg through the wrap
        self.bus.done_after = 2
        db.straight(50)
        self.assertTrue(abs(self.bus.headings[-1] - 2.0) < 0.01,
                        self.bus.headings)

    def test_use_gyro_without_imu_raises(self):
        db = self._db()
        with self.assertRaises(ValueError):
            db.use_gyro(True)

    def test_no_heading_traffic_with_gyro_off(self):
        db = self._db()
        self.bus.done_after = 3
        db.straight(100)
        self.assertEqual(self.bus.headings, [])


class EStopGateTests(_Base):
    def test_engaged_estop_blocks_moves(self):
        db = self._db()
        estop.engage()
        try:
            # The e-stop contract: check() raises KeyboardInterrupt
            # (the same unwind the stop button injects).
            with self.assertRaises(KeyboardInterrupt):
                db.straight(100)
        finally:
            estop.clear()


if __name__ == "__main__":
    unittest.main()
