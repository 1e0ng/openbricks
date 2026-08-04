# SPDX-License-Identifier: MIT
"""Tests for the ONE-CLASS serial-native path: ``DriveBase`` with
serial-bus Motor objects adopts them onto the hard-tick engine
transparently (user decision, 1.45.0 — no separate class, no user
burden).

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
from openbricks.robotics.native_drivebase import _SerialNativeEngine


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

    def db_move_wheels(self, l, r):
        self.calls.append(("db_move_wheels", l, r))
        return getattr(self, "move_wheels_ok", True)

    def db_stop(self, mode=None):
        # mode None = yield-only (abort paths); 0/1/2 = the atomic
        # coast/brake/hold staged in C. stop_ok=False models the
        # C layer refusing a hold before odometry is live.
        self.calls.append(("db_stop", mode))
        return getattr(self, "stop_ok", True)

    def db_done(self):
        left = getattr(self, "_left", 0)
        self._left = left - 1
        return left <= 0

    def db_use_gyro(self, on):
        self.calls.append(("db_use_gyro", on))

    def db_set_heading(self, deg):
        self.headings.append(deg)

    def db_gyro_source(self, mode):
        self.calls.append(("db_gyro_source", mode))

    def db_set_accel(self, dps2):
        self.calls.append(("db_set_accel", dps2))

    def servo_run(self, slot, steps):
        self.calls.append(("servo_run", slot, steps))
        return True

    def servo_coast(self, slot):
        self.calls.append(("servo_coast", slot))
        return True

    def servo_counts(self, slot):
        self.calls.append(("servo_counts", slot))
        return 0

    def servo_feedback(self, slot):
        self.calls.append(("servo_feedback", slot))
        return getattr(self, "feedback", (0, 0, True))

    def servo_move(self, slot, delta_counts, speed_cps, accel_cps2):
        self.calls.append(("servo_move", slot, delta_counts,
                           speed_cps, accel_cps2))
        self.move_refuse = getattr(self, "move_refuse", False)
        if self.move_refuse:
            return False
        self._move_left = self.done_after
        return True

    def servo_hold(self, slot):
        self.calls.append(("servo_hold", slot))
        return not getattr(self, "move_refuse", False)

    def servo_move_done(self, slot):
        left = getattr(self, "_move_left", 0)
        self._move_left = left - 1
        return left <= 0

    def reset_runtime(self):
        self.calls.append(("reset_runtime",))


class _FakeMP:
    @staticmethod
    def hard_tick_selftest():
        return True


class _FakeIMU:
    def __init__(self):
        self.h = 0.0

    def heading(self):
        return self.h


class _FakeHardIMU(_FakeIMU):
    # The ICM-45686 marker: heading lives in the hard-tick
    # integrator; the engine must select source 1 and never pump.
    _hard_heading_source = True


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
        return _SerialNativeEngine(**args)


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

    def test_hard_source_imu_selects_source_1_and_never_pumps(self):
        # ICM-45686 path: heading is computed in the hard tick; the
        # engine selects db_gyro_source(1) (which captures the frame
        # ref in C) and the Python pump must stay silent.
        db = self._db(imu=_FakeHardIMU())
        db.use_gyro(True)
        self.assertIn(("db_gyro_source", 1), self.bus.calls)
        self.bus.done_after = 3
        db.straight(100)
        self.assertEqual(self.bus.headings, [])

    def test_classic_imu_selects_source_0_and_pumps(self):
        db = self._db(imu=_FakeIMU())
        db.use_gyro(True)
        self.assertIn(("db_gyro_source", 0), self.bus.calls)
        self.bus.done_after = 3
        db.straight(100)
        self.assertTrue(len(self.bus.headings) >= 1)

    def test_use_gyro_without_imu_raises(self):
        db = self._db()
        with self.assertRaises(ValueError):
            db.use_gyro(True)

    def test_no_heading_traffic_with_gyro_off(self):
        db = self._db()
        self.bus.done_after = 3
        db.straight(100)
        self.assertEqual(self.bus.headings, [])


class AdoptionTests(_Base):
    """The one-class contract end to end: real ST3032Motor objects
    (fake machine.UART), real registry, real adopt_motors — DriveBase
    detects, releases the UART, adopts, and every drivebase + motor
    call routes through the stub bus."""

    def _motors(self):
        from openbricks.drivers.st3215 import ST3215
        from openbricks.drivers.st3032 import ST3032Motor
        ST3215._buses.clear()
        left = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                           invert=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        return left, right

    def _drivebase(self, **kw):
        from openbricks.robotics import DriveBase
        left, right = self._motors()
        db = DriveBase(left, right, wheel_diameter_mm=88,
                       axle_track_mm=138, **kw)
        return db, left, right

    def test_drivebase_adopts_and_releases_the_uart(self):
        from openbricks.drivers.st3215 import ST3215
        db, left, right = self._drivebase()
        self.assertIsNotNone(db._serial_engine)
        # UART handed over: MP driver released, registry emptied.
        self.assertEqual(len(ST3215._buses), 0)
        # Slots attached with the motors' ids + inverts.
        att = [c for c in self.bus.calls if c[0] == "servo_attach"]
        self.assertEqual(att[0][1:4], (0, 2, True))
        self.assertEqual(att[1][1:4], (1, 1, False))

    def test_straight_routes_through_the_engine(self):
        db, _, _ = self._drivebase()
        db.settings(straight_speed=200)
        self.bus.done_after = 2
        db.straight(150)
        call = [c for c in self.bus.calls if c[0] == "db_straight"][0]
        self.assertEqual(call[1], 150.0)

    def test_adopted_motor_wheel_api_routes_via_slots(self):
        db, left, right = self._drivebase()
        left.run_speed(90)      # -> servo_run on slot 0
        right.coast()           # -> servo_coast on slot 1
        names = [c[0] for c in self.bus.calls]
        self.assertIn("servo_run", names)
        self.assertIn("servo_coast", names)
        # angle() reads slot odometry.
        self.assertEqual(left.angle(), 0.0)

    def test_adopted_run_angle_routes_through_servo_move(self):
        # 1.46.0: step mode runs in C. run_angle(dps, deg) becomes a
        # per-slot position move in counts: delta = deg*4096/360,
        # speed/accel scaled by steps_per_dps.
        db, left, _ = self._drivebase()
        left.run_angle(100, 90)
        moves = [c for c in self.bus.calls if c[0] == "servo_move"]
        self.assertEqual(len(moves), 1)
        _, slot, delta, speed, accel = moves[0]
        self.assertEqual(slot, 0)
        self.assertAlmostEqual(delta, 90 * 4096 / 360.0, places=3)
        self.assertAlmostEqual(speed, 100 * 4096 / 360.0, places=3)
        self.assertGreater(accel, 0)
        # Default then="coast": end-of-move dispatch coasts the slot.
        self.assertIn(("servo_coast", 0), self.bus.calls)

    def test_adopted_run_angle_then_hold_leaves_the_c_hold(self):
        db, left, _ = self._drivebase()
        left.run_angle(100, 90, then="hold")
        # No coast, no zero-speed write after the move: the C move's
        # own position hold is the end state. (assertFalse/in — MP's
        # unittest has no assertNotIn.)
        self.assertFalse(("servo_coast", 0) in self.bus.calls)
        self.assertFalse(("servo_run", 0, 0) in self.bus.calls)

    def test_adopted_run_angle_wait_false_defers_then_to_done(self):
        db, left, _ = self._drivebase()
        left.run_angle(100, 90, wait=False)
        self.assertFalse(("servo_coast", 0) in self.bus.calls)
        while not left.done():
            pass
        self.assertIn(("servo_coast", 0), self.bus.calls)
        self.assertTrue(left.done())    # idempotent after dispatch

    def test_adopted_run_angle_refused_raises(self):
        # The C layer refuses a move while the DriveBase owns the
        # slot (or before odometry is live) — surfaced as a loud
        # RuntimeError, not a silent no-op.
        db, left, _ = self._drivebase()
        self.bus.move_refuse = True
        try:
            left.run_angle(100, 90)
            self.fail("expected RuntimeError")
        except RuntimeError:
            pass

    def test_adopted_hold_routes_through_servo_hold(self):
        db, left, _ = self._drivebase()
        left.hold()
        self.assertIn(("servo_hold", 0), self.bus.calls)

    def test_adopted_speed_load_stalled_route_through_feedback(self):
        # 1.50.0: the pump's widened read arms the last three Motor
        # methods. steps -> dps via steps_per_dps; load_raw (0.1% of
        # stall) -> mNm via STALL_TORQUE_MNM; user frame comes from
        # the slot (no double invert).
        db, left, _ = self._drivebase()
        self.bus.feedback = (455, -200, True)   # ~40 dps, -20% stall
        self.assertAlmostEqual(left.speed(), 455 / (4096 / 360.0),
                               places=3)
        self.assertAlmostEqual(left.load(),
                               -200 * left.STALL_TORQUE_MNM / 1000.0,
                               places=3)
        self.assertFalse(left.stalled())        # fast + light load

    def test_adopted_stalled_contract(self):
        db, left, _ = self._drivebase()
        # Heavy load (>= STALL_LOAD_PCT*10) + barely moving.
        self.bus.feedback = (
            int(left.STALL_SPEED_DPS * (4096 / 360.0)) - 1,
            left.STALL_LOAD_PCT * 10, True)
        self.assertTrue(left.stalled())
        # Same load but spinning fast: not stalled.
        self.bus.feedback = (3000, left.STALL_LOAD_PCT * 10, True)
        self.assertFalse(left.stalled())

    def test_adopted_feedback_stale_is_loud(self):
        # Silent bus: speed/load read None, stalled RAISES (a silent
        # bus must never read as "not stalled") — classic contract.
        db, left, _ = self._drivebase()
        self.bus.feedback = (0, 0, False)
        self.assertIsNone(left.speed())
        self.assertIsNone(left.load())
        try:
            left.stalled()
            self.fail("expected OSError")
        except OSError:
            pass

    def test_settings_acceleration_reaches_the_engine(self):
        db, _, _ = self._drivebase()
        db.settings(acceleration=800)
        self.assertIn(("db_set_accel", 800.0), self.bus.calls)

    def test_no_native_bus_raises_no_silent_fallback(self):
        # 1.45.0 contract: a runtime with serial-bus motors but no
        # st_bus behind them can't drive them closed-loop, and the
        # Python fallback loop is gone — construction must raise, not
        # quietly degrade.
        del _native.st_bus
        with self.assertRaises(RuntimeError):
            self._drivebase()


class AtomicStopTests(_Base):
    """DriveBase.stop(then=...) on the serial path routes the WHOLE
    stop — end-state included — through one C call (db_stop(mode)),
    never through per-motor coast/brake/hold dispatch: that released
    the wheels one bus transaction apart."""

    def _drivebase(self):
        from openbricks.drivers.st3215 import ST3215
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics import DriveBase
        ST3215._buses.clear()
        left = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                           invert=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        db = DriveBase(left, right, wheel_diameter_mm=88,
                       axle_track_mm=138)
        return db, left, right

    def test_stop_coast_is_one_atomic_engine_call(self):
        db, _, _ = self._drivebase()
        self.bus.calls = []
        db.stop()                              # default then="coast"
        self.assertIn(("db_stop", 0), self.bus.calls)
        names = [c[0] for c in self.bus.calls]
        self.assertFalse("servo_coast" in names)

    def test_stop_brake_and_hold_map_to_modes(self):
        db, _, _ = self._drivebase()
        self.bus.calls = []
        db.stop(then="brake")
        self.assertIn(("db_stop", 1), self.bus.calls)
        db.stop(then="hold")
        self.assertIn(("db_stop", 2), self.bus.calls)
        names = [c[0] for c in self.bus.calls]
        self.assertFalse("servo_run" in names)
        self.assertFalse("servo_hold" in names)

    def test_wait_false_done_dispatch_is_atomic_too(self):
        db, _, _ = self._drivebase()
        self.bus.done_after = 2
        db.straight(150, then="hold", wait=False)
        while not db.done():
            pass
        self.assertIn(("db_stop", 2), self.bus.calls)
        names = [c[0] for c in self.bus.calls]
        self.assertFalse("servo_hold" in names)

    def test_stop_supersedes_motor_level_pending_moves(self):
        # A wait=False run_angle's deferred then= must not fire after
        # the drivebase stop took the wheels back (new command wins —
        # the per-motor dispatch used to clear this as a side effect).
        db, left, _ = self._drivebase()
        left.run_angle(100, 90, wait=False)
        self.assertEqual(left._native_pending, "coast")
        db.stop()
        self.assertIsNone(left._native_pending)

    def test_refused_hold_raises(self):
        # C refuses a hold before slot odometry is live; the engine
        # must surface that loudly, not park a wrong silent state.
        db, _, _ = self._drivebase()
        self.bus.stop_ok = False
        try:
            db.stop(then="hold")
            self.fail("expected RuntimeError")
        except RuntimeError:
            pass

    def test_engine_abort_paths_still_yield_only(self):
        # The settle-timeout abort inside _wait calls db_stop()
        # WITHOUT a mode: the end-state dispatch belongs to the
        # caller's then=, not to the abort.
        db = self._db()
        self.bus._left = 10_000          # never done
        try:
            db._deadline = 0             # expire immediately
            db._wait()
            self.fail("expected RuntimeError")
        except RuntimeError:
            pass
        self.assertIn(("db_stop", None), self.bus.calls)


class MoveWheelsTests(_Base):
    """``move_wheels`` — the drivebase-owned SyncServoGroup
    replacement. One engine call carrying both speeds, converted
    from wheel-deg/s to encoder steps/s."""

    def _drivebase(self):
        from openbricks.drivers.st3215 import ST3215
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics import DriveBase
        ST3215._buses.clear()
        left = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                           invert=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        return DriveBase(left, right, wheel_diameter_mm=88,
                         axle_track_mm=138), left, right

    def test_speeds_convert_to_steps_and_ride_one_call(self):
        db, _, _ = self._drivebase()
        self.bus.calls = []
        db.move_wheels(200, -100)
        steps = 4096 / 360.0
        self.assertEqual(
            self.bus.calls,
            [("db_move_wheels", int(200 * steps), int(-100 * steps))])

    def test_move_wheels_supersedes_a_pending_wait_false_move(self):
        db, left, _ = self._drivebase()
        self.bus.done_after = 10
        db.straight(300, wait=False)
        left.run_angle(100, 90, wait=False)
        db.move_wheels(150, 150)
        self.assertTrue(db.done())            # db move cleared
        self.assertIsNone(left._native_pending)

    def test_refusal_raises(self):
        db, _, _ = self._drivebase()
        self.bus.move_wheels_ok = False
        try:
            db.move_wheels(100, 100)
            self.fail("expected RuntimeError")
        except RuntimeError:
            pass

    def test_move_wheels_is_estop_gated(self):
        db, _, _ = self._drivebase()
        estop.engage()
        try:
            db.move_wheels(100, 100)
            self.fail("expected KeyboardInterrupt")
        except KeyboardInterrupt:
            pass
        finally:
            estop.clear()


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
