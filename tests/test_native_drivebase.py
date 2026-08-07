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
import time
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

    def servo_slot_of(self, servo_id):
        # One physical servo, one slot — the real bus answers this so
        # a re-run reuses a claim instead of consuming a second slot.
        for slot, sid in getattr(self, "slot_ids", {}).items():
            if sid == servo_id:
                return slot
        return -1

    def servo_attach(self, *a):
        # Slot-claim model: attach fails on an in-use slot, exactly
        # like the C layer — what the re-construction test rides on.
        self.calls.append(("servo_attach",) + a)
        if a[0] in self.attached:
            return False
        self.attached.add(a[0])
        if not hasattr(self, "slot_ids"):
            self.slot_ids = {}
        self.slot_ids[a[0]] = a[1]
        return True

    def db_config(self, *a):
        self.calls.append(("db_config",) + a)

    def db_straight(self, mm, mm_s):
        self.calls.append(("db_straight", mm, mm_s))
        self._left = self.done_after

    def db_turn(self, deg, dps):
        self.calls.append(("db_turn", deg, dps))
        self._left = self.done_after

    def servo_stats(self, slot):
        # (reads_ok, reads_failed, stale). ``dead_slots`` marks wheels
        # that never answered — the unplugged / wrong-id / no-power
        # case. A slot named by ``fault_bits`` reports the shape the C
        # layer would actually have when it latches: the fault fires
        # only after a run of failures, so "faulted but 0 stale" is
        # not a state real hardware can be in.
        if slot in getattr(self, "wedged_slots", ()):
            return (0, 0, 0)     # pump never ASKED — a wedged bus
        if slot in getattr(self, "config_failed_slots", ()):
            # An unconfigured slot never gets a feedback read: the C
            # planner requires config_step 3 before polling. So the
            # READ counters look exactly like the wedged-pump case —
            # which is why the write counters must be consulted first.
            return (0, 0, 0)
        if slot in getattr(self, "dead_slots", ()):
            return (0, 137, 137)  # asked repeatedly, got silence
        if getattr(self, "fault_bits", 0) & (1 << slot):
            return (500, 42, 42)
        return (500, 0, 0)

    def servo_write_stats(self, slot):
        # (writes_failed, config_failed). ``config_failed_slots``
        # models the servo that never ACKed its wheel-mode setup —
        # the C layer's retry-then-latch outcome for a dead servo.
        if slot in getattr(self, "config_failed_slots", ()):
            return (8, 1)
        return (0, 0)

    def db_fault(self):
        return getattr(self, "fault_bits", 0)

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

    # ---- user register transactions (duty_limit) ----
    # The fake resolves instantly: a stage records the value and the
    # next poll answers done. Tests shadow ``servo_user_poll`` on the
    # instance for the loss/timeout paths.

    def servo_user_write(self, slot, reg, val, length):
        self.calls.append(("servo_user_write", slot, reg, val, length))
        self.user_regs = getattr(self, "user_regs", {})
        self.user_regs[reg] = val
        self.user_result = (1, val)
        return True

    def servo_user_read(self, slot, reg, length):
        self.calls.append(("servo_user_read", slot, reg, length))
        regs = getattr(self, "user_regs", {})
        self.user_result = (1, regs.get(reg, 1000))
        return True

    def servo_user_poll(self, slot):
        return getattr(self, "user_result", (-2, 0))


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
        # Slots are claimed by scanning for a free one (the right
        # wheel's first probe hits the left's slot), so the exact
        # attach count is an implementation detail — pin the shape.
        self.assertEqual(names[0], "db_disable")
        self.assertEqual(names[1], "attach_uart")
        self.assertEqual(names[-1], "db_config")
        self.assertTrue(names.count("servo_attach") >= 2, names)
        self.assertFalse("servo_detach" in names, names)
        # Bench defaults: UART1 @1M on 14/6; left slot0 id2 inverted.
        by = {}
        for c in self.bus.calls:
            by.setdefault(c[0], []).append(c)
        self.assertEqual(by["attach_uart"][0],
                         ("attach_uart", 1, 1_000_000, 14, 6))
        cfg = by["db_config"][0]
        self.assertEqual(cfg[1:3], (0, 1))      # left slot, right slot
        ok = [c for c in self.bus.calls
              if c[0] == "servo_attach" and c[1] in (0, 1)]
        self.assertEqual(ok[0][1:4], (0, 2, True))     # left: id 2
        self.assertEqual(ok[-1][1:4], (1, 1, False))   # right: id 1

    def test_goal_acc_encoding_matches_the_driver_formula(self):
        # st3215.py::_encode_goal_acc — steps/100 units, clamped 254.
        self._db(accel_dps2=400.0)
        att = [c for c in self.bus.calls if c[0] == "servo_attach"]
        self.assertEqual(att[0][4], int(400.0 * (4096 / 360.0) / 100.0))
        self.bus = _FakeBus()
        _native.st_bus = self.bus
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
        mark = len(self.bus.calls)
        self._db()          # must not raise
        # The second engine REUSES the slots its servo ids already
        # hold rather than consuming two more — on a 4-slot bus,
        # claiming again would exhaust it.
        second = [c for c in self.bus.calls[mark:] if c[0] == "servo_attach"]
        self.assertEqual(second, [], second)

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
        cfg = [c for c in self.bus.calls if c[0] == "db_config"][0]
        self.assertEqual(cfg[1:3], (0, 1))
        att = [c for c in self.bus.calls
               if c[0] == "servo_attach" and c[1] in (0, 1)]
        self.assertEqual(att[0][1:4], (0, 2, True))
        self.assertEqual(att[-1][1:4], (1, 1, False))

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

    def _timeout_msg(self, moved_counts, motor_kw=None):
        """Force the stall path with a chosen amount of travel and
        return what got REPORTED. A stall is loud, not fatal: the
        console and run log get the diagnosis and run_angle returns
        False, so a mission does not abort over one stuck motor."""
        from openbricks.drivers.st3215 import ST3215Motor
        db, left, _ = self._drivebase(**(motor_kw or {}))
        self.bus.done_after = 10_000              # never completes
        counts = [0, moved_counts]
        self.bus.servo_counts = (
            lambda slot: counts.pop(0) if counts else moved_counts)
        seen = []
        orig = ST3215Motor._report_stall
        ST3215Motor._report_stall = staticmethod(seen.append)
        try:
            result = left.run_angle(100, 90)
        finally:
            ST3215Motor._report_stall = orig
        self.assertFalse(result, "a stall must report False, not True")
        return seen[0]

    def test_stall_report_says_it_never_moved(self):
        msg = self._timeout_msg(0)
        self.assertTrue("never moved" in msg, msg)
        self.assertTrue("servo id 2" in msg, msg)

    def test_stall_report_says_stopped_partway(self):
        # Half of 90 deg, in counts.
        msg = self._timeout_msg(int(45 * 4096 / 360.0))
        self.assertTrue("stopped partway" in msg, msg)
        self.assertTrue("overload" in msg, msg)
        self.assertTrue("45.0 deg of the 90.0" in msg, msg)

    def test_stall_report_distinguishes_a_move_that_did_not_latch(self):
        # Travelled the whole way: mechanically fine, so pointing the
        # user at a jam or a stall would send them to the wrong place.
        msg = self._timeout_msg(int(90 * 4096 / 360.0))
        self.assertTrue("never latched" in msg, msg)
        self.assertTrue("NOT a mechanical fault" in msg, msg)

    def test_drive_supersedes_pending_and_ships_one_sync(self):
        # drive() was the ONE motion verb that skipped new-command-
        # wins: a still-running straight() overwrote its per-motor
        # speeds every tick, and a later done() poll dispatched the
        # stale move's then= on top. It now routes through
        # move_wheels — engine move aborted, both setpoints in one
        # sync-write — and clears pending state like every other verb.
        db, left, _ = self._drivebase()
        left.run_angle(100, 90, wait=False)
        self.assertIsNotNone(left._native_pending)
        db.drive(100, 0)
        self.assertIsNone(left._native_pending)
        mw = [c for c in self.bus.calls if c[0] == "db_move_wheels"]
        self.assertEqual(len(mw), 1)
        self.assertAlmostEqual(mw[0][1], mw[0][2], places=3)  # straight

    def test_a_dead_bus_mid_move_raises_not_stall_reports(self):
        # Bus death freezes counts exactly like a jam, and the stall
        # reporter used to call it one — "the shaft is jammed" for an
        # unplugged servo sends the user to the wrong part of the
        # robot. A wiring fault is not survivable, so it raises even
        # with the default raise_on_stall=False, naming the real
        # fault.
        db, left, _ = self._drivebase()
        self.bus.done_after = 10_000
        self.bus.dead_slots = (0,)
        try:
            left.run_angle(100, 90)
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("SILENT" in msg, msg)
        self.assertTrue("wiring" in msg, msg)
        self.assertTrue("jam" in msg, msg)     # names the trap it dodges

    def test_wait_false_polling_detects_the_stall(self):
        # THE documented pattern: while not m.done(): ... — 1.62.0's
        # stall detection covered only wait=True, so this loop hung
        # forever on a jammed motor.
        from openbricks.drivers.st3215 import ST3215Motor
        db, left, _ = self._drivebase()
        self.bus.done_after = 10_000           # never arrives
        left._stall_idle_ms = 30
        seen = []
        orig = ST3215Motor._report_stall
        ST3215Motor._report_stall = staticmethod(seen.append)
        try:
            left.run_angle(100, 90, wait=False)
            detected = False
            for _ in range(200):
                if left.done():
                    detected = True
                    break
                time.sleep_ms(5)
            self.assertTrue(detected,
                            "done() never detected the stall")
        finally:
            ST3215Motor._report_stall = orig
        self.assertTrue(seen and "gave up" in seen[0], seen)
        self.assertIn(("servo_run", 0, 0), self.bus.calls)  # stopped
        self.assertTrue(left.done())           # pending cleared

    def test_wait_false_polling_raises_on_a_dead_bus(self):
        db, left, _ = self._drivebase()
        self.bus.done_after = 10_000
        left.run_angle(100, 90, wait=False)
        self.bus.dead_slots = (0,)             # bus dies mid-move
        try:
            left.done()
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("SILENT" in str(e), str(e))

    def test_raise_on_stall_restores_the_fatal_behaviour(self):
        # Opt-in for callers who would rather abort than continue.
        from openbricks.drivers.st3215 import ST3215
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics import DriveBase
        ST3215._buses.clear()
        left = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                           invert=True, raise_on_stall=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        DriveBase(left, right, wheel_diameter_mm=88, axle_track_mm=138)
        self.bus.done_after = 10_000
        try:
            left.run_angle(100, 90)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("gave up" in str(e), str(e))

    def test_a_stuck_move_gives_up_on_stillness_not_on_the_budget(self):
        # A shaft that has not advanced a count in a second is stuck,
        # whatever the budget says — so the report names STILLNESS,
        # and the wait is ~1 s rather than the multi-second budget.
        msg = self._timeout_msg(0)
        self.assertTrue("stopped moving for" in msg, msg)
        self.assertFalse("ran out of its" in msg, msg)

    def test_a_move_that_keeps_inching_is_not_called_stuck(self):
        # The opposite error: a move fighting a heavy load is still a
        # move. Advancing counts must keep resetting the stillness
        # clock, so only the total budget can end it.
        from openbricks.drivers.st3215 import ST3215Motor
        db, left, _ = self._drivebase()
        self.bus.done_after = 10_000
        creep = [0]
        def _creep(slot):
            creep[0] += 8          # ~0.7 deg per poll: slow but real
            return creep[0]
        self.bus.servo_counts = _creep
        seen = []
        orig = ST3215Motor._report_stall
        ST3215Motor._report_stall = staticmethod(seen.append)
        try:
            left.run_angle(100, 90)
        finally:
            ST3215Motor._report_stall = orig
        self.assertTrue(seen, "expected a report")
        self.assertTrue("ran out of its" in seen[0], seen[0])

    def test_a_completed_run_angle_reports_success(self):
        # The return value is the contract now, so it must be True on
        # the happy path — otherwise callers that branch on it would
        # treat every good move as a stall.
        db, left, _ = self._drivebase()
        self.bus.done_after = 2
        self.assertTrue(left.run_angle(100, 90))

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
        # _native_pending is the watch dict since 1.65.0 (it carries
        # the wait=False stall detection); "then" is the deferred
        # end-state.
        self.assertEqual(left._native_pending["then"], "coast")
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


class DeadMotorDiagnosisTests(_Base):
    """A dead motor must raise, and the message must say WHICH motor
    and where it's wired — "nothing moved" is the least actionable
    error a robot can give."""

    def test_construction_raises_naming_the_silent_motor(self):
        self.bus.dead_slots = (1,)          # right wheel never answers
        try:
            self._db()
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        # Identity: side, bus id, slot, UART and pins.
        self.assertTrue("right wheel" in msg, msg)
        self.assertTrue("servo id 1" in msg, msg)
        self.assertTrue("slot 1" in msg, msg)
        self.assertTrue("UART1" in msg, msg)
        self.assertTrue("tx=14" in msg, msg)
        self.assertTrue("rx=6" in msg, msg)
        # Evidence + a next step.
        self.assertTrue("137" in msg, msg)
        self.assertTrue("servo-id" in msg, msg)

    def test_left_motor_is_named_when_it_is_the_dead_one(self):
        self.bus.dead_slots = (0,)
        try:
            self._db()
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("left wheel" in str(e), str(e))
            self.assertTrue("servo id 2" in str(e), str(e))

    def test_healthy_pair_constructs_silently(self):
        self._db()                           # must not raise

    def test_mid_move_fault_raises_before_the_settle_timeout(self):
        db = self._db()
        self.bus.done_after = 10_000         # would never finish
        self.bus.fault_bits = 0x02           # right wheel went quiet
        try:
            db.straight(200)
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("right wheel" in msg, msg)
        self.assertTrue("running away" in msg, msg)

    def test_faulted_move_never_reports_success(self):
        # Halting on a fault latches the controller's ``done`` flag,
        # so a wait loop that tested done first would exit reporting
        # success. The health check must come first.
        db = self._db()
        self.bus.fault_bits = 0x01
        self.bus._left = 0                   # db_done() would say True
        try:
            db.straight(200)
            self.fail("faulted move reported success")
        except OSError:
            pass

    def test_move_wheels_surfaces_a_dead_motor(self):
        # Nothing waits on move_wheels, so without this check a dead
        # motor is silent forever — the line-follow shape.
        db = self._db()
        self.bus.fault_bits = 0x01
        try:
            db.move_wheels(100, 100)
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("left wheel" in str(e), str(e))

    def test_check_motors_is_available_on_the_drivebase(self):
        from openbricks.drivers.st3215 import ST3215
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics import DriveBase
        ST3215._buses.clear()
        left = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                           invert=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        db = DriveBase(left, right, wheel_diameter_mm=88,
                       axle_track_mm=138)
        db.check_motors()                    # healthy: silent
        self.bus.fault_bits = 0x02
        try:
            db.check_motors()
            self.fail("expected OSError")
        except OSError:
            pass

    def test_settle_timeout_reports_both_wheels_traffic(self):
        # Both wheels talking but the target unreached = mechanical.
        # The per-wheel counters localise it.
        db = self._db()
        self.bus.done_after = 10_000
        try:
            db.straight(200)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            msg = str(e)
        self.assertTrue("did not reach target" in msg, msg)
        self.assertTrue("left" in msg and "right" in msg, msg)


class OneBusOneOwnerTests(_Base):
    """Four ST-3032s on ONE UART: two wheels plus two task motors.

    A task motor that opened its own ``machine.UART`` on a bus the
    native driver already owns puts two drivers on one wire — the
    hard tick's replies land in the other driver's buffer and get
    consumed as the wrong packet's answer (bench 2026-08-04: a write
    to id 4 acknowledged by id 1). So every motor on a natively-owned
    UART takes a slot instead.
    """

    def setUp(self):
        _Base.setUp(self)
        from openbricks.drivers.st3215 import ST3215
        ST3215._buses.clear()
        self.bus.uart_num = lambda: -1      # native bus owns nothing yet

    def _motor(self, servo_id, **kw):
        from openbricks.drivers.st3032 import ST3032Motor
        return ST3032Motor(servo_id=servo_id, uart_id=1, tx=14, rx=6, **kw)

    def _drivebase(self):
        from openbricks.robotics import DriveBase
        left = self._motor(2, invert=True)
        right = self._motor(1)
        db = DriveBase(left, right, wheel_diameter_mm=88,
                       axle_track_mm=138)
        # Adoption has taken the UART; the native bus owns it now.
        self.bus.uart_num = lambda: 1
        return db, left, right

    def test_failed_adoption_restores_the_micropython_bus(self):
        # Engine construction raises BY DESIGN (dead wheel, slot
        # exhaustion) — but the MicroPython bus was already deinited
        # and deregistered. A caller that caught the error and tried
        # plain motor commands wrote into a dead UART for the rest of
        # the program. The handover is now unwound on failure.
        from openbricks.drivers.st3215 import ST3215
        from openbricks.robotics import DriveBase
        left = self._motor(2, invert=True)
        right = self._motor(1)
        self.bus.dead_slots = (0, 1)          # liveness gate will raise
        try:
            DriveBase(left, right, wheel_diameter_mm=88,
                      axle_track_mm=138)
            self.fail("expected OSError")
        except OSError:
            pass
        self.assertEqual(len(ST3215._buses), 1)   # re-registered
        self.assertIsNotNone(left._bus)
        left.run_speed(50)                    # and it still drives

    def test_adoption_refuses_a_uart_shared_with_position_servos(self):
        # A position-mode ST3215 has no native-slot path (slots
        # configure wheel mode). Adoption used to close the UART
        # under it silently; it now refuses BEFORE the handover,
        # naming the servo and the remedy.
        from openbricks.drivers.st3215 import ST3215
        from openbricks.robotics import DriveBase
        left = self._motor(2, invert=True)
        right = self._motor(1)
        ST3215(servo_id=7, uart_id=1, tx=14, rx=6)   # gripper
        try:
            DriveBase(left, right, wheel_diameter_mm=88,
                      axle_track_mm=138)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            msg = str(e)
        self.assertTrue("position-mode" in msg, msg)
        self.assertTrue("7" in msg, msg)
        self.assertTrue("second UART" in msg, msg)
        self.assertEqual(len(ST3215._buses), 1)   # bus untouched

    def test_position_servo_refused_on_a_natively_owned_uart(self):
        # Constructed AFTER adoption: opening its own machine.UART on
        # pins the IDF driver owns is the two-drivers-one-wire fault
        # (bench 2026-08-04). Refused with the remedy.
        from openbricks.drivers.st3215 import ST3215
        db, _, _ = self._drivebase()      # native bus owns UART 1 now
        try:
            ST3215(servo_id=7, uart_id=1, tx=14, rx=6)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            msg = str(e)
        self.assertTrue("owned by the native bus" in msg, msg)
        self.assertTrue("second UART" in msg, msg)

    def test_task_motor_built_after_adoption_takes_a_slot(self):
        db, _, _ = self._drivebase()
        task = self._motor(4)
        self.assertIsNotNone(task._native_slot)
        self.assertIsNone(task._bus)            # no competing UART
        self.assertIn(task._native_slot, (2, 3))

    def test_two_task_motors_get_the_two_free_slots(self):
        db, _, _ = self._drivebase()
        a, b = self._motor(3), self._motor(4)
        self.assertEqual(sorted([a._native_slot, b._native_slot]), [2, 3])

    def test_task_slots_never_steal_the_drivebase_wheels(self):
        # Slots 0/1 are attached by fixed index by the engine, which
        # detaches whatever is there first — a task motor parked in
        # one would be evicted mid-run.
        db, left, right = self._drivebase()
        self._motor(3), self._motor(4)
        self.assertEqual(left._native_slot, 0)
        self.assertEqual(right._native_slot, 1)

    def test_a_fifth_motor_is_refused_with_the_reason(self):
        db, _, _ = self._drivebase()
        self._motor(3), self._motor(4)
        try:
            self._motor(5)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            msg = str(e)
        self.assertTrue("no free native slot" in msg, msg)
        self.assertTrue("second UART" in msg, msg)

    def test_task_motor_built_BEFORE_the_drivebase_is_migrated(self):
        # Construction order must not matter: adoption takes the UART
        # away, so a motor already on the MicroPython bus has to come
        # across or it talks into a closed UART.
        task = self._motor(4)
        self.assertIsNone(task._native_slot)     # plain bus for now
        db, _, _ = self._drivebase()
        self.assertIsNotNone(task._native_slot)
        self.assertIsNone(task._bus)

    def test_task_motor_is_usable_the_moment_it_is_constructed(self):
        # THE bench failure (2026-08-05): a slot has no odometry until
        # the pump's round-robin reaches it, and the C layer refuses a
        # position move until then — so a run_angle issued right after
        # construction died with "slot odometry is not live yet". The
        # constructor now waits for the first feedback read.
        db, _, _ = self._drivebase()
        task = self._motor(4)
        self.bus.calls = []
        task.run_angle(100, 90)
        self.assertTrue(any(c[0] == "servo_move" for c in self.bus.calls),
                        self.bus.calls)

    def test_task_motor_that_never_answers_raises_at_construction(self):
        # Same liveness standard the drivebase wheels get: a silent
        # servo is a wiring/id/power fault, named at construction
        # rather than surfacing as a puzzling refusal later.
        db, _, _ = self._drivebase()
        self.bus.dead_slots = (2, 3)
        try:
            self._motor(4)
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("servo id 4" in msg, msg)
        self.assertTrue("not answering" in msg, msg)
        self.assertTrue("137 failed reads" in msg, msg)
        self.assertTrue("servo-id" in msg, msg)

    def test_task_motor_that_never_acked_config_names_the_write_loss(self):
        # A servo that never ACKs its wheel-mode setup has 0 reads
        # attempted (unconfigured slots are never polled) — the same
        # READ counters as a wedged pump. Without consulting the
        # write counters this raised "the bus pump never polled it —
        # not a wiring fault" for what IS a wiring fault.
        db, _, _ = self._drivebase()
        self.bus.config_failed_slots = (2, 3)
        try:
            self._motor(4)
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("servo id 4" in msg, msg)
        self.assertTrue("configuration" in msg, msg)
        self.assertTrue("8" in msg, msg)               # the loss count
        self.assertTrue("wiring" in msg, msg)
        self.assertTrue("never polled" not in msg, msg)

    def test_a_wheel_that_never_acked_config_is_named_at_adoption(self):
        # The drivebase's liveness gate gets the same discrimination.
        self.bus.config_failed_slots = (0,)
        try:
            self._drivebase()
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("left wheel" in msg, msg)
        self.assertTrue("never ACKed its configuration" in msg, msg)
        self.assertTrue("8 writes unacknowledged" in msg, msg)
        self.assertTrue("never polled" not in msg, msg)

    def test_a_pump_that_never_polled_is_not_blamed_on_wiring(self):
        # 0 replies AND 0 failed reads means the bus never ASKED —
        # a wedged pump, not a dead servo. Saying "check your wiring"
        # there sent a bench session hunting the wrong fault (1.57.2).
        db, _, _ = self._drivebase()
        self.bus.wedged_slots = (2, 3)
        try:
            self._motor(4)
            self.fail("expected OSError")
        except OSError as e:
            msg = str(e)
        self.assertTrue("never polled" in msg, msg)
        self.assertTrue("not a wiring fault" in msg, msg)
        self.assertFalse("servo-id --scan" in msg, msg)

    def test_wheels_built_on_an_already_native_bus_still_adopt(self):
        # THE bench ordering (seasonq4.py): task motors first, so the
        # UART is already native by the time the WHEELS are built —
        # they go straight onto slots and have no MicroPython bus.
        # Adoption used to demand a registry entry for a bus that
        # never existed ("motor bus not found in the registry").
        # The UART survives program boundaries, so on any run after
        # the first it is ALREADY native when the script starts —
        # every motor goes straight onto a slot, wheels included.
        self.bus.uart_num = lambda: 1
        task_a, task_b = self._motor(3), self._motor(4)
        left, right = self._motor(2, invert=True), self._motor(1)
        self.assertIsNone(left._bus)           # straight onto a slot
        from openbricks.robotics import DriveBase
        db = DriveBase(left, right, wheel_diameter_mm=88,
                       axle_track_mm=138)
        self.assertIsNotNone(db._serial_engine)
        # All four motors hold distinct slots.
        slots = sorted([task_a._native_slot, task_b._native_slot,
                        left._native_slot, right._native_slot])
        self.assertEqual(slots, [0, 1, 2, 3])
        # And the drivebase drives the slots its wheels actually hold.
        cfg = [c for c in self.bus.calls if c[0] == "db_config"][-1]
        self.assertEqual(cfg[1:3], (left._native_slot,
                                    right._native_slot))

    def test_sync_servo_group_refuses_adopted_wheels(self):
        # Bench 2026-08-05: a line-align routine built a
        # SyncServoGroup over the drivebase wheels. Those wheels are
        # driven by the native bus, but the group writes through the
        # MicroPython UART — two drivers on one wire, and each reads
        # the other's replies ("write to servo id 2 acknowledged by
        # servo id 4"). Refuse it, and name the replacement.
        from openbricks.drivers.st3215 import SyncServoGroup
        db, left, right = self._drivebase()
        try:
            SyncServoGroup([left, right])
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            msg = str(e)
        self.assertTrue("native bus" in msg, msg)
        self.assertTrue("move_wheels" in msg, msg)

    def test_group_built_BEFORE_adoption_is_refused_on_use(self):
        # THE order-dependent case (bench 2026-08-05: "run second
        # time still this error, first time ok"). On a first run the
        # motors are still on the MicroPython bus when the group is
        # built, so construction passes — and then a DriveBase adopts
        # them a line later, leaving the group holding wheels it can
        # no longer reach. It used to write into a contended bus
        # silently. Refusing only at construction made the failure
        # depend on run order, which is worse than either.
        from openbricks.drivers.st3215 import SyncServoGroup
        left, right = self._motor(2, invert=True), self._motor(1)
        group = SyncServoGroup([left, right])      # allowed: no slots yet
        from openbricks.robotics import DriveBase
        DriveBase(left, right, wheel_diameter_mm=88, axle_track_mm=138)
        self.bus.uart_num = lambda: 1
        try:
            group.set_goal_speeds([100, 100])
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("move_wheels" in str(e), str(e))

    def test_sync_servo_group_still_works_off_the_native_bus(self):
        # Non-adopted servos on a plain MicroPython bus are exactly
        # what this class is for (grippers, multi-axis arms).
        from openbricks.drivers.st3215 import SyncServoGroup
        a, b = self._motor(6), self._motor(7)
        SyncServoGroup([a, b])            # must not raise

    def test_without_a_native_bus_nothing_changes(self):
        # No drivebase, no native ownership: the plain MicroPython
        # driver path is untouched.
        task = self._motor(4)
        self.assertIsNone(task._native_slot)
        self.assertIsNotNone(task._bus)


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


class AdoptedDutyLimitTests(_Base):
    """duty_limit on a natively-adopted task motor: the torque-cap
    register transaction must ride the native pump's staged user ops
    — Python never talks on a natively-owned UART — and every
    outcome (done / lost / refused / hung) surfaces loudly."""

    def setUp(self):
        _Base.setUp(self)
        from openbricks.drivers.st3215 import ST3215
        ST3215._buses.clear()
        self.bus.uart_num = lambda: -1

    def _adopted_task_motor(self):
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics import DriveBase
        left = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                           invert=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        DriveBase(left, right, wheel_diameter_mm=88,
                  axle_track_mm=138)
        self.bus.uart_num = lambda: 1     # native bus owns UART 1 now
        task = ST3032Motor(servo_id=4, uart_id=1, tx=14, rx=6)
        self.assertTrue(task._native_slot is not None)
        return task

    def test_push_rides_the_native_user_ops(self):
        task = self._adopted_task_motor()
        prev = task._duty_limit_push(30)
        self.assertEqual(prev, 1000)      # the fake's register default
        reads = [c for c in self.bus.calls
                 if c[0] == "servo_user_read"]
        writes = [c for c in self.bus.calls
                  if c[0] == "servo_user_write"]
        self.assertEqual(reads[-1],
                         ("servo_user_read", task._native_slot,
                          0x30, 2))
        self.assertEqual(writes[-1],
                         ("servo_user_write", task._native_slot,
                          0x30, 300, 2))
        self.assertEqual(task._duty_limit_raw, 300)

    def test_pop_restores_via_the_native_path(self):
        task = self._adopted_task_motor()
        prev = task._duty_limit_push(30)
        task._duty_limit_pop(prev)
        write = [c for c in self.bus.calls
                 if c[0] == "servo_user_write"][-1]
        self.assertEqual(write[2:], (0x30, 1000, 2))
        self.assertEqual(task._duty_limit_raw, 1000)

    def test_lost_transaction_raises_named(self):
        task = self._adopted_task_motor()
        self.bus.servo_user_poll = lambda slot: (-1, 0)
        try:
            task._duty_limit_push(30)
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("0x30" in str(e), e)
            self.assertTrue("servo id 4" in str(e), e)

    def test_stage_refusal_raises(self):
        task = self._adopted_task_motor()
        self.bus.servo_user_read = lambda slot, reg, n: False
        try:
            task._duty_limit_push(30)
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("could not stage" in str(e), e)

    def test_hung_transaction_times_out(self):
        # A poll that never resolves must not spin forever — the
        # virtual clock drives the deadline.
        task = self._adopted_task_motor()
        self.bus.servo_user_poll = lambda slot: (0, 0)
        try:
            task._duty_limit_push(30)
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("timed out" in str(e), e)


if __name__ == "__main__":
    unittest.main()
