# SPDX-License-Identifier: MIT
"""Tests for per-slot position moves on the native bus
(``st_bus.servo_move`` / ``servo_hold`` / ``servo_move_done`` —
the C side of ``run_angle``/``hold`` on adopted serial motors).

Closes the loop with the same PERFECT WHEEL harness as
test_st_drivebase: commanded speeds integrate into position, position
feeds back through the wire protocol. Also pins the 1.46.0
arbitration contract: the drivebase owns its slots only while a db
move is in flight (db_straight/db_turn .. db_stop); yielded, per-slot
moves and plain speed commands own the wheels.
"""

import unittest

try:
    from _openbricks_native import st_bus as sb
except ImportError:
    sb = None

from tests.test_st_drivebase import _PerfectWheels


class _Base(unittest.TestCase):
    def setUp(self):
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()
        self.w = _PerfectWheels()
        # Bench mapping: left slot0 id2 inverted, right slot1 id1.
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.w.advance(50)          # let odometry go live


class MoveTests(_Base):
    def test_move_converges_on_relative_target(self):
        self.assertTrue(sb.servo_move(1, 4096.0, 2000.0, 8000.0))
        self.assertFalse(sb.servo_move_done(1))
        self.w.advance(3500)
        self.assertTrue(sb.servo_move_done(1))
        self.assertLess(abs(sb.servo_counts(1) - 4096), 40)

    def test_negative_move_converges(self):
        self.assertTrue(sb.servo_move(1, -2048.0, 1500.0, 6000.0))
        self.w.advance(2500)
        self.assertTrue(sb.servo_move_done(1))
        self.assertLess(abs(sb.servo_counts(1) + 2048), 40)

    def test_inverted_slot_moves_in_user_frame(self):
        # Slot 0 is inverted; servo_counts and servo_move both live in
        # the user frame, so a positive delta lands at positive counts.
        self.assertTrue(sb.servo_move(0, 2048.0, 1500.0, 6000.0))
        self.w.advance(2500)
        self.assertTrue(sb.servo_move_done(0))
        self.assertLess(abs(sb.servo_counts(0) - 2048), 40)

    def test_move_refused_without_live_odometry(self):
        # A move anchored before the first feedback read would slam
        # the shaft toward a wrong absolute position — refused.
        sb.test_reset()
        self.w = _PerfectWheels()
        sb.servo_attach(1, 1, False, 45)
        self.assertFalse(sb.servo_move(1, 1000.0, 1000.0, 4000.0))

    def test_servo_run_cancels_a_move(self):
        self.assertTrue(sb.servo_move(1, 40960.0, 2000.0, 8000.0))
        self.w.advance(100)
        sb.servo_run(1, 500)        # new command wins
        self.w.advance(1000)
        self.assertFalse(sb.servo_move_done(1))
        # The wheel follows the speed command, not the dead profile:
        # ~500 counts/s for ~1 s.
        self.assertGreater(sb.servo_counts(1), 300)
        self.assertLess(sb.servo_counts(1), 40000)

    def test_hold_resists_disturbance(self):
        self.w.advance(10)
        self.assertTrue(sb.servo_hold(1))
        self.assertTrue(sb.servo_move_done(1))   # a hold is not a move
        held = sb.servo_counts(1)
        self.w.pos[1] += 300                     # shove the shaft
        self.w.advance(1500)
        self.assertLess(abs(sb.servo_counts(1) - held), 40)

    def test_reset_runtime_clears_moves(self):
        self.assertTrue(sb.servo_move(1, 4096.0, 2000.0, 8000.0))
        sb.reset_runtime()
        self.assertFalse(sb.servo_move_done(1))


class ArbitrationTests(_Base):
    def setUp(self):
        super().setUp()
        sb.db_config(0, 1, 88.0, 136.0, 400.0)

    def test_configured_but_idle_db_leaves_wheels_free(self):
        # Pre-1.46.0 a configured db held pose zero with torque from
        # construction. Now it yields until its first move: a direct
        # speed command owns the wheel.
        sb.servo_run(1, 800)
        self.w.advance(1000)
        self.assertGreater(sb.servo_counts(1), 500)

    def test_move_refused_while_db_move_in_flight(self):
        sb.db_straight(200.0, 60.0)
        self.assertFalse(sb.servo_move(1, 1000.0, 1000.0, 4000.0))
        self.assertFalse(sb.servo_hold(0))

    def test_move_allowed_again_after_db_stop(self):
        sb.db_straight(200.0, 60.0)
        self.w.advance(300)
        sb.db_stop()
        self.assertTrue(sb.servo_move(1, 1024.0, 1500.0, 6000.0))
        self.w.advance(2000)
        self.assertTrue(sb.servo_move_done(1))

    def test_db_stop_yields_the_wheels(self):
        sb.db_straight(500.0, 60.0)
        self.w.advance(300)
        sb.db_stop()
        pos = sb.servo_counts(1)
        sb.servo_run(1, 600)
        self.w.advance(1000)
        # The db no longer re-asserts its hold: the wheel followed the
        # speed command instead of being dragged back.
        self.assertGreater(sb.servo_counts(1) - pos, 400)

    def test_db_arm_cancels_slot_moves_and_owns_the_wheels(self):
        sb.servo_move(1, 40960.0, 2000.0, 8000.0)
        sb.db_straight(100.0, 60.0)
        self.w.advance(50)
        self.assertFalse(sb.servo_move_done(1))
        # And the db still converges normally with the move canceled.
        self.w.advance(6000)
        self.assertTrue(sb.db_done())


if __name__ == "__main__":
    unittest.main()
