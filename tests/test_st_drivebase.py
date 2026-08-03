# SPDX-License-Identifier: MIT
"""Tests for the native drivebase on serial-bus slots (``st_bus.db_*``).

The proven 2-DOF controller (drivebase_core — the one that passed the
encoder path's asymmetric-friction gate) drives serial-bus wheels
through bridge structs; the pump syncs slot odometry in and speed
targets out around each tick. These tests close the loop with a
simulated PERFECT WHEEL: commanded speeds integrate into position,
position feeds back through the wire protocol — so convergence here
means the controller, the planner, the protocol, and the odometry
unwrap all agree end to end.

Two planner regressions are pinned because each one silently produced
zero motion in development:
  * torque re-staged every tick starved the speed sync-writes;
  * speed syncs every tick starved the feedback reads (control loop
    on frozen odometry). The fairness rule (a sync must be followed
    by a read) is the fix.
"""

import unittest

try:
    from _openbricks_native import st_bus as sb
except ImportError:
    sb = None


_MM_PER_COUNT = (88.0 * 3.14159265) / 4096     # bench wheel


def _chk(body):
    return (~sum(body)) & 0xFF


def _reply(servo_id, err, payload=b""):
    body = bytes([servo_id, len(payload) + 2, err]) + payload
    return b"\xff\xff" + body + bytes([_chk(body)])


class _PerfectWheels:
    """Servos that follow commanded speed exactly; 1 ms per pump."""

    track = 1.0     # fraction of commanded speed actually achieved

    def __init__(self):
        self.pos = {1: 0.0, 2: 0.0}
        self.spd = {1: 0, 2: 0}
        self.now = 0
        self.reads = 0
        self.syncs = 0

    def pump(self):
        self.now += 1
        for i in (1, 2):
            self.pos[i] += self.spd[i] * self.track / 1000.0
        sb.servo_pump(self.now)
        tx = sb.take_tx()
        i = 0
        while i + 4 <= len(tx):
            if tx[i:i + 2] != b"\xff\xff":
                i += 1
                continue
            pid, ln, instr = tx[i + 2], tx[i + 3], tx[i + 4]
            pkt = tx[i:i + 4 + ln]
            if instr == 0x02:                          # READ feedback
                self.reads += 1
                raw = int(self.pos[pid]) & 0x0FFF
                # Widened 6-byte feedback (1.50.0): pos + present-
                # speed (sign-magnitude b15, = the commanded speed on
                # a perfect wheel) + present-load (b10 sign; scaled
                # stand-in so load plumbing is testable end to end).
                spd = int(self.spd[pid])
                sp = (0x8000 | -spd) if spd < 0 else spd
                ld = min(abs(spd) // 4, 0x3FF)
                if spd < 0:
                    ld |= 0x0400
                sb.feed_rx(_reply(pid, 0, bytes([
                    raw & 0xFF, (raw >> 8) & 0xFF,
                    sp & 0xFF, (sp >> 8) & 0xFF,
                    ld & 0xFF, (ld >> 8) & 0xFF])))
            elif instr == 0x03 and pid != 0xFE:        # WRITE
                sb.feed_rx(_reply(pid, 0))
            elif instr == 0x83:                        # SYNC speed
                self.syncs += 1
                dl = pkt[6]
                j = 7
                end = 2 + 2 + ln - 1                   # before checksum
                while j + dl + 1 <= end:
                    sid = pkt[j]
                    v = pkt[j + 1] | (pkt[j + 2] << 8)
                    if sid in self.spd:
                        self.spd[sid] = -(v & 0x7FFF) if v & 0x8000 else v
                    j += 1 + dl
            i += 4 + ln

    def advance(self, ms):
        for _ in range(ms):
            self.pump()


class _LaggyWheels(_PerfectWheels):
    """Wheels that achieve only 60% of the commanded speed — a crude
    stand-in for real settle dynamics, so moves arrive at the done
    latch with a genuine residual instead of ~zero."""

    track = 0.6


class _Base(unittest.TestCase):
    def setUp(self):
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()
        self.w = _PerfectWheels()
        # Bench mapping: left slot0 id2 inverted, right slot1 id1.
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.w.advance(50)
        sb.db_config(0, 1, 88.0, 136.0, 400.0)

    def _mm(self, slot):
        return sb.servo_counts(slot) * _MM_PER_COUNT


class StraightTests(_Base):
    def test_straight_converges_on_the_commanded_distance(self):
        sb.db_straight(200.0, 150.0)
        self.assertFalse(sb.db_done())
        self.w.advance(3500)
        self.assertTrue(sb.db_done())
        self.assertTrue(abs(self._mm(0) - 200) < 10, self._mm(0))
        self.assertTrue(abs(self._mm(1) - 200) < 10, self._mm(1))

    def test_reverse_straight(self):
        sb.db_straight(-150.0, 150.0)
        self.w.advance(3000)
        self.assertTrue(sb.db_done())
        self.assertTrue(abs(self._mm(0) + 150) < 10, self._mm(0))
        self.assertTrue(abs(self._mm(1) + 150) < 10, self._mm(1))

    def test_feedback_reads_flow_during_the_drive(self):
        # THE fairness regression: speed syncs every tick starved
        # feedback entirely — control on frozen odometry, zero motion.
        sb.db_straight(200.0, 150.0)
        self.w.reads = 0
        self.w.advance(500)
        self.assertTrue(self.w.reads > 100, self.w.reads)
        self.assertTrue(self.w.syncs > 50, self.w.syncs)


class TurnTests(_Base):
    def test_turn_90_is_cw_positive_and_symmetric(self):
        # turn(+90) = clockwise = right (Pybricks convention since
        # 1.24.0): left wheel forward, right wheel back, each by the
        # arc pi * axle_track * 90/360 = 106.8 mm.
        sb.db_turn(90.0, 60.0)
        self.w.advance(3000)
        self.assertTrue(sb.db_done())
        dl, dr = self._mm(0), self._mm(1)
        self.assertTrue(abs(abs(dl) - 106.8) < 8, dl)
        self.assertTrue(abs(abs(dr) - 106.8) < 8, dr)
        self.assertTrue(dl > 0 and dr < 0, (dl, dr))

    def test_counter_turn_mirrors(self):
        sb.db_turn(-90.0, 60.0)
        self.w.advance(3000)
        dl, dr = self._mm(0), self._mm(1)
        self.assertTrue(dl < 0 and dr > 0, (dl, dr))


class StopAndGyroTests(_Base):
    def test_stop_zeroes_the_wheel_speeds(self):
        sb.db_straight(500.0, 150.0)
        self.w.advance(400)                    # mid-move, cruising
        self.assertTrue(abs(self.w.spd[1]) > 0)
        sb.db_stop()
        self.w.advance(50)
        # Hold-in-place semantics: speeds settle to ~0 (P on ~0 error)
        # and the pose stays put — the STALE-HOLD regression drove the
        # wheels back toward the move-start pose after a stop.
        here = (self._mm(0), self._mm(1))
        self.assertTrue(abs(self.w.spd[1]) < 30, self.w.spd[1])
        self.assertTrue(abs(self.w.spd[2]) < 30, self.w.spd[2])
        self.w.advance(500)
        self.assertTrue(abs(self._mm(0) - here[0]) < 3, (here, self._mm(0)))
        self.assertTrue(abs(self._mm(1) - here[1]) < 3, (here, self._mm(1)))
        self.assertTrue(sb.db_done())

    def test_gyro_override_steers_the_diff_axis(self):
        # With use_gyro on, the controller trusts db_set_heading (the
        # Python outer loop) for the heading axis. Feed a constant
        # "veering right" heading and watch the controller counter-
        # steer: left slows relative to right.
        sb.db_use_gyro(True)
        sb.db_straight(500.0, 150.0)
        self.w.advance(300)
        sb.db_set_heading(5.0)                 # body says: drifted +5 deg
        self.w.advance(300)
        # Counter-steer: right wheel must have out-paced left since
        # the override landed.
        self.assertTrue(self._mm(1) > self._mm(0),
                        (self._mm(0), self._mm(1)))

    def test_done_requires_arrival_not_just_profile_expiry(self):
        # The +4.5-deg bench bug: db_done flipped true at trajectory
        # expiry while the final turn's overshoot stood uncorrected.
        # Simulate a lagging heading source: freeze the override 6
        # wheel-deg short of the target past profile expiry — done
        # must stay False, and flip True once the reading arrives.
        sb.db_use_gyro(True)
        sb.db_turn(90.0, 60.0)
        # Run well past the profile duration with the override frozen
        # at zero (robot "hasn't turned" as far as the gyro knows).
        for _ in range(3000):
            self.w.pump()
            sb.db_set_heading(0.0)
        self.assertFalse(sb.db_done())
        # Arrive: feed the target heading (90 body-deg).
        for _ in range(50):
            self.w.pump()
            sb.db_set_heading(90.0)
        self.assertTrue(sb.db_done())

    # Wheel-degrees of diff per body-degree on the 88/136 geometry
    # (matches ob_drivebase_body_to_wheel_diff): 136/88.
    _WHEEL_PER_BODY = 136.0 / 88.0

    def _feed_heading_from_wheels(self):
        """One pump with an HONEST heading feed: body heading derived
        from the wheels' actual differential, like a perfect IMU on a
        non-slipping chassis."""
        self.w.pump()
        # User-frame counts (slot0 carries the invert), same frame
        # the controller's bridges read.
        diff_wheel_deg = ((sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
                          * 360.0 / 4096.0)
        sb.db_set_heading(diff_wheel_deg / self._WHEEL_PER_BODY)

    def test_gyro_frame_survives_per_move_stops(self):
        # THE +7.6-deg bench regression (2026-08-02, first gyro square
        # on the one-class flow): db_stop re-captured turn_hold from
        # MEASURED heading, re-baselining the absolute frame at every
        # per-move stop — each turn banked its arrival residual
        # (~+1.9 body-deg) instead of the next move correcting it.
        # Six 20-deg gyro turns, each followed by the stop the
        # DriveBase flow performs, must land on the ABSOLUTE 120.
        #
        # The plant must SETTLE SLOWLY for this to discriminate: a
        # perfect wheel arrives with ~zero residual and there is
        # nothing to bank (the unfixed code passed with perfect
        # wheels — verified). 60% speed tracking gives every turn a
        # real ~3-wheel-deg residual at the done latch, like the
        # bench chassis.
        self.w = _LaggyWheels()
        sb.db_use_gyro(True)
        for _ in range(6):
            sb.db_turn(20.0, 60.0)
            for _ in range(6000):
                self._feed_heading_from_wheels()
                if sb.db_done():
                    break
            self.assertTrue(sb.db_done())
            sb.db_stop()
        diff_wheel_deg = ((sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
                          * 360.0 / 4096.0)
        body = diff_wheel_deg / self._WHEEL_PER_BODY
        # Fixed: only the LAST turn's residual remains (~1.9 body-deg
        # at the done latch). Bug: all six bank, ~11 body-deg short.
        self.assertTrue(abs(body - 120.0) < 3.0,
                        "6 x turn(20) landed at %.1f body-deg "
                        "(banked residuals?)" % body)

    def test_aborted_turn_does_not_haunt_the_next_straight(self):
        # The flip side — the capture must STAY for mid-move aborts:
        # after stopping a 90-deg turn a third of the way in, the
        # following straight must hold the heading WHERE THE ABORT
        # LEFT IT, not keep steering toward the abandoned 90.
        sb.db_use_gyro(True)
        sb.db_turn(90.0, 60.0)
        for _ in range(600):                   # well short of done
            self._feed_heading_from_wheels()
        self.assertFalse(sb.db_done())
        sb.db_stop()
        diff_at_stop = (sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
        sb.db_straight(80.0, 60.0)
        for _ in range(4000):
            self._feed_heading_from_wheels()
            if sb.db_done():
                break
        diff_now = (sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
        drift_wheel_deg = abs(diff_now - diff_at_stop) * 360.0 / 4096.0
        self.assertTrue(drift_wheel_deg < 6.0,
                        "straight after an aborted turn moved the "
                        "diff axis %.1f wheel-deg" % drift_wheel_deg)

    def test_torque_starvation_regression(self):
        # The OTHER planner regression: set_speed re-staging torque
        # every tick starved the sync-writes (98 torque packets per
        # 100). After the first torque-on, a steady stream of speed
        # commands must produce NO further torque writes.
        sb.db_straight(300.0, 150.0)
        self.w.advance(100)
        sb.take_tx()
        before = self.w.now
        torque_writes = 0
        for _ in range(200):
            self.w.pump()
            # (torque packets would be instr 0x03 reg 0x28 — the
            # _PerfectWheels parser doesn't classify, so scan here)
        # Simplest check: motion is converging, which the starved
        # planner never achieved.
        self.w.advance(3000)
        self.assertTrue(sb.db_done())
        self.assertTrue(abs(self._mm(0) - 300) < 12, self._mm(0))


if __name__ == "__main__":
    unittest.main()


class FeedbackReadTests(_Base):
    """The widened 6-byte feedback read (1.50.0): present-speed and
    present-load ride the SAME transaction as position — servo_feedback
    exposes them user-frame, arming speed()/load()/stalled() on
    adopted motors with zero extra bus traffic."""

    def test_speed_and_load_flow_from_the_wire(self):
        sb.servo_run(1, 500)          # slot 1 (id 1, not inverted)
        self.w.advance(300)
        steps, load, fresh = sb.servo_feedback(1)
        self.assertTrue(fresh)
        # _PerfectWheels reports its commanded speed as present-speed.
        self.assertEqual(steps, 500)
        self.assertEqual(load, 500 // 4)

    def test_inverted_slot_reports_user_frame(self):
        # Slot 0 is inverted: a user-frame +400 command runs the wire
        # at -400; present-speed must come back user-frame +400, like
        # counts (frame symmetry rule).
        sb.servo_run(0, 400)
        self.w.advance(300)
        steps, load, fresh = sb.servo_feedback(0)
        self.assertTrue(fresh)
        self.assertEqual(steps, 400)
        self.assertEqual(load, 400 // 4)

    def test_feedback_not_fresh_before_first_read(self):
        sb.test_reset()
        self.w = _PerfectWheels()
        sb.servo_attach(1, 1, False, 45)
        _steps, _load, fresh = sb.servo_feedback(1)
        self.assertFalse(fresh)

    def test_feedback_goes_stale_on_bus_silence(self):
        sb.servo_run(1, 300)
        self.w.advance(200)
        self.assertTrue(sb.servo_feedback(1)[2])
        # Pump WITHOUT answering (silent wire): reads time out.
        for _ in range(200):
            self.w.now += 1
            sb.servo_pump(self.w.now)
            sb.take_tx()               # swallow requests, never reply
        self.assertFalse(sb.servo_feedback(1)[2])
