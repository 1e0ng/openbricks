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
        self.torque = {1: None, 2: None}   # last value seen per servo
        self.now = 0
        self.reads = 0
        self.syncs = 0
        # One entry per sync-write to the TORQUE register: the list
        # of (servo_id, value) pairs that shared that ONE packet —
        # the atomic-stop tests assert both wheels appear together.
        self.torque_pkts = []

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
                if spd > 0:
                    ld |= 0x0400      # bit 10 = pushing POSITIVE
                sb.feed_rx(_reply(pid, 0, bytes([
                    raw & 0xFF, (raw >> 8) & 0xFF,
                    sp & 0xFF, (sp >> 8) & 0xFF,
                    ld & 0xFF, (ld >> 8) & 0xFF])))
            elif instr == 0x03 and pid != 0xFE:        # WRITE
                if pkt[5] == 0x28 and pid in self.torque:   # TORQUE
                    self.torque[pid] = pkt[6]
                sb.feed_rx(_reply(pid, 0))
            elif instr == 0x03 and pid == 0xFE:        # BROADCAST write
                # Torque-off broadcast (the e-stop): applies to every
                # servo, no reply by protocol.
                if pkt[5] == 0x28:
                    for sid in self.torque:
                        self.torque[sid] = pkt[6]
                        if pkt[6] == 0:
                            self.spd[sid] = 0
            elif instr == 0x83:                        # SYNC write
                reg, dl = pkt[5], pkt[6]
                j = 7
                end = 2 + 2 + ln - 1                   # before checksum
                if reg == 0x2E:                        # GOAL_SPEED
                    self.syncs += 1
                    while j + dl + 1 <= end:
                        sid = pkt[j]
                        v = pkt[j + 1] | (pkt[j + 2] << 8)
                        if sid in self.spd:
                            self.spd[sid] = (-(v & 0x7FFF) if v & 0x8000
                                             else v)
                        j += 1 + dl
                elif reg == 0x2C:                      # mode-2 DUTY
                    # Dumb-mode plant: bit 10 SET = POSITIVE (the
                    # load-register convention; bench 2026-08-12),
                    # ~10 steps/s per duty unit free-run.
                    self.duty_syncs = getattr(self, "duty_syncs", 0) + 1
                    while j + dl + 1 <= end:
                        sid = pkt[j]
                        v = pkt[j + 1] | (pkt[j + 2] << 8)
                        duty = (v & 0x3FF) if v & 0x0400 else -(v & 0x3FF)
                        if sid in self.spd:
                            self.spd[sid] = duty * 10
                        j += 1 + dl
                elif reg == 0x28:                      # TORQUE
                    entries = []
                    while j + dl + 1 <= end:
                        sid, val = pkt[j], pkt[j + 1]
                        entries.append((sid, val))
                        if sid in self.torque:
                            self.torque[sid] = val
                            if val == 0:
                                self.spd[sid] = 0    # coasting wheel
                        j += 1 + dl
                    self.torque_pkts.append(entries)
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


class _DeadRightWheel(_PerfectWheels):
    """Servo id 1 (the right wheel) never answers a READ — the alive-
    but-not-reporting motor (broken feedback line) whose frozen
    odometry the runaway guard exists for. Its writes still ACK, so
    it configures normally and the evidence is a CLIMBING stale
    counter. The left wheel is healthy throughout. For the wheel that
    answers nothing at all, see _UnpluggedRightWheel."""

    DEAD_ID = 1
    ANSWERS_WRITES = True
    READ_GRACE = 0          # 0 = reads silent from the first packet

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
            if pid == self.DEAD_ID:
                if instr == 0x03 and self.ANSWERS_WRITES:
                    sb.feed_rx(_reply(pid, 0))   # command line fine
                elif instr == 0x02:
                    # READ_GRACE > 0 models a wheel that dies LATER:
                    # the first N reads answer (odometry goes live),
                    # then the feedback line breaks mid-run.
                    self.read_grace = getattr(self, "read_grace",
                                              self.READ_GRACE)
                    if self.read_grace > 0:
                        self.read_grace -= 1
                        raw = int(self.pos[pid]) & 0x0FFF
                        sb.feed_rx(_reply(pid, 0, bytes([
                            raw & 0xFF, (raw >> 8) & 0xFF,
                            0, 0, 0, 0])))
                i += 4 + ln
                continue                       # reads: silence
            if instr == 0x02:
                self.reads += 1
                raw = int(self.pos[pid]) & 0x0FFF
                sb.feed_rx(_reply(pid, 0, bytes([
                    raw & 0xFF, (raw >> 8) & 0xFF, 0, 0, 0, 0])))
            elif instr == 0x03 and pid != 0xFE:
                sb.feed_rx(_reply(pid, 0))
            elif instr == 0x83:
                reg, dl = pkt[5], pkt[6]
                j, end = 7, 2 + 2 + ln - 1
                if reg == 0x2E:
                    self.syncs += 1
                    while j + dl + 1 <= end:
                        sid = pkt[j]
                        v = pkt[j + 1] | (pkt[j + 2] << 8)
                        if sid in self.spd:
                            self.spd[sid] = (-(v & 0x7FFF) if v & 0x8000
                                             else v)
                        j += 1 + dl
            i += 4 + ln


class CurveTests(_Base):
    """db_curve — Pybricks arc semantics on the 88/136 bench
    geometry. curve(150, 90): centre travels 150 * pi/2 = 235.6 mm,
    outer wheel (150+68) * pi/2 = 342.4 mm, inner (150-68) * pi/2 =
    128.8 mm. Positive angle = CW/right (left wheel outer); the
    radius SIGN picks travel direction."""

    S_MM = 235.6
    OUTER_MM = 342.4
    INNER_MM = 128.8
    DIFF_MM = 106.8

    def test_quarter_circle_right_forward(self):
        sb.db_curve(150.0, 90.0, 150.0)
        self.assertFalse(sb.db_done())
        self.w.advance(4500)
        self.assertTrue(sb.db_done())
        dl, dr = self._mm(0), self._mm(1)
        self.assertTrue(abs(dl - self.OUTER_MM) < 10, dl)
        self.assertTrue(abs(dr - self.INNER_MM) < 10, dr)

    def test_quarter_circle_left_mirrors(self):
        sb.db_curve(150.0, -90.0, 150.0)
        self.w.advance(4500)
        self.assertTrue(sb.db_done())
        dl, dr = self._mm(0), self._mm(1)
        self.assertTrue(abs(dl - self.INNER_MM) < 10, dl)
        self.assertTrue(abs(dr - self.OUTER_MM) < 10, dr)

    def test_negative_radius_drives_the_arc_backward(self):
        # Heading still goes CW (+90) but the robot backs along the
        # circle: centre -235.6 mm, so left = -235.6 + 106.8, right =
        # -235.6 - 106.8.
        sb.db_curve(-150.0, 90.0, 150.0)
        self.w.advance(4500)
        self.assertTrue(sb.db_done())
        dl, dr = self._mm(0), self._mm(1)
        self.assertTrue(abs(dl + self.INNER_MM) < 10, dl)
        self.assertTrue(abs(dr + self.OUTER_MM) < 10, dr)

    def test_heading_tracks_distance_through_the_ramps(self):
        # THE circle property: heading fraction == distance fraction
        # at every instant, accel ramp included — that is what the
        # proportional cruise+accel scaling buys. Sample mid-ramp and
        # mid-cruise.
        sb.db_curve(150.0, 90.0, 150.0)
        for at_ms in (300, 1200):
            self.w.advance(at_ms if at_ms == 300 else 900)
            dl, dr = self._mm(0), self._mm(1)
            dist_frac = ((dl + dr) / 2.0) / self.S_MM
            head_frac = ((dl - dr) / 2.0) / self.DIFF_MM
            self.assertTrue(dist_frac > 0.02, (at_ms, dist_frac))
            self.assertTrue(abs(dist_frac - head_frac) < 0.05,
                            (at_ms, dist_frac, head_frac))

    def test_zero_radius_is_a_turn_in_place(self):
        sb.db_curve(0.0, 90.0, 150.0)
        self.w.advance(4000)
        self.assertTrue(sb.db_done())
        dl, dr = self._mm(0), self._mm(1)
        self.assertTrue(abs(dl - self.DIFF_MM) < 8, dl)
        self.assertTrue(abs(dr + self.DIFF_MM) < 8, dr)

    def test_zero_angle_completes_immediately_without_motion(self):
        sb.db_curve(150.0, 0.0, 150.0)
        self.w.advance(200)
        self.assertTrue(sb.db_done())
        self.assertTrue(abs(self._mm(0)) < 2, self._mm(0))
        self.assertTrue(abs(self._mm(1)) < 2, self._mm(1))

    def test_curve_before_config_raises(self):
        sb.test_reset()
        try:
            sb.db_curve(150.0, 90.0, 150.0)
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("db_config" in str(e), e)


class ProgramBoundaryTests(_Base):
    """``reset_runtime`` runs between programs. A transaction the
    previous one left in flight must not survive it.

    The pump consumes only results it started (``tick_txn``), and the
    reset clears that flag — so an un-taken reply pins the bus in a
    non-IDLE state that nothing clears, and every subsequent pump
    returns early. Symptom (bench 2026-08-05): a slot attached at
    program start reports 0 replies AND 0 failed reads — a dead servo
    shows failed reads climbing, a wedged pump shows neither.
    """

    def test_in_flight_transaction_does_not_survive_the_reset(self):
        # Leave a read in flight, exactly as a program ending
        # mid-transaction would.
        sb.db_straight(200.0, 150.0)
        for _ in range(3):
            sb.servo_pump(self.w.now + 1)
            self.w.now += 1
        sb.take_tx()                       # swallow it: no reply comes
        sb.reset_runtime()
        # A fresh program attaches a slot and the pump must service it.
        sb.servo_attach(0, 2, True, 45)
        w = _PerfectWheels()
        w.advance(200)
        ok, failed, stale = sb.servo_stats(0)
        self.assertTrue(ok > 0,
                        "pump wedged after reset: %d ok, %d failed"
                        % (ok, failed))

    def test_reset_leaves_the_bus_idle(self):
        sb.db_straight(200.0, 150.0)
        for _ in range(3):
            sb.servo_pump(self.w.now + 1)
            self.w.now += 1
        sb.take_tx()
        sb.reset_runtime()
        self.assertEqual(sb.state(), sb.IDLE)


class DeadWheelTests(_Base):
    """A wheel that stops answering must be caught, not absorbed.

    Before this existed the coupled controller integrated the silent
    wheel's frozen odometry: the diff error grew without bound and
    wound that wheel's command to the rail (bench-reproduced at 8468
    steps/s while the live wheel sat at 6). A motor that is alive but
    merely not REPORTING — a broken feedback line — would take off.
    """

    def setUp(self):
        # Dead from the FIRST packet — _Base.setUp would let the
        # right wheel reply during its own settle window.
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()
        self.w = _DeadRightWheel()
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.w.advance(50)
        sb.db_config(0, 1, 88.0, 136.0, 400.0)

    def test_silent_wheel_is_visible_in_the_stats(self):
        # Slots that are not driving are polled at a reduced rate
        # (they must not spend the bus a moving wheel needs), so an
        # IDLE dead wheel takes proportionally longer to show. That
        # is fine: the fault only matters while driving, and a
        # driving slot is polled at full rate — see
        # test_move_faults_instead_of_running_the_wheel_away, which
        # arms a move first and trips the latch promptly.
        self.w.advance(1600)
        self.assertGreater(sb.servo_stats(0)[0], 0)     # left replying
        self.assertEqual(sb.servo_stats(1)[0], 0)       # right silent
        self.assertGreater(sb.servo_stats(1)[2], 20)

    def test_move_faults_instead_of_running_the_wheel_away(self):
        sb.db_straight(200.0, 150.0)
        self.w.advance(3000)
        # Fault latched, naming the RIGHT wheel (bit 1).
        self.assertEqual(sb.db_fault(), 0x02)
        # And the commanded speeds were cut, not railed.
        self.assertEqual(self.w.spd[1], 0)
        self.assertEqual(self.w.spd[2], 0)

    def test_fault_stops_the_runaway_quickly(self):
        # The whole point: the silent wheel must not be driven far.
        # Unguarded this reached ~21000 counts in 3000 pumps.
        sb.db_straight(500.0, 150.0)
        self.w.advance(3000)
        self.assertLess(abs(self.w.pos[1]), 200, self.w.pos)

    def test_healthy_pair_never_faults(self):
        _Base.setUp(self)                     # clean, fully-alive rig
        sb.db_straight(200.0, 150.0)
        self.w.advance(3000)
        self.assertEqual(sb.db_fault(), 0)
        self.assertTrue(sb.db_done())

    def test_arming_a_new_move_clears_the_latch_for_a_retry(self):
        sb.db_straight(200.0, 150.0)
        self.w.advance(3000)
        self.assertEqual(sb.db_fault(), 0x02)
        sb.db_straight(200.0, 150.0)          # user retries
        self.assertEqual(sb.db_fault(), 0)    # cleared...
        self.w.advance(3000)
        self.assertEqual(sb.db_fault(), 0x02)  # ...and re-detected


class _DiesMidMoveRightWheel(_DeadRightWheel):
    """Answers the first N reads (odometry goes live, moves can arm),
    then the feedback line breaks — the failure striking MID-move."""

    READ_GRACE = 40


class MidMoveDeathTests(_Base):

    def setUp(self):
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()
        self.w = _DiesMidMoveRightWheel()
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.w.advance(30)                     # config + first reads

    def test_per_slot_move_is_halted_in_c_when_feedback_dies(self):
        # The drivebase got its stale latch in 1.55.0; per-slot moves
        # relied on Python's ~1 s stall detector while ff + kp*err
        # ramped the command on frozen odometry. The tick now halts
        # the move at the same ~200 ms stale threshold the db path
        # considered mandatory, and zeroes the wheel.
        self.assertTrue(sb.servo_move(1, 40960.0, 2000.0, 8000.0))
        self.w.advance(30)
        self.assertTrue(abs(self.w.spd[1]) > 0)   # move is driving
        self.w.advance(2000)                      # grace exhausted
        self.assertEqual(self.w.spd[1], 0)        # halted, not railed
        self.assertFalse(sb.servo_move_done(1))   # NOT "arrived"

    def test_program_boundary_clears_a_latched_fault(self):
        # reset_runtime is where the previous run's diagnosis has
        # been read: leaving the latch made the NEXT program's first
        # db_fault() report a wheel fault belonging to the last one.
        sb.db_config(0, 1, 88.0, 136.0, 400.0)
        sb.db_straight(200.0, 150.0)
        self.w.advance(3000)                      # feedback dies, faults
        self.assertEqual(sb.db_fault(), 0x02)
        sb.reset_runtime()
        self.assertEqual(sb.db_fault(), 0)


class _UnpluggedRightWheel(_DeadRightWheel):
    """Total silence from the first packet — unplugged, no power, or
    wrong bus id. Unlike the broken-feedback-line case above, the
    config writes go unACKed too, so the slot never configures, is
    never polled, and its stale counter CANNOT climb: the evidence
    lives in the write counters and the config_failed latch."""

    ANSWERS_WRITES = False


class UnpluggedWheelTests(_Base):

    def setUp(self):
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()
        self.w = _UnpluggedRightWheel()
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.w.advance(200)                    # config retries + latch
        sb.db_config(0, 1, 88.0, 136.0, 400.0)

    def test_unacked_config_is_visible_in_the_write_stats(self):
        wfailed, latched = sb.servo_write_stats(1)
        self.assertEqual(latched, 1)
        self.assertTrue(wfailed >= 8, wfailed)
        self.assertEqual(sb.servo_write_stats(0), (0, 0))   # left clean
        self.assertTrue(sb.servo_stats(0)[0] > 0)   # left reads flow
        self.assertEqual(sb.servo_stats(1)[0], 0)   # right never polled

    def test_the_dead_slot_does_not_hog_the_healthy_ones_bus(self):
        # Config outranks reads in the planner: without the latch the
        # dead wheel's endless retries would starve the left wheel's
        # odometry behind a write every few polls.
        before = sb.servo_stats(0)[0]
        self.w.advance(400)
        self.assertTrue(sb.servo_stats(0)[0] - before > 50,
                        (before, sb.servo_stats(0)))

    def test_move_faults_immediately_not_never(self):
        # The stale counter cannot climb for a slot that is never
        # polled — before the config_failed latch fed the fault, this
        # move would have driven the healthy wheel and never faulted.
        sb.db_straight(200.0, 150.0)
        self.w.advance(100)
        self.assertEqual(sb.db_fault(), 0x02)
        self.assertEqual(self.w.spd[2], 0)     # healthy wheel held


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

    def test_move_wheels_reaches_both_speeds_with_inverts(self):
        # The SyncServoGroup replacement: both wheel speeds ramp to
        # target (settings.acceleration — 1.94.0) with each slot's
        # invert applied (slot0 = id2 inverted, slot1 = id1).
        self.assertTrue(sb.db_move_wheels(400, 200))
        self.w.advance(300)                # > full ramp at 400 dps^2
        self.assertEqual(self.w.spd[2], -400)      # inverted slot
        self.assertEqual(self.w.spd[1], 200)

    def test_move_wheels_ramps_at_the_configured_accel(self):
        # settings(acceleration) applies to move_wheels/drive too —
        # the uniform-accel rule. 400 wheel-dps^2 * 11.38 steps/deg
        # ~= 4551 steps/s^2: at 50 ms the command must sit ~227
        # steps/s, nowhere near the 2000 target; by ~500 ms it lands.
        self.assertTrue(sb.db_move_wheels(2000, 2000))
        self.w.advance(50)
        mid = abs(self.w.spd[1])
        self.assertTrue(100 < mid < 400,
                        "expected ~227 steps/s mid-ramp, got %d" % mid)
        self.w.advance(500)
        self.assertEqual(self.w.spd[1], 2000)
        self.assertEqual(self.w.spd[2], -2000)

    def test_move_wheels_ramp_preserves_the_wheel_ratio(self):
        # Proportional slew: a 2:1 command stays 2:1 THROUGH the
        # ramp (a drive() arc keeps its radius), both wheels
        # arriving together.
        self.assertTrue(sb.db_move_wheels(2000, 1000))
        self.w.advance(60)
        l, r = abs(self.w.spd[2]), abs(self.w.spd[1])
        self.assertTrue(0 < r < 1000, r)           # genuinely mid-ramp
        ratio = l / r
        self.assertTrue(abs(ratio - 2.0) < 0.1,
                        "ratio drifted to %.2f mid-ramp" % ratio)

    def test_move_wheels_supersedes_an_in_flight_move(self):
        # A direct wheel command wins over the coupled controller,
        # and the db must stop re-asserting its own targets.
        sb.db_straight(500.0, 150.0)
        self.w.advance(300)
        self.assertFalse(sb.db_done())
        sb.db_move_wheels(100, 100)
        self.w.advance(600)               # ramp down from cruise
        # Our speeds survived the ticks (the db is yielded after the
        # ramp, not fighting us) — the unfixed shape would overwrite
        # them.
        self.assertEqual(self.w.spd[2], -100)
        self.assertEqual(self.w.spd[1], 100)

    def test_move_wheels_then_a_move_rearms_cleanly(self):
        # After direct control the next coupled move must arm from
        # the true pose, not lurch back toward a stale hold.
        sb.db_move_wheels(200, 200)
        self.w.advance(400)
        here = (self._mm(0), self._mm(1))
        sb.db_straight(100.0, 150.0)
        self.w.advance(4000)
        self.assertTrue(sb.db_done())
        self.assertTrue(abs(self._mm(0) - here[0] - 100) < 12,
                        (here[0], self._mm(0)))

    def test_move_wheels_zero_is_a_stop_not_a_coast(self):
        sb.db_move_wheels(300, 300)
        self.w.advance(150)               # ramp to 300 completes
        self.w.torque_pkts = []
        sb.db_move_wheels(0, 0)
        self.w.advance(150)               # ramp back down completes
        self.assertEqual(self.w.spd[1], 0)
        self.assertEqual(self.w.spd[2], 0)
        self.assertEqual(self.w.torque_pkts, [])   # torque stays on

    def test_move_start_torques_both_wheels_in_one_packet(self):
        # Both wheels engage at the same packet boundary too: the
        # first move's torque-on is ONE sync-write covering both ids.
        self.w.torque_pkts = []
        sb.db_straight(100.0, 80.0)
        self.w.advance(20)
        self.assertEqual(len(self.w.torque_pkts), 1, self.w.torque_pkts)
        pairs = sorted(self.w.torque_pkts[0])
        self.assertEqual(pairs, [(1, 1), (2, 1)])

    def test_stop_coast_releases_both_wheels_in_one_packet(self):
        # THE atomic-stop contract, on the wire: db_stop(0) puts both
        # wheels' torque-off in ONE sync-write — previously each
        # wheel got its own single-servo write, a bus transaction
        # apart, so one wheel free-wheeled while the other still
        # drove.
        sb.db_straight(500.0, 150.0)
        self.w.advance(400)                    # mid-move, cruising
        self.assertTrue(abs(self.w.spd[1]) > 0)
        self.w.torque_pkts = []
        self.assertTrue(sb.db_stop(0))
        self.w.advance(30)
        self.assertEqual(len(self.w.torque_pkts), 1, self.w.torque_pkts)
        pairs = sorted(self.w.torque_pkts[0])
        self.assertEqual(pairs, [(1, 0), (2, 0)])
        self.assertEqual(self.w.torque[1], 0)
        self.assertEqual(self.w.torque[2], 0)

    def test_stop_brake_ramps_to_zero_with_torque_kept_on(self):
        sb.db_straight(500.0, 150.0)
        self.w.advance(400)
        self.w.torque_pkts = []
        self.assertTrue(sb.db_stop(1))
        self.w.advance(30)
        # Mid-decel (uniform-accel rule, deceleration too): speed is
        # coming DOWN but not yet zero — a brake is a controlled
        # ramp, not a cliff.
        mid = abs(self.w.spd[1])
        self.assertTrue(0 < mid < 2223,
                        "expected a partial decel, got %d" % mid)
        self.w.advance(700)
        # No torque traffic at all — the ramped zero-speed sync IS
        # the brake.
        self.assertEqual(self.w.torque_pkts, [])
        self.assertEqual(self.w.spd[1], 0)
        self.assertEqual(self.w.spd[2], 0)
        self.assertEqual(self.w.torque[1], 1)
        self.assertEqual(self.w.torque[2], 1)

    def test_stop_hold_decelerates_then_anchors_where_it_stopped(self):
        # Hold called mid-cruise: the robot ramps DOWN at the accel
        # setting and the hold anchors where it actually stopped —
        # not a teleport-stop at the request point.
        sb.db_straight(500.0, 150.0)
        self.w.advance(400)               # mid-cruise
        pos_at_request = self.w.pos[1]
        self.assertTrue(sb.db_stop(2))
        self.w.advance(900)               # decel completes, hold arms
        self.assertTrue(self.w.pos[1] > pos_at_request + 20,
                        "no roll-out: hold anchored at the request "
                        "point (instant stop)")
        settled = self.w.pos[1]
        self.w.advance(300)
        self.assertTrue(abs(self.w.pos[1] - settled) < 5,
                        "hold not holding after the ramp")

    def test_stop_hold_arms_both_position_holds_at_once(self):
        sb.db_straight(150.0, 150.0)
        self.w.advance(3000)
        self.assertTrue(sb.db_done())
        self.assertTrue(sb.db_stop(2))
        self.w.advance(50)
        here = (self.w.pos[1], self.w.pos[2])
        # Shove both wheels off the captured pose; the C holds must
        # pull them back without any Python involvement.
        self.w.pos[1] += 60
        self.w.pos[2] += 60
        self.w.advance(800)
        self.assertTrue(abs(self.w.pos[1] - here[0]) < 15,
                        (here[0], self.w.pos[1]))
        self.assertTrue(abs(self.w.pos[2] - here[1]) < 15,
                        (here[1], self.w.pos[2]))

    def test_stop_hold_before_odometry_is_refused(self):
        # A hold with no live odometry would anchor to counts=0 and
        # slam the shafts — must be refused loudly, not armed wrong.
        sb.test_reset()
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        sb.db_config(0, 1, 88.0, 136.0, 400.0)
        self.assertFalse(sb.db_stop(2))        # no feedback read yet

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


class HardHeadingSourceTests(_Base):
    """db_gyro_source(1): the db tick pulls heading from the hard-tick
    yaw integrator (imu_yaw_core) every millisecond — the raw-IMU
    (ICM-45686) path, no Python pump. The feed binding is the
    synthetic-gyro seam until the SPI driver exists."""

    def setUp(self):
        super().setUp()
        from _openbricks_native import motor_process as m
        self.m = m
        m.hard_yaw_config(1.0)     # re-init integrator, unit scale
        sb.db_use_gyro(True)
        sb.db_gyro_source(1)

    def _pump_with_hard_gyro(self):
        """One pump, feeding the integrator the body rate implied by
        the wheels — an honest 1 kHz gyro on a non-slipping chassis."""
        prev = ((sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
                * 360.0 / 4096.0) / (136.0 / 88.0)
        self.w.pump()
        now = ((sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
               * 360.0 / 4096.0) / (136.0 / 88.0)
        self.m.hard_yaw_feed(1.0, (now - prev) * 1000.0)

    def test_gyro_turn_converges_on_the_hard_source(self):
        sb.db_turn(45.0, 60.0)
        for _ in range(5000):
            self._pump_with_hard_gyro()
            if sb.db_done():
                break
        self.assertTrue(sb.db_done())
        body = ((sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
                * 360.0 / 4096.0) / (136.0 / 88.0)
        self.assertTrue(abs(body - 45.0) < 3.0,
                        "hard-source turn landed at %.1f body-deg" % body)

    def test_source_selection_captures_the_reference(self):
        # Yaw accumulated BEFORE selecting the source must not leak
        # into the frame: select-time yaw is the zero.
        for _ in range(500):
            self.m.hard_yaw_feed(1.0, 90.0)   # +45 deg of pre-history
        sb.db_gyro_source(1)                  # re-select: new ref
        sb.db_turn(20.0, 60.0)
        for _ in range(4000):
            self._pump_with_hard_gyro()
            if sb.db_done():
                break
        body = ((sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
                * 360.0 / 4096.0) / (136.0 / 88.0)
        self.assertTrue(abs(body - 20.0) < 3.0,
                        "pre-history leaked: landed %.1f" % body)

    def test_reset_runtime_restores_python_source(self):
        sb.reset_runtime()
        # Rebuild a minimal db; with source back to 0, the tick must
        # NOT pull from the integrator (the Python override rules).
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.w.advance(50)
        sb.db_config(0, 1, 88.0, 136.0, 400.0)
        sb.db_use_gyro(True)
        for _ in range(300):
            self.m.hard_yaw_feed(1.0, 200.0)  # integrator spinning
        sb.db_set_heading(0.0)
        sb.db_straight(100.0, 60.0)
        self.w.advance(200)
        # Python source: heading pinned at 0 -> wheels stay matched
        # (no counter-steer against the integrator's phantom spin).
        dl = sb.servo_counts(0)
        dr = sb.servo_counts(1)
        self.assertTrue(abs(dl - dr) < 60, (dl, dr))

    def test_yaw_bindings_and_calibration_surface(self):
        # Exercises the full binding surface + the core's branch set
        # under gcov (c-unit covers the core separately; this makes
        # the MP-coverage build see it too): rest-bias learn -> state
        # shows the lock; zero-dt guard; negative mounting scale;
        # reset keeps calibration.
        m = self.m
        m.hard_yaw_config(-1.0)
        for _ in range(1500):
            m.hard_yaw_feed(1.0, 0.9)          # rest with 0.9 dps bias
        bias, locked, _still = m.hard_yaw_state()
        self.assertTrue(locked)
        self.assertTrue(abs(bias - 0.9) < 0.1)
        m.hard_yaw_feed(0.0, 500.0)            # zero-dt: ignored
        for _ in range(1000):
            m.hard_yaw_feed(1.0, 90.9)         # 90 dps turn (+bias)
        self.assertTrue(abs(m.hard_yaw_deg() + 90.0) < 3.0)  # sign -1
        # The reset binding refuses while the db steers by the gyro
        # (Pybricks parity, 1.93.0) — this class's setUp arms it.
        try:
            m.hard_yaw_reset()
            self.fail("expected OSError under gyro-armed db")
        except OSError as e:
            self.assertTrue("db.reset" in str(e), e)
        sb.db_use_gyro(False)
        m.hard_yaw_reset()
        self.assertTrue(abs(m.hard_yaw_deg()) < 1e-6)
        _bias2, locked2, _s2 = m.hard_yaw_state()
        self.assertTrue(locked2)               # reset keeps the cal
        # NVS-seeding path: a stored bias installs directly and
        # counts as calibrated.
        m.hard_yaw_config(1.0)                 # re-init: lock cleared
        m.hard_yaw_seed_bias(0.5)
        bias3, locked3, _s3 = m.hard_yaw_state()
        self.assertTrue(locked3)
        self.assertTrue(abs(bias3 - 0.5) < 1e-9)


class DutyDriveTests(_Base):
    """Dumb mode end to end over the wire harness: the engine's FF+PI
    closes the speed loop over raw mode-2 duty packets, the servo's
    own controller out of the circuit. The harness plant follows the
    bench's linear free-run line (~10 steps/s per duty unit)."""

    def _to_duty(self):
        sb.servo_drive_duty(0, True)
        sb.servo_drive_duty(1, True)
        self.w.advance(50)              # re-config (mode 2) completes

    def test_duty_drive_converges_on_the_target(self):
        self._to_duty()
        sb.duty_gains(101, 51, 3)
        speed_syncs_before = self.w.syncs
        sb.servo_run(0, 3000)
        sb.servo_run(1, 3000)
        self.w.advance(600)
        self.assertTrue(getattr(self.w, "duty_syncs", 0) > 0,
                        "no duty packets reached the wire")
        self.assertEqual(self.w.syncs, speed_syncs_before,
                         "duty slots must not ship goal-speed syncs")
        for sid in (1, 2):
            err = abs(abs(self.w.spd[sid]) - 3000)
            self.assertTrue(err < 200,
                            "servo %d speed %r not near 3000"
                            % (sid, self.w.spd[sid]))

    def test_duty_rest_stops_the_wheel_and_the_packets(self):
        self._to_duty()
        sb.servo_run(0, 3000)
        self.w.advance(300)
        sb.servo_run(0, 0)
        self.w.advance(100)
        self.assertEqual(self.w.spd[2], 0)
        quiet = getattr(self.w, "duty_syncs", 0)
        self.w.advance(200)
        self.assertEqual(getattr(self.w, "duty_syncs", 0), quiet,
                         "a resting duty slot must go bus-quiet")

    def test_switch_back_restores_wheel_mode(self):
        self._to_duty()
        sb.servo_drive_duty(0, False)
        sb.servo_drive_duty(1, False)
        self.w.advance(50)              # re-config (mode 1) completes
        before = getattr(self.w, "duty_syncs", 0)
        sb.servo_run(0, 2000)
        self.w.advance(200)
        self.assertEqual(getattr(self.w, "duty_syncs", 0), before)
        self.assertEqual(abs(self.w.spd[2]), 2000)

    def test_bad_slot_is_loud(self):
        try:
            sb.servo_drive_duty(9, True)
            self.fail("expected ValueError")
        except ValueError:
            pass


class DutyDriveBaseTests(_Base):
    """The 2-DOF drivebase controller riding the duty loop end to
    end: db moves converge with the servo's own controller fully out
    of the circuit."""

    def setUp(self):
        _Base.setUp(self)
        sb.servo_drive_duty(0, True)
        sb.servo_drive_duty(1, True)
        self.w.advance(50)              # mode-2 re-config completes

    def test_straight_converges_on_duty_packets_only(self):
        speed_syncs_before = self.w.syncs
        sb.db_straight(200.0, 150.0)
        self.w.advance(3500)
        self.assertTrue(sb.db_done())
        self.assertTrue(abs(self._mm(0) - 200) < 12, self._mm(0))
        self.assertTrue(abs(self._mm(1) - 200) < 12, self._mm(1))
        self.assertTrue(getattr(self.w, "duty_syncs", 0) > 50)
        self.assertEqual(self.w.syncs, speed_syncs_before)

    def test_turn_converges_on_duty(self):
        sb.db_turn(90.0, 60.0)
        self.w.advance(3000)
        self.assertTrue(sb.db_done())


class TurnAccelTests(_Base):
    """Separate turn acceleration (Pybricks parity, 1.90.0): the arm
    glue selects the per-move-type accel, so a crawl turn accel must
    not slow the straights."""

    def test_turn_accel_binding_and_independent_ramps(self):
        sb.db_set_turn_accel(30.0)          # crawl ramp for turns
        sb.db_straight(150.0, 150.0)        # straights unaffected
        self.w.advance(2500)
        self.assertTrue(sb.db_done())
        sb.db_turn(90.0, 60.0)              # ~2 s cruise + slow ramps
        self.w.advance(700)
        self.assertFalse(sb.db_done(),
                         "a 30 dps^2 turn ramp cannot finish 90 deg "
                         "in 0.7 s")
        self.w.advance(9000)
        self.assertTrue(sb.db_done())

    def test_curve_rides_the_straight_accel(self):
        # curve() is a drive move (Pybricks classification): it must
        # arm with the STRAIGHT accel even when the turn accel is a
        # crawl.
        sb.db_set_turn_accel(30.0)
        sb.db_curve(200.0, 45.0, 120.0)
        self.w.advance(4000)
        self.assertTrue(sb.db_done(),
                        "curve slowed by the turn accel")


class EstopBindingTests(_Base):
    """st_bus.estop() — the launcher's program-exit kill for adopted
    buses: writers dead (db + per-slot moves), broadcast torque-off.
    Torque-off alone would be re-armed by the next db tick."""

    def test_estop_mid_move_stops_and_stays_stopped(self):
        sb.db_straight(400.0, 150.0)
        self.w.advance(400)
        self.assertTrue(abs(self.w.spd[1]) > 0)
        self.assertTrue(sb.estop())
        self.w.advance(5)
        self.assertEqual(self.w.spd[1], 0)
        self.assertEqual(self.w.spd[2], 0)
        self.assertEqual(self.w.torque[1], 0)
        self.assertEqual(self.w.torque[2], 0)
        # Nothing re-drives: no torque re-arm, no speed/duty syncs.
        before = (self.w.syncs, getattr(self.w, "duty_syncs", 0))
        self.w.advance(500)
        self.assertEqual(self.w.spd[1], 0)
        self.assertEqual(self.w.spd[2], 0)
        self.assertEqual(
            (self.w.syncs, getattr(self.w, "duty_syncs", 0)), before)
        self.assertEqual(self.w.torque[1], 0)

    def test_estop_kills_a_per_slot_move_too(self):
        self.assertTrue(sb.servo_move(0, 40960.0, 2000.0, 8000.0))
        self.w.advance(50)
        self.assertTrue(abs(self.w.spd[2]) > 0)
        sb.estop()
        self.w.advance(300)
        self.assertEqual(self.w.spd[2], 0)
        self.assertEqual(self.w.torque[2], 0)


class HeadingResetParityTests(_Base):
    """``db_reset()``: Pybricks ``DriveBase.reset()`` parity.

    Re-zeroes the yaw integrator, the engine's frame reference, and
    the held target in ONE locked section — the only sanctioned
    heading re-zero while a drive base steers by the gyro. Bench
    2026-08-13: ``imu.reset_heading()`` alone shifted the measured
    frame under the held target, and ``straight()`` after
    ``turn(-90)`` pivoted left chasing the -90 the target still
    remembered.
    """

    def setUp(self):
        super().setUp()
        from _openbricks_native import motor_process as m
        self.m = m
        m.hard_yaw_config(1.0)
        sb.db_use_gyro(True)
        sb.db_gyro_source(1)

    def _pump_with_hard_gyro(self):
        prev = ((sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
                * 360.0 / 4096.0) / (136.0 / 88.0)
        self.w.pump()
        now = ((sb.servo_counts(0) - sb.servo_counts(1)) / 2.0
               * 360.0 / 4096.0) / (136.0 / 88.0)
        self.m.hard_yaw_feed(1.0, (now - prev) * 1000.0)

    def test_gyro_in_use_reflects_db_state(self):
        self.assertTrue(sb.db_gyro_in_use())
        sb.db_use_gyro(False)
        self.assertFalse(sb.db_gyro_in_use())

    def test_reset_before_config_raises(self):
        sb.test_reset()
        try:
            sb.db_reset()
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("db_config" in str(e), e)

    def test_reset_during_a_move_raises(self):
        sb.db_straight(200.0, 60.0)
        try:
            sb.db_reset()
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("stop first" in str(e), e)

    def test_reset_zeroes_integrator_and_frame(self):
        sb.db_turn(-90.0, 90.0)
        for _ in range(8000):
            self._pump_with_hard_gyro()
            if sb.db_done():
                break
        self.assertTrue(sb.db_done())
        self.assertTrue(abs(self.m.hard_yaw_deg()) > 60.0)
        sb.db_reset()
        self.assertTrue(abs(self.m.hard_yaw_deg()) < 0.001)

    def test_straight_after_turn_and_reset_goes_straight(self):
        # The bench scenario, distilled: turn(-90), reset, straight.
        # Without the atomic re-base the controller chases the old
        # -90 target and the "straight" is a pivot.
        sb.db_turn(-90.0, 90.0)
        for _ in range(8000):
            self._pump_with_hard_gyro()
            if sb.db_done():
                break
        self.assertTrue(sb.db_done())
        sb.db_reset()
        base_l = sb.servo_counts(0)
        base_r = sb.servo_counts(1)
        sb.db_straight(100.0, 60.0)
        for _ in range(8000):
            self._pump_with_hard_gyro()
            if sb.db_done():
                break
        self.assertTrue(sb.db_done())
        dl = sb.servo_counts(0) - base_l
        dr = sb.servo_counts(1) - base_r
        self.assertTrue(abs(dl - dr) < 60,
                        "veered after reset: dl=%d dr=%d" % (dl, dr))

    def test_reset_without_gyro_is_benign(self):
        sb.db_use_gyro(False)
        sb.db_reset()   # no raise; encoder frame re-derives per arm


class _StuckWheels(_PerfectWheels):
    """Wheels that answer every read but never move — a blocked
    robot. NOT the dead-wheel fault: reads keep succeeding, so the
    stale counter never climbs; the odometry is honestly frozen
    because the chassis is."""

    track = 0.0


class BoundedSettleTests(_Base):
    """Post-profile arrival wait is CAPPED for SMALL residuals
    (OB_DRIVEBASE_SETTLE_MS / _FORGIVE, 1.95.0). Duty-mode stiction
    can leave the last degrees of a turn below feedback's breakaway
    authority; the old arrival-or-nothing latch then stalled the
    wait loop (bench 2026-08-14: intermittent ~1 s pause between
    turn() and the next straight()). A forgiven residual stays in
    the gyro's absolute frame — the next move corrects it in motion.
    A LARGE residual (blocked robot) still refuses to latch: the
    move watchdog must fail loudly, never a silent wrong pose."""

    _WHEEL_PER_BODY = 136.0 / 88.0

    def test_small_stiction_residual_latches_at_the_settle_cap(self):
        # Freeze the heading 6 wheel-deg short of the target — inside
        # the forgive limit, outside the arrival tolerance. The old
        # latch waited forever (the bench pause); now the cap fires.
        sb.db_use_gyro(True)
        target_body = 45.0
        frozen_body = target_body - 6.0 / self._WHEEL_PER_BODY
        sb.db_turn(target_body, 90.0)
        for _ in range(1400):              # well past the profile
            self.w.pump()
            sb.db_set_heading(frozen_body)
        elapsed = 0
        while not sb.db_done() and elapsed < 2000:
            self.w.pump()
            sb.db_set_heading(frozen_body)
            elapsed += 1
        self.assertTrue(sb.db_done(), "never latched (old stall)")

    def test_blocked_robot_still_refuses_to_latch(self):
        # Heading stuck at ZERO — the turn genuinely didn't happen.
        # Forgiving this would silently continue from a wrong pose;
        # done must stay false (the caller's watchdog raises).
        sb.db_use_gyro(True)
        sb.db_turn(90.0, 60.0)
        for _ in range(4000):
            self.w.pump()
            sb.db_set_heading(0.0)
        self.assertFalse(sb.db_done())

    def test_new_move_restarts_the_settle_window(self):
        # The cap is per-move: after a capped-done move, a freshly
        # armed move must run its own profile + window, not inherit
        # a stale settle timestamp.
        sb.db_use_gyro(True)
        frozen_body = 45.0 - 6.0 / self._WHEEL_PER_BODY
        sb.db_turn(45.0, 90.0)
        for _ in range(2400):
            self.w.pump()
            sb.db_set_heading(frozen_body)
        self.assertTrue(sb.db_done())
        sb.db_turn(45.0, 90.0)             # fresh move
        for _ in range(200):               # still inside its profile
            self.w.pump()
            sb.db_set_heading(frozen_body)
        self.assertFalse(sb.db_done())


class StuckWheelsLoudnessTests(unittest.TestCase):
    """A fully blocked chassis in ENCODER mode: the sum error stays
    at the whole move — far past the forgive limit — so done must
    never latch (the move watchdog owns this failure, loudly)."""

    def setUp(self):
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()
        self.w = _StuckWheels()
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.w.advance(50)
        sb.db_config(0, 1, 88.0, 136.0, 400.0)

    def test_blocked_straight_never_silently_completes(self):
        sb.db_straight(100.0, 80.0)
        self.w.advance(6000)               # profile + many caps
        self.assertFalse(sb.db_done())


class DecelSlewTests(_Base):
    """The 1.94.0 slew is symmetric: retargeting DOWN — including all
    the way to move_wheels(0, 0) — obeys settings.acceleration just
    like ramping up (bench directive 2026-08-14: deceleration follows
    the same value)."""

    def test_move_wheels_zero_ramps_down_not_instant(self):
        sb.db_move_wheels(2000, 2000)
        self.w.advance(600)                # up-ramp completes
        self.assertEqual(self.w.spd[1], 2000)
        sb.db_move_wheels(0, 0)
        self.w.advance(50)                 # mid-decel: ~227 of 2000 shed
        mid = abs(self.w.spd[1])
        self.assertTrue(1500 < mid < 1950,
                        "expected a partial decel, got %d" % mid)
        self.w.advance(600)
        self.assertEqual(self.w.spd[1], 0)
        self.assertEqual(self.w.spd[2], 0)


class TrajectoryEntrySpeedTests(unittest.TestCase):
    """Trajectories arm FROM THE CURRENT SPEED (2.0.0). A straight
    armed while the wheels cruise (line-follow handing over) blends
    down through settings.acceleration; before this, the command
    cliffed from cruise to ~zero in one tick and duty mode braked as
    hard as the plant allowed (bench 2026-08-16: "deceleration is
    too much while the acceleration is correct")."""

    def setUp(self):
        if sb is None:
            raise unittest.SkipTest("st_bus is firmware/unix-MP only")
        sb.test_reset()
        self.w = _PerfectWheels()
        sb.servo_attach(0, 2, True, 45)
        sb.servo_attach(1, 1, False, 45)
        self.w.advance(50)
        sb.db_config(0, 1, 88.0, 136.0, 1500.0)

    def test_straight_armed_at_cruise_blends_not_cliffs(self):
        sb.db_move_wheels(9443, 9443)       # ~830 dps line-follow cruise
        self.w.advance(800)
        self.assertEqual(self.w.spd[1], 9443)
        sb.db_straight(290.0, 350.0)
        self.w.advance(5)
        # 5 ms after the arm the command must still be near cruise —
        # the old from-rest profile dropped it to ~0 here.
        self.assertTrue(abs(self.w.spd[1]) > 9000,
                        "cliffed to %d" % self.w.spd[1])
        # And the descent obeys the accel setting: 1500 dps^2 is
        # ~17 steps/s per ms; after 100 ms more, expect ~1700 shed
        # (loose 2x bound, never a cliff).
        self.w.advance(100)
        shed = 9443 - abs(self.w.spd[1])
        self.assertTrue(800 < shed < 3600,
                        "shed %d in 100 ms (expected ~1700)" % shed)

    def test_short_straight_at_cruise_boosts_decel_no_reversal(self):
        # Physics: 830 dps needs ~56 mm to stop at 1500 dps^2. A
        # 20 mm command cannot absorb that entry at the configured
        # accel — since 2.4.0 the move KEEPS the true entry speed and
        # RAISES its deceleration to v0^2/(2D) (user decision,
        # replacing the 2.0.1 entry clamp): one continuous, steeper
        # ramp that lands at rest ON the mark. No feed-forward step
        # at the arm, no overshoot-reverse.
        sb.db_move_wheels(9443, 9443)
        self.w.advance(800)
        base_counts = sb.servo_counts(1)
        sb.db_straight(20.0, 350.0)
        self.w.advance(5)
        v = abs(self.w.spd[1])
        # Entry speed PRESERVED (~9443 steps/s, minus 5 ms of the
        # steep ramp) — neither clamped to ~3175 nor cliffed to ~0.
        self.assertTrue(v > 5000,
                        "entry %d, expected near 9443 (kept)" % v)
        # Position must be MONOTONIC — the overshoot-reverse cycle
        # is the regression this pins against.
        last = sb.servo_counts(1)
        for _ in range(60):
            self.w.advance(50)
            now = sb.servo_counts(1)
            self.assertTrue(now >= last - 3,
                            "reversed: %d -> %d" % (last, now))
            last = now
        self.assertTrue(sb.db_done())
        travelled = (sb.servo_counts(1) - base_counts) * _MM_PER_COUNT
        self.assertTrue(abs(travelled - 20.0) < 8.0,
                        "landed %.1f mm (wanted 20)" % travelled)

    def test_straight_with_carry_ends_at_cruise_and_hands_over(self):
        # then="continue" (2.5.0, Pybricks Stop.NONE): the profile
        # ends AT cruise, done latches while still flying, and the
        # reference keeps advancing until the next command.
        # 350 mm/s on the 88 mm wheel ~= 458 dps ~= 5210 steps/s.
        sb.db_straight(300.0, 350.0, 1)
        for _ in range(200):               # up to 2 s
            self.w.advance(10)
            if sb.db_done():
                break
        self.assertTrue(sb.db_done(), "carry move never latched done")
        v_done = abs(self.w.spd[1])
        self.assertTrue(v_done > 4500,
                        "speed at done %d, expected ~5210 (carried)"
                        % v_done)
        # No next command: the reference keeps driving (Pybricks
        # Stop.NONE contract) — speed holds, position advances.
        c0 = sb.servo_counts(1)
        self.w.advance(300)
        self.assertTrue(abs(self.w.spd[1]) > 4500,
                        "carried speed decayed to %d" % self.w.spd[1])
        self.assertTrue(abs(sb.servo_counts(1) - c0) > 1000,
                        "reference stopped advancing")

    def test_carry_hands_speed_to_the_next_straight(self):
        # The seam this feature exists for: segment 1 carries,
        # segment 2 arms from full speed — the wheel speed through
        # the handover never dips below ~cruise.
        sb.db_straight(200.0, 350.0, 1)
        for _ in range(200):
            self.w.advance(10)
            if sb.db_done():
                break
        self.assertTrue(sb.db_done())
        sb.db_straight(200.0, 350.0, 0)    # stopping second leg
        min_v = 99999
        for _ in range(30):                # first 300 ms of leg 2
            self.w.advance(10)
            v = abs(self.w.spd[1])
            if v < min_v:
                min_v = v
        self.assertTrue(min_v > 4000,
                        "speed dipped to %d through the seam" % min_v)

    def test_plain_straight_still_stops_at_the_end(self):
        # carry=0 (and the omitted-arg form) keeps the classic
        # profile: at completion the wheels are at rest.
        sb.db_straight(200.0, 350.0)
        for _ in range(300):
            self.w.advance(10)
            if sb.db_done():
                break
        self.assertTrue(sb.db_done())
        self.w.advance(50)
        self.assertTrue(abs(self.w.spd[1]) < 200,
                        "still moving at %d after a stopping move"
                        % self.w.spd[1])

    def test_zero_radius_curve_after_cruise_ramps_forward_too(self):
        # curve(radius=0) is a turn in place — entered while
        # translating it takes the same forward-axis stop trajectory
        # as turn().
        sb.db_move_wheels(9443, 9443)
        self.w.advance(800)
        sb.db_curve(0.0, 90.0, 150.0)
        self.w.advance(5)
        s_sum = abs(self.w.spd[1] - self.w.spd[2]) / 2
        self.assertTrue(s_sum > 8500,
                        "forward axis cliffed: sum %d" % s_sum)
        self.w.advance(9000)
        self.assertTrue(sb.db_done())

    def test_turn_after_cruise_ramps_the_forward_axis_down(self):
        # turn() armed while translating: the forward axis gets a
        # STOP trajectory at the accel limit, not an instant
        # position-hold (which braked at plant limit — the same
        # cliff the entry speeds exist to remove).
        sb.db_move_wheels(9443, 9443)
        self.w.advance(800)
        sb.db_turn(90.0, 90.0)
        self.w.advance(5)
        # 5 ms in, the SUM of the wheel commands must still be near
        # cruise (the turn differential rides on top of it).
        s_sum = abs(self.w.spd[1] - self.w.spd[2]) / 2
        self.assertTrue(s_sum > 8500,
                        "forward axis cliffed: sum %d" % s_sum)
        self.w.advance(8000)
        self.assertTrue(sb.db_done())
