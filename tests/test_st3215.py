# SPDX-License-Identifier: MIT
"""Tests for the ST-3215 serial bus servo driver."""

import tests._fakes  # noqa: F401

import unittest

from openbricks.drivers import st3215 as st3215_mod
from openbricks.drivers.st3215 import ST3215, ST3215Motor, SyncServoGroup


_REG_OP_MODE       = 0x21
_REG_TORQUE        = 0x28
_REG_GOAL_POSITION = 0x2A
_REG_GOAL_SPEED    = 0x2E
_REG_PRESENT_POS   = 0x38
_HEADER            = b"\xFF\xFF"


def _checksum(body):
    s = 0
    for b in body:
        s += b
    return (~s) & 0xFF


class TestST3215(unittest.TestCase):
    def setUp(self):
        # The bus registry is class-level state; reset it so each test sees
        # a fresh UART.
        ST3215._buses = {}

    def test_move_to_writes_goal_position_packet(self):
        servo = ST3215(servo_id=1, uart_id=1, tx=17, rx=16)
        servo.move_to(180, wait=False)

        # 180 deg on a 0..4095 raw range maps to int(4095 * 180 / 360) = 2047.
        raw = 2047
        data = bytes([raw & 0xFF, (raw >> 8) & 0xFF])
        params = bytes([_REG_GOAL_POSITION]) + data
        length = len(data) + 3
        body = bytes([1, length, 0x03]) + params
        expected = _HEADER + body + bytes([_checksum(body)])

        self.assertEqual(servo._bus._uart._tx_log[0], expected)

    def test_move_to_with_speed_writes_two_packets(self):
        servo = ST3215(servo_id=2)
        servo.move_to(90, speed=500, wait=False)

        # First packet: set goal speed. Second: goal position.
        tx = servo._bus._uart._tx_log
        self.assertEqual(len(tx), 2)

        speed = 500
        speed_body = bytes([2, 5, 0x03, _REG_GOAL_SPEED,
                            speed & 0xFF, (speed >> 8) & 0xFF])
        self.assertEqual(tx[0], _HEADER + speed_body + bytes([_checksum(speed_body)]))

    def test_checksum_is_ones_complement_of_body_sum(self):
        servo = ST3215(servo_id=1)
        servo.move_to(0, wait=False)
        packet = servo._bus._uart._tx_log[0]
        # packet = 0xFF 0xFF <body> <chk>
        body = packet[2:-1]
        chk = packet[-1]
        self.assertEqual(chk, _checksum(body))

    def test_ping_emits_ping_instruction(self):
        servo = ST3215(servo_id=7)
        servo.ping()
        packet = servo._bus._uart._tx_log[0]
        body = packet[2:-1]
        # Instruction byte is at offset 2 within body: [id, length, instr].
        self.assertEqual(body[0], 7)      # servo id
        self.assertEqual(body[2], 0x01)   # PING

    def test_angle_returns_none_when_bus_is_silent(self):
        # The fake UART returns no RX data, so the read times out and angle()
        # reports None rather than a bogus zero.
        servo = ST3215(servo_id=1)
        self.assertIsNone(servo.angle())

    def test_tx_drains_stale_rx_bytes_first(self):
        # Bug: half-duplex bus echo or a timed-out previous reply can
        # leave bytes in the RX FIFO. Without draining before TX, the
        # next read() pulls those stale bytes — symptom on real
        # hardware: ping returns True (any 6 bytes pass the length
        # check) but read() returns None (stale bytes don't form a
        # valid reply). _tx must drain first.
        servo = ST3215(servo_id=1)
        # Pre-stuff the RX buffer with stale junk + the actual valid
        # reply we expect to see for a present-position read.
        # Real reply layout for a 2-byte read: FF FF ID LEN ERR D0 D1 CHK.
        valid_reply = b"\xFF\xFF\x01\x04\x00\x39\x05\xBC"
        servo._bus._uart._rx_buf = b"\xAA\x55\xDE\xAD" + valid_reply
        # The read should drain the junk in _tx, send the read packet,
        # then read the valid reply that we placed AFTER the junk.
        # Since the fake UART has only one buffer, draining empties
        # everything — so we restage the valid reply post-drain via
        # a write-side hook on the fake.
        original_write = servo._bus._uart.write
        def write_then_stage(data):
            servo._bus._uart._rx_buf = valid_reply
            return original_write(data)
        servo._bus._uart.write = write_then_stage
        # Read should now succeed against the valid reply (D0=0x39 D1=0x05).
        result = servo._bus.read(servo._id, _REG_PRESENT_POS, 2)
        self.assertEqual(result, b"\x39\x05")

    def test_buses_are_shared_per_uart(self):
        # Two servos on the same UART params share one _SCServoBus instance.
        s1 = ST3215(servo_id=1, uart_id=2, tx=17, rx=16)
        s2 = ST3215(servo_id=2, uart_id=2, tx=17, rx=16)
        self.assertIs(s1._bus, s2._bus)

        # A different UART id gets a separate bus.
        s3 = ST3215(servo_id=3, uart_id=1, tx=17, rx=16)
        self.assertIsNot(s1._bus, s3._bus)


def _decode_write(packet):
    """Pull (servo_id, register, data_bytes) out of an SCServo write packet."""
    assert packet.startswith(_HEADER)
    body = packet[2:-1]
    sid, length, instr = body[0], body[1], body[2]
    assert instr == 0x03   # WRITE
    register = body[3]
    data     = bytes(body[4:])
    return sid, register, data


def _writes_to(packets, register):
    """Filter a UART tx log down to writes targeting one register."""
    out = []
    for pkt in packets:
        sid, reg, data = _decode_write(pkt)
        if reg == register:
            out.append((sid, data))
    return out


class TestST3215Motor(unittest.TestCase):
    def setUp(self):
        ST3215._buses = {}

    def test_constructor_switches_servo_into_wheel_mode(self):
        m = ST3215Motor(servo_id=1)
        mode_writes = _writes_to(m._bus._uart._tx_log, _REG_OP_MODE)
        self.assertEqual(mode_writes, [(1, bytes([1]))])   # 1 = wheel

    def test_constructor_enables_torque(self):
        m = ST3215Motor(servo_id=2)
        torque_writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE)
        self.assertEqual(torque_writes, [(2, bytes([1]))])

    def test_run_speed_writes_signed_magnitude_to_goal_speed(self):
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        m.run_speed(50)   # → magnitude = 500, sign bit clear
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        self.assertEqual(speed_writes[-1], (3, bytes([500 & 0xFF, 500 >> 8])))

    def test_run_speed_negative_sets_high_bit(self):
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        m.run_speed(-50)
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        # magnitude 500, plus the direction bit at bit 15 of the 16-bit value
        v = 500 | 0x8000
        self.assertEqual(speed_writes[-1], (3, bytes([v & 0xFF, (v >> 8) & 0xFF])))

    def test_run_speed_clamps_to_max_dps(self):
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=100.0)
        m.run_speed(99999)
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        # Clamped to 100 dps × 10 steps/dps = 1000
        self.assertEqual(speed_writes[-1], (3, bytes([1000 & 0xFF, 1000 >> 8])))

    def test_invert_flips_run_speed_direction(self):
        # Both servos default to uart_id=1 → they share a _SCServoBus,
        # so the tx_log holds packets from both. Filter by servo_id to
        # isolate each motor's last command.
        m_plain = ST3215Motor(servo_id=4, steps_per_dps=10.0, max_dps=1000.0)
        m_inv   = ST3215Motor(servo_id=5, steps_per_dps=10.0, max_dps=1000.0,
                              invert=True)
        m_plain.run_speed(50)
        m_inv.run_speed(50)
        all_speed_writes = _writes_to(m_plain._bus._uart._tx_log,
                                      _REG_GOAL_SPEED)
        plain = next(d for sid, d in reversed(all_speed_writes) if sid == 4)
        inv   = next(d for sid, d in reversed(all_speed_writes) if sid == 5)
        # plain should have sign bit clear; inv should have it set
        self.assertEqual(plain[1] & 0x80, 0)
        self.assertEqual(inv[1] & 0x80, 0x80)

    def test_brake_writes_zero_speed(self):
        m = ST3215Motor(servo_id=6)
        m.brake()
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        self.assertEqual(speed_writes[-1], (6, bytes([0, 0])))

    def test_coast_disables_torque(self):
        m = ST3215Motor(servo_id=7)
        m.coast()
        torque_writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE)
        # Constructor wrote a 1; coast should append a 0.
        self.assertEqual(torque_writes[-1], (7, bytes([0])))

    def test_motor_base_class_hold_raises_not_implemented(self):
        # ``hold`` is opt-in: open-loop drivers and any Motor subclass
        # that doesn't override it fall through to the base raise so
        # callers see a clear failure rather than a silent no-op.
        from openbricks.interfaces import Motor
        with self.assertRaises(NotImplementedError):
            Motor().hold()

    def test_angle_accumulates_across_positive_wrap(self):
        # Synthesise the bus reads on the 12-bit (0..4095) absolute-
        # position register. 3800 → 100 means the wheel moved +396
        # counts the short way (across the 0/4095 boundary), not
        # -3700 the long way. The wrap heuristic must pick the
        # short-arc interpretation.
        m = ST3215Motor(servo_id=8)

        # MicroPython doesn't allow attribute access on closures, so the
        # queue lives in an enclosing list (mutated via .pop()).
        queue = [3800, 100]

        def fake_read(servo_id, register, nbytes):
            assert servo_id == 8 and register == _REG_PRESENT_POS and nbytes == 2
            v = queue.pop(0) & 0xFFFF
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        m._bus.read = fake_read

        first  = m.angle()
        second = m.angle()
        # First read = absolute baseline at 3800 counts.
        self.assertAlmostEqual(first, 3800 * 360.0 / 4096, places=2)
        # Wrap correction: delta = 100 - 3800 = -3700, < -2048, so
        # delta += 4096 → +396. accum = 3800 + 396 = 4196.
        expected_total = (3800 + 396) * 360.0 / 4096
        self.assertAlmostEqual(second, expected_total, places=2)

    def test_reset_angle_zeroes_the_reading(self):
        m = ST3215Motor(servo_id=9)

        # 12-bit absolute position; pin all reads to the same raw value
        # so reset_angle observes a stable baseline.
        queue = [2000, 2000, 2000]

        def fake_read(servo_id, register, nbytes):
            v = queue[0] if len(queue) == 1 else queue.pop(0)
            v &= 0xFFFF
            return bytes([v & 0xFF, (v >> 8) & 0xFF])
        m._bus.read = fake_read

        before = m.angle()
        self.assertGreater(before, 0)
        m.reset_angle(0)
        after = m.angle()
        self.assertAlmostEqual(after, 0.0, places=2)


class TestST3215MotorRunAngle(unittest.TestCase):
    """``run_angle`` switches the servo into native position mode
    (op_mode=0) for the move and restores wheel mode (op_mode=1)
    when done. These tests pin the protocol-level contract; actual
    position-PID convergence is tested on hardware."""

    def setUp(self):
        ST3215._buses = {}

    def _patch_present_pos(self, motor, sequence):
        """Make successive ``motor._read_present_pos()`` calls return
        the raw 12-bit values in ``sequence``. Last value sticks once
        the sequence runs out, so the in-chunk completion loop sees a
        stable 'arrived' reading."""
        values = list(sequence)

        def fake():
            return values.pop(0) if len(values) > 1 else values[0]
        motor._read_present_pos = fake

    def test_run_angle_anchors_goal_position_before_mode_switch(self):
        # Bench regression: on the first run_angle after open-loop
        # spins, the servo would undershoot by 5-10° because the
        # mode 1→0 flip activated the position PID against a stale
        # goal-position register, drifting the wheel for a few ms
        # before we wrote the real goal. Fix: write goal-position =
        # present BEFORE flipping mode, so the PID activates "at
        # target" and can't drift.
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_present_pos(m, [500, 500, 500 + 1024])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)

        # Filter to just the WRITE instructions we care about.
        log = m._bus._uart._tx_log[baseline:]
        ordered_targets = []
        for pkt in log:
            sid, reg, _ = _decode_write(pkt)
            if reg in (_REG_OP_MODE, _REG_GOAL_POSITION, _REG_GOAL_SPEED):
                ordered_targets.append(reg)
        # Required ordering: goal-position(anchor), then mode-switch
        # to position, then goal-speed, then the actual move's
        # goal-position(s), then mode-switch back to wheel, then
        # goal-speed=0 (brake).
        self.assertEqual(ordered_targets[0], _REG_GOAL_POSITION)
        self.assertEqual(ordered_targets[1], _REG_OP_MODE)

    def test_run_angle_then_brake_switches_into_position_mode_then_back(self):
        # With ``then="brake"`` the move ends by restoring wheel mode
        # so the next ``run_speed`` is interpreted as a signed velocity
        # rather than a position-mode speed cap.
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        target_counts = int(round(90 * 4096 / 360))   # = 1024
        self._patch_present_pos(m, [0, 0, target_counts])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90, then="brake")

        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        # First mode write into the move = position (0); last = wheel (1).
        self.assertEqual(mode_writes[0][1], bytes([0]))
        self.assertEqual(mode_writes[-1][1], bytes([1]))

    def test_run_angle_default_coasts_and_does_not_restore_wheel_mode(self):
        # New default: ``then="coast"`` cuts torque and leaves op_mode in
        # position. Skipping the wheel-mode re-flip avoids the ~85-raw
        # post-brake "settle" artifact observed on the bench. The next
        # ``run_speed`` / ``brake`` transparently restores wheel mode via
        # ``_ensure_mode``.
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        target_counts = int(round(90 * 4096 / 360))
        self._patch_present_pos(m, [0, 0, target_counts])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)   # default then="coast"

        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        # Only one mode write in the move: into position. No wheel-mode
        # flip back — coast doesn't need it.
        self.assertEqual(len(mode_writes), 1)
        self.assertEqual(mode_writes[0][1], bytes([0]))
        # And torque was cut at the end.
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes[-1][1], bytes([0]))

    def test_run_angle_writes_goal_position_at_present_plus_delta(self):
        m = ST3215Motor(servo_id=2, steps_per_dps=10.0, max_dps=1000.0)
        # Present pos starts at 100; +90° = +1024 counts → goal = 1124.
        # Anchor read first (→ 100), then chunk-start (→ 100),
        # then arrival (→ 1124).
        self._patch_present_pos(m, [100, 100, 100 + 1024])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)

        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        # Two writes: anchor (= present = 100) and the actual goal (1124).
        self.assertEqual(len(pos_writes), 2)
        anchor_goal = pos_writes[0][1][0] | (pos_writes[0][1][1] << 8)
        self.assertEqual(anchor_goal, 100)
        move_goal = pos_writes[1][1][0] | (pos_writes[1][1][1] << 8)
        self.assertEqual(move_goal, 1124)

    def test_run_angle_chunks_moves_larger_than_180_degrees(self):
        # 270° forward = 3072 counts. Can't be one chunk (would be
        # routed as 90° backward by the servo's shortest-path PID).
        # Implementation caps each chunk at 2000 counts (≈175.8°),
        # so we expect ⌈3072 / 2000⌉ = 2 chunks.
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        # Reads: 1 anchor + per-chunk (chunk-start + inner-arrival +
        # end-of-chunk self.angle()) = 1 + 2×3 = 7.
        self._patch_present_pos(m, [
            0,    # anchor (pre-mode-switch hold)
            0,    # iter1: chunk-start
            2000, # iter1: inner-loop arrival
            2000, # iter1: self.angle() update
            2000, # iter2: chunk-start
            3072, # iter2: inner-loop arrival
            3072, # iter2: self.angle() update (sticks for final)
        ])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=270)

        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        # Anchor (0), chunk1 (2000), chunk2 (3072).
        self.assertEqual(len(pos_writes), 3)
        goals = [w[1][0] | (w[1][1] << 8) for w in pos_writes]
        self.assertEqual(goals, [0, 2000, 3072])

    def test_run_angle_negative_target_writes_correct_goal(self):
        m = ST3215Motor(servo_id=4, steps_per_dps=10.0, max_dps=1000.0)
        # Start at 2000; anchor=2000, -90° = -1024 → goal = 976.
        self._patch_present_pos(m, [2000, 2000, 976])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=-90)
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        # Anchor (2000) + move goal (976).
        move_goal = pos_writes[1][1][0] | (pos_writes[1][1][1] << 8)
        self.assertEqual(move_goal, 976)

    def test_run_angle_invert_flips_direction(self):
        m = ST3215Motor(servo_id=5, steps_per_dps=10.0, max_dps=1000.0,
                        invert=True)
        # +90° asked, but invert means we should command -1024 counts.
        # Anchor=1024, present=1024 + (-1024) = 0.
        self._patch_present_pos(m, [1024, 1024, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        move_goal = pos_writes[1][1][0] | (pos_writes[1][1][1] << 8)
        self.assertEqual(move_goal, 0)

    def test_run_angle_wraps_goal_position_modulo_one_revolution(self):
        # Present 3500 + 1024 = 4524 → must wrap to 4524 % 4096 = 428.
        m = ST3215Motor(servo_id=6, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_present_pos(m, [3500, 3500, 428])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        # pos_writes[0] is the anchor (= present = 3500).
        # pos_writes[1] is the move goal, which must wrap.
        move_goal = pos_writes[1][1][0] | (pos_writes[1][1][1] << 8)
        self.assertEqual(move_goal, 428)

    def test_run_angle_writes_goal_speed_clamped_to_register_range(self):
        m = ST3215Motor(servo_id=7, steps_per_dps=10.0, max_dps=10000.0)
        self._patch_present_pos(m, [0, 0, 1024])
        baseline = len(m._bus._uart._tx_log)
        # 5000 dps × 10 steps/dps = 50000 steps — exceeds the 0x7FFF
        # register cap and must clamp. Use ``then="brake"`` to also
        # cover the trailing goal_speed=0 write that brake emits.
        m.run_angle(deg_per_s=5000, target_angle=90, then="brake")
        speed_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                  _REG_GOAL_SPEED)
        # Two speed writes: the cruise cap, then a final 0 (brake).
        v_cruise = speed_writes[0][1][0] | (speed_writes[0][1][1] << 8)
        self.assertEqual(v_cruise, 0x7FFF)
        self.assertEqual(speed_writes[-1][1], bytes([0, 0]))

    def test_run_angle_no_wait_writes_goal_position_and_returns(self):
        m = ST3215Motor(servo_id=8, steps_per_dps=10.0, max_dps=1000.0)
        # Two reads: anchor (pre-mode-switch hold) + the no-wait branch's
        # own present read. Both at 500 since the wheel hasn't moved yet.
        self._patch_present_pos(m, [500, 500])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=45, wait=False)
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        # Anchor (= 500) and the chunk goal (45° = 512 counts → 500+512=1012).
        self.assertEqual(len(pos_writes), 2)
        anchor_goal = pos_writes[0][1][0] | (pos_writes[0][1][1] << 8)
        self.assertEqual(anchor_goal, 500)
        chunk_goal = pos_writes[1][1][0] | (pos_writes[1][1][1] << 8)
        self.assertEqual(chunk_goal, 1012)

    def test_run_angle_zero_target_is_a_noop(self):
        m = ST3215Motor(servo_id=9)
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=0)
        # No mode switch, no goal-position write — early return before
        # the anchor read even happens.
        self.assertEqual(len(m._bus._uart._tx_log), baseline)

    def test_run_angle_silent_anchor_read_bails_before_mode_switch(self):
        # If the very first (anchor) present-pos read fails — bus
        # totally silent — run_angle returns BEFORE flipping into
        # position mode. No mode writes at all means there's nothing
        # to restore, and the wheel stays in whatever state it was in.
        m = ST3215Motor(servo_id=10, steps_per_dps=10.0, max_dps=1000.0)
        m._read_present_pos = lambda: None
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertEqual(mode_writes, [])

    def test_run_angle_then_brake_silent_after_anchor_still_restores_wheel_mode(self):
        # If the anchor read succeeds but a later read fails (bus
        # drops mid-move), the try/finally must still restore wheel
        # mode so the next ``run_speed`` works normally. Only the
        # ``then="brake"`` path explicitly restores wheel mode in
        # finally — coast leaves op_mode alone and relies on the next
        # call's ``_ensure_mode`` to do the flip.
        m = ST3215Motor(servo_id=11, steps_per_dps=10.0, max_dps=1000.0)
        reads = [500]   # anchor succeeds, then None forever.
        def fake():
            return reads.pop(0) if reads else None
        m._read_present_pos = fake
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90, then="brake")
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertEqual(mode_writes[0][1], bytes([0]))   # position
        self.assertEqual(mode_writes[-1][1], bytes([1]))  # wheel restored

    def test_run_angle_then_hold_locks_goal_position_in_position_mode(self):
        # ``then="hold"`` re-anchors goal_position to where the wheel
        # actually stopped (which may differ from the loop's target by
        # up to tol_counts) and leaves op_mode in position so the PID
        # actively resists rotation. No wheel-mode flip, no torque cut.
        m = ST3215Motor(servo_id=12, steps_per_dps=10.0, max_dps=1000.0)
        # Anchor + chunk-start + inner-arrival + post-loop self.angle()
        # + the hold path's re-read (= 5 reads). The final value 1022
        # is what hold should write back as goal_position.
        self._patch_present_pos(m, [0, 0, 1024, 1024, 1022])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90, then="hold")

        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        # Only the flip INTO position mode; no flip back to wheel.
        self.assertEqual(len(mode_writes), 1)
        self.assertEqual(mode_writes[0][1], bytes([0]))
        # Torque is NOT cut by hold.
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes, [])
        # The last goal-position write should be the re-anchored stop
        # position (1022), not the loop's target (1024).
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        final_goal = pos_writes[-1][1][0] | (pos_writes[-1][1][1] << 8)
        self.assertEqual(final_goal, 1022)

    def test_run_angle_then_invalid_raises_value_error(self):
        m = ST3215Motor(servo_id=13, steps_per_dps=10.0, max_dps=1000.0)
        with self.assertRaises(ValueError):
            m.run_angle(deg_per_s=200, target_angle=90, then="freewheel")

    def test_run_speed_after_coast_re_enables_torque(self):
        # After ``coast`` (or ``run_angle(then="coast")``) the torque
        # register is 0. The next ``run_speed`` must write torque=1
        # before its goal_speed packet — otherwise the motor stays
        # un-torqued and silently ignores the command.
        m = ST3215Motor(servo_id=14, steps_per_dps=10.0, max_dps=1000.0)
        m.coast()
        baseline = len(m._bus._uart._tx_log)
        m.run_speed(50)
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes[-1][1], bytes([1]))

    def test_brake_after_coast_re_enables_torque(self):
        m = ST3215Motor(servo_id=15)
        m.coast()
        baseline = len(m._bus._uart._tx_log)
        m.brake()
        torque_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_TORQUE)
        self.assertEqual(torque_writes[-1][1], bytes([1]))

    def test_run_speed_after_hold_restores_wheel_mode(self):
        # ``hold`` leaves the servo in position mode; the next
        # ``run_speed`` must flip back to wheel mode before writing
        # goal_speed, otherwise a negative dps value (sign bit at 0x8000)
        # is misread as the position-mode unsigned speed cap.
        m = ST3215Motor(servo_id=16, steps_per_dps=10.0, max_dps=1000.0)
        m._read_present_pos = lambda: 500
        m.hold()
        baseline = len(m._bus._uart._tx_log)
        m.run_speed(50)
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        self.assertEqual(mode_writes[-1][1], bytes([1]))   # wheel


class TestSyncServoGroup(unittest.TestCase):
    def setUp(self):
        ST3215._buses = {}

    def test_constructor_rejects_servos_on_different_buses(self):
        s1 = ST3215Motor(servo_id=1, uart_id=1, tx=17, rx=16)
        s2 = ST3215Motor(servo_id=2, uart_id=2, tx=17, rx=16)
        with self.assertRaises(ValueError):
            SyncServoGroup([s1, s2])

    def test_constructor_rejects_empty_list(self):
        with self.assertRaises(ValueError):
            SyncServoGroup([])

    def test_set_goal_speeds_emits_one_sync_write_packet(self):
        s1 = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        s2 = ST3215Motor(servo_id=2, steps_per_dps=10.0, max_dps=1000.0)
        group = SyncServoGroup([s1, s2])

        # Drain any constructor packets so we examine only the sync write.
        baseline = len(s1._bus._uart._tx_log)
        group.set_goal_speeds([50, -50])

        new_packets = s1._bus._uart._tx_log[baseline:]
        self.assertEqual(len(new_packets), 1)
        pkt = new_packets[0]

        # Header + body + checksum.
        self.assertTrue(pkt.startswith(_HEADER))
        body = pkt[2:-1]
        # Body layout: ID(0xFE) | LEN | INSTR(0x83) | ADDR | DATA_LEN |
        #              ID1 | D1_lo | D1_hi | ID2 | D2_lo | D2_hi
        self.assertEqual(body[0], 0xFE)            # broadcast
        self.assertEqual(body[2], 0x83)            # SYNC WRITE
        self.assertEqual(body[3], _REG_GOAL_SPEED) # register
        self.assertEqual(body[4], 2)               # data_len
        # Servo 1: speed = +50 dps × 10 steps/dps = 500 (no sign bit)
        self.assertEqual(body[5], 1)
        self.assertEqual(body[6], 500 & 0xFF)
        self.assertEqual(body[7], (500 >> 8) & 0xFF)
        # Servo 2: speed = -50 dps → magnitude 500 + sign bit
        v2 = 500 | 0x8000
        self.assertEqual(body[8], 2)
        self.assertEqual(body[9],  v2 & 0xFF)
        self.assertEqual(body[10], (v2 >> 8) & 0xFF)

    def test_set_goal_speeds_respects_per_servo_invert(self):
        s1 = ST3215Motor(servo_id=10, steps_per_dps=10.0, max_dps=1000.0)
        s2 = ST3215Motor(servo_id=11, steps_per_dps=10.0, max_dps=1000.0,
                         invert=True)
        group = SyncServoGroup([s1, s2])
        baseline = len(s1._bus._uart._tx_log)
        group.set_goal_speeds([50, 50])   # both commanded forward
        body = s1._bus._uart._tx_log[baseline:][0][2:-1]
        # s1 (no invert) → sign bit clear in high byte
        self.assertEqual(body[7] & 0x80, 0)
        # s2 (invert=True) → sign bit set in high byte
        self.assertEqual(body[10] & 0x80, 0x80)

    def test_set_goal_speeds_packet_length_field_matches_payload(self):
        servos = [ST3215Motor(servo_id=i + 1) for i in range(4)]
        group = SyncServoGroup(servos)
        baseline = len(servos[0]._bus._uart._tx_log)
        group.set_goal_speeds([0, 0, 0, 0])
        pkt = servos[0]._bus._uart._tx_log[baseline:][0]
        body = pkt[2:-1]
        # LEN = 4 + N × (1 + data_len) = 4 + 4 × 3 = 16
        self.assertEqual(body[1], 16)
        # Total body = ID + LEN + INSTR + ADDR + DATA_LEN + 4×(ID+2bytes) = 5 + 12 = 17
        self.assertEqual(len(body), 17)

    def test_set_goal_speeds_count_must_match_servo_count(self):
        s1 = ST3215Motor(servo_id=1)
        s2 = ST3215Motor(servo_id=2)
        group = SyncServoGroup([s1, s2])
        with self.assertRaises(ValueError):
            group.set_goal_speeds([100])      # too few
        with self.assertRaises(ValueError):
            group.set_goal_speeds([1, 2, 3])  # too many

    def test_set_goal_speeds_rejects_position_mode_servos(self):
        # Position-mode ST3215 doesn't have ``_encode_goal_speed`` —
        # SyncServoGroup should refuse rather than write nonsense.
        s_pos   = ST3215(servo_id=1)
        s_wheel = ST3215Motor(servo_id=2)
        group = SyncServoGroup([s_pos, s_wheel])
        with self.assertRaises(TypeError):
            group.set_goal_speeds([100, 100])

    def test_sync_write_does_not_read_response(self):
        # Broadcast writes: no per-servo reply. Ensure we don't block
        # on the RX path waiting for one.
        s1 = ST3215Motor(servo_id=1)
        s2 = ST3215Motor(servo_id=2)
        group = SyncServoGroup([s1, s2])
        # Track _rx calls — sync_write must not call into them.
        original_rx = s1._bus._rx
        rx_calls = [0]

        def counting_rx(*args, **kwargs):
            rx_calls[0] += 1
            return original_rx(*args, **kwargs)
        s1._bus._rx = counting_rx
        before = rx_calls[0]
        group.set_goal_speeds([100, 100])
        self.assertEqual(rx_calls[0], before)


if __name__ == "__main__":
    # Keep the linter quiet about the unused module import used for reloading.
    assert st3215_mod is not None
    unittest.main()
