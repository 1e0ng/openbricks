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

    def test_run_angle_switches_into_position_mode_then_back(self):
        m = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        # Start at raw 0; servo arrives at the goal on the second read.
        target_counts = int(round(90 * 4096 / 360))   # = 1024
        self._patch_present_pos(m, [0, target_counts])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)

        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        # First mode write into the move = position (0); last = wheel (1).
        self.assertEqual(mode_writes[0][1], bytes([0]))
        self.assertEqual(mode_writes[-1][1], bytes([1]))

    def test_run_angle_writes_goal_position_at_present_plus_delta(self):
        m = ST3215Motor(servo_id=2, steps_per_dps=10.0, max_dps=1000.0)
        # Present pos starts at 100; +90° = +1024 counts → goal = 1124.
        self._patch_present_pos(m, [100, 100 + 1024])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)

        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        self.assertEqual(len(pos_writes), 1)
        sid, data = pos_writes[0]
        goal = data[0] | (data[1] << 8)
        self.assertEqual(goal, 1124)

    def test_run_angle_chunks_moves_larger_than_180_degrees(self):
        # 270° forward = 3072 counts. Can't be one chunk (would be
        # routed as 90° backward by the servo's shortest-path PID).
        # Implementation caps each chunk at 2000 counts (≈175.8°),
        # so we expect ⌈3072 / 2000⌉ = 2 chunks.
        m = ST3215Motor(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        # Each chunk consumes 3 reads: chunk-start position, in-loop
        # arrival, end-of-chunk self.angle(). For two chunks that's
        # six reads. Sequence below also feeds the final angle() call
        # via the "last value sticks" rule in _patch_present_pos.
        self._patch_present_pos(m, [
            0,    # iter1: chunk-start
            2000, # iter1: inner-loop arrival
            2000, # iter1: self.angle() update
            2000, # iter2: chunk-start
            3072, # iter2: inner-loop arrival (sticks for final angle())
        ])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=270)

        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        self.assertEqual(len(pos_writes), 2)
        # First chunk goal: present(0) + 2000 = 2000.
        g0 = pos_writes[0][1][0] | (pos_writes[0][1][1] << 8)
        self.assertEqual(g0, 2000)
        # Second chunk goal: present(2000) + 1072 = 3072.
        g1 = pos_writes[1][1][0] | (pos_writes[1][1][1] << 8)
        self.assertEqual(g1, 3072)

    def test_run_angle_negative_target_writes_correct_goal(self):
        m = ST3215Motor(servo_id=4, steps_per_dps=10.0, max_dps=1000.0)
        # Start at 2000; -90° = -1024 → goal = 976.
        self._patch_present_pos(m, [2000, 976])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=-90)
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        goal = pos_writes[0][1][0] | (pos_writes[0][1][1] << 8)
        self.assertEqual(goal, 976)

    def test_run_angle_invert_flips_direction(self):
        m = ST3215Motor(servo_id=5, steps_per_dps=10.0, max_dps=1000.0,
                        invert=True)
        # +90° asked, but invert means we should command -1024 counts.
        # Present 1024 + (-1024) = 0.
        self._patch_present_pos(m, [1024, 0])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        goal = pos_writes[0][1][0] | (pos_writes[0][1][1] << 8)
        self.assertEqual(goal, 0)

    def test_run_angle_wraps_goal_position_modulo_one_revolution(self):
        # Present 3500 + 1024 = 4524 → must wrap to 4524 % 4096 = 428.
        m = ST3215Motor(servo_id=6, steps_per_dps=10.0, max_dps=1000.0)
        self._patch_present_pos(m, [3500, 428])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        goal = pos_writes[0][1][0] | (pos_writes[0][1][1] << 8)
        self.assertEqual(goal, 428)

    def test_run_angle_writes_goal_speed_clamped_to_register_range(self):
        m = ST3215Motor(servo_id=7, steps_per_dps=10.0, max_dps=10000.0)
        self._patch_present_pos(m, [0, 1024])
        baseline = len(m._bus._uart._tx_log)
        # 5000 dps × 10 steps/dps = 50000 steps — exceeds the 0x7FFF
        # register cap and must clamp.
        m.run_angle(deg_per_s=5000, target_angle=90)
        speed_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                  _REG_GOAL_SPEED)
        # Two speed writes happen: the cruise cap, then a final 0
        # (brake) when wheel mode is restored.
        v_cruise = speed_writes[0][1][0] | (speed_writes[0][1][1] << 8)
        self.assertEqual(v_cruise, 0x7FFF)
        self.assertEqual(speed_writes[-1][1], bytes([0, 0]))

    def test_run_angle_no_wait_writes_one_goal_position_and_returns(self):
        m = ST3215Motor(servo_id=8, steps_per_dps=10.0, max_dps=1000.0)
        # No completion poll happens — only one present-pos read.
        self._patch_present_pos(m, [500])
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=45, wait=False)
        pos_writes = _writes_to(m._bus._uart._tx_log[baseline:],
                                _REG_GOAL_POSITION)
        self.assertEqual(len(pos_writes), 1)
        # 45° = 512 counts, present 500 → goal 1012.
        goal = pos_writes[0][1][0] | (pos_writes[0][1][1] << 8)
        self.assertEqual(goal, 1012)

    def test_run_angle_zero_target_is_a_noop(self):
        m = ST3215Motor(servo_id=9)
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=0)
        # No mode switch, no goal-position write — early return.
        self.assertEqual(len(m._bus._uart._tx_log), baseline)

    def test_run_angle_restores_wheel_mode_even_when_bus_goes_silent(self):
        m = ST3215Motor(servo_id=10, steps_per_dps=10.0, max_dps=1000.0)
        # _read_present_pos returns None on the very first read inside
        # the loop → run_angle should bail BUT still restore wheel mode.
        m._read_present_pos = lambda: None
        baseline = len(m._bus._uart._tx_log)
        m.run_angle(deg_per_s=200, target_angle=90)
        mode_writes = _writes_to(m._bus._uart._tx_log[baseline:], _REG_OP_MODE)
        # First write switches into position mode; the finally block
        # writes wheel mode back even though the move never completed.
        self.assertEqual(mode_writes[0][1], bytes([0]))
        self.assertEqual(mode_writes[-1][1], bytes([1]))


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
