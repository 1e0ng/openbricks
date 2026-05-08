# SPDX-License-Identifier: MIT
"""Tests for the ST-3215 serial bus servo driver."""

import tests._fakes  # noqa: F401

import unittest

from openbricks.drivers import st3215 as st3215_mod
from openbricks.drivers.st3215 import ST3215, ST3215Wheel


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


class TestST3215Wheel(unittest.TestCase):
    def setUp(self):
        ST3215._buses = {}

    def test_constructor_switches_servo_into_wheel_mode(self):
        m = ST3215Wheel(servo_id=1)
        mode_writes = _writes_to(m._bus._uart._tx_log, _REG_OP_MODE)
        self.assertEqual(mode_writes, [(1, bytes([1]))])   # 1 = wheel

    def test_constructor_enables_torque(self):
        m = ST3215Wheel(servo_id=2)
        torque_writes = _writes_to(m._bus._uart._tx_log, _REG_TORQUE)
        self.assertEqual(torque_writes, [(2, bytes([1]))])

    def test_run_speed_writes_signed_magnitude_to_goal_speed(self):
        m = ST3215Wheel(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        m.run_speed(50)   # → magnitude = 500, sign bit clear
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        self.assertEqual(speed_writes[-1], (3, bytes([500 & 0xFF, 500 >> 8])))

    def test_run_speed_negative_sets_high_bit(self):
        m = ST3215Wheel(servo_id=3, steps_per_dps=10.0, max_dps=1000.0)
        m.run_speed(-50)
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        # magnitude 500, plus the direction bit at bit 15 of the 16-bit value
        v = 500 | 0x8000
        self.assertEqual(speed_writes[-1], (3, bytes([v & 0xFF, (v >> 8) & 0xFF])))

    def test_run_speed_clamps_to_max_dps(self):
        m = ST3215Wheel(servo_id=3, steps_per_dps=10.0, max_dps=100.0)
        m.run_speed(99999)
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        # Clamped to 100 dps × 10 steps/dps = 1000
        self.assertEqual(speed_writes[-1], (3, bytes([1000 & 0xFF, 1000 >> 8])))

    def test_invert_flips_run_speed_direction(self):
        # Both servos default to uart_id=1 → they share a _SCServoBus,
        # so the tx_log holds packets from both. Filter by servo_id to
        # isolate each motor's last command.
        m_plain = ST3215Wheel(servo_id=4, steps_per_dps=10.0, max_dps=1000.0)
        m_inv   = ST3215Wheel(servo_id=5, steps_per_dps=10.0, max_dps=1000.0,
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
        m = ST3215Wheel(servo_id=6)
        m.brake()
        speed_writes = _writes_to(m._bus._uart._tx_log, _REG_GOAL_SPEED)
        self.assertEqual(speed_writes[-1], (6, bytes([0, 0])))

    def test_coast_disables_torque(self):
        m = ST3215Wheel(servo_id=7)
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
        m = ST3215Wheel(servo_id=8)

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
        m = ST3215Wheel(servo_id=9)

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


if __name__ == "__main__":
    # Keep the linter quiet about the unused module import used for reloading.
    assert st3215_mod is not None
    unittest.main()
