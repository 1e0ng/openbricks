# SPDX-License-Identifier: MIT
"""Tests for the ST-3215 serial bus servo driver."""

import tests._fakes  # noqa: F401

import unittest

from openbricks.drivers import st3215 as st3215_mod
from openbricks.drivers.st3215 import ST3215


_REG_GOAL_POSITION = 0x2A
_REG_GOAL_SPEED    = 0x2E
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


if __name__ == "__main__":
    # Keep the linter quiet about the unused module import used for reloading.
    assert st3215_mod is not None
    unittest.main()
