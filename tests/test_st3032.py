# SPDX-License-Identifier: MIT
"""Tests for the ST-3032 marker subclasses.

ST-3032 shares the SCS protocol verbatim with the ST-3215, so these
tests don't re-cover packet formatting (that's done in
``test_st3215.py``). Instead they pin the contract that:

* ``ST3032`` and ``ST3032Motor`` ARE-A ``ST3215`` / ``ST3215Motor`` —
  any ``DriveBase`` / ``Servo`` consumer that accepts an ST-3215 must
  also accept an ST-3032 without isinstance gymnastics.
* The bus registry is shared with ``ST3215``, so an ST-3032 and an
  ST-3215 daisy-chained on the same UART reuse one ``_SCServoBus``.
* The classes emit byte-identical packets to their ST-3215
  counterparts — no silent overrides have crept in.
"""

import tests._fakes  # noqa: F401

import unittest

from openbricks.drivers.st3215 import ST3215, ST3215Motor, SyncServoGroup
from openbricks.drivers.st3032 import ST3032, ST3032Motor
from openbricks.interfaces import Motor, Servo


class TestST3032(unittest.TestCase):
    def setUp(self):
        ST3215._buses = {}

    def test_is_a_st3215_and_a_servo(self):
        self.assertTrue(issubclass(ST3032, ST3215))
        self.assertTrue(issubclass(ST3032, Servo))

    def test_shares_bus_registry_with_st3215(self):
        # Same UART params → same bus instance, regardless of model.
        s_3215 = ST3215(servo_id=1, uart_id=1, tx=17, rx=16)
        s_3032 = ST3032(servo_id=2, uart_id=1, tx=17, rx=16)
        self.assertIs(s_3215._bus, s_3032._bus)

    def test_emits_identical_move_to_packet_as_st3215(self):
        a = ST3215(servo_id=5)
        b = ST3032(servo_id=5)
        a.move_to(123, wait=False)
        b.move_to(123, wait=False)
        self.assertEqual(a._bus._uart._tx_log, b._bus._uart._tx_log)


class TestST3032Motor(unittest.TestCase):
    def setUp(self):
        ST3215._buses = {}

    def test_is_a_st3215_motor_and_a_motor(self):
        self.assertTrue(issubclass(ST3032Motor, ST3215Motor))
        self.assertTrue(issubclass(ST3032Motor, Motor))

    def test_shares_bus_registry_with_st3215(self):
        s_motor_3215 = ST3215Motor(servo_id=1, uart_id=1, tx=17, rx=16)
        s_motor_3032 = ST3032Motor(servo_id=2, uart_id=1, tx=17, rx=16)
        self.assertIs(s_motor_3215._bus, s_motor_3032._bus)

    def test_constructor_emits_same_mode_and_torque_packets_as_st3215(self):
        a = ST3215Motor(servo_id=7)
        b = ST3032Motor(servo_id=7)
        self.assertEqual(a._bus._uart._tx_log, b._bus._uart._tx_log)

    def test_run_speed_emits_identical_packet_as_st3215(self):
        a = ST3215Motor(servo_id=3)
        b = ST3032Motor(servo_id=3)
        baseline_a = len(a._bus._uart._tx_log)
        baseline_b = len(b._bus._uart._tx_log)
        a.run_speed(120)
        b.run_speed(120)
        self.assertEqual(
            a._bus._uart._tx_log[baseline_a:],
            b._bus._uart._tx_log[baseline_b:],
        )


class TestMixedFamilySyncGroup(unittest.TestCase):
    """An ST-3032 and an ST-3215 on the same daisy chain should drive
    fine from one ``SyncServoGroup`` — the SCS protocol doesn't
    distinguish models."""

    def setUp(self):
        ST3215._buses = {}

    def test_mixed_st3215_and_st3032_motor_share_one_sync_packet(self):
        s_3215 = ST3215Motor(servo_id=1, steps_per_dps=10.0, max_dps=1000.0)
        s_3032 = ST3032Motor(servo_id=2, steps_per_dps=10.0, max_dps=1000.0)
        group = SyncServoGroup([s_3215, s_3032])
        baseline = len(s_3215._bus._uart._tx_log)
        group.set_goal_speeds([50, -50])
        new_packets = s_3215._bus._uart._tx_log[baseline:]
        # One SYNC WRITE for both, regardless of model mix.
        self.assertEqual(len(new_packets), 1)
        body = new_packets[0][2:-1]
        self.assertEqual(body[0], 0xFE)   # broadcast
        self.assertEqual(body[2], 0x83)   # SYNC WRITE
