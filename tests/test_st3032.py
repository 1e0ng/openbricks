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
  counterparts — no silent overrides have crept in. (The ONE
  deliberate override is ``ST3032Motor``'s ``max_dps=888`` default,
  pinned in ``TestST3032MaxDpsDefault``; packet-identity tests stay
  below both clamps.)
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
        s_3215 = ST3215(servo_id=1, uart_id=1, tx=14, rx=6)
        s_3032 = ST3032(servo_id=2, uart_id=1, tx=14, rx=6)
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
        s_motor_3215 = ST3215Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        s_motor_3032 = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6)
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


class TestST3032MaxDpsDefault(unittest.TestCase):
    """The ONE deliberate ST-3032 specialisation: ``max_dps`` defaults
    to the datasheet no-load speed (888 °/s). The inherited ST-3215
    default (600) silently clamped every wire command ~290 °/s below
    the servo's spec — bench symptom: "top speed is capped"."""

    def setUp(self):
        ST3215._buses = {}

    def test_default_max_dps_is_datasheet_no_load_speed(self):
        from openbricks.drivers.st3032 import ST3032_NO_LOAD_DPS
        m = ST3032Motor(servo_id=1)
        self.assertEqual(m._max_dps, 888.0)
        self.assertEqual(ST3032_NO_LOAD_DPS, 888.0)

    def test_st3215_default_unchanged(self):
        # The parent keeps its own protective default — only the
        # ST-3032 subclass knows its servo is faster.
        m = ST3215Motor(servo_id=2)
        self.assertEqual(m._max_dps, 600.0)

    def test_speed_between_600_and_888_reaches_the_wire_unclamped(self):
        # The bench bug: 800 dps commanded, 600 written. The register
        # value must encode the REQUESTED speed.
        m = ST3032Motor(servo_id=3)
        expected = int(800 * m._steps_per_dps)
        self.assertEqual(m._encode_goal_speed(800) & 0x7FFF, expected)

    def test_speed_above_no_load_clamps_at_888(self):
        m = ST3032Motor(servo_id=4)
        expected = int(888.0 * m._steps_per_dps)
        self.assertEqual(m._encode_goal_speed(2000) & 0x7FFF, expected)

    def test_explicit_max_dps_still_wins(self):
        m = ST3032Motor(servo_id=5, max_dps=120.0)
        expected = int(120.0 * m._steps_per_dps)
        self.assertEqual(m._encode_goal_speed(500) & 0x7FFF, expected)

    def test_all_other_constructor_defaults_match_st3215(self):
        # The subclass re-states the parent signature to change one
        # default; the rest must not drift.
        a = ST3215Motor(servo_id=6, uart_id=1, tx=14, rx=6)
        b = ST3032Motor(servo_id=7, uart_id=1, tx=14, rx=6)
        self.assertIs(a._bus, b._bus)          # same uart/tx/rx/baud
        self.assertEqual(a._steps_per_dps, b._steps_per_dps)
        self.assertEqual(a._invert, b._invert)


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
        group.set_goal_speeds([0, 0])   # pre-warm: first call enables torque
        baseline = len(s_3215._bus._uart._tx_log)
        group.set_goal_speeds([50, -50])
        new_packets = s_3215._bus._uart._tx_log[baseline:]
        # One SYNC WRITE for both, regardless of model mix.
        self.assertEqual(len(new_packets), 1)
        body = new_packets[0][2:-1]
        self.assertEqual(body[0], 0xFE)   # broadcast
        self.assertEqual(body[2], 0x83)   # SYNC WRITE
