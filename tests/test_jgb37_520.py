# SPDX-License-Identifier: MIT
"""Tests for the JGB37-520 closed-loop motor driver."""

import tests._fakes  # noqa: F401

import time
import unittest

from machine import Timer  # type: ignore[import-not-found]

from openbricks._native import motor_process
from openbricks.drivers.jgb37_520 import JGB37Motor


def _make_motor(**overrides):
    kwargs = dict(
        in1=1, in2=2, pwm=3,
        encoder_a=4, encoder_b=5,
        counts_per_output_rev=1320,
    )
    kwargs.update(overrides)
    return JGB37Motor(**kwargs)


class TestJGB37Motor(unittest.TestCase):
    def setUp(self):
        motor_process.reset()
        Timer.reset_for_test()

    def test_run_passes_through_to_h_bridge(self):
        m = _make_motor()
        m.run(50)
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertGreater(m._driver._pwm.duty(), 0)

    def test_brake_and_coast_passthrough(self):
        m = _make_motor()
        m.brake()
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 1)
        m.coast()
        self.assertEqual(m._driver._pwm.duty(), 0)

    def test_angle_from_encoder_count(self):
        m = _make_motor()
        # 1320 counts per output rev -> 660 counts = 180 deg.
        m._enc._count = 660
        self.assertAlmostEqual(m.angle(), 180.0, places=6)

    def test_reset_angle_sets_encoder_count(self):
        m = _make_motor()
        m.reset_angle(90)
        # 90 deg * 1320 / 360 = 330 counts.
        self.assertEqual(m._enc._count, 330)
        m.reset_angle(0)
        self.assertEqual(m._enc._count, 0)

    def test_run_speed_sets_target_and_attaches(self):
        m = _make_motor()
        m.run_speed(300)
        self.assertEqual(m._target_dps, 300.0)
        self.assertTrue(m._active)

    def test_control_step_positive_clamps_to_full_forward(self):
        m = _make_motor()
        m.run_speed(300)
        m._control_step()
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 0)
        self.assertEqual(m._driver._pwm.duty(), 1023)

    def test_control_step_negative_clamps_to_full_reverse(self):
        m = _make_motor()
        m.run_speed(-300)
        m._control_step()
        self.assertEqual(m._driver._in1.value(), 0)
        self.assertEqual(m._driver._in2.value(), 1)
        self.assertEqual(m._driver._pwm.duty(), 1023)

    def test_run_speed_registers_control_step_with_scheduler(self):
        m = _make_motor()
        m.run_speed(100)
        self.assertIn(m._control_step, motor_process._callbacks)

    def test_run_speed_is_idempotent_for_registration(self):
        m = _make_motor()
        m.run_speed(100)
        m.run_speed(150)
        self.assertEqual(
            motor_process._callbacks.count(m._control_step), 1,
        )
        self.assertEqual(m._target_dps, 150.0)

    def test_brake_detaches_and_brakes(self):
        m = _make_motor()
        m.run_speed(300)
        m._control_step()
        self.assertEqual(m._driver._pwm.duty(), 1023)

        m.brake()
        self.assertNotIn(m._control_step, motor_process._callbacks)
        self.assertFalse(m._active)
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._in2.value(), 1)

    def test_coast_detaches_and_coasts(self):
        m = _make_motor()
        m.run_speed(300)
        m.coast()
        self.assertNotIn(m._control_step, motor_process._callbacks)
        self.assertFalse(m._active)
        self.assertEqual(m._driver._pwm.duty(), 0)

    def test_run_open_loop_detaches(self):
        m = _make_motor()
        m.run_speed(300)
        self.assertTrue(m._active)
        m.run(50)
        self.assertFalse(m._active)
        self.assertNotIn(m._control_step, motor_process._callbacks)

    def test_scheduler_drives_bridge_during_sleep(self):
        m = _make_motor()
        m.run_speed(300)
        # First attach starts the scheduler; one 10 ms tick period elapses.
        time.sleep_ms(10)
        self.assertEqual(m._driver._in1.value(), 1)
        self.assertEqual(m._driver._pwm.duty(), 1023)
        m.brake()

    def test_run_angle_reaches_target_when_encoder_is_fed(self):
        m = _make_motor()

        # Simulate the encoder advancing every scheduler tick.
        def fake_rotation():
            m._enc._count += 13

        motor_process.register(fake_rotation)
        m.run_angle(300, 90)  # block until we reach +90 deg.
        self.assertGreaterEqual(m.angle(), 90)
        self.assertFalse(m._active)  # run_angle braked, which detaches


if __name__ == "__main__":
    unittest.main()
