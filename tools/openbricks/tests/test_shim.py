# SPDX-License-Identifier: MIT
"""Tests for the ``openbricks_sim.shim`` driver-shim layer.

Two surfaces:

  * ``install`` / ``uninstall`` lifecycle — ``sys.modules`` swaps and
    ``time.*`` patches are reversible.
  * The shim ``Servo`` / ``DriveBase`` classes accept the firmware's
    constructor signatures and route them onto a SimRuntime + chassis.

The integration test at the bottom drives the chassis via *real
firmware code* — by importing ``openbricks.drivers.jgb37_520.JGB37Motor``
and ``openbricks.robotics.drivebase.DriveBase`` after installing the
shim. That's the full "run firmware code unchanged in the sim"
scenario.
"""

import sys
import time
import unittest

from openbricks_sim.robot import SimRobot
from openbricks_sim import shim


class _ShimTestBase(unittest.TestCase):
    """Make sure each test starts from a clean state and the shim
    is uninstalled afterwards even when an assertion fails."""

    def setUp(self):
        if shim.is_installed():
            shim.uninstall()
        self.robot = SimRobot()
        shim.install(self.robot.runtime)

    def tearDown(self):
        shim.uninstall()

    def _serial_db(self, imu=None):
        """The user's actual robot shape: two ST-3032 serial servos
        through the one-class DriveBase; adoption hands them to the
        serial engine over the emulated st_bus."""
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics.drivebase import DriveBase

        left  = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        right = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                            invert=True)
        db = DriveBase(left, right,
                       wheel_diameter_mm=65, axle_track_mm=120,
                       imu=imu)
        return db, left, right


class InstallLifecycleTests(unittest.TestCase):

    def test_install_and_uninstall_round_trip_sys_modules(self):
        # Save baseline state.
        prev_machine = sys.modules.get("machine")
        prev_native  = sys.modules.get("_openbricks_native")

        robot = SimRobot()
        shim.install(robot.runtime)
        try:
            # Both fakes are now installed.
            self.assertIn("machine", sys.modules)
            self.assertIn("_openbricks_native", sys.modules)
            # And different from whatever was there before.
            import machine
            self.assertIs(machine, sys.modules["machine"])
            self.assertNotEqual(machine, prev_machine)
        finally:
            shim.uninstall()

        # Uninstall restores prior entries (or removes if absent).
        self.assertEqual(sys.modules.get("machine"), prev_machine)
        self.assertEqual(sys.modules.get("_openbricks_native"), prev_native)

    def test_install_twice_raises(self):
        robot = SimRobot()
        shim.install(robot.runtime)
        try:
            with self.assertRaises(RuntimeError):
                shim.install(robot.runtime)
        finally:
            shim.uninstall()

    def test_uninstall_when_not_installed_is_noop(self):
        # Should not raise even when nothing's installed.
        shim.uninstall()
        self.assertFalse(shim.is_installed())

    def test_install_patches_time_sleep_ms_to_advance_sim(self):
        robot = SimRobot()
        shim.install(robot.runtime)
        try:
            self.assertEqual(robot.runtime.now_ms, 0)
            time.sleep_ms(50)   # patched: 50 ms of sim time
            self.assertEqual(robot.runtime.now_ms, 50)
        finally:
            shim.uninstall()
        # After uninstall, time.sleep_ms is restored to its previous
        # state — typically nonexistent on CPython.
        self.assertFalse(hasattr(time, "sleep_ms"),
                          "uninstall should remove the sleep_ms patch")


class MotorSlotAllocationTests(_ShimTestBase):

    def test_first_servo_binds_left_second_binds_right(self):
        from _openbricks_native import Servo
        s_left  = Servo(in1=12, in2=14, pwm=27, encoder=None,
                        counts_per_rev=1320, kp=0.3)
        s_right = Servo(in1=13, in2=15, pwm=26, encoder=None,
                        counts_per_rev=1320, kp=0.3)
        # The shim allocates chassis_motor_l → first, _r → second.
        self.assertNotEqual(s_left._adapter._actuator_id,
                             s_right._adapter._actuator_id)

    def test_third_servo_construction_raises(self):
        from _openbricks_native import Servo
        Servo(in1=12, in2=14, pwm=27, encoder=None)
        Servo(in1=13, in2=15, pwm=26, encoder=None)
        with self.assertRaises(RuntimeError):
            Servo(in1=99, in2=98, pwm=97, encoder=None)


class ShimServoBehaviourTests(_ShimTestBase):

    def test_run_speed_drives_actuator(self):
        from _openbricks_native import Servo
        s = Servo(in1=12, in2=14, pwm=27, encoder=None)
        s.run_speed(180.0)
        # Step a few times so the controller writes a non-zero ctrl.
        for _ in range(10):
            self.robot.runtime.step()
        adapter = s._adapter
        ctrl = float(self.robot.data.ctrl[adapter._actuator_id])
        self.assertGreater(abs(ctrl), 0.0,
                            "run_speed should drive the actuator")

    def test_run_target_completes_via_sleep_busy_wait(self):
        # Mirrors the firmware's JGB37Motor.run_angle wait pattern.
        from _openbricks_native import Servo
        s = Servo(in1=12, in2=14, pwm=27, encoder=None,
                  counts_per_rev=360, kp=0.0)   # open-loop trajectory
        s.run_target(delta_deg=90.0, cruise_dps=180.0, accel=720.0)
        # Drive the wait loop the same way openbricks's wrappers do.
        deadline = 0
        while not s.is_done() and deadline < 5000:
            time.sleep_ms(10)   # patched: advances sim
            deadline += 10
        self.assertTrue(s.is_done())

    def test_brake_detaches(self):
        from _openbricks_native import Servo
        s = Servo(in1=12, in2=14, pwm=27, encoder=None)
        s.run_speed(100.0)
        self.assertTrue(s._adapter._attached)
        s.brake()
        self.assertFalse(s._adapter._attached)


class ShimDriveBaseTests(_ShimTestBase):

    def test_construct_with_shim_servos(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        self.assertTrue(db.is_done())   # idle at construction

    def test_construct_with_non_shim_servo_raises(self):
        from _openbricks_native import DriveBase
        with self.assertRaises(TypeError):
            DriveBase(left="not a servo", right=None,
                       wheel_diameter_mm=60, axle_track_mm=150)

    def test_set_accel_delegates_to_native(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        # The ValueError lives in the C extension — its propagation
        # proves the call traverses ShimDriveBase → SimDriveBase →
        # _native rather than dying in a missing passthrough.
        with self.assertRaises(ValueError):
            db.set_accel(-1.0)
        db.set_accel(90.0)   # valid value accepted

    def test_straight_via_busy_wait(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        db.straight(50.0, 80.0)
        deadline = 0
        while not db.is_done() and deadline < 5000:
            time.sleep_ms(10)
            deadline += 10
        self.assertTrue(db.is_done())

    def test_construction_resizes_chassis_wheels_to_match_dims(self):
        # The user's robot.py is the same script the firmware runs;
        # ``DriveBase(wheel_diameter_mm=80, axle_track_mm=200)`` is
        # the single source of truth for chassis dims. Pin: at
        # ShimDriveBase construction time, the sim model's wheel geom
        # gets resized to the user's wheel_diameter_mm and the wheel
        # bodies are repositioned for axle_track_mm. Without this,
        # encoders rotate a default-size wheel while the user's
        # odometry math thinks they're rotating a different-size wheel.
        import mujoco
        from _openbricks_native import Servo, DriveBase

        m = self.robot.model
        wl_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis_wheel_l")
        wr_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis_wheel_r")
        # Find each wheel body's cylinder geom (one per body).
        def _wheel_geom(bid):
            for gid in range(m.ngeom):
                if int(m.geom_bodyid[gid]) == bid:
                    return gid
            raise AssertionError("no geom found for body %d" % bid)
        wl_gid = _wheel_geom(wl_id)
        wr_gid = _wheel_geom(wr_id)

        # Construct DriveBase with non-default dims.
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        DriveBase(left=l, right=r,
                  wheel_diameter_mm=80.0,    # default is 60 mm
                  axle_track_mm=200.0)       # default is 150 mm

        # Wheel cylinder radius in metres = 80 / 2000 = 0.040.
        self.assertAlmostEqual(float(m.geom_size[wl_gid, 0]), 0.040, delta=1e-6)
        self.assertAlmostEqual(float(m.geom_size[wr_gid, 0]), 0.040, delta=1e-6)
        # Axle Y on each side = ±200 / 2000 = ±0.100 m.
        self.assertAlmostEqual(float(m.body_pos[wl_id, 1]), +0.100, delta=1e-6)
        self.assertAlmostEqual(float(m.body_pos[wr_id, 1]), -0.100, delta=1e-6)

    def test_use_gyro_without_imu_raises(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        with self.assertRaises(RuntimeError):
            db.use_gyro(True)

    def test_use_gyro_with_imu_installs_imu_tick(self):
        from _openbricks_native import Servo, DriveBase, BNO055
        imu = BNO055(i2c=None, address=0x28)
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0,
                        imu=imu)
        # Toggling on captures the heading offset and installs the
        # imu tick. Subsequent steps must not crash.
        db.use_gyro(True)
        for _ in range(10):
            self.robot.runtime.step()
        # And toggling off restores the encoder-differential path
        # without leaving the imu tick behind.
        db.use_gyro(False)
        for _ in range(10):
            self.robot.runtime.step()

    def test_use_gyro_false_is_always_allowed(self):
        from _openbricks_native import Servo, DriveBase
        l = Servo(in1=12, in2=14, pwm=27, encoder=None)
        r = Servo(in1=13, in2=15, pwm=26, encoder=None)
        db = DriveBase(left=l, right=r,
                        wheel_diameter_mm=60.0, axle_track_mm=150.0)
        # Should NOT raise — turning gyro feedback off is the default
        # state and never depends on an IMU.
        db.use_gyro(False)


class MachineFakeTests(_ShimTestBase):
    """Verify the ``machine`` fake covers the interfaces openbricks
    drivers actually instantiate."""

    def test_pin_pwm_construct_no_args_no_kwargs(self):
        import machine
        machine.Pin(0)
        machine.PWM(machine.Pin(1), freq=20_000, duty=0)
        machine.I2C(0, sda=21, scl=22, freq=400_000)

    def test_pin_value_returns_int(self):
        import machine
        p = machine.Pin(0, machine.Pin.OUT, value=0)
        self.assertEqual(p.value(), 0)
        p.value(1)   # write — accepted, return ignored
        # Reads return 0 by convention (no-op fake).


class ShimSerialMotorTests(_ShimTestBase):
    """ST3215Motor / ST3032Motor resolve to shim classes and answer
    the Motor API from MuJoCo. These are the classes the serial-bus
    (fallback) DriveBase path drives."""

    def test_st3032motor_resolves_to_shim_class(self):
        from openbricks.drivers.st3032 import ST3032Motor
        self.assertIs(ST3032Motor, shim.ShimST3032Motor)

    def test_st3215motor_resolves_to_shim_class(self):
        from openbricks.drivers.st3215 import ST3215Motor
        self.assertIs(ST3215Motor, shim.ShimST3215Motor)

    def test_run_speed_advances_angle(self):
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        start = m.angle()
        m.run_speed(300)
        time.sleep_ms(500)
        self.assertGreater(m.angle(), start + 10)
        m.coast()

    def test_invert_is_ignored_as_wiring_concern(self):
        # The sim chassis defines both wheel hinges on the same axis
        # (+speed = forward on both sides), so the mirrored-mounting
        # compensation must not flip the sim wheel.
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)
        start = m.angle()
        m.run_speed(300)
        time.sleep_ms(500)
        self.assertGreater(m.angle(), start + 10)
        m.coast()

    def test_run_speed_clamps_to_max_dps(self):
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6, max_dps=600)
        m.run_speed(5000)
        self.assertEqual(m._target_dps, 600.0)
        m.coast()

    def test_st3032_default_max_dps_matches_firmware_not_st3215(self):
        # Regression: ShimST3032Motor is a marker subclass of
        # ShimST3215Motor and used to inherit its 600 dps default
        # verbatim. The real firmware ST3032Motor raises its default
        # to 888 (the servo's actual no-load speed; see
        # ST3032_NO_LOAD_DPS in openbricks/drivers/st3032.py) because
        # 600 silently capped the servo below its own spec. A
        # default-constructed sim motor must clamp at the same 888,
        # not the ST-3215's 600, or a script tuned against real
        # hardware would quietly under-perform in the sim.
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.drivers.st3215 import ST3215Motor
        st3032 = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        st3032.run_speed(5000)
        self.assertEqual(st3032._target_dps, 888.0)
        st3032.coast()

        st3215 = ST3215Motor(servo_id=2, uart_id=1, tx=14, rx=6)
        st3215.run_speed(5000)
        self.assertEqual(st3215._target_dps, 600.0)   # unchanged
        st3215.coast()

    def test_run_angle_blocking_reaches_target(self):
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        m.reset_angle(0)
        m.run_angle(300, 180)
        self.assertTrue(m.done())
        self.assertAlmostEqual(m.angle(), 180.0, delta=15.0)

    def test_run_angle_wait_false_completes_via_done(self):
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        m.reset_angle(0)
        m.run_angle(300, 90, wait=False)
        self.assertFalse(m.done())
        deadline = 0
        while not m.done() and deadline < 5000:
            time.sleep_ms(10)
            deadline += 10
        self.assertTrue(m.done())
        self.assertAlmostEqual(m.angle(), 90.0, delta=15.0)

    def test_ping_reports_present(self):
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        self.assertTrue(m.ping())

    def test_third_serial_motor_exhausts_slots(self):
        from openbricks.drivers.st3032 import ST3032Motor
        ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6)
        with self.assertRaises(RuntimeError):
            ST3032Motor(servo_id=3, uart_id=1, tx=14, rx=6)


class FullFirmwareCodeIntegrationTest(_ShimTestBase):
    """End-to-end: import the *real* openbricks driver classes through
    the shim, construct them with hardware-style pin numbers, and drive
    a straight move. If this passes, firmware-targeting user code can
    run unchanged inside the sim."""

    def test_st3032_drivebase_straight(self):
        # The user's actual robot: two ST-3032 serial servos through
        # the openbricks DriveBase wrapper. Adoption hands them to the
        # ONE serial engine over the emulated st_bus (_SimStBus) — the
        # same code path as firmware — driving MuJoCo wheels. The
        # bench script's invert=True on the right motor rides along as
        # an ignored wiring concern.
        db, _, _ = self._serial_db()
        db.settings(straight_speed=150, acceleration=360)
        db.straight(50)
        x_mm, _, _ = self.robot.chassis_pose()
        self.assertGreater(x_mm, 5.0,
                           "serial-bus drivebase.straight should have "
                           "moved the chassis +X (got x=%.1f mm)" % x_mm)

    def test_settings_acceleration_reaches_the_sim_core(self):
        # The knob a user sets in firmware code (settings(acceleration=…))
        # must land in the sim's C core through wrapper → ShimDriveBase →
        # SimDriveBase → _native. The C-side ValueError propagating back
        # up through all four layers is the proof.
        from openbricks.drivers.jgb37_520 import JGB37Motor
        from openbricks.robotics.drivebase import DriveBase

        m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                              encoder_a=18, encoder_b=19)
        m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                              encoder_a=20, encoder_b=21)
        db = DriveBase(m_left, m_right,
                        wheel_diameter_mm=60, axle_track_mm=150)
        db.settings(acceleration=90)          # accepted end to end
        with self.assertRaises(ValueError):
            db.settings(acceleration=0)       # wrapper-level gate
        with self.assertRaises(ValueError):
            db._native.set_accel(-5.0)        # C-level gate, via shim

    def test_jgb37_drivebase_straight(self):
        # Add the openbricks package to sys.path the same way
        # shim.install does (for our module-level ``openbricks`` import
        # below). install() already did this — verify the import works.
        from openbricks.drivers.jgb37_520 import JGB37Motor
        from openbricks.robotics.drivebase import DriveBase

        m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                              encoder_a=18, encoder_b=19)
        m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                              encoder_a=20, encoder_b=21)
        db = DriveBase(m_left, m_right,
                        wheel_diameter_mm=60, axle_track_mm=150)
        # The drivebase wrapper at the openbricks side sets cruise via
        # settings(); use the default 200 dps.
        db.settings(straight_speed=180, turn_rate=120)
        # Blocking call: straight() busy-waits on time.sleep_ms which
        # the shim patched to step the sim. So this returns when the
        # native trajectory is done.
        db.straight(50)
        # After return, the chassis should have translated some +X.
        x_mm, _, _ = self.robot.chassis_pose()
        self.assertGreater(x_mm, 5.0,
                            "drivebase.straight should have moved the "
                            "chassis +X (got x=%.1f mm)" % x_mm)

    @unittest.expectedFailure
    def test_jgb37_drivebase_straight_lands_near_target(self):
        # Tighter version of ``test_jgb37_drivebase_straight``: pin
        # what users *expect* — straight(50) leaves the chassis near
        # x=50, not just somewhere positive. As of this commit the
        # chassis significantly overshoots the target because the
        # native drivebase's ``is_done()`` fires when the planned
        # trajectory distance is reached, not when the chassis has
        # actually settled at the target. ``db.stop()`` then cuts
        # power without active braking, so the chassis coasts past.
        # Real hardware masks this with motor-internal friction;
        # the sim doesn't, so a 50 mm requested move can land at
        # 100-300 mm depending on speed.
        #
        # Marked ``expectedFailure`` so the test runs (catches a
        # future fix that flips it green) without breaking the
        # current suite. Fix landing-place: deceleration phase that
        # brings velocity to ~0 at target, and/or active braking
        # in ``ob_drivebase_stop``.
        from openbricks.drivers.jgb37_520 import JGB37Motor
        from openbricks.robotics.drivebase import DriveBase
        m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                              encoder_a=18, encoder_b=19)
        m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                              encoder_a=20, encoder_b=21)
        db = DriveBase(m_left, m_right,
                        wheel_diameter_mm=60, axle_track_mm=150)
        db.settings(straight_speed=180)
        db.straight(50)
        x_mm, _, _ = self.robot.chassis_pose()
        # Within ±20% of the requested 50 mm.
        self.assertGreaterEqual(x_mm, 40.0)
        self.assertLessEqual(x_mm, 60.0)


class SimStBusEngineTests(_ShimTestBase):
    """The emulated st_bus surface (``_SimStBus``) under the ONE
    serial engine — turn, gyro feed, the driver-facing servo verbs,
    and the runtime-reset/estop hooks. Mirrors what the firmware
    st_bus answers to the same calls."""

    def _heading(self):
        from openbricks_sim.runtime import SimIMU
        return SimIMU(self.robot.runtime).heading()

    def test_turn_rotates_chassis_cw_and_completes(self):
        db, _, _ = self._serial_db()
        db.settings(turn_rate=120, acceleration=360)
        h0 = self._heading()
        db.turn(90)
        turned = ((self._heading() - h0 + 180.0) % 360.0) - 180.0
        # Pybricks CW-positive; the sim IMU heading uses the same
        # convention (see test_runtime's gyro turn tests).
        self.assertTrue(60.0 < turned < 120.0,
                        "turn(90) rotated %+.1f deg" % turned)
        self.assertTrue(db.done())

    def test_use_gyro_turn_feeds_heading_and_lands(self):
        # use_gyro(True) makes the engine's wait loop pump the IMU
        # heading into db_set_heading; the C controller then ends the
        # turn on MEASURED rotation. Covers db_use_gyro +
        # db_set_heading end-to-end in physics.
        from openbricks_sim.shim import ShimBNO055
        imu = ShimBNO055(i2c=None, address=0x29)
        db, _, _ = self._serial_db(imu=imu)
        db.settings(turn_rate=120, acceleration=360)
        db.use_gyro(True)
        h0 = self._heading()
        db.turn(90)
        turned = ((self._heading() - h0 + 180.0) % 360.0) - 180.0
        self.assertTrue(60.0 < turned < 120.0,
                        "gyro turn(90) rotated %+.1f deg" % turned)
        db.use_gyro(False)

    def test_native_path_stop_applies_the_end_state_to_both_wheels(self):
        # Firmware parity for the ENCODER path (drivebase.c db_stop):
        # DriveBase.stop(then=) routes ONE ShimDriveBase.stop(mode)
        # call that puts both wheels into the end state, instead of
        # per-motor coast()/brake() dispatch from Python.
        from openbricks.drivers.jgb37_520 import JGB37Motor
        from openbricks.robotics.drivebase import DriveBase

        left  = JGB37Motor(in1=1, in2=2, pwm=17, encoder_a=7, encoder_b=8)
        right = JGB37Motor(in1=9, in2=10, pwm=11, encoder_a=12, encoder_b=13)
        db = DriveBase(left, right, wheel_diameter_mm=65, axle_track_mm=120)
        self.assertIsNotNone(db._native)
        db.straight(200, wait=False)
        time.sleep_ms(100)
        self.assertTrue(left._servo._adapter._attached)
        self.assertTrue(right._servo._adapter._attached)
        db.stop()                                  # then="coast"
        # Both detached from the tick loop and both actuators zeroed
        # by the one call — neither wheel outlives the other.
        rt = left._servo._adapter.runtime
        self.assertFalse(left._servo._adapter._attached)
        self.assertFalse(right._servo._adapter._attached)
        self.assertEqual(rt.data.ctrl[left._servo._adapter._actuator_id], 0.0)
        self.assertEqual(rt.data.ctrl[right._servo._adapter._actuator_id], 0.0)

        # move_wheels on the native path: both wheels re-subscribed,
        # each at its own speed.
        db.move_wheels(150, 90)
        self.assertTrue(left._servo._adapter._attached)
        self.assertTrue(right._servo._adapter._attached)
        db.stop()

        # turn + then="brake" take the same one-call route (mode 1).
        db.turn(45, wait=False)
        time.sleep_ms(100)
        self.assertTrue(left._servo._adapter._attached)
        self.assertTrue(right._servo._adapter._attached)
        db.stop(then="brake")
        self.assertFalse(left._servo._adapter._attached)
        self.assertFalse(right._servo._adapter._attached)
        self.assertEqual(rt.data.ctrl[left._servo._adapter._actuator_id], 0.0)
        self.assertEqual(rt.data.ctrl[right._servo._adapter._actuator_id], 0.0)

    def test_move_wheels_drives_each_wheel_at_its_own_speed(self):
        # The SyncServoGroup replacement, in physics: unequal wheel
        # speeds must actually turn the chassis, and a later coupled
        # move must still work (the db yields, then re-arms).
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        l0, r0 = sb.servo_counts(0), sb.servo_counts(1)
        db.move_wheels(200, 100)
        time.sleep_ms(600)
        dl = sb.servo_counts(0) - l0
        dr = sb.servo_counts(1) - r0
        self.assertGreater(dl, 0)
        self.assertGreater(dr, 0)
        self.assertGreater(dl, dr * 1.3)    # left genuinely faster
        db.stop()
        # And the coupled controller still owns the wheels afterwards.
        db.settings(straight_speed=150, acceleration=360)
        db.straight(50)
        self.assertTrue(db.done())

    def test_servo_verbs_proxy_to_mujoco_wheels(self):
        # servo_run / servo_counts / servo_coast are the st_bus verbs
        # the FIRMWARE driver's adopted wheel-mode API calls (the shim
        # motors answer those from MuJoCo directly, so exercise the
        # bus surface itself — same contract as the C module). Since
        # 1.46.0 an idle drivebase YIELDS its wheels (it writes only
        # from db_straight/db_turn until db_stop), so direct verbs
        # work right after construction — no db_disable needed.
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        c0 = sb.servo_counts(0)
        self.assertTrue(sb.servo_run(0, 120 * sb._STEPS_PER_DEG))
        time.sleep_ms(300)
        self.assertGreater(sb.servo_counts(0), c0 + 10)
        self.assertTrue(sb.servo_coast(0))

    def test_servo_move_drives_the_wheel_by_delta(self):
        # The C per-slot move (RawServoMove = st_move_core) in
        # physics: half a wheel-rev commanded through the bus surface,
        # arrival-latched done, wheel lands within tolerance.
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        c0 = sb.servo_counts(0)
        self.assertTrue(sb.servo_move(0, 2048.0, 2000.0, 8000.0))
        self.assertFalse(sb.servo_move_done(0))
        # The sim wheel's inner velocity loop settles the last few
        # counts exponentially — give the arrival latch ~4.5 s.
        time.sleep_ms(4500)
        self.assertTrue(sb.servo_move_done(0))
        self.assertLess(abs(sb.servo_counts(0) - c0 - 2048), 80)

    def test_servo_move_refused_while_db_move_in_flight(self):
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        db.straight(300, wait=False)
        self.assertFalse(sb.servo_move(0, 1000.0, 1000.0, 4000.0))
        self.assertFalse(sb.servo_hold(0))
        db.stop()
        self.assertTrue(sb.servo_move(0, 1000.0, 1000.0, 4000.0))

    def test_db_stop_yields_the_wheels(self):
        # After stop() the db no longer re-asserts its hold, so a
        # direct speed command moves the chassis.
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        db.settings(straight_speed=150, acceleration=360)
        db.straight(500, wait=False)
        time.sleep_ms(300)
        db.stop()
        c0 = sb.servo_counts(0)
        sb.servo_run(0, 120 * sb._STEPS_PER_DEG)
        time.sleep_ms(400)
        self.assertGreater(sb.servo_counts(0) - c0, 60)

    def test_stop_then_applies_the_end_state_to_both_wheels_atomically(self):
        # Firmware-parity for the atomic stop: DriveBase.stop(then=)
        # routes ONE db_stop(mode) call instead of per-motor
        # dispatch. hold arms the REAL C position holds (st_move_core)
        # on both wheels in the same call; coast releases both.
        db, left, right = self._serial_db()
        sb = db._serial_engine._sb
        db.settings(straight_speed=150, acceleration=360)
        db.straight(300, wait=False)
        time.sleep_ms(200)
        db.stop(then="hold")
        self.assertTrue(sb._moves[0].is_active())
        self.assertTrue(sb._moves[1].is_active())
        db.stop(then="coast")
        self.assertFalse(sb._moves[0].is_active())
        self.assertFalse(sb._moves[1].is_active())
        self.assertEqual(left._mode, "idle")
        self.assertEqual(right._mode, "idle")

    def test_gyro_square_drift_stays_bounded_across_stops(self):
        # THE +7.6-deg bench regression: RawDriveBase.stop() (like the
        # firmware binding) re-captured turn_hold from measured
        # heading, re-baselining the absolute gyro frame at every
        # per-move stop — the one-class flow stops after EVERY move,
        # so each turn banked its arrival residual. A full gyro square
        # in physics must return near the start heading.
        from openbricks_sim.shim import ShimBNO055
        imu = ShimBNO055(i2c=None, address=0x29)
        db, _, _ = self._serial_db(imu=imu)
        db.settings(straight_speed=150, turn_rate=120, acceleration=360)
        db.use_gyro(True)
        h0 = self._heading()
        for _ in range(4):
            db.straight(120)
            db.turn(90)
        drift = ((self._heading() - h0 + 180.0) % 360.0) - 180.0
        self.assertTrue(abs(drift) < 5.0,
                        "gyro square drifted %+.1f deg" % drift)

    def test_servo_feedback_reports_live_wheel_speed(self):
        # The 1.50.0 feedback surface: an adopted motor's speed()
        # reads through servo_feedback; in the sim that's the MuJoCo
        # wheel's actual velocity (load is 0 — the shim wheel model
        # has no torque estimate; fresh stays True because speed IS
        # live).
        db, _, right = self._serial_db()
        sb = db._serial_engine._sb
        sb.servo_run(0, int(120 * sb._STEPS_PER_DEG))
        time.sleep_ms(500)
        steps, load, fresh = sb.servo_feedback(0)
        self.assertTrue(fresh)
        self.assertEqual(load, 0)
        dps = steps / sb._STEPS_PER_DEG
        self.assertTrue(60 < dps < 180, "wheel dps=%.0f" % dps)
        sb.servo_coast(0)

    def test_bus_health_surface_reports_permanently_healthy(self):
        # The engine consults servo_stats / servo_write_stats /
        # db_fault when a wheel misbehaves. Sim wheels cannot go
        # silent or lose writes, so the CONTRACT answers healthy —
        # the failure modes themselves are hardware ones (documented
        # sim limitation, mirrored in the firmware fakes).
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        self.assertEqual(sb.servo_stats(0), (1, 0, 0))
        self.assertEqual(sb.servo_write_stats(0), (0, 0))
        self.assertEqual(sb.servo_write_stats(1), (0, 0))
        self.assertEqual(sb.db_fault(), 0)

    def test_db_and_runtime_verbs_cancel_armed_moves(self):
        # New-command-wins in every direction: each db/runtime verb
        # must cancel an ARMED per-slot move, not just tolerate an
        # empty move table.
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        self.assertTrue(sb.servo_move(0, 40960.0, 2000.0, 8000.0))
        sb.db_straight(50.0, 60.0)
        self.assertFalse(sb._moves[0].is_active())
        sb.db_stop()
        self.assertTrue(sb.servo_move(0, 40960.0, 2000.0, 8000.0))
        sb.db_turn(30.0, 60.0)
        self.assertFalse(sb._moves[0].is_active())
        sb.db_stop()
        self.assertTrue(sb.servo_move(0, 40960.0, 2000.0, 8000.0))
        sb.torque_off_all()
        self.assertFalse(sb._moves[0].is_active())
        self.assertTrue(sb.servo_move(1, 40960.0, 2000.0, 8000.0))
        sb.reset_runtime()
        self.assertFalse(sb._moves[1].is_active())
        self.assertTrue(sb.servo_move(0, 40960.0, 2000.0, 8000.0))
        sb.db_config(0, 1, 65.0, 120.0, 400.0)
        self.assertFalse(sb._moves[0].is_active())

    def test_adopted_motor_run_angle_via_the_engine_surface(self):
        # End-to-end for the user-visible path: the ADOPTED firmware
        # driver routes run_angle through servo_move — in the sim the
        # shim motor keeps its own MuJoCo implementation, so pin the
        # equivalent contract at the bus surface with hold: hold is
        # done immediately and holds position.
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        self.assertTrue(sb.servo_hold(1))
        self.assertTrue(sb.servo_move_done(1))
        held = sb.servo_counts(1)
        time.sleep_ms(500)
        self.assertLess(abs(sb.servo_counts(1) - held), 40)

    def test_torque_off_all_stops_ticking_the_controller(self):
        # The estop broadcast surface: after torque_off_all the bus
        # deactivates and _tick's guard skips the controller, so a
        # pending move stops advancing the chassis.
        db, _, _ = self._serial_db()
        db.settings(straight_speed=150, acceleration=360)
        db.straight(500, wait=False)
        time.sleep_ms(200)
        sb = db._serial_engine._sb
        self.assertTrue(sb.torque_off_all())
        x0, _, _ = self.robot.chassis_pose()
        time.sleep_ms(300)   # sim keeps stepping; controller must not
        x1, _, _ = self.robot.chassis_pose()
        self.assertLess(abs(x1 - x0), 3.0,
                        "chassis kept driving after torque_off_all "
                        "(%.1f -> %.1f mm)" % (x0, x1))

    def test_reset_runtime_clears_the_drivebase_config(self):
        # launcher._reset_motor_process parity: reset_runtime drops
        # the controller so the NEXT program's db_config starts clean.
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        self.assertIsNotNone(sb._raw)
        sb.reset_runtime()
        self.assertIsNone(sb._raw)
        self.assertFalse(sb._active)
        # Ticking in the unconfigured state is a no-op, not a crash.
        time.sleep_ms(50)


if __name__ == "__main__":
    unittest.main()


class WorldAliasTableTests(unittest.TestCase):
    """``cli.py`` and ``robot.py`` each carry a world-alias table.

    They must agree: a world registered in one and not the other
    loads from the CLI and fails from ``SimRobot`` (or the reverse),
    which is exactly how ``practice-line`` first failed.
    """

    def test_the_two_alias_tables_match(self):
        from openbricks_sim.cli import _BUILTIN_WORLDS as cli_worlds
        from openbricks_sim.robot import _BUILTIN_WORLDS as robot_worlds
        self.assertEqual(cli_worlds, robot_worlds)

    def test_every_aliased_world_file_exists(self):
        import pathlib
        from openbricks_sim.robot import _BUILTIN_WORLDS
        root = pathlib.Path(
            __import__("openbricks_sim").__file__).resolve().parent
        for alias, rel in _BUILTIN_WORLDS.items():
            if rel is None:
                continue
            self.assertTrue((root / rel).is_file(),
                            "%s -> %s missing" % (alias, rel))
