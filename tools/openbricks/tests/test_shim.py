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
from openbricks.parameters import Stop


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

    def test_uninstall_evicts_openbricks_modules(self):
        # openbricks.* imported UNDER the shim capture the fake
        # machine at module level; leaving them cached poisons a
        # later import in the same process (order-dependent tests).
        robot = SimRobot()
        shim.install(robot.runtime)
        try:
            import openbricks.drivers.st3215  # noqa: F401
            self.assertIn("openbricks.drivers.st3215", sys.modules)
        finally:
            shim.uninstall()
        self.assertNotIn("openbricks.drivers.st3215", sys.modules)

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

    def test_third_encoder_servo_is_refused_but_serial_takes_the_slot(self):
        # Two physical wheel slots. The kinematic third/fourth slots
        # are serial-only — an encoder motor's whole point is the
        # physics behind it — so a third Servo still raises, while a
        # third serial motor binds a kinematic slot (bench shape).
        from _openbricks_native import Servo
        from openbricks.drivers.st3032 import ST3032Motor
        Servo(in1=12, in2=14, pwm=27, encoder=None)
        Servo(in1=13, in2=15, pwm=26, encoder=None)
        with self.assertRaises(RuntimeError):
            Servo(in1=99, in2=98, pwm=97, encoder=None)
        # The refusal must not burn the slot: serial task motors
        # still get BOTH kinematic slots...
        t1 = ST3032Motor(servo_id=3, uart_id=1, tx=14, rx=6)
        t2 = ST3032Motor(servo_id=4, uart_id=1, tx=14, rx=6)
        self.assertIsNone(t1._plumb)            # kinematic
        self.assertIsNone(t2._plumb)
        # ...and the fifth motor overall is refused with the count.
        with self.assertRaises(RuntimeError):
            ST3032Motor(servo_id=5, uart_id=1, tx=14, rx=6)


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


class BenchShapeTests(_ShimTestBase):
    """The user's actual robot: four ST-3032s on ONE UART — two
    DriveBase wheels + two task motors, exactly the firmware's four
    native slots. That main.py must construct AND run under the sim;
    it used to be refused at the third motor."""

    def _bench(self):
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics.drivebase import DriveBase
        left  = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6,
                            invert=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        db = DriveBase(left, right,
                       wheel_diameter_mm=88, axle_track_mm=136)
        t1 = ST3032Motor(servo_id=3, uart_id=1, tx=14, rx=6)
        t2 = ST3032Motor(servo_id=4, uart_id=1, tx=14, rx=6)
        return db, t1, t2

    def test_four_motor_script_constructs_and_everything_moves(self):
        db, t1, t2 = self._bench()
        # Kinematic task motor: run_angle completes and lands.
        t1.run_angle(200, 90)
        self.assertTrue(80 < t1.angle() < 100, t1.angle())
        # Speed mode integrates too.
        t2.run_speed(120)
        time.sleep_ms(500)
        self.assertTrue(t2.angle() > 30, t2.angle())
        t2.coast()
        # And the chassis still drives — task motors took the
        # kinematic slots, not the wheels'.
        x0 = float(self.robot.runtime.data.qpos[0])
        db.straight(40)
        self.assertTrue(db.done())
        self.assertTrue(abs(float(self.robot.runtime.data.qpos[0]) - x0)
                        > 0.005)

    def test_task_motor_velocity_reads_the_kinematic_integrator(self):
        # A task motor has no MuJoCo actuator (or qvel) behind it,
        # so the velocity accessor must answer from the kinematic
        # integrator — pinned here so any caller gets a live value,
        # not a crash on the missing DOF.
        db, _, t2 = self._bench()
        t2.run_speed(120)
        time.sleep_ms(300)
        self.assertTrue(abs(t2._vel_dps() - 120) < 40, t2._vel_dps())
        t2.coast()
        time.sleep_ms(100)
        self.assertEqual(t2._vel_dps(), 0.0)

    def test_drivebase_adopts_the_physical_wheels_whatever_the_order(self):
        # Firmware rule (st3215 _attach_task_slot): a DriveBase adopts
        # whatever slots its wheels hold, so construction order never
        # matters. The sim's physical/kinematic split follows: the
        # adopted pair gets the chassis wheels even when the task
        # motors were built first (the bench main.py's actual order),
        # and the displaced task motors become kinematic shafts.
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics.drivebase import DriveBase
        front = ST3032Motor(servo_id=4, uart_id=1, tx=14, rx=6)
        back = ST3032Motor(servo_id=3, uart_id=1, tx=14, rx=6)
        left = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        self.assertIsNotNone(front._plumb)      # held the wheels so far
        self.assertIsNone(left._plumb)
        db = DriveBase(left, right, wheel_diameter_mm=88, axle_track_mm=136)
        self.assertIsNotNone(left._plumb)
        self.assertIsNotNone(right._plumb)
        self.assertNotEqual(left._actuator_id, right._actuator_id)
        self.assertIsNone(front._plumb)
        self.assertIsNone(back._plumb)
        x0 = float(self.robot.runtime.data.qpos[0])
        db.straight(40)
        self.assertTrue(db.done())
        self.assertTrue(abs(float(self.robot.runtime.data.qpos[0]) - x0)
                        > 0.005)
        # ...and the task motors still run, kinematically.
        front.run_angle(200, 90)
        self.assertTrue(80 < front.angle() < 100, front.angle())

    def test_same_servo_id_is_the_same_motor(self):
        # One servo, one slot: the bus keys by id, so re-constructing
        # a motor for an id yields the SAME motor (the bench main.py
        # re-constructs its front/back motors to swap their names),
        # and it consumes no further slot — six constructions over
        # four ids fit the four-slot bus.
        from openbricks.drivers.st3032 import ST3032Motor
        a = ST3032Motor(servo_id=4, uart_id=1, tx=14, rx=6)
        b = ST3032Motor(servo_id=3, uart_id=1, tx=14, rx=6)
        ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6)
        ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        b2 = ST3032Motor(servo_id=4, uart_id=1, tx=14, rx=6)
        a2 = ST3032Motor(servo_id=3, uart_id=1, tx=14, rx=6)
        self.assertIs(b2, a)
        self.assertIs(a2, b)
        a.run_speed(120)
        time.sleep_ms(200)
        self.assertGreater(b2.angle(), 10.0)     # same shaft
        with self.assertRaises(RuntimeError):
            ST3032Motor(servo_id=5, uart_id=1, tx=14, rx=6)


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

    def test_health_reports_nominal_supply_and_no_flags(self):
        # Firmware parity (3.3.0): a program that logs motor.health()
        # runs unchanged in the sim — nominal 12 V, room temperature,
        # a speed-following current, never a protection flag.
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        h = m.health()
        self.assertEqual(h.voltage, 12.0)
        self.assertEqual(h.temperature, 25.0)
        self.assertEqual(h.flags, ())
        self.assertEqual(h.status, 0)
        idle = h.current
        m.run_speed(200)
        time.sleep_ms(400)
        self.assertGreater(m.health().current, idle)
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

    def test_velocity_loop_holds_the_commanded_speed(self):
        # The servo's inner loop is PI over a DC model free-running
        # at THIS servo's no-load speed: a commanded speed is the
        # wheel's speed, up to the ST-3032's 888. (P alone against
        # the generic 300 dps model settled 350 -> ~200.)
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        for cmd in (120.0, 350.0, 600.0):
            m.run_speed(cmd)
            time.sleep_ms(1500)
            self.assertAlmostEqual(m.speed(), cmd, delta=5.0)
        m.coast()

    def test_chassis_speed_follows_the_wheel_command(self):
        # 350 dps on 86.4 mm wheels is 264 mm/s of floor.
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics.drivebase import DriveBase
        l = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)
        r = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        db = DriveBase(l, r, wheel_diameter_mm=86.4, axle_track_mm=135)
        db.settings(acceleration=1000)
        db.move_wheels(350, 350)
        time.sleep_ms(1500)                    # ramp + settle
        x0 = self.robot.chassis_pose()[0]
        time.sleep_ms(1000)
        v = self.robot.chassis_pose()[0] - x0
        self.assertTrue(240 < v < 290, "%.0f mm/s" % v)
        db.stop()

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

    def test_blocked_run_angle_reports_and_continues(self):
        # A physically blocked wheel defeats the crawl floor; the
        # wait loop used to spin forever (a CI timeout instead of a
        # named stall). Firmware-style budget: report, return False,
        # park — the mission continues.
        from openbricks.drivers.st3032 import ST3032Motor
        m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
        m._apply_v = lambda now_ms, v_cmd: None    # shaft never moves
        result = m.run_angle(200, 90)
        self.assertFalse(result)
        self.assertEqual(m._mode, "idle")          # gave up, parked
        # And an unblocked move still completes with True.
        m2 = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6)
        self.assertTrue(m2.run_angle(200, 45))

    def test_unmapped_mux_channel_raises_not_impersonates(self):
        # The bench has a third TCS34725 on mux channel 2; the shim
        # silently fell back to the CENTRE camera for it — plausible
        # readings from the wrong sensor. A sensor the sim cannot
        # model must say so.
        from openbricks.drivers.tca9548a import TCA9548A
        from openbricks.drivers.tcs34725 import TCS34725
        from machine import I2C, Pin
        mux = TCA9548A(I2C(0, sda=Pin(15), scl=Pin(16)))
        TCS34725(mux[0])                    # mapped: fine
        try:
            TCS34725(mux[2])
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("channel 2" in str(e), e)

    def test_fifth_serial_motor_exhausts_slots(self):
        # Four slots now — 2 physical wheels + 2 kinematic task
        # motors, the firmware bus's count and the bench robot's
        # shape (a third used to be refused outright).
        from openbricks.drivers.st3032 import ST3032Motor
        for sid in (1, 2, 3, 4):
            ST3032Motor(servo_id=sid, uart_id=1, tx=14, rx=6)
        with self.assertRaises(RuntimeError):
            ST3032Motor(servo_id=5, uart_id=1, tx=14, rx=6)


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

    def test_st3032_drivebase_curve_quarter_circle(self):
        # curve(150, 90): a forward quarter-circle to the right.
        # Start facing +X (world yaw 0, CCW-positive): forward AND
        # rightward (-Y) displacement with a substantial CW yaw.
        # Tolerances match the suite's other chassis assertions —
        # this sim rig under-rotates the chassis relative to the
        # encoder frame (a plain turn(90) lands at ~-65 deg here
        # too), so exact pose is the C-level tests' job
        # (tests/test_st_drivebase.py pins the wheel geometry
        # precisely); THIS test pins the end-to-end wiring and the
        # arc's shape.
        db, _, _ = self._serial_db()
        db.settings(straight_speed=150, acceleration=360)
        db.curve(150, 90)
        x_mm, y_mm, yaw_deg = self.robot.chassis_pose()
        self.assertTrue(x_mm > 80,
                        "expected forward motion, got x=%.1f" % x_mm)
        self.assertTrue(y_mm < -60,
                        "expected rightward arc (-Y), got y=%.1f" % y_mm)
        self.assertTrue(yaw_deg < -45,
                        "expected substantial CW yaw, got %.1f" % yaw_deg)

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

    def test_jgb37_drivebase_curve_arcs_right(self):
        # The PWM sim path (ShimDriveBase -> SimDriveBase -> the C
        # extension's DriveBase.curve): a CW arc moves the chassis
        # forward and rightward with CW yaw — the serial path's curve
        # is covered by test_st3032_drivebase_curve_quarter_circle.
        from openbricks.drivers.jgb37_520 import JGB37Motor
        from openbricks.robotics.drivebase import DriveBase

        m_left  = JGB37Motor(in1=12, in2=14, pwm=27,
                              encoder_a=18, encoder_b=19)
        m_right = JGB37Motor(in1=13, in2=15, pwm=26,
                              encoder_a=20, encoder_b=21)
        db = DriveBase(m_left, m_right,
                        wheel_diameter_mm=60, axle_track_mm=150)
        db.settings(straight_speed=180, turn_rate=120)
        db.curve(150, 90)
        x_mm, y_mm, yaw_deg = self.robot.chassis_pose()
        self.assertGreater(x_mm, 30.0,
                           "curve should move the chassis +X "
                           "(got x=%.1f mm)" % x_mm)
        self.assertLess(y_mm, -20.0,
                        "CW curve should move the chassis -Y "
                        "(got y=%.1f mm)" % y_mm)
        self.assertLess(yaw_deg, -30.0,
                        "CW curve should yaw negative "
                        "(got %.1f deg)" % yaw_deg)

    def test_jgb37_drivebase_straight_lands_near_target(self):
        # Tighter version of ``test_jgb37_drivebase_straight``: pin
        # what users *expect* — straight(50) leaves the chassis near
        # x=50, not just somewhere positive. This was an
        # ``expectedFailure`` ("the chassis coasts 100-300 mm past
        # the target") until the chassis resize stopped burying the
        # wheels: ``apply_drivebase_dims_to_model`` placed a resized
        # wheel by the body-underside formula chassis_mjcf had
        # already abandoned (issue #234), so every native-path
        # drivebase ran on 20 mm of floor penetration — and THAT was
        # the coasting. On wheels that touch the floor the move
        # lands within the tolerance below.
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

    def test_move_wheels_ramps_at_the_configured_accel(self):
        # 1.94.0: move_wheels (and drive(), which routes through it)
        # obey settings.acceleration — the uniform-accel rule. The
        # 1.89.0 duty default removed the servo's internal goal_acc
        # from the circuit, which had been papering over the step.
        import time as _t
        db, left, right = self._serial_db()
        db.settings(acceleration=600)
        db.move_wheels(300, 300)
        _t.sleep_ms(100)           # mid-ramp: 600 dps^2 * 0.1 s = 60
        mid = getattr(left, "_target_dps", None)
        self.assertTrue(mid is not None and 20 < abs(mid) < 150,
                        "expected ~60 dps mid-ramp, got %r" % mid)
        _t.sleep_ms(800)           # ramp completes, db yields
        self.assertTrue(abs(getattr(left, "_target_dps")) >= 295,
                        getattr(left, "_target_dps"))
        db.stop()

    def test_move_wheels_ramp_preserves_the_wheel_ratio(self):
        import time as _t
        db, left, right = self._serial_db()
        db.settings(acceleration=600)
        db.move_wheels(300, 150)
        _t.sleep_ms(100)           # genuinely mid-ramp
        l = abs(getattr(left, "_target_dps"))
        r = abs(getattr(right, "_target_dps"))
        self.assertTrue(0 < r < 150, r)
        self.assertTrue(abs(l / r - 2.0) < 0.1,
                        "ratio drifted to %.2f mid-ramp" % (l / r))
        db.stop()

    def test_stop_wait_blocks_until_wheels_settle(self):
        # stop(wait=True) — extension beyond Pybricks: dispatch the
        # ramped brake, then block on MEASURED wheel speeds. On
        # return, the physics wheels must actually be near rest.
        import time as _t
        db, left, right = self._serial_db()
        db.settings(acceleration=800)
        db.move_wheels(500, 500)
        _t.sleep_ms(900)
        self.assertTrue(abs(left.speed()) > 200, left.speed())
        db.stop(then=Stop.BRAKE, wait=True)
        self.assertTrue(abs(left.speed()) < 15, left.speed())
        self.assertTrue(abs(right.speed()) < 15, right.speed())

    def test_straight_after_cruise_blends_in_physics(self):
        # 2.0.0: trajectories arm from the current speed. In
        # MuJoCo, a straight() right after a move_wheels cruise must
        # not slam the wheel command to zero — the commanded speed
        # 30 ms into the move is still a large fraction of cruise.
        import time as _t
        db, left, right = self._serial_db()
        db.settings(acceleration=1500)
        db.move_wheels(500, 500)
        _t.sleep_ms(600)                # cruise established
        pre = abs(getattr(left, "_target_dps"))
        self.assertTrue(pre > 450, pre)
        db.straight(200, wait=False)
        _t.sleep_ms(30)
        mid = abs(getattr(left, "_target_dps"))
        self.assertTrue(mid > 300,
                        "command cliffed to %.0f dps" % mid)
        db.stop()

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
        db.stop()                                  # then=Stop.COAST
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

        # turn + then=Stop.BRAKE take the same one-call route (mode 1).
        db.turn(45, wait=False)
        time.sleep_ms(100)
        self.assertTrue(left._servo._adapter._attached)
        self.assertTrue(right._servo._adapter._attached)
        db.stop(then=Stop.BRAKE)
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

    def test_then_continue_chains_without_stopping(self):
        # then=Stop.NONE (Pybricks Stop.NONE): the first leg ends AT
        # speed and the second leg takes it over — the wheels never
        # rest between segments.
        db, left, _ = self._serial_db()
        db.settings(straight_speed=150, acceleration=360)
        db.straight(150, then=Stop.NONE)     # blocking, ends at cruise
        v_seam = abs(left.speed())
        self.assertGreater(v_seam, 90.0,
                           "wheels at %.0f dps at the seam - carried "
                           "speed lost" % v_seam)
        db.straight(150)                      # stopping second leg
        db.stop(then=Stop.COAST)
        time.sleep_ms(600)                    # freewheel decay
        self.assertLess(abs(left.speed()), 60.0)

    def test_db_stop_yields_the_wheels(self):
        # After stop(then=Stop.COAST) the db no longer re-asserts its
        # hold, so a direct speed command moves the chassis. Coast is
        # explicit: the brake default actively resists at zero
        # velocity, which is ownership, not release.
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        db.settings(straight_speed=150, acceleration=360)
        db.straight(500, wait=False)
        time.sleep_ms(300)
        db.stop(then=Stop.COAST)
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
        db.stop(then=Stop.HOLD)
        # Hold DECELERATES at settings.acceleration first (uniform-
        # accel rule, 2026-08-14) and anchors where the robot stops —
        # the position holds arm at ramp completion, not instantly.
        time.sleep_ms(800)
        self.assertTrue(sb._moves[0].is_active())
        self.assertTrue(sb._moves[1].is_active())
        db.stop(then=Stop.COAST)
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

    def test_brake_after_a_follow_keeps_the_heading_the_follow_reached(self):
        # Firmware parity (3.2.0): a brake/hold decelerates as a move
        # of the coupled controller with the heading loop closed, and
        # after move_wheels (a line-follow) the heading target
        # re-anchors to where the follow REACHED — the brake and the
        # next straight hold that heading instead of unwinding the
        # follow's rotation back to the pre-follow target.
        from openbricks_sim.shim import ShimBNO055
        imu = ShimBNO055(i2c=None, address=0x29)
        db, _, _ = self._serial_db(imu=imu)
        db.settings(straight_speed=150, acceleration=360)
        db.use_gyro(True)                        # absolute target: here
        h0 = self._heading()
        db.move_wheels(200, 100)                 # arcing: chassis rotates
        time.sleep_ms(1500)
        reached = ((self._heading() - h0 + 180.0) % 360.0) - 180.0
        self.assertGreater(abs(reached), 6.0,
                           "arc rotated only %+.1f deg" % reached)
        db.stop(then=Stop.BRAKE, wait=True)
        after = ((self._heading() - h0 + 180.0) % 360.0) - 180.0
        self.assertLess(abs(after - reached), 6.0,
                        "brake steered from %+.1f to %+.1f deg"
                        % (reached, after))
        db.straight(60)
        end = ((self._heading() - h0 + 180.0) % 360.0) - 180.0
        self.assertLess(abs(end - after), 5.0,
                        "straight after the follow steered from %+.1f "
                        "to %+.1f deg" % (after, end))

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

    def test_straight_after_move_wheels_goes_forward_not_backward(self):
        # THE stale-bridge regression (this week's review, verified
        # by execution): 2 s of move_wheels advanced the chassis
        # ~177 mm, then straight(50) drove it BACKWARD 142.6 mm — the
        # db armed against the pose from before the yielded stretch,
        # because bridge odometry was only fed while the db was
        # WRITING. Firmware syncs the bridges every hard tick; the
        # shim now does too (RawDriveBase.sync).
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        db.move_wheels(200, 200)
        time.sleep_ms(2000)
        db.stop()
        x0 = float(self.robot.runtime.data.qpos[0])
        db.straight(50)
        self.assertTrue(db.done())
        moved = float(self.robot.runtime.data.qpos[0]) - x0
        # Forward, and in the right ballpark — the bug was -0.14 m.
        self.assertTrue(0.01 < moved < 0.15,
                        "straight(50) after move_wheels moved %+.1f mm"
                        % (moved * 1000.0))

    def test_wheels_claim_distinct_slots_and_reattach_is_refused(self):
        # Firmware-faithful slot bookkeeping: an occupied slot
        # refuses, so the engine's first-free-slot loop hands out 0
        # then 1. Before this, both wheels reported slot 0.
        db, _, _ = self._serial_db()
        eng = db._serial_engine
        self.assertEqual((eng._slot_l, eng._slot_r), (0, 1))
        sb = eng._sb
        self.assertEqual(sb.servo_slot_of(1), 0)     # left_id
        self.assertEqual(sb.servo_slot_of(2), 1)     # right_id
        self.assertEqual(sb.servo_slot_of(9), -1)
        self.assertFalse(sb.servo_attach(0, 9, False, 0))  # occupied

    def test_duty_parity_surface_exists(self):
        # Firmware's dumb-mode switches must run unchanged in sim:
        # accepted for good slots, loud for bad ones.
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        sb.servo_drive_duty(0, True)
        sb.servo_drive_duty(0, False)
        sb.duty_gains(101, 51, 3)
        sb.db_set_turn_accel(800.0)
        try:
            sb.servo_drive_duty(9, True)
            self.fail("expected ValueError")
        except ValueError:
            pass

    def test_db_curve_cancels_an_armed_move(self):
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        self.assertTrue(sb.servo_move(0, 40960.0, 2000.0, 8000.0))
        sb.db_curve(200.0, 45.0, 60.0)
        self.assertFalse(sb._moves[0].is_active())
        sb.db_stop()

    def test_servo_detach_frees_the_slot(self):
        db, _, _ = self._serial_db()
        sb = db._serial_engine._sb
        self.assertEqual(sb.servo_slot_of(2), 1)
        sb.servo_detach(1)
        self.assertEqual(sb.servo_slot_of(2), -1)
        self.assertTrue(sb.servo_attach(1, 9, False, 0))
        self.assertEqual(sb.servo_slot_of(9), 1)

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
        # A freewheeling chassis coasts a few mm from 150 dps (~5 on
        # wheels that hold their commanded speed); what must not
        # happen is the controller DRIVING on — 14 mm in 300 ms.
        self.assertLess(abs(x1 - x0), 10.0,
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


class ShimQTRTests(unittest.TestCase):
    """``QTRArray`` / ``QTRChannel`` / ``QTRLineSensor`` resolve to
    the shim subclasses and read the chassis line site — the bench
    line sensor's whole discipline (modes, edge error, calibration
    contract) running on simulated reflectance."""

    def setUp(self):
        if shim.is_installed():
            shim.uninstall()
        self.robot = SimRobot(world="practice-line")
        shim.install(self.robot.runtime)
        self.addCleanup(self._uninstall)

    @staticmethod
    def _uninstall():
        if shim.is_installed():
            shim.uninstall()

    def _line_sensor(self):
        from openbricks.drivers.qtr import QTRLineSensor
        from openbricks.parameters import LineMode
        qtr = QTRLineSensor()
        qtr.load_calibration("/qtr.cal")     # the hub path: no file here
        return qtr, LineMode

    def test_classes_resolve_to_the_shim(self):
        from openbricks.drivers import qtr as qtr_mod
        self.assertIs(qtr_mod.QTRLineSensor, shim.ShimQTRLineSensor)
        self.assertIs(qtr_mod.QTRArray, shim.ShimQTRArray)
        self.assertIs(qtr_mod.QTRChannel, shim.ShimQTRChannel)
        # ...and are still the firmware classes underneath: the
        # geometry table and setpoints come from there.
        self.assertTrue(issubclass(shim.ShimQTRLineSensor, shim._RealQTRLineSensor))
        self.assertEqual(shim.ShimQTRLineSensor.RIGHT_SETPOINT_MM, 16.0)

    def test_reading_before_calibration_still_raises(self):
        from openbricks.drivers.qtr import QTRLineSensor
        with self.assertRaises(RuntimeError):
            QTRLineSensor().read()

    def test_load_calibration_touches_no_file(self):
        import os
        qtr, _ = self._line_sensor()
        self.assertFalse(os.path.exists("/qtr.cal"))
        self.assertEqual(len(qtr.read()), 10)

    def test_calibrate_spends_its_sim_time(self):
        from openbricks.drivers.qtr import QTRLineSensor
        qtr = QTRLineSensor()
        t0 = self.robot.runtime.now_ms
        qtr.calibrate(duration_ms=300)
        self.assertEqual(self.robot.runtime.now_ms - t0, 300)
        self.assertEqual(len(qtr.read()), 10)

    def test_save_calibration_writes_nothing_but_keeps_the_contract(self):
        import os, tempfile
        from openbricks.drivers.qtr import QTRLineSensor
        qtr = QTRLineSensor()
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "qtr.cal")
            with self.assertRaises(RuntimeError):
                qtr.save_calibration(path)    # uncalibrated: same raise
            qtr.calibrate(duration_ms=10)
            qtr.save_calibration(path)
            self.assertFalse(os.path.exists(path))

    def test_on_the_practice_line_the_centre_elements_are_dark(self):
        # Spawn puts the array over the 20 mm line: +/-4 mm dark, the
        # rest over mat; the centroid sits on the array centre.
        qtr, LineMode = self._line_sensor()
        self.robot.run_for(0.3)               # settle on the wheels
        r = qtr.read()
        self.assertTrue(r[4].dark() and r[5].dark(), r)
        self.assertTrue(r[0].white() and r[9].white(), r)
        self.assertAlmostEqual(r.position(), 0.0, delta=2.0)
        qtr.set_mode(LineMode.CENTER)
        self.assertAlmostEqual(r.edge_error(), 0.0, delta=5.0)

    def test_right_edge_follower_tracks_the_line_and_stops_at_the_bar(self):
        # The bench main.py's inner loop, verbatim shape: hold the
        # line's right edge under channel 12, steer P on the edge
        # error, stop when the whole window goes dark. Practice-line
        # has a stop bar at x = 1.2 m and a LEFT-side branch stub at
        # x = 0.6 that must darken the left elements once.
        import time as _t
        from openbricks.drivers.st3032 import ST3032Motor
        from openbricks.robotics.drivebase import DriveBase
        qtr, LineMode = self._line_sensor()
        qtr.set_mode(LineMode.RIGHT)
        left = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41, invert=True)
        right = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
        db = DriveBase(left, right, wheel_diameter_mm=86.4,
                       axle_track_mm=135)
        db.settings(acceleration=1000)
        # P gain 1 on the +/-50 edge error at 250 dps cruise: tracks
        # within ~20 mm here (gain 2 rides over the branch stub and
        # reads it as the bar; the bench's own 0.5 wanders 50 mm —
        # exactly the kind of thing the sim exists to show before
        # the mat does).
        branches, in_branch, worst_y = 0, False, 0.0
        for _ in range(6000):
            r = qtr.read()
            if all(e.ambient() < 50 for e in r):
                db.stop()
                break
            err = r.edge_error()
            db.move_wheels(250 + 1.0 * err, 250 - 1.0 * err)
            left_dark = all(e.ambient() <= 50 for e in r[:7])
            if left_dark and not in_branch:
                branches += 1
            in_branch = left_dark
            _t.sleep_ms(5)
            worst_y = max(worst_y, abs(self.robot.chassis_pose()[1]))
        else:
            self.fail("never reached the stop bar")
        x_mm, _, _ = self.robot.chassis_pose()
        # The bar is at x = 1.2 m, the array 60 mm ahead of the axle.
        self.assertGreater(x_mm, 1100.0, "stopped short at %.0f mm" % x_mm)
        self.assertLess(x_mm, 1200.0, "overran the bar to %.0f mm" % x_mm)
        self.assertLess(worst_y, 30.0, "wandered %.0f mm off" % worst_y)
        self.assertGreaterEqual(branches, 1)

    def test_qtr_channel_is_one_element(self):
        from openbricks.drivers.qtr import QTRChannel
        ch = QTRChannel(pin=9)
        ch.calibrate(duration_ms=10)
        self.robot.run_for(0.3)
        # The lone element sits at the site centre, on the line
        # (practice-line's ink is a very dark grey, not 0/0/0).
        self.assertTrue(ch.dark())
        self.assertGreater(ch.value(), 900)


class SimIcm45686Tests(_ShimTestBase):
    """The REAL firmware ICM-45686 driver runs unchanged in the sim —
    no Shim class: ``_native.icm45686`` plus the motor_process
    hard-yaw surfaces are emulated, and the esp32 NVS fake lets
    calibration persistence run. The default gyro story, sim side."""

    def _icm(self):
        from openbricks.drivers.icm45686 import ICM45686
        return ICM45686(sck=12, mosi=13, miso=11, cs=17)

    def test_constructs_calibrated_with_gravity(self):
        imu = self._icm()
        self.assertTrue(imu.calibrated())
        self.assertTrue(imu.stats()[2])
        time.sleep_ms(50)          # sensors populate after stepping
        az = imu.acceleration()[2]
        self.assertTrue(abs(az - 1.0) < 0.2, az)     # g units

    def test_hard_gyro_turn_lands_and_heading_agrees(self):
        imu = self._icm()
        db, _, _ = self._serial_db(imu=imu)
        imu.reset_heading()        # allowed: gyro not driving yet
        db.use_gyro(True)          # hard source: db_gyro_source(1)
        db.turn(90)
        h = imu.heading()
        # +/-10: on the 65/120 geometry this lands at ~96. A turn on
        # any non-default chassis runs ~4% long in MuJoCo (measured
        # 93.9-94.4 for 65/120 and 86.4/135, built at compile time
        # or resized at adoption alike; the 60/150 default lands at
        # 90.3), and the gyro loop trims only part of it before the
        # trajectory ends. It landed inside 5 before 3.5.0 because
        # the resized wheels were buried 20 mm in the floor.
        self.assertTrue(abs(h - 90) < 10, h)
        db.reset()                 # the sanctioned mid-mission zero
        self.assertTrue(abs(imu.heading()) < 1, imu.heading())

    def test_reset_heading_refused_while_gyro_drives(self):
        # Pybricks parity ("Can't reset heading while gyro in use"):
        # bench 2026-08-13 — resetting the integrator under the armed
        # controller made the next straight() pivot left chasing the
        # frame the held target still remembered.
        imu = self._icm()
        db, _, _ = self._serial_db(imu=imu)
        db.use_gyro(True)
        try:
            imu.reset_heading()
            self.fail("expected OSError")
        except OSError as e:
            self.assertTrue("db.reset" in str(e), e)
        db.use_gyro(False)
        imu.reset_heading()        # allowed again once gyro released

    def test_db_reset_refused_while_a_move_is_active(self):
        imu = self._icm()
        db, _, _ = self._serial_db(imu=imu)
        db.use_gyro(True)
        db.straight(300, wait=False)
        try:
            db.reset()
            self.fail("expected RuntimeError")
        except RuntimeError as e:
            self.assertTrue("stop first" in str(e), e)
        db.stop()
        db.reset()                 # idle again: allowed

    def test_straight_after_turn_and_reset_goes_straight(self):
        # The bench script, end to end in physics: straight, turn
        # -90, reset, straight — the last leg must hold the NEW zero,
        # not pivot chasing the pre-reset -90 target.
        imu = self._icm()
        db, _, _ = self._serial_db(imu=imu)
        db.use_gyro(True)
        db.straight(100)
        db.turn(-90)
        db.reset()
        self.assertTrue(abs(imu.heading()) < 1, imu.heading())
        db.straight(130)
        h = imu.heading()
        self.assertTrue(abs(h) < 5,
                        "veered to %.1f deg after reset" % h)

    def test_calibration_round_trips_through_the_nvs_fake(self):
        imu = self._icm()
        imu.save_calibration()     # locked in sim: must not raise
        again = self._icm()        # reconstruction seeds from NVS
        self.assertTrue(again.calibrated())

    def test_heading_unwraps_across_the_180_boundary(self):
        # The firmware integrator is continuous multi-turn; SimIMU
        # wraps at +/-180 — both unwrap directions must survive.
        imu = self._icm()
        db, _, _ = self._serial_db(imu=imu)
        imu.reset_heading()
        db.use_gyro(True)
        db.turn(200)
        h = imu.heading()
        self.assertTrue(abs(h - 200) < 8, h)      # NOT wrapped to -160
        db.turn(-400)
        h = imu.heading()
        self.assertTrue(abs(h + 200) < 10, h)
        db.use_gyro(False)                        # gyro_source(0) leg

    def test_esp32_nvs_fake_blob_and_error_paths(self):
        import esp32
        nvs = esp32.NVS("openbricks")
        nvs.set_blob("k", b"hello")
        buf = bytearray(16)
        n = nvs.get_blob("k", buf)
        self.assertEqual(bytes(buf[:n]), b"hello")
        nvs.commit()
        try:
            nvs.get_blob("missing", buf)
            self.fail("expected OSError")
        except OSError:
            pass
        try:
            nvs.get_i32("k")                      # blob is not an int
            self.fail("expected OSError")
        except OSError:
            pass
        nvs.set_i32("n", 42)
        try:
            nvs.get_blob("n", buf)                # int is not a blob
            self.fail("expected OSError")
        except OSError:
            pass

    def test_native_icm_surface_extras(self):
        import _openbricks_native as n
        self.assertTrue(n.icm45686.available())

    def test_native_selftest_matches_firmware_tuple(self):
        import _openbricks_native as n
        self.assertEqual(tuple(n.icm45686.selftest()),
                         (0, 258, 772, 1286, -2, 300, -5))
