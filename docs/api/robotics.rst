``robotics`` — DriveBase
========================

Drive a robot, not two motors: ``DriveBase`` couples a left and a
right motor into one chassis with moves in millimeters and
body-degrees.

.. code-block:: python

    from openbricks.drivers.st3032 import ST3032Motor
    from openbricks.robotics import DriveBase

    left  = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=6)
    right = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=6, invert=True)

    db = DriveBase(left, right, wheel_diameter_mm=88, axle_track_mm=138)
    db.settings(straight_speed=200, turn_rate=150)

    for _ in range(4):          # a 300 mm square
        db.straight(300)
        db.turn(90)

For direct control of each wheel — line-following, tank-style
teleop, or any controller that computes its own per-wheel outputs —
``move_wheels`` takes two speeds in wheel-deg/s:

.. code-block:: python

    db.move_wheels(200, 120)     # gentle right-hand arc
    time.sleep_ms(500)
    db.stop()

Both setpoints leave in a single sync-write packet on serial-bus
motors, so the wheels change speed at the same packet boundary.
Reach for this rather than a :class:`~openbricks.drivers.st3215.SyncServoGroup`
over the wheels: a DriveBase hands their UART to the native bus
driver when it adopts them, so a SyncServoGroup can't drive them at
all.

With an IMU attached, ``use_gyro(True)`` steers by measured body
rotation instead of the encoder differential — immune to wheel slip:

.. code-block:: python

    from machine import I2C, Pin
    from openbricks.drivers.bno055 import BNO055

    imu = BNO055(i2c=I2C(0, sda=Pin(15), scl=Pin(16), freq=400_000))
    db = DriveBase(left, right, wheel_diameter_mm=88,
                   axle_track_mm=138, imu=imu)
    db.use_gyro(True)

Accurate ``wheel_diameter_mm`` / ``axle_track_mm`` values matter more
than any tuning — calibrate both with two short test drives:
:doc:`/measuring`.

.. automodule:: openbricks.robotics.drivebase
   :members:
   :undoc-members:
   :show-inheritance:
