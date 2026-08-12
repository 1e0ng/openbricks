``robotics`` — DriveBase
========================

Drive a robot, not two motors: ``DriveBase`` couples a left and a
right motor into one chassis with moves in millimeters and
body-degrees.

.. code-block:: python

    from openbricks.drivers.st3032 import ST3032Motor
    from openbricks.robotics import DriveBase

    left  = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)
    right = ST3032Motor(servo_id=2, uart_id=1, tx=14, rx=41, invert=True)

    db = DriveBase(left, right, wheel_diameter_mm=88, axle_track_mm=138)
    db.settings(straight_speed=250, turn_rate=200,
                acceleration=1000, turn_acceleration=800)

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

A wheel that stops answering the bus — no power, a knocked-loose
TX/RX wire, the wrong ``servo_id`` — raises instead of quietly doing
nothing, and the error names the motor:

.. code-block:: text

    OSError: motor is not responding on the bus: right wheel
    (servo id 1, slot 1) on UART1 tx=14 rx=6 — 0 replies, 137 failed
    reads (137 in a row). Check the servo's power and TX/RX wiring,
    and that it really has that bus id — `openbricks servo-id --scan`
    lists the ids actually answering on the bus.

Both wheels are verified when the DriveBase is constructed, and on
every move afterwards. If one goes silent mid-move the controller
halts immediately rather than winding that wheel's command to the
rail — a frozen odometry reading would otherwise look like "infinite
error" to the heading loop. ``db.check_motors()`` runs the same check
on demand.

With an IMU attached, ``use_gyro(True)`` steers by measured body
rotation instead of the encoder differential — immune to wheel slip.
The preferred IMU is the :class:`~openbricks.drivers.icm45686.ICM45686`:
it is read *inside* the 1 kHz control tick over SPI, so the heading
correction runs every millisecond in C with no Python in the loop
(bench: +0.6° total drift over a four-turn square):

.. code-block:: python

    from openbricks.drivers.icm45686 import ICM45686

    imu = ICM45686(sck=12, mosi=13, miso=11, cs=17)
    db = DriveBase(left, right, wheel_diameter_mm=88,
                   axle_track_mm=138, imu=imu)
    db.use_gyro(True)

A :class:`~openbricks.drivers.bno055.BNO055` on the I2C bus works
too — its fused heading is pumped from Python between ticks, which
is accurate but slower to correct (typically +0.5° to +1.8° per
turn):

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
