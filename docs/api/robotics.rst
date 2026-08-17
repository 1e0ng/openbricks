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

Moves block by default. Pass ``wait=False`` to return immediately
and poll ``done()`` — the Pybricks pattern for driving while
reading sensors; any new move command supersedes the pending one::

    db.straight(600, wait=False)
    while not db.done():
        if bumper_pressed():
            db.stop()
            break
        time.sleep_ms(10)

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

Moves take a ``then=`` end state: ``"stop"`` (alias ``"coast"``,
the default) decelerates to rest and free-wheels; ``"brake"`` /
``"hold"`` end actively. ``then="continue"`` on ``straight`` and
``curve`` — Pybricks ``Stop.NONE`` — does NOT decelerate at the
end: the move finishes at cruise speed and the wheels keep it until
the next command, so chained segments flow through their seams::

    db.straight(300, then="continue")   # ends AT cruise
    db.curve(150, 90, then="continue")  # picks the speed up
    db.straight(300)                    # decelerates to rest

Move endings are SHAPED all the way down (2.6.0): the controller
runs position integral action (pbio's integrator rules — the same
control law Pybricks uses) so tracking error is squeezed out near
the target, and any residual left when a profile expires is closed
by a small landing trajectory under the same acceleration limit as
every other motion — never a raw feedback step. A robot that ends a
mission simply comes to rest on its mark; a genuinely stuck robot
still refuses to report ``done()`` and the stall watchdog raises.

``stop()`` is Pybricks parity: it coasts and returns immediately.
``then`` picks the end state (``"coast"``, ``"brake"``,
``"hold"``). Short moves armed while the robot is already fast
raise their own deceleration to land at rest exactly on target, so
you rarely need more — but ``wait=True`` is available to block
until both wheels' measured speeds read ~0 (the decel ramp plus
settle for brake/hold, the physical freewheel decay for coast). It
raises ``ValueError`` on open-loop pairs (no measured speed) and,
if the wheels never settle within 5 s, ``RuntimeError`` naming the
measured speeds — a stopped robot that is still moving is a fault,
not a detail to hide.

A wheel that stops answering the bus — no power, a knocked-loose
TX/RX wire, the wrong ``servo_id`` — raises instead of quietly doing
nothing, and the error names the motor:

.. code-block:: text

    OSError: motor is not responding on the bus: right wheel
    (servo id 1, slot 1) on UART1 tx=14 rx=6 — 0 replies, 137 failed
    reads (137 in a row). Check the servo's power and TX/RX wiring,
    and that it really has that bus id — `openbricks servo-id --scan`
    lists the ids actually answering on the bus.

(``openbricks servo-id`` talks through the URT-2's USB port. With
the servo already wired to the hub,
``openbricks run -n NAME examples/servo_set_id.py`` scans and
re-IDs through the hub instead — same safety contract.)

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

(A legacy :class:`~openbricks.drivers.bno055.BNO055` on I2C still
works — its fused heading is pumped from Python between ticks,
which corrects noticeably slower, typically +0.5° to +1.8° per
turn. New builds should use the ICM-45686.)

To re-zero the heading frame mid-mission (say, after squaring up on
a line), call ``db.reset()`` between moves — afterwards the robot's
CURRENT pose is heading zero for both the drive base and
``imu.heading()``, atomically:

.. code-block:: python

    db.straight(100)
    db.turn(-90)
    db.reset()          # here, now = heading zero
    db.straight(130)    # drives straight along the NEW zero

``imu.reset_heading()`` refuses (``OSError``) while a drive base
steers by the gyro — same rule as Pybricks ("can't reset heading
while gyro in use"): zeroing the integrator under an armed heading
controller shifts the measurement out from under the held target,
and the next move veers chasing the old frame. Use ``db.reset()``,
or ``use_gyro(False)`` first. ``db.reset()`` itself raises while a
move is in progress — stop first.

Accurate ``wheel_diameter_mm`` / ``axle_track_mm`` values matter more
than any tuning — calibrate both with two short test drives:
:doc:`/measuring`.

.. automodule:: openbricks.robotics.drivebase
   :members:
   :undoc-members:
   :show-inheritance:
