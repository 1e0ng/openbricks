# SPDX-License-Identifier: MIT
"""
Example: drive forward 500 mm, turn 90°, repeat.

Hardware:
    * ESP32
    * 2x JGB37-520 motors with quadrature encoders
    * 1x L298N dual H-bridge (powers both motors)
    * Two 18650 cells or similar 7.4V battery on the L298N motor supply

Wiring (edit the GPIOs to match your board):

    Left motor  (L298N channel A)
        IN1  -> GPIO 25
        IN2  -> GPIO 26
        ENA  -> GPIO 27   (PWM)
        ENC A-> GPIO 32
        ENC B-> GPIO 33

    Right motor (L298N channel B)
        IN3  -> GPIO 14
        IN4  -> GPIO 12
        ENB  -> GPIO 13   (PWM)
        ENC A-> GPIO 34   (input-only pin is fine)
        ENC B-> GPIO 35

The closed-loop control loop runs on the shared ``MotorProcess`` scheduler,
driven by ``machine.Timer`` at 100 Hz. ``DriveBase.straight`` / ``.turn``
start and stop the scheduler around each move automatically, but you can
``.start()`` it once for the whole program if you prefer one timer warm the
entire time — which also lets you mix scheduler-driven ``drive()`` calls
with blocking ``straight()`` / ``turn()`` seamlessly.
"""

from openbricks.drivers.jgb37_520 import JGB37Motor
from openbricks.robotics import DriveBase
from openbricks.tools import wait
from openbricks.tools.scheduler import MotorProcess


left = JGB37Motor(
    in1=25, in2=26, pwm=27,
    encoder_a=32, encoder_b=33,
    counts_per_output_rev=1320,  # 11 CPR * 30:1 gearbox * 4 (quadrature)
)
right = JGB37Motor(
    in1=14, in2=12, pwm=13,
    encoder_a=34, encoder_b=35,
    counts_per_output_rev=1320,
    invert=True,  # mirror image of the left motor
)

db = DriveBase(left, right, wheel_diameter_mm=65, axle_track_mm=120)
db.settings(straight_speed=200, turn_rate=120)

# Keep the control loop running across all moves so the scheduler doesn't
# thrash its timer between each call. ``stop()`` at the end returns the
# hardware timer so nothing else contends for it.
scheduler = MotorProcess.instance()
scheduler.start()
try:
    for _ in range(4):
        db.straight(500)
        wait(200)
        db.turn(90)
        wait(200)
finally:
    scheduler.stop()
