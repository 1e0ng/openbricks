# SPDX-License-Identifier: MIT
"""Drive a rounded square with DriveBase.curve().

Four straights joined by four quarter-circle arcs — the robot never
stops to pivot, so the lap is faster and smoother than the
straight+turn square. curve(radius, angle) follows Pybricks: positive
angle arcs right, the radius sign picks forward/backward, and the
outer wheel is automatically capped at the straight_speed setting.

Run with:
    openbricks run -n ls examples/st3032_drivebase_curve.py
"""

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

LEFT_ID, RIGHT_ID = 2, 1
UART_ID, TX, RX = 1, 14, 41

WHEEL_DIAMETER_MM = 88
AXLE_TRACK_MM = 136

SIDE_MM = 150
RADIUS_MM = 60
NUM_LAPS = 1

left = ST3032Motor(servo_id=LEFT_ID, uart_id=UART_ID, tx=TX, rx=RX,
                   invert=True)
right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX)
db = DriveBase(left, right,
               wheel_diameter_mm=WHEEL_DIAMETER_MM,
               axle_track_mm=AXLE_TRACK_MM)
db.settings(straight_speed=150, turn_rate=200)

print("rounded square: %d mm sides, %d mm corner radius" %
      (SIDE_MM, RADIUS_MM))
for lap in range(NUM_LAPS):
    for side in range(4):
        db.straight(SIDE_MM)
        db.curve(radius=RADIUS_MM, angle=90)
print("done")
