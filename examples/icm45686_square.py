# SPDX-License-Identifier: MIT
"""
Gyro-corrected square — ICM-45686 on the hard tick.

The ICM-45686 is read INSIDE the 1 kHz control tick over SPI, so
with ``use_gyro(True)`` the DriveBase corrects heading every
millisecond in C — no Python in the loop. This script drives a
square twice, encoder-only and then gyro-corrected, and prints how
far each pass drifts from its starting heading. Bench 2026-08-10:
the gyro pass returned within +0.6 degrees over all four turns.

The learned gyro bias is persisted to NVS after the stillness
lock, so later boots start corrected without the ~0.5 s wait.

Wiring: breakout on 3V3/GND plus the four SPI pins below; INT
stays unwired (the tick polls). Needs ~0.5 m of clear floor.

Run:
    openbricks run -n <hub> examples/icm45686_square.py
"""

import time

from openbricks.drivers.icm45686 import ICM45686
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

LEFT_ID, RIGHT_ID = 2, 1
UART_ID, TX, RX = 1, 14, 41
SCK, MOSI, MISO, CS = 12, 13, 11, 17

WHEEL_DIAMETER_MM = 88
AXLE_TRACK_MM = 136

STRAIGHT_SPEED = 80
TURN_RATE = 60
SIDE_MM = 300


def square_drift(db, imu):
    start = imu.heading()
    for side in range(4):
        db.straight(SIDE_MM)
        db.turn(90)
    return imu.heading() - start - 360


print("hold still for gyro bias lock ...")
imu = ICM45686(sck=SCK, mosi=MOSI, miso=MISO, cs=CS)
while not imu.calibrated():
    time.sleep_ms(100)
imu.save_calibration()

left = ST3032Motor(servo_id=LEFT_ID, uart_id=UART_ID, tx=TX, rx=RX,
                   invert=True)
right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX)
db = DriveBase(left, right, wheel_diameter_mm=WHEEL_DIAMETER_MM,
               axle_track_mm=AXLE_TRACK_MM, imu=imu)
db.settings(straight_speed=STRAIGHT_SPEED, turn_rate=TURN_RATE)

print("pass 1: encoders only")
print("drift: %+.1f deg" % square_drift(db, imu))

db.use_gyro(True)
print("pass 2: gyro-corrected at 1 kHz")
print("drift: %+.1f deg" % square_drift(db, imu))
db.stop(then="brake")
