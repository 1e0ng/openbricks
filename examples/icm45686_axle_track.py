# SPDX-License-Identifier: MIT
"""
Axle-track calibration — the gyro replaces the protractor.

Encoder-only turns convert wheel travel into body rotation through
``axle_track_mm``, so an error there lands in every turn. This
script spins TURNS full turns on encoder math alone (the
drivebase's gyro stays off) while the ICM-45686 independently
measures how far the robot really rotated, then prints the
corrected track. Overshooting the mark means the real track is
SMALLER than configured; undershooting means larger. Ten turns
make each 0.1 degree of gyro error only 0.003% of track.

Calibrate wheel diameter first (:doc:`/measuring`), put the number
this prints into your ``DriveBase(...)`` call, and re-run to
confirm the measured rotation lands on the commanded one.

Run:
    openbricks run -n <hub> examples/icm45686_axle_track.py
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
TURN_RATE = 120

# --- control law (pure logic, unit-tested in tests/test_axle_track_cal.py) ---

TURNS = 10


def corrected_track(track_mm, commanded_deg, measured_deg):
    return track_mm * commanded_deg / measured_deg

# --- end control law ---


print("hold still for gyro bias lock ...")
imu = ICM45686(sck=SCK, mosi=MOSI, miso=MISO, cs=CS)
while not imu.calibrated():
    time.sleep_ms(100)

left = ST3032Motor(servo_id=LEFT_ID, uart_id=UART_ID, tx=TX, rx=RX,
                   invert=True)
right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX)
db = DriveBase(left, right, wheel_diameter_mm=WHEEL_DIAMETER_MM,
               axle_track_mm=AXLE_TRACK_MM, imu=imu)
db.settings(turn_rate=TURN_RATE)

imu.reset_heading()
print("spinning %d turns on encoder math alone ..." % TURNS)
db.turn(360 * TURNS)
db.stop()
time.sleep_ms(500)

commanded = 360 * TURNS
measured = imu.heading()
print("commanded %d deg, gyro measured %.1f deg" % (commanded, measured))
print("axle_track_mm: %.1f -> %.2f" % (
    AXLE_TRACK_MM, corrected_track(AXLE_TRACK_MM, commanded, measured)))
