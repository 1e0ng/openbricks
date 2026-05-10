# SPDX-License-Identifier: MIT
"""
Smoke test for a 2x ST-3215-C018 drivebase.

Drives forward 200 mm, brakes, prints both wheels' final angles. Use
it to verify the ST-3215 wheel-mode pipeline (driver + DriveBase) on
the bench.

Hardware:
    * 2x ST-3215-C018 (continuous-rotation variant) on one TTL bus
    * ESP32-S3 on UART1 (TX=17, RX=16) — adjust if your board is
      wired differently. Some adapters need a direction-enable GPIO
      for true half-duplex; pass ``dir_pin=...`` if so.
    * 7.4V supply (2S Li-ion or 7.4V LiPo) on the servo VCC rail.
      The ESP32 logic side stays at 3.3V — the bus board takes care
      of level shifting.
    * Servo IDs: assign each servo a unique ID using a Feetech
      programmer / FD-debug tool before wiring them onto a shared
      bus. Defaults to ID 1 (left) and ID 2 (right) below.

Edit the IDs / UART pins below to match your wiring:
"""

from openbricks.drivers.st3215 import ST3215Motor
from openbricks.robotics import DriveBase
from openbricks.tools import wait

LEFT_ID, RIGHT_ID = 1, 2
UART_ID, TX, RX   = 1, 17, 16

left = ST3215Motor(
    servo_id=LEFT_ID,
    uart_id=UART_ID, tx=TX, rx=RX,
)
right = ST3215Motor(
    servo_id=RIGHT_ID,
    uart_id=UART_ID, tx=TX, rx=RX,
    invert=True,        # right motor mirrors left mounting; flip if it spins backwards
)

# wheel_diameter / axle_track in mm — EDIT to your chassis.
db = DriveBase(left, right, wheel_diameter_mm=65, axle_track_mm=120)
db.settings(straight_speed=80, turn_rate=90)

print("ping left  =", left.ping())
print("ping right =", right.ping())

print("before: left=%.1f° right=%.1f°" % (left.angle() or 0.0,
                                          right.angle() or 0.0))
db.straight(200)
wait(200)
print("after:  left=%.1f° right=%.1f°" % (left.angle() or 0.0,
                                          right.angle() or 0.0))
print("done.")
