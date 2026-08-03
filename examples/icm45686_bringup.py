# SPDX-License-Identifier: MIT
"""
ICM-45686 bring-up (first silicon contact).

Wire the breakout to 4 free GPIOs (defaults below — edit to your
wiring), power at 3.3 V, then run this. It verifies each layer in
order and prints the evidence:

  1. WHO_AM_I answers 0xE9 (construction raises loudly if not —
     try ``mode=0`` if you get a mismatch; SPI mode is the one
     reference-derived guess in the driver).
  2. Accel sanity: at rest expect ~(0, 0, +1 g) — confirms burst
     byte order on real silicon.
  3. Gyro Z sign: rotate the robot CLOCKWISE (viewed from above)
     during the window; heading must INCREASE (CW-positive). If it
     decreases, construct with ``scale=+1.0`` (mounted upside down).
  4. Bias lock: leave it still and watch calibrated() flip True.
  5. Heading stream while you rotate by hand — compare against a
     protractor turn (~360 commanded by hand should read ~360).

Run:
    openbricks run -n ls examples/icm45686_bringup.py
"""

import time

from openbricks.drivers.icm45686 import ICM45686

SCK, MOSI, MISO, CS = 8, 9, 17, 18      # EDIT to your wiring

imu = ICM45686(sck=SCK, mosi=MOSI, miso=MISO, cs=CS)
print("WHO_AM_I ok (construction verified it)")
print("stats:", imu.stats())

time.sleep_ms(300)
ax, ay, az = imu.acceleration()
print("accel at rest (want ~0, ~0, ~+1 g): (%.2f, %.2f, %.2f)"
      % (ax, ay, az))

print()
print(">>> keep it STILL ~2 s for bias lock ...")
for i in range(30):
    time.sleep_ms(100)
    if imu.calibrated():
        break
print("calibrated:", imu.calibrated())

print()
print(">>> rotate CLOCKWISE (top view) in the next 15 s;")
print(">>> heading should INCREASE. Streaming at 200 ms:")
h0 = imu.heading()
for i in range(75):
    time.sleep_ms(200)
    gz = imu.gyro()[2]
    print("%2d heading %+8.1f  gyro_z %+7.1f dps  stats %s"
          % (i, imu.heading() - h0, gz, imu.stats()))

if imu.calibrated():
    imu.save_calibration()
    print("calibration saved to NVS — next boot seeds from it")
print("--- done. Paste this output back. ---")
