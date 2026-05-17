# SPDX-License-Identifier: MIT
"""
Bench test for a 2× ST-3032 drivebase on a URT-2 adapter.

Parallel to ``st3215_drivebase_test.py`` — same protocol, same
register layout, same ``run_angle`` code path (ST3032Motor is a
marker subclass of ST3215Motor with no behavioural override). The
only differences are the class name and the speed knobs: ST-3032
is a smaller, faster motor (no-load ~300 dps at 12 V; ST-3215 is
slower with more torque), so we set ``SPEED_DPS = 300`` instead
of the conservative 60 dps used for the larger servo.

Hardware:
    * 2× ST-3032 (continuous-rotation, wheel mode), daisy-chained on
      one URT-2. Left at ID=1, right at ID=2 (re-ID with
      ``examples/st3215_reid.py`` if both are factory-default —
      same protocol, the re-ID script works for ST-3032 too).
    * ESP32-S3 GPIO14 → URT-2 RX, GPIO6 → URT-2 TX, common GND.
    * 12 V into URT-2 servo rail (ST-3032 brown-out floor is ~9 V;
      do NOT share a 6 V rail with ST-3215s).

Run with:
    openbricks run -n ls examples/st3032_drivebase_test.py

Chassis geometry — EDIT to your robot. Defaults are placeholders;
on a bench with bare motors the values don't correspond to physical
distance, but the coordinated-motion check still works.
"""

import time

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase


LEFT_ID, RIGHT_ID = 1, 2
UART_ID, TX, RX   = 1, 14, 6

WHEEL_DIAMETER_MM = 65    # EDIT to your wheels
AXLE_TRACK_MM     = 120   # EDIT to your chassis

# Full-driver-cap speed for ST-3032. Per the Feetech STS3032 datasheet
# (``docs/datasheets/feetech_sts3032.pdf`` §5-3), no-load top speed at
# 12 V is 148 RPM = 888 °/s; under rated 3.3 kg·cm load the working
# point drops to ~2/3 of that. The driver's default ``max_dps=600``
# clamps requests at the loaded operating point — instantiate with an
# explicit ``max_dps=900`` if you want to chase the no-load number.
# Bench/chassis speeds are kept conservative regardless: a small
# robot on a table doesn't want 500 mm/s.
SPEED_DPS         = 600
STRAIGHT_SPEED_MM = 150
TURN_RATE_DPS     = 200


def line(msg):
    print(msg)


def main():
    line("--- ST-3032 drivebase bench test (full speed) ---")

    left  = ST3032Motor(servo_id=LEFT_ID,  uart_id=UART_ID, tx=TX, rx=RX)
    right = ST3032Motor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX,
                        invert=True)   # flip if right wheel spins the wrong way

    # --- 1. ping both ---------------------------------------------------
    line("[1] ping both servos ...")
    pl, pr = left.ping(), right.ping()
    line("    left  (id=%d): %s" % (LEFT_ID,  "OK" if pl else "NO RESPONSE"))
    line("    right (id=%d): %s" % (RIGHT_ID, "OK" if pr else "NO RESPONSE"))
    if not (pl and pr):
        line("    Aborting — fix the bus before continuing.")
        return

    # --- 2. baseline angles --------------------------------------------
    line("[2] baseline angles ...")
    al, ar = left.angle(), right.angle()
    line("    left=%s°  right=%s°" % (al, ar))
    if al is None or ar is None:
        line("    angle() returned None — RX path is dead for at least one servo.")
        return

    # --- 3. individual run_speed (exercises each motor in isolation) ---
    line("[3a] left  run_speed(+%d dps) for 1 s ..." % SPEED_DPS)
    left.run_speed(SPEED_DPS); time.sleep(1.0); left.brake()
    line("    left delta = %.1f°" % ((left.angle() or 0) - (al or 0)))

    time.sleep(0.3)

    line("[3b] right run_speed(+%d dps) for 1 s ..." % SPEED_DPS)
    right.run_speed(SPEED_DPS); time.sleep(1.0); right.brake()
    line("    right delta = %.1f°" % ((right.angle() or 0) - (ar or 0)))
    line("    Expect both deltas to be the SAME SIGN (forward for both).")
    line("    If right is opposite, change invert= in this script.")

    time.sleep(0.5)

    # --- 4. coordinated motion via DriveBase ---------------------------
    db = DriveBase(left, right,
                   wheel_diameter_mm=WHEEL_DIAMETER_MM,
                   axle_track_mm=AXLE_TRACK_MM)
    db.settings(straight_speed=STRAIGHT_SPEED_MM, turn_rate=TURN_RATE_DPS)

    line("[4] DriveBase straight(200 mm) at %d mm/s ..." % STRAIGHT_SPEED_MM)
    al_b, ar_b = left.angle(), right.angle()
    db.straight(200)
    al_a, ar_a = left.angle(), right.angle()
    dl, dr = (al_a or 0) - (al_b or 0), (ar_a or 0) - (ar_b or 0)
    # 200 mm at 65 mm wheel diameter → 200 / (π × 65) × 360 ≈ 352.7° per wheel.
    line("    left delta  = %.1f° (target ~%.1f°)" %
         (dl, 200 / (3.14159 * WHEEL_DIAMETER_MM) * 360))
    line("    right delta = %.1f° (target ~%.1f°)" %
         (dr, 200 / (3.14159 * WHEEL_DIAMETER_MM) * 360))
    line("    Expect: left ≈ right (within ~5%%). Mismatch = slip or stuck wheel.")

    time.sleep(0.5)

    # --- 5. turn in place ----------------------------------------------
    line("[5] DriveBase turn(+90°) at %d deg/s ..." % TURN_RATE_DPS)
    al_b, ar_b = left.angle(), right.angle()
    db.turn(90)
    al_a, ar_a = left.angle(), right.angle()
    dl, dr = (al_a or 0) - (al_b or 0), (ar_a or 0) - (ar_b or 0)
    line("    left delta  = %+.1f°" % dl)
    line("    right delta = %+.1f°" % dr)
    line("    Expect: opposite signs (one wheel forward, one reverse).")

    time.sleep(0.5)

    # --- 6. run_angle precision moves on each motor --------------------
    # Same case set as the ST-3215 version. Cases [6d] and [6e] failed
    # by ~+7° on the previous ST-3215 bench (consistent with one
    # ~85-raw post-brake settle artifact); we want to know if the same
    # asymmetry shows up on ST-3032 hardware.
    line("[6a] left  run_angle(%d dps, +90°) ..." % SPEED_DPS)
    a_before = left.angle()
    left.run_angle(SPEED_DPS, 90)
    a_after = left.angle()
    err = (a_after or 0) - (a_before or 0) - 90
    line("    delta = %+.2f°  error = %+.2f° (pass: |err| ≤ 0.5°)" %
         ((a_after or 0) - (a_before or 0), err))

    time.sleep(0.3)

    line("[6b] left  run_angle(%d dps, -90°) — return ..." % SPEED_DPS)
    a_before = left.angle()
    left.run_angle(SPEED_DPS, -90)
    a_after = left.angle()
    err = (a_after or 0) - (a_before or 0) + 90
    line("    delta = %+.2f°  error = %+.2f° (pass: |err| ≤ 0.5°)" %
         ((a_after or 0) - (a_before or 0), err))

    time.sleep(0.3)

    line("[6c] right run_angle(%d dps, +90°) ..." % SPEED_DPS)
    a_before = right.angle()
    right.run_angle(SPEED_DPS, 90)
    a_after = right.angle()
    err = (a_after or 0) - (a_before or 0) - 90
    line("    delta = %+.2f°  error = %+.2f° (pass: |err| ≤ 0.5°)" %
         ((a_after or 0) - (a_before or 0), err))

    time.sleep(0.3)

    line("[6d] right run_angle(%d dps, -90°) — return ..." % SPEED_DPS)
    a_before = right.angle()
    right.run_angle(SPEED_DPS, -90)
    a_after = right.angle()
    err = (a_after or 0) - (a_before or 0) + 90
    line("    delta = %+.2f°  error = %+.2f° (pass: |err| ≤ 0.5°)" %
         ((a_after or 0) - (a_before or 0), err))

    time.sleep(0.3)

    # Multi-chunk: 720° exercises the ≤180° sub-move chunking.
    line("[6e] left  run_angle(%d dps, +720°) — multi-chunk ..." % SPEED_DPS)
    a_before = left.angle()
    left.run_angle(SPEED_DPS, 720)
    a_after = left.angle()
    err = (a_after or 0) - (a_before or 0) - 720
    line("    delta = %+.2f°  error = %+.2f° (pass: |err| ≤ 1°)" %
         ((a_after or 0) - (a_before or 0), err))

    line("--- done ---")


main()
