# SPDX-License-Identifier: MIT
"""Hardware probe: confirm ST3032Motor.run_angle rotates the FULL
commanded angle past one turn now that run_angle drives the servo in
step mode (op_mode=3, signed relative goal) instead of single-turn
position mode (op_mode=0).

Background
----------
In single-turn position mode the goal-position register is an absolute
angle in 0..4095 and the servo physically cannot cross the 0/4095
boundary, so a target that wrapped past it was executed the *wrong way
round* — capping real motion at roughly half a turn. Step mode makes
the goal a signed relative step (negative = reverse) with a ±7-turn
envelope, so a 540°/720° move is one clean relative command. See
docs/datasheets + FeeTech STS tutorial §13.

What to look for
----------------
Each case prints the angle() before and after the move and the error
vs the commanded angle. A correct fix shows |error| within a couple of
degrees on EVERY case, including the ones past 360°. The old code would
stall the >180° cases at ~±180° (large error) or reverse them.

Bench wiring (hub ``yt``): ESP32-S3 GPIO14 → URT-2 TX, GPIO41 → URT-2 RX,
ST3032 at servo_id=1 on the 12 V rail.

    openbricks run -n yt examples/st3032_run_angle_multiturn.py
"""

import time

from openbricks.drivers.st3032 import ST3032Motor


SPEED_DPS = 120
TOL_DEG   = 3.0

m = ST3032Motor(servo_id=1, uart_id=1, tx=14, rx=41)


def probe(label, target_deg, then="hold"):
    m.reset_angle(0)
    time.sleep_ms(50)
    start = m.angle()
    m.run_angle(SPEED_DPS, target_deg, then=then)
    time.sleep_ms(150)
    end = m.angle()
    if start is None or end is None:
        print("  %-28s BUS SILENT (start=%s end=%s)" % (label, start, end))
        return
    moved = end - start
    err = moved - target_deg
    verdict = "PASS" if abs(err) <= TOL_DEG else "FAIL"
    print("  %-28s cmd=%+7.1f  moved=%+8.2f  err=%+6.2f  [%s]"
          % (label, target_deg, moved, err, verdict))


print("\n=== run_angle multi-turn probe (step mode) ===")
print("  servo_id=1  speed=%d dps  tol=%.1f deg\n" % (SPEED_DPS, TOL_DEG))

probe("[1] +90  (sub-turn)",      +90)
probe("[2] -90  (sub-turn)",      -90)
probe("[3] +270 (was capped!)",  +270)
probe("[4] +540 (1.5 turns)",    +540)
probe("[5] -540 (1.5 turns back)", -540)
probe("[6] +720 (2 full turns)", +720)

m.coast()
print("\n--- done (motor coasting) ---")
