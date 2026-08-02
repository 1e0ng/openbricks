# SPDX-License-Identifier: MIT
"""
Native-drivebase square — the arc's floor test.

Drives a 150 mm square through the RAW ``st_bus.db_*`` bindings — the
same 2-DOF controller ``DriveBase`` runs, but with no Python class in
between: useful for isolating a bug below the robotics layer. For
normal use construct ``DriveBase`` with ST3032Motor objects instead
(see ``st3032_drivebase_gyro_test.py``); it adopts the motors onto
this controller automatically.

Speeds honour the bench cap (straight 60 mm/s ~= 78 wheel-dps,
turn 45 body-dps). The stop button works throughout (e-stop
broadcasts reach the native bus).

Run:
    openbricks run -n ls examples/native_drivebase_square.py
"""

import time

from openbricks._native import motor_process, st_bus

LEFT_SLOT, LEFT_ID   = 0, 2      # bench mapping
RIGHT_SLOT, RIGHT_ID = 1, 1
UART_ID, BAUD, TX, RX = 1, 1_000_000, 14, 6

WHEEL_MM, AXLE_MM = 88.0, 138.0
SIDE_MM      = 150.0
SPEED_MM_S   = 60.0              # <= bench cap
TURN_DPS     = 45.0
ACCEL_DPS2   = 400.0


def wait_done(timeout_ms=10000):
    t0 = time.ticks_ms()
    while not st_bus.db_done():
        if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
            raise RuntimeError("move did not complete")
        time.sleep_ms(20)


def main():
    motor_process.hard_tick_selftest()
    if not st_bus.attach_uart(UART_ID, BAUD, TX, RX):
        raise RuntimeError("attach_uart failed")
    assert st_bus.servo_attach(LEFT_SLOT, LEFT_ID, True, 45)
    assert st_bus.servo_attach(RIGHT_SLOT, RIGHT_ID, False, 45)
    time.sleep_ms(200)
    st_bus.db_config(LEFT_SLOT, RIGHT_SLOT, WHEEL_MM, AXLE_MM, ACCEL_DPS2)

    mm = (WHEEL_MM * 3.14159265) / 4096
    l0 = st_bus.servo_counts(LEFT_SLOT)
    r0 = st_bus.servo_counts(RIGHT_SLOT)

    for side in range(4):
        print("side %d ..." % (side + 1))
        st_bus.db_straight(SIDE_MM, SPEED_MM_S)
        wait_done()
        st_bus.db_turn(90.0, TURN_DPS)
        wait_done()

    st_bus.db_stop()
    time.sleep_ms(300)
    dl = (st_bus.servo_counts(LEFT_SLOT) - l0) * mm
    dr = (st_bus.servo_counts(RIGHT_SLOT) - r0) * mm
    # A closed square: each wheel's net distance = 4 sides + 4 quarter
    # turns; the turns cancel between wheels, so dl - dr ~= 2 * full
    # turn arc and dl + dr ~= 2 * 4 * SIDE.
    print("net wheel travel: L=%.1f mm R=%.1f mm" % (dl, dr))
    print("closure (sum vs 4 sides x2): %.1f mm vs %.1f mm"
          % (dl + dr, 2 * 4 * SIDE_MM))
    print("stats: L=%s R=%s bus=%s" % (
        st_bus.servo_stats(LEFT_SLOT), st_bus.servo_stats(RIGHT_SLOT),
        st_bus.stats()[:3]))
    print("done — measure the robot's return-to-start error on the "
          "floor and compare with the Python-fallback square.")


main()
