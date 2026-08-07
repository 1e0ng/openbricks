# SPDX-License-Identifier: MIT
"""
First MOTION through the native serial-bus path — wheels-up test.

What this proves when it passes: speed commands staged from Python
reach the servos via the hard-tick pump's sync-writes, odometry
accumulates from the pump's position reads, and coast stops the
wheel — all with the control traffic below the Python scheduler.

SAFETY: put the robot on a stand, WHEELS OFF THE GROUND. Speeds
honour the bench cap (<=120 dps). The stop button works: the e-stop
broadcast reaches the native bus (1.41.0+).

Run:
    openbricks run -n ls examples/native_bus_first_motion.py
"""

import time

from openbricks._native import motor_process, st_bus

LEFT_SLOT, LEFT_ID   = 0, 2
RIGHT_SLOT, RIGHT_ID = 1, 1
UART_ID, BAUD, TX, RX = 1, 1_000_000, 14, 6

STEPS_PER_DPS = 4096 / 360.0
TEST_DPS      = 100
GOAL_ACC      = 45


def main():
    motor_process.hard_tick_selftest()
    if not st_bus.attach_uart(UART_ID, BAUD, TX, RX):
        raise RuntimeError("attach_uart failed")
    assert st_bus.servo_attach(LEFT_SLOT, LEFT_ID, True, GOAL_ACC)
    assert st_bus.servo_attach(RIGHT_SLOT, RIGHT_ID, False, GOAL_ACC)
    time.sleep_ms(200)

    steps = int(TEST_DPS * STEPS_PER_DPS)
    print("forward %d dps (%d steps/s) for 2 s ..." % (TEST_DPS, steps))
    c0 = (st_bus.servo_counts(LEFT_SLOT), st_bus.servo_counts(RIGHT_SLOT))
    st_bus.servo_run(LEFT_SLOT, steps)
    st_bus.servo_run(RIGHT_SLOT, steps)
    time.sleep_ms(2000)
    st_bus.servo_coast(LEFT_SLOT)
    st_bus.servo_coast(RIGHT_SLOT)
    time.sleep_ms(300)
    c1 = (st_bus.servo_counts(LEFT_SLOT), st_bus.servo_counts(RIGHT_SLOT))

    moved = (c1[0] - c0[0], c1[1] - c0[1])
    expect = int(TEST_DPS * STEPS_PER_DPS * 2)
    print("moved counts: L=%+d R=%+d (expect ~%d each, +/- ramp)"
          % (moved[0], moved[1], expect))
    print("read stats:  L=%s R=%s" % (st_bus.servo_stats(LEFT_SLOT),
                                      st_bus.servo_stats(RIGHT_SLOT)))
    print("bus stats:   ok/timeout/bad = %s" % (st_bus.stats()[:3],))

    ok = (moved[0] > expect // 2 and moved[1] > expect // 2)
    print("RESULT: %s" % ("PASS — both wheels moved forward the "
                          "expected order of magnitude" if ok else
                          "FAIL — see counts/stats above"))


main()
