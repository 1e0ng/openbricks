# SPDX-License-Identifier: MIT
"""
Instrument the fallback DriveBase tick during one long straight.

Bench observation (2026-07-25, real 88 mm chassis): drivebase motion
feels non-continuous ("start-stop"). Raising cruise from 80 to 200
wheel-deg/s helped but didn't eliminate it, so low-speed stick-slip
isn't the whole story — this probe measures what the control loop
is actually doing instead of guessing further:

  * Poll-to-poll period of the ``done()`` loop, bucketed. Each poll
    runs ONE fallback tick = two servo angle reads + two speed
    writes over the half-duplex bus (~10 ms sleep + bus time). A
    healthy loop sits in the 10-30 ms buckets; entries in >60 ms
    mean a bus read blocked on its timeout — long enough to feel as
    a hitch if the servo coasts through it.
  * Dropped (None) angle reads per motor — how often the bus
    actually failed vs merely ran slow.
  * Min/max speed commands sent — reveals whether the commanded
    speed itself oscillates (correction fighting) or stays flat
    while the MOTION stutters (pointing below the loop, at the
    servo/mechanics). Note the min will show the 15 dps launch
    floor from the profile ramp; that's expected.

Run with:
    openbricks run -n ls examples/st3032_drivebase_tick_probe.py

Give it ~1 m of clear floor (DISTANCE_MM below).
"""

import time

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase


LEFT_ID, RIGHT_ID = 2, 1
UART_ID, TX, RX   = 1, 14, 6

WHEEL_DIAMETER_MM  = 88
AXLE_TRACK_MM      = 138
STRAIGHT_SPEED_DPS = 200
DISTANCE_MM        = 600


class ProbeMotor(ST3032Motor):
    """ST3032Motor that counts failed angle reads and tracks the
    range of speed commands it was given."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.none_reads = 0
        self.cmds = 0
        self.cmd_min = None
        self.cmd_max = None

    def angle(self):
        a = super().angle()
        if a is None:
            self.none_reads += 1
        return a

    def run_speed(self, dps):
        self.cmds += 1
        m = abs(dps)
        if self.cmd_min is None or m < self.cmd_min:
            self.cmd_min = m
        if self.cmd_max is None or m > self.cmd_max:
            self.cmd_max = m
        return super().run_speed(dps)


def main():
    left  = ProbeMotor(servo_id=LEFT_ID,  uart_id=UART_ID, tx=TX, rx=RX,
                       invert=True)
    right = ProbeMotor(servo_id=RIGHT_ID, uart_id=UART_ID, tx=TX, rx=RX)
    db = DriveBase(left, right,
                   wheel_diameter_mm=WHEEL_DIAMETER_MM,
                   axle_track_mm=AXLE_TRACK_MM)
    db.settings(straight_speed=STRAIGHT_SPEED_DPS)

    print("--- tick probe: straight(%d mm) at %d wheel-dps ---"
          % (DISTANCE_MM, STRAIGHT_SPEED_DPS))

    db.straight(DISTANCE_MM, wait=False)
    # <15 / 15-30 / 30-60 / >60 ms buckets.
    buckets = [0, 0, 0, 0]
    max_period = 0
    t_prev = time.ticks_ms()
    while not db.done():
        time.sleep_ms(10)
        now = time.ticks_ms()
        p = time.ticks_diff(now, t_prev)
        t_prev = now
        if p > max_period:
            max_period = p
        if p < 15:
            buckets[0] += 1
        elif p < 30:
            buckets[1] += 1
        elif p < 60:
            buckets[2] += 1
        else:
            buckets[3] += 1

    print("tick period ms   <15: %d   15-30: %d   30-60: %d   >60: %d   max: %d"
          % (buckets[0], buckets[1], buckets[2], buckets[3], max_period))
    print("dropped angle reads   left: %d   right: %d"
          % (left.none_reads, right.none_reads))
    print("speed cmds   left  n=%d min=%.0f max=%.0f" %
          (left.cmds, left.cmd_min or 0.0, left.cmd_max or 0.0))
    print("             right n=%d min=%.0f max=%.0f" %
          (right.cmds, right.cmd_min or 0.0, right.cmd_max or 0.0))
    print("--- done ---")


main()
