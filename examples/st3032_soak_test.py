# SPDX-License-Identifier: MIT
"""ST-3032 soak test — run a servo for an hour and watch it age.

Before a competition, or after a crash: one servo on the bus swings
+/-90 deg at a gentle speed for the chosen duration, and every
REPORT_EVERY cycles it logs supply voltage, temperature, current and
its own gear play (one target, approached from below and from above
under hold). The summary at the end is the drift: a temperature that
keeps climbing, or play that grows through the run, is a servo to
retire before it retires you on the mat.

    openbricks upload -n <hub> examples/st3032_soak_test.py
    openbricks log -n <hub>
"""

import time

from openbricks.drivers.st3032 import ST3032Motor
from openbricks.parameters import Stop

UART_ID, TX, RX = 1, 14, 41
SERVO_ID = 3
DURATION_MIN = 60
SPEED_DPS = 120
SWING_DEG = 90
PLAY_APPROACH_DEG = 20
REPORT_EVERY = 20


# --- soak summary ---
def play_deg(from_below, from_above):
    """Gear play: the rest angle reached approaching one target from
    above minus the one reached approaching it from below."""
    return abs(from_above - from_below)


def soak_summary(rows):
    """``rows`` = (t_min, voltage, temperature, current, play_deg) in
    time order. The drift from the first report to the last, the
    peak temperature and the lowest supply voltage seen."""
    first = rows[0]
    last = rows[-1]
    return {
        "temp_rise": last[2] - first[2],
        "temp_peak": max(r[2] for r in rows),
        "current_change": last[3] - first[3],
        "play_change": last[4] - first[4],
        "voltage_min": min(r[1] for r in rows),
    }
# --- end soak summary ---


m = ST3032Motor(servo_id=SERVO_ID, uart_id=UART_ID, tx=TX, rx=RX)

print("t_min,voltage_v,temperature_c,current_a,play_deg,cycles")
rows = []
start = time.ticks_ms()
cycles = 0
while time.ticks_diff(time.ticks_ms(), start) < DURATION_MIN * 60000:
    m.run_angle(SPEED_DPS, SWING_DEG)
    m.run_angle(SPEED_DPS, -SWING_DEG)
    cycles += 1
    if cycles % REPORT_EVERY:
        continue
    m.run_angle(SPEED_DPS, -PLAY_APPROACH_DEG)
    m.run_angle(SPEED_DPS, PLAY_APPROACH_DEG, then=Stop.HOLD)
    time.sleep_ms(300)
    below = m.angle()
    m.run_angle(SPEED_DPS, PLAY_APPROACH_DEG)
    m.run_angle(SPEED_DPS, -PLAY_APPROACH_DEG, then=Stop.HOLD)
    time.sleep_ms(300)
    above = m.angle()
    if below is None or above is None:
        raise OSError("servo %d went silent during the play check"
                      % SERVO_ID)
    h = m.health()
    t_min = time.ticks_diff(time.ticks_ms(), start) / 60000.0
    rows.append((t_min, h.voltage, h.temperature, h.current,
                 play_deg(below, above)))
    print("%.1f,%.1f,%.0f,%.3f,%.2f,%d"
          % (t_min, h.voltage, h.temperature, h.current,
             rows[-1][4], cycles))
    if h.flags:
        print("protection flags: %s" % ",".join(h.flags))
m.coast()

s = soak_summary(rows)
print("# --- soak summary ---")
print("%d cycles: temperature %+.0f C (peak %.0f C, shutdown at 80), "
      "current %+.3f A, play %+.2f deg, supply dipped to %.1f V"
      % (cycles, s["temp_rise"], s["temp_peak"], s["current_change"],
         s["play_change"], s["voltage_min"]))
