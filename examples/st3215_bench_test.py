# SPDX-License-Identifier: MIT
"""Bench test for one ST-3215 (or ST-3032) on a URT-2 adapter.

Wiring assumed:
    ESP32-S3 GPIO14  ── URT-2 RX  (i.e. data INTO URT-2)
    ESP32-S3 GPIO41  ── URT-2 TX  (i.e. data OUT of URT-2)
    ESP32-S3 GND     ── URT-2 GND ── servo GND
    URT-2 12 V in    ── bench supply
    Servo 3-pin      ── URT-2 servo header

If `ping` fails, try swapping GPIO14 ↔ GPIO41 — the URT-2 silkscreen
sometimes labels TX/RX from the board's perspective rather than the
MCU's.

Run with:
    openbricks run examples/st3215_bench_test.py
"""

import time

from openbricks.drivers.st3215 import ST3215Motor


SERVO_ID = 1
TX_PIN   = 14
RX_PIN   = 41


def line(msg):
    print(msg)


def main():
    line("--- ST-3215 bench test ---")

    m = ST3215Motor(servo_id=SERVO_ID, uart_id=1, tx=TX_PIN, rx=RX_PIN)

    line("[1] ping servo id=%d ..." % SERVO_ID)
    if not m.ping():
        line("    NO RESPONSE — check wiring / 12 V / servo ID / baud.")
        return
    line("    OK")

    line("[2] baseline angle ...")
    a0 = m.angle()
    line("    angle = %s deg" % a0)
    if a0 is None:
        line("    angle() returned None — bus is one-way (TX works, RX doesn't).")
        line("    Most likely: GPIO14/GPIO41 swapped, or no common ground.")
        return

    line("[3] run_speed(+60 dps) for 1 s ...")
    m.run_speed(60)
    time.sleep(1.0)
    m.brake()
    a1 = m.angle()
    line("    angle now = %s deg (delta = %.1f)" % (a1, (a1 or 0) - (a0 or 0)))
    line("    Expect: ~+60° (positive direction).")

    time.sleep(0.3)

    line("[4] run_speed(-60 dps) for 1 s ...")
    m.run_speed(-60)
    time.sleep(1.0)
    m.brake()
    a2 = m.angle()
    line("    angle now = %s deg (delta from a1 = %.1f)" % (a2, (a2 or 0) - (a1 or 0)))
    line("    Expect: ~-60°.")

    time.sleep(0.3)

    line("[5] run_angle(60 dps, +90°) — precision forward ...")
    a_before = m.angle()
    m.run_angle(60, 90)
    a_after = m.angle()
    delta = (a_after or 0) - (a_before or 0)
    line("    delta = %.2f°  (target +90°, error %.2f°)" % (delta, delta - 90))

    time.sleep(0.3)

    line("[6] run_angle(60 dps, -90°) — precision return ...")
    a_before = m.angle()
    m.run_angle(60, -90)
    a_after = m.angle()
    delta = (a_after or 0) - (a_before or 0)
    line("    delta = %.2f°  (target -90°, error %.2f°)" % (delta, delta + 90))

    time.sleep(0.3)

    line("[7] run_angle(120 dps, +720°) — two-rev, multi-chunk ...")
    a_before = m.angle()
    m.run_angle(120, 720)
    a_after = m.angle()
    delta = (a_after or 0) - (a_before or 0)
    line("    delta = %.2f°  (target +720°, error %.2f°)" % (delta, delta - 720))

    line("--- done ---")
    line("Pass criteria:")
    line("  [5][6] error within ±0.5° (matches LEGO Prime).")
    line("  [7]    error within ±1° (chunk-handoff transient may add ~0.1°/chunk).")


main()
