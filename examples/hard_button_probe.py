# SPDX-License-Identifier: MIT
"""
Hard-button diagnostic probe (bench 2026-08-03: stop works, but the
stop line read hard-button stats (0, 0, 0, False) — the hard tick
never SAW the press; the Python watcher delivered the stop).

Phase 1 checks each link in the chain with the existing bindings:
is the hard tick dispatcher alive, did config take, what does the
raw pad read. Phase 2 prints the stats tuple every 300 ms for 20 s —
press the STOP button during it. Whatever cuts the stream, the final
"hard button (...)" line from the stop handler is the verdict:
presses > 0 means sampling works; hard_stops > 0 means the hard
path delivered the stop.

Run:
    openbricks run -n ls examples/hard_button_probe.py
"""

import time

from machine import Pin

from _openbricks_native import motor_process as m

BUTTON_PIN = 4      # launcher default (program button)

print("hard_tick_available:", m.hard_tick_available())
t0 = m.hard_tick_count()
time.sleep_ms(500)
t1 = m.hard_tick_count()
print("hard ticks in 500 ms:", t1 - t0, "(want ~500)")
print("hard_button_config(%d) ->" % BUTTON_PIN,
      m.hard_button_config(BUTTON_PIN))
print("stats now:", m.hard_button_stats())
# No mode args: reads the pad without reconfiguring it.
print("raw Pin(%d).value() while NOT pressed (want 1):" % BUTTON_PIN,
      Pin(BUTTON_PIN).value())
print()
print(">>> press the STOP button within 20 s; stats print every 300 ms")
for i in range(66):
    time.sleep_ms(300)
    print(i, m.hard_button_stats())
print("no press detected in 20 s — done (that itself is a data point).")
