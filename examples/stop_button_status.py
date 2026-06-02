# SPDX-License-Identifier: MIT
"""Run after flashing the stop-button fix. The launcher auto-installs the
native stop_tick Timer and arms it for the duration of this run, so just
loop and press — and watch the debug counter.

    openbricks run -n ls examples/stop_button_status.py

debug tuple = (stop_tick_ticks, fires, armed, requested)
  * stop_tick_ticks climbing -> the native C-function Timer callback runs
  * armed = 1                -> _exec_program_raw armed it for this run
  * on a press: requested flips 1 then fires increments, loop stops
"""

import time

import openbricks
import _openbricks_native as n

print("fw", openbricks.__version__)
print("debug = (stop_tick_ticks, fires, armed, requested)")
print(">>> PRESS the GPIO-4 button — the loop should stop. <<<")
i = 0
try:
    while i < 600:                      # ~30 s
        i += 1
        if i % 20 == 0:                 # ~once a second
            print("debug:", n.stop_button_debug())
        time.sleep_ms(50)
except KeyboardInterrupt:
    print("STOPPED — stop button WORKS. final:", n.stop_button_debug())
print("end. final debug:", n.stop_button_debug())
