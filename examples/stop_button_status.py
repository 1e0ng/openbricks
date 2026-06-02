# SPDX-License-Identifier: MIT
"""Run after flashing v1.8.6. Installs+arms the FreeRTOS-task stop button
and prints its live debug counter while you press.

    openbricks run -n ls examples/stop_button_status.py

debug tuple = (task_started, loop_ticks, edges_fired, armed, gpio)
  * loop_ticks climbing   -> the poll task IS running
  * edges_fired climbs on a press -> it detected the press while armed
If the loop STOPS on a press, the stop button works. If not, the tuple
says exactly which link is broken — no more guessing.
"""

import time

import openbricks
import _openbricks_native as n

print("fw", openbricks.__version__)
n.install_stop_button(4)
n.set_stop_armed(True)
print("installed + armed. debug = (task_started, ticks, edges, armed, gpio)")
print(">>> PRESS the button — the loop should stop. <<<")
i = 0
try:
    while i < 600:                      # ~30 s
        i += 1
        if i % 20 == 0:                 # ~once a second
            print("debug:", n.stop_button_debug())
        time.sleep_ms(50)
except KeyboardInterrupt:
    print("STOPPED — stop button WORKS. final:", n.stop_button_debug())
finally:
    n.set_stop_armed(False)
print("end. final debug:", n.stop_button_debug())
