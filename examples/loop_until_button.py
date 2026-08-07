# SPDX-License-Identifier: MIT
"""Discriminating test for the stop button.

Run this via ``openbricks run`` (NOT the button). ``openbricks run``
execs the program in the MAIN THREAD (top level), whereas a button
*start* runs it inside MicroPython's scheduler. We want to know whether
a program is stoppable when it lives in the main thread.

    openbricks run -n ls examples/loop_until_button.py

Then PRESS THE GPIO-4 BUTTON once and watch:

* If the loop STOPS (you see ``openbricks: stopped by button press.``)
  → main-thread programs ARE stoppable on this hardware. The real bug
  is only that button-*started* programs run inside the scheduler, and
  the fix is to route button-start through the main-thread idle loop.

* If the loop KEEPS GOING after pressing the button → the stop
  *mechanism* (micropython.schedule raising KeyboardInterrupt) doesn't
  preempt a running loop on this chip at all, and the fix has to be a
  different stop mechanism (not just where the program runs).

Either result tells me exactly what to change — no more guessing.
You can always stop it from the host with Ctrl-C if the button doesn't.
"""

import time

i = 0
print("looping in the MAIN THREAD via openbricks run.")
print("Press the GPIO-4 stop button now...")
while True:
    i += 1
    if i % 20 == 0:
        print("still looping, i =", i)
    time.sleep_ms(50)
