# SPDX-License-Identifier: MIT
"""
Measure what a ``print()`` costs while a run log is active.

Background: the run-log tee used to commit to flash on every line —
``file.flush()``, which on littlefs forces a metadata commit. On the
ESP32 bench that measured ~60-90 ms PER LINE, paid synchronously on
the main thread between the user program's own bytecodes. A 29-line
``dump_events()`` took 1.9 seconds, and any program that printed while
driving had a tenth-of-a-second stall injected into its control loop.

Since 1.35.0 ``print`` only appends to a RAM buffer. The bytes reach
flash from the launcher's Timer tick (``log.pump()``), and a real
commit is paid only where durability matters: program end, the stop
button, and ``write_text`` (the "started:" / "Exception:" lines).

This script reports both costs so the improvement is a number rather
than a claim:

    openbricks run -n ls examples/log_write_benchmark.py

Run it under ``openbricks run`` (or from a button press) — both enter
a log session, which is what makes ``print`` a tee'd call. Without an
active session there is nothing to measure and the script says so.
"""

import time

from openbricks import log


N = 50
LINE = "benchmark line %d with a realistic amount of text to write"


def _buffered():
    for i in range(N):
        print(LINE % i)


def _committed():
    for i in range(N):
        print(LINE % i)
        log.flush()


def main():
    if log._ACTIVE is None:
        print("no active run-log session — start this with "
              "'openbricks run' or the program button, otherwise "
              "print() is not tee'd and there is nothing to measure.")
        return

    print("warming up")
    log.flush()

    t0 = time.ticks_us()
    _buffered()
    buffered_us = time.ticks_diff(time.ticks_us(), t0)

    t0 = time.ticks_us()
    _committed()
    committed_us = time.ticks_diff(time.ticks_us(), t0)

    print("")
    print("--- run-log write cost, %d lines each ---" % N)
    print("buffered print (current):  %7.1f us/line  (%d us total)"
          % (buffered_us / N, buffered_us))
    print("commit-per-line (old):     %7.1f us/line  (%d us total)"
          % (committed_us / N, committed_us))
    if buffered_us > 0:
        print("speedup:                   %7.1fx" %
              (committed_us / float(buffered_us)))
    print("")
    print("A program printing 100 lines pays %.2f s under the old "
          "behaviour vs %.2f s now."
          % (committed_us * 2 / 1e6, buffered_us * 2 / 1e6))


main()
