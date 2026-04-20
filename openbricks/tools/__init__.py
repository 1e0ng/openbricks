# SPDX-License-Identifier: MIT
"""Small utilities mirroring the ``pybricks.tools`` module."""

import time


def wait(milliseconds):
    """Pause for ``milliseconds`` ms."""
    time.sleep_ms(int(milliseconds))


class StopWatch:
    """Elapsed-time stopwatch in milliseconds."""

    def __init__(self):
        self._start = time.ticks_ms()
        self._paused_at = None

    def time(self):
        if self._paused_at is not None:
            return time.ticks_diff(self._paused_at, self._start)
        return time.ticks_diff(time.ticks_ms(), self._start)

    def reset(self):
        self._start = time.ticks_ms()
        if self._paused_at is not None:
            self._paused_at = self._start

    def pause(self):
        if self._paused_at is None:
            self._paused_at = time.ticks_ms()

    def resume(self):
        if self._paused_at is not None:
            # Shift start forward by how long we were paused.
            paused_for = time.ticks_diff(time.ticks_ms(), self._paused_at)
            self._start = time.ticks_add(self._start, paused_for)
            self._paused_at = None
