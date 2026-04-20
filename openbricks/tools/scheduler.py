# SPDX-License-Identifier: MIT
"""
Cooperative motor scheduler — openbricks' analogue of Pybricks' ``pbio``
``motor_process``.

A ``MotorProcess`` singleton owns a periodic ``machine.Timer``; motor
drivers (``JGB37Motor`` and future peers) register a per-tick callable
which the timer ISR invokes at a fixed rate. Closed-loop control then
runs deterministically even when user code is blocking in
``DriveBase.straight()`` or sleeping.

Design:

* **Always-on, pbio-style.** Because openbricks ships as a custom
  MicroPython firmware, the runtime owns the hardware. The first call
  to ``instance()`` starts the timer and it stays running for the life
  of the interpreter. Motors just flip between "actively controlled"
  and "idle" states by registering or unregistering their tick
  callback; the scheduler itself never idles down.
* **Singleton.** One physical timer drives everything; motors share it.
  Period defaults to 10 ms (100 Hz), overridable via
  ``MotorProcess(period_ms=…)`` or ``configure()``.
* **Motors manage themselves.** User-facing motor methods —
  ``run_speed``, ``brake``, ``coast``, ``run`` — register or unregister
  their control step internally. Callers of those methods never touch
  the scheduler.
* **Manual subscription is still available.** ``register()`` /
  ``unregister()`` are public for custom periodic work (e.g. a user
  wiring up their own sensor-fusion loop). Both are idempotent.
* **Tick under test.** ``tick()`` fires all registered callables
  synchronously — useful when a test wants to run exactly one beat
  without sleeping.
"""

from machine import Timer


class MotorProcess:
    _instance = None

    def __init__(self, period_ms=10):
        self._period_ms = int(period_ms)
        self._callbacks = []
        self._timer = None

    # ---- singleton accessors ----

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls()
            cls._instance.start()
        return cls._instance

    @classmethod
    def reset(cls):
        """Drop the singleton. Tests call this in setUp to isolate state."""
        if cls._instance is not None:
            cls._instance.stop()
        cls._instance = None

    def configure(self, period_ms):
        """Change the tick period. Takes effect immediately if the timer is
        already running."""
        self._period_ms = int(period_ms)
        if self._timer is not None:
            self.stop()
            self.start()

    # ---- subscription ----

    def register(self, callback):
        if callback not in self._callbacks:
            self._callbacks.append(callback)

    def unregister(self, callback):
        try:
            self._callbacks.remove(callback)
        except ValueError:
            pass

    # ---- lifecycle ----
    # Rarely needed directly — ``instance()`` starts the timer and it stays
    # running. These are kept public for tests and for power users who
    # want to pause/resume the whole process.

    def start(self):
        if self._timer is not None:
            return
        self._timer = Timer(-1)
        self._timer.init(
            period=self._period_ms,
            mode=Timer.PERIODIC,
            callback=self._on_tick,
        )

    def stop(self):
        if self._timer is None:
            return
        self._timer.deinit()
        self._timer = None

    def is_running(self):
        return self._timer is not None

    # ---- tick ----

    def tick(self):
        """Run one beat synchronously. ISR-driven firing uses the same path
        via ``_on_tick``."""
        self._on_tick(None)

    def _on_tick(self, _timer):
        # Snapshot in case a callback unregisters itself.
        for cb in list(self._callbacks):
            cb()
