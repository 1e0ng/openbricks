# SPDX-License-Identifier: MIT
"""
Cooperative motor scheduler — openbricks' analogue of Pybricks' ``pbio``
``motor_process``.

A ``MotorProcess`` singleton owns a periodic ``machine.Timer``; drivers
register a per-tick callable (``_control_step`` on ``JGB37Motor``, for
example) which the timer ISR invokes at a fixed rate. That lets closed-loop
control run deterministically even when user code is busy blocking in
``straight()`` or sleeping.

Design:

* **Singleton.** One physical timer drives everything; motors share it
  rather than each allocating their own. ``MotorProcess.instance()`` is the
  access point; the first call instantiates it.
* **Period is fixed at 10 ms by default** — a 100 Hz control loop is ample
  for gear motors at this API's accuracy. Set via ``MotorProcess(period_ms=…)``
  before the first ``instance()`` call, or via ``configure()``.
* **Idempotent.** ``start()`` / ``stop()`` are safe to call repeatedly.
  Registering the same callable twice is a no-op.
* **Tick under test.** ``tick()`` fires all registered callables
  synchronously — useful when a test wants to run exactly one beat without
  sleeping.
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
        return cls._instance

    @classmethod
    def reset(cls):
        """Drop the singleton. Tests call this in setUp to isolate state."""
        if cls._instance is not None:
            cls._instance.stop()
        cls._instance = None

    def configure(self, period_ms):
        """Change the tick period. Takes effect on the next ``start()``."""
        self._period_ms = int(period_ms)

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
        """Run one beat synchronously. ``start()``-driven ISR firing calls
        the same path via ``_on_tick``."""
        self._on_tick(None)

    def _on_tick(self, _timer):
        # Snapshot in case a callback unregisters itself.
        for cb in list(self._callbacks):
            cb()
