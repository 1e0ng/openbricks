# SPDX-License-Identifier: MIT
"""
Emergency-stop latch: once engaged, no motor moves until it's cleared.

The stop button's fundamental problem is that killing the *program*
rides on ``mp_sched_keyboard_interrupt``, which raises in whatever
main-thread frame happens to be executing — scheduled callbacks
included — so a single injection can be eaten by an innocent callback
and the program never notices. Stopping the *robot* must not depend on
that lottery.

This latch decouples the two:

* ``engage()`` — called from the launcher's button poll (its own timer
  callback, independent of the program) the moment a stop press lands.
  It latches, then best-effort kills everything that moves: the native
  1 kHz scheduler and a torque-off broadcast on every serial servo
  bus. From here on the robot is physically stopping, interrupt or no
  interrupt.
* ``check()`` — called by every motion command in every driver
  (``run`` / ``run_speed`` / ``run_angle`` / ``move_to`` / drive).
  While engaged it raises ``KeyboardInterrupt`` *synchronously in the
  caller's own frame* — undroppable, and it turns the still-running
  program's next motor command into the stop's delivery vehicle. The
  tug-of-war where the program re-commands speed after the kill is
  unwinnable for the program: its commands raise instead of reaching
  the bus.
* ``clear()`` — called by the launcher when it regains control (the
  program is dead) and defensively before each program start.

Stopping commands (``brake`` / ``coast`` / ``hold`` / ``stop``) are
deliberately NOT gated — they're always safe to execute.
"""

_engaged = False


def is_engaged():
    return _engaged


def engage():
    """Latch the e-stop and best-effort halt everything that moves.

    Safe to call from a soft timer callback (allocations are fine
    there; every step is wrapped so one wedged bus can't stop the
    others from being stopped). Idempotent.
    """
    global _engaged
    _engaged = True
    # Halt the native 1 kHz scheduler (closed-loop PWM/encoder motors).
    try:
        from _openbricks_native import motor_process
        motor_process.stop()
    except Exception:
        pass
    # Torque-off broadcast on every known serial-servo bus. Write-only
    # packet, idempotent — collisions with an in-flight program
    # transaction are possible but harmless to retry; the synchronous
    # ``check()`` raise keeps the program from re-commanding anyway.
    try:
        from openbricks.drivers.st3215 import (
            ST3215, _REG_TORQUE, _BROADCAST_ID)
        for bus in list(ST3215._buses.values()):
            try:
                bus.write(_BROADCAST_ID, _REG_TORQUE, bytes([0]))
            except Exception:
                pass
    except Exception:
        pass
    # NATIVE serial bus (st_bus, 1.41.0+): when the hard-tick pump
    # owns the UART, the Python bus objects above can't reach those
    # motors — the native broadcast can, jumps any in-flight
    # transaction, and voids every staged speed so the pump can't
    # re-drive. Harmless no-op when nothing is attached; absent
    # entirely off-firmware (guarded import).
    try:
        from _openbricks_native import st_bus
        st_bus.torque_off_all()
    except Exception:
        pass


def clear():
    """Release the latch. Motors accept commands again."""
    global _engaged
    _engaged = False


def check():
    """Gate for motion commands: no-op normally; raises while engaged.

    Raises ``KeyboardInterrupt`` (not a custom type) so the unwind
    takes exactly the same path as the injected stop — the launcher's
    ``except KeyboardInterrupt`` handler — and so user code that
    already handles stop presses keeps working unchanged.
    """
    if _engaged:
        raise KeyboardInterrupt("stop button (e-stop engaged)")
