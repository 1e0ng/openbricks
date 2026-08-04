# SPDX-License-Identifier: MIT
"""
Diagnostic: why does the SAME ``run_angle`` call move at different
speeds on different runs?

Reported from the bench: ``motor.run_angle(200, -145)`` in a
one-motor script lands on the same angle every time, but sometimes
runs fast and sometimes slow.

Reading the driver settles one thing: openbricks writes the
goal-speed register (0x2E) and the goal-acceleration register (0x29)
explicitly on every call, so it is not *commanding* different speeds.
Something between the command and the shaft is varying. This script
finds out which, by measuring instead of guessing.

Each repetition records, around one move:

* the registers the servo ACTUALLY holds afterwards — goal speed,
  goal acc, op mode, angle limits. ``_SCServoBus.write`` does not
  verify the servo's acknowledgement (it sends and discards the
  status reply), so a dropped or rejected register write is silent.
  If a slow run shows a goal-speed readback that differs from the
  commanded value, that is the answer.
* supply voltage and temperature. A Feetech servo that can't hold
  the commanded rate under a sagging supply, or that has entered
  overload protection after repeated runs, moves slower with every
  register perfectly correct.
* wall-clock duration and the resulting degrees-per-second. This is
  the objective version of "fast" vs "slow" — eyeballing can't tell
  0.8 s from 1.4 s reliably.

Read the numbers like this:

* goal-speed readback differs between runs -> a register write is
  being lost; the bug is in the driver's unverified writes.
* registers identical, VOLTAGE lower on the slow runs -> supply sag
  (battery, wiring, or a brownout under load).
* registers identical, TEMPERATURE climbing across runs -> the
  servo's overload protection is throttling it.
* registers identical, voltage and temperature steady, duration
  still varying -> mechanical: binding, a rubbing shaft, or the
  load changing with the arm's resting position.

Hardware: one ST-3032 on the bench UART (id 4, UART1 tx=14 rx=6 —
edit below to match). Nothing else should be running: a drivebase
script left resident in the same power session keeps its own bus
traffic going, which would muddy every reading here.
"""

import time

from openbricks.drivers.st3032 import ST3032Motor


# --- the move under investigation (the user's actual call) ---
SERVO_ID   = 4
UART_ID, TX, RX = 1, 14, 6
DEG_PER_S  = 200
ANGLE      = -145
REPS       = 6
SETTLE_MS  = 700      # pause between reps, so each starts at rest

# Feetech STS register map (the driver's private names, spelled out
# here so this script stays readable on its own).
REG_MIN_ANGLE   = 0x09
REG_MAX_ANGLE   = 0x0B
REG_OP_MODE     = 0x21
REG_GOAL_ACC    = 0x29
REG_GOAL_SPEED  = 0x2E
REG_VOLTAGE     = 0x3E     # 0.1 V per count
REG_TEMPERATURE = 0x3F     # degrees C


def _u8(motor, reg):
    raw = motor._bus.read(motor._id, reg, 1)
    return None if raw is None else raw[0]


def _u16(motor, reg):
    raw = motor._bus.read(motor._id, reg, 2)
    return None if raw is None else (raw[0] | (raw[1] << 8))


def snapshot(motor):
    """Registers as the SERVO actually holds them right now."""
    return {
        "goal_speed": _u16(motor, REG_GOAL_SPEED),
        "goal_acc":   _u8(motor, REG_GOAL_ACC),
        "op_mode":    _u8(motor, REG_OP_MODE),
        "min_angle":  _u16(motor, REG_MIN_ANGLE),
        "max_angle":  _u16(motor, REG_MAX_ANGLE),
        "voltage_dv": _u8(motor, REG_VOLTAGE),
        "temp_c":     _u8(motor, REG_TEMPERATURE),
    }


def main():
    motor = ST3032Motor(servo_id=SERVO_ID, uart_id=UART_ID, tx=TX, rx=RX)

    # What the driver INTENDS to write, from its own arithmetic — the
    # number every readback below should match.
    commanded = int(round(min(DEG_PER_S, motor._max_dps)
                          * motor._steps_per_dps))
    print("commanded goal_speed = %d steps/s (%d dps)"
          % (commanded, DEG_PER_S))
    print("commanded goal_acc   = %d" % motor._encode_goal_acc())
    print("")

    for rep in range(1, REPS + 1):
        before = snapshot(motor)
        t0 = time.ticks_ms()
        motor.run_angle(DEG_PER_S, ANGLE)
        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        after = snapshot(motor)

        measured_dps = (abs(ANGLE) * 1000.0 / elapsed) if elapsed else 0.0
        print("rep %d: %d ms  -> %.0f dps average" % (rep, elapsed,
                                                      measured_dps))
        print("   goal_speed after=%s (commanded %d)%s"
              % (after["goal_speed"], commanded,
                 "  <-- MISMATCH" if after["goal_speed"] not in
                 (commanded, None) else ""))
        print("   goal_acc=%s op_mode=%s limits=%s/%s"
              % (after["goal_acc"], after["op_mode"],
                 after["min_angle"], after["max_angle"]))
        v = after["voltage_dv"]
        print("   voltage=%s V  temp=%s C   (before: %s V, %s C)"
              % ("?" if v is None else "%.1f" % (v / 10.0),
                 after["temp_c"],
                 "?" if before["voltage_dv"] is None
                 else "%.1f" % (before["voltage_dv"] / 10.0),
                 before["temp_c"]))
        time.sleep_ms(SETTLE_MS)

    motor.coast()
    print("\ndone — compare the rows: what differs on the slow ones?")


if __name__ == "__main__":
    main()
