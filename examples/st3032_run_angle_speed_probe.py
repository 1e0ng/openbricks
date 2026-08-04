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

Round 1 established the facts: the "slow" moves are the CORRECT
ones (196 dps against a commanded 200) and the fast ones genuinely
travel the full angle at ~700 dps — the servo's no-load ceiling is
895 dps, so those moves run flat out. Supply (11.9 V) and
temperature (40 C) are steady throughout, and goal-speed always
reads back exactly as commanded. So it is not supply sag, not
overload throttling, and not a lost speed write.

Round 1 also killed the first theory. "Only the first move is fast,
because only it writes the EEPROM angle limits" cannot survive a run
where moves 1 AND 2 were both fast — move 2 writes no EEPROM at all,
and the count of fast moves changed between runs.

So this round separates the variables instead of guessing again:

* **A** repeats the same move, so move number and accumulated shaft
  position advance together (the original reproduction).
* **B** alternates direction, so the shaft stays near where it began
  while the move number keeps climbing. A and B agreeing means the
  effect follows the MOVE NUMBER; B breaking the pattern means it
  follows the POSITION.
* **C** halves the commanded speed. If "fast" really means
  "unlimited", it lands near the same ~700 dps regardless; if it
  scales with the command, this is a units problem instead.

The ``from`` column is the shaft position each move started at —
that is where a position-dependent effect shows up.

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


def sweep(motor, label, angles, commanded):
    """Run a sequence of moves, reporting speed and where the shaft
    was when each one started."""
    print("\n=== %s ===" % label)
    for i, ang in enumerate(angles, 1):
        start = motor.angle()
        t0 = time.ticks_ms()
        motor.run_angle(DEG_PER_S, ang)
        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        travelled = motor.angle() - start
        dps = (abs(travelled) * 1000.0 / elapsed) if elapsed else 0.0
        verdict = "FAST" if dps > commanded * 1.5 else "ok  "
        # ``start`` is the shaft position this move began from. If the
        # fast/slow split tracks position rather than move number,
        # this column is where it shows.
        print("  %d: %s %4d ms  %+7.1f deg from %+8.1f  -> %3.0f dps"
              % (i, verdict, elapsed, travelled, start, dps))
        time.sleep_ms(SETTLE_MS)


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

    # The first theory — "only the first move is fast, because only it
    # writes the EEPROM angle limits" — died when a second run showed
    # TWO fast moves, the second of which writes no EEPROM at all. So
    # stop guessing and separate the variables.
    #
    # A: same direction every time, so the shaft walks steadily away
    #    from where it started. Move number and accumulated position
    #    advance together — this is the original reproduction.
    # B: alternating direction, so the shaft stays near where it
    #    began while the move number keeps climbing. If the fast moves
    #    follow the MOVE NUMBER, A and B look alike. If they follow
    #    the POSITION, B's pattern breaks.
    # C: half speed. If "fast" is really "unlimited", it lands near
    #    the same ~700 dps no matter what was asked; if it scales with
    #    the command, it is a units/scale problem instead.
    sweep(motor, "A: same direction (walks away)",
          [ANGLE] * 6, DEG_PER_S)
    sweep(motor, "B: alternating (stays near start)",
          [ANGLE, -ANGLE] * 3, DEG_PER_S)

    print("\n=== C: half speed, same direction ===")
    for i in range(1, 5):
        start = motor.angle()
        t0 = time.ticks_ms()
        motor.run_angle(DEG_PER_S // 2, ANGLE)
        elapsed = time.ticks_diff(time.ticks_ms(), t0)
        travelled = motor.angle() - start
        dps = (abs(travelled) * 1000.0 / elapsed) if elapsed else 0.0
        print("  %d: %4d ms  %+7.1f deg from %+8.1f  -> %3.0f dps "
              "(asked %d)" % (i, elapsed, travelled, start, dps,
                              DEG_PER_S // 2))
        time.sleep_ms(SETTLE_MS)

    motor.coast()
    print("\ndone — A vs B says whether it tracks the move number or "
          "the shaft position; C says whether 'fast' means 'unlimited'.")


if __name__ == "__main__":
    main()
