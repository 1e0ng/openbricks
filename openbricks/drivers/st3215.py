# SPDX-License-Identifier: MIT
"""
ST-3215 (Waveshare/FeeTech) serial bus servo.

These servos daisy-chain on a half-duplex UART bus: one TX line shared by
host and all servos, each servo addressed by a 1-byte ID. Packet format (as
in the FeeTech SCServo / Dynamixel-style protocol):

    0xFF 0xFF  ID  LEN  INSTR  PARAM...  CHECKSUM

    CHECKSUM = ~(ID + LEN + INSTR + sum(PARAM)) & 0xFF
    LEN      = number of params + 2

Common instructions::

    0x01 PING            ->  probe the servo
    0x02 READ            ->  READ  reg len
    0x03 WRITE           ->  WRITE reg value...

Key registers (ST-3215)::

    0x21  Operation Mode — 0 = position, 1 = wheel/continuous
    0x28  Torque Switch  — 1 = enable, 0 = coast
    0x2A  Goal position (low, high) — int16, 0..4095 over ~360°
    0x2E  Goal speed (low, high) — sign-magnitude; in wheel mode this
          is the velocity setpoint (bit 15 of high byte = direction)
    0x38  Present position (low, high) — read only. 12-bit absolute
          angle within one revolution: 0..4095 over 360°. Wraps to 0
          at every full turn; we accumulate multi-turn revolutions in
          software via a wrap heuristic in ``ST3215Motor.angle``.

Half-duplex wiring: most ST-3215 boards use a single data line driven by a
TX/RX switching circuit, but MicroPython UART pins are usually separate. If
your adapter exposes TX and RX separately, just wire them normally. If you
have a true half-duplex bus, you'll need a driver chip or a direction-enable
GPIO — set ``dir_pin`` below.

Two classes here:

* ``ST3215`` — position-mode (Servo interface), for grippers / lifts /
  sensor turrets. ``move_to(angle)`` blocks until reached.

* ``ST3215Motor`` — continuous-rotation mode (Motor interface), for
  drivebase wheels. Switches the servo into mode=1 at construction
  and exposes ``run_speed(dps)`` / ``angle()`` / ``brake()`` so it
  drops into ``DriveBase`` the same way ``MG370Motor`` does.

The ST-3032 (smaller sibling — 12 V, ~10 kg·cm, same SCS protocol)
ships as marker subclasses in ``openbricks.drivers.st3032``.

Only a minimal subset of the protocol is implemented here. PR welcome.
"""

import time

from machine import UART, Pin

from openbricks import pins
from openbricks import estop
from openbricks.interfaces import Motor, Servo

_HEADER = b"\xFF\xFF"

_BROADCAST_ID = 0xFE

_INSTR_PING       = 0x01
_INSTR_READ       = 0x02
_INSTR_WRITE      = 0x03
_INSTR_SYNC_WRITE = 0x83

# Angle-limit registers (each int16, low byte first). When BOTH are
# written to 0 the STS servo leaves single-turn position mode and
# allows multi-turn moves — the prerequisite for step mode (see
# ``run_angle`` / the FeeTech STS tutorial §13 "How to realize the
# step function").
_REG_MIN_ANGLE     = 0x09
_REG_MAX_ANGLE     = 0x0B

_REG_OP_MODE       = 0x21
_REG_TORQUE        = 0x28
_REG_GOAL_ACC      = 0x29
_REG_GOAL_POSITION = 0x2A
_REG_GOAL_SPEED    = 0x2E
_REG_PRESENT_POS   = 0x38
_REG_PRESENT_SPEED = 0x3A
_REG_PRESENT_LOAD  = 0x3C

_MODE_POSITION = 0   # single-turn absolute position (0..4095 = 0..360°)
_MODE_WHEEL    = 1   # continuous velocity (wheel)
_MODE_STEP     = 3   # step servo: goal_position is a SIGNED RELATIVE
                     # step; multi-turn capable (needs angle limits=0)

# Hardware: 4096 encoder counts per output revolution.
_COUNTS_PER_REV = 4096

# Largest relative step issued to the servo in a single goal-position
# write while in step mode. The STS multi-turn range is ±7 turns
# (datasheet 7-13); we cap one step at exactly 7 turns so a single
# write never exceeds the absolute-position envelope. Moves larger
# than this are issued as back-to-back steps, each completing before
# the next — there is no boundary to cross, so direction is always
# unambiguous (unlike the old single-turn ``% 4096`` chunking, which
# reversed past the 0/4095 wrap). 7 × 4096 = 28672 (< 0x7FFF reg max).
_MAX_STEP_COUNTS = 7 * _COUNTS_PER_REV

# Speed register units. The Feetech datasheet uses "step/sec"; one step
# is 360/4096 ≈ 0.0879 deg, so 1 dps ≈ 11.378 step/sec. Exposed as a
# kwarg in case future ST-3215 revisions ship with a different scale.
_DEFAULT_STEPS_PER_DPS = _COUNTS_PER_REV / 360.0   # = 11.378


class _SCServoBus:
    """Shared UART bus. One instance per physical bus; many servos per bus."""

    def __init__(self, uart_id, tx, rx, baud=1_000_000, dir_pin=None):
        pins.check(tx, "serial-bus UART TX")
        pins.check(rx, "serial-bus UART RX", output=False)
        if dir_pin is not None:
            pins.check(dir_pin, "serial-bus direction pin")
        self._uart = UART(uart_id, baudrate=baud, tx=tx, rx=rx, timeout=50)
        self._dir = Pin(dir_pin, Pin.OUT, value=0) if dir_pin is not None else None
        # UART hardware takes ~10 ms to be ready for clean TX on
        # ESP32-S3 — without this settle, the first packet sent
        # (typically ST3215Motor's constructor write to op_mode)
        # leaves the peripheral as malformed bits on the wire. The
        # servo doesn't reply, the URT-2 adapter sometimes enters a
        # sulk, and every subsequent read returns junk → ``ping()``
        # falsely reports False even though the bus is otherwise
        # fine. Bench-confirmed: settle here makes the constructor-
        # plus-immediate-ping pattern work; skip and it doesn't.
        time.sleep_ms(20)

    def _checksum(self, parts):
        s = 0
        for p in parts:
            s += p
        return (~s) & 0xFF

    def _tx(self, data):
        # Drain any bytes still sitting in the RX FIFO from boot, from
        # half-duplex bus echo, or from a previous reply we didn't fully
        # consume. Without this, the very next ``_rx`` would return
        # stale residue and the SCS header check (``starts with 0xFFFF``)
        # could either fail outright or — worse — succeed against
        # noise that happens to start with 0xFFFF, mis-parsing the
        # rest of the packet. Symptom: ``ping`` returns True (6 bytes
        # of anything come back) but ``read`` returns None.
        while self._uart.any():
            self._uart.read(self._uart.any())
        if self._dir is not None:
            self._dir.value(1)
        self._uart.write(data)
        if self._dir is not None:
            # Wait for transmission to flush before releasing the line.
            time.sleep_us(len(data) * 10_000_000 // 1_000_000)  # rough
            self._dir.value(0)

    def _rx(self, n, timeout_ms=50):
        deadline = time.ticks_ms() + timeout_ms
        buf = b""
        while len(buf) < n and time.ticks_diff(deadline, time.ticks_ms()) > 0:
            chunk = self._uart.read(n - len(buf))
            if chunk:
                buf += chunk
            else:
                time.sleep_ms(1)
        return buf

    def write(self, servo_id, register, data):
        length = len(data) + 3  # register + params + checksum + instr -> LEN = params + 2 conceptually; +1 for register
        params = bytes([register]) + bytes(data)
        body = bytes([servo_id, length, _INSTR_WRITE]) + params
        packet = _HEADER + body + bytes([self._checksum(body)])
        self._tx(packet)
        # Discard any status response.
        self._rx(6, timeout_ms=10)

    def read(self, servo_id, register, nbytes):
        length = 4
        params = bytes([register, nbytes])
        body = bytes([servo_id, length, _INSTR_READ]) + params
        packet = _HEADER + body + bytes([self._checksum(body)])
        self._tx(packet)
        # Response: FF FF ID LEN ERR DATA... CHK
        resp = self._rx(6 + nbytes)
        if len(resp) < 6 + nbytes or not resp.startswith(_HEADER):
            return None
        return resp[5:5 + nbytes]

    def ping(self, servo_id):
        body = bytes([servo_id, 2, _INSTR_PING])
        packet = _HEADER + body + bytes([self._checksum(body)])
        self._tx(packet)
        return len(self._rx(6)) == 6

    def sync_write(self, register, data_len, servo_data):
        """Broadcast SYNC WRITE: one packet writes ``register`` on N
        servos simultaneously.

        ``servo_data`` is a list of ``(servo_id, data_bytes)`` tuples
        where each ``data_bytes`` is exactly ``data_len`` bytes long.

        Two reasons to prefer this over N individual ``write()`` calls
        when coordinating multiple servos on one bus:

        * **Time alignment.** All servos apply their setpoint at the
          same packet boundary; with individual writes, servo A gets
          its command 1–5 ms before servo B and the wheels start at
          slightly different times.
        * **Bus bandwidth.** N writes = N packets + N status replies +
          N round-trips. SYNC WRITE = one packet, no replies — about
          5–10× less UART time on a 4-servo bus.

        Servos do NOT reply to SYNC WRITE (it's broadcast, ID 0xFE),
        so this method doesn't poll the RX line.
        """
        n = len(servo_data)
        if n == 0:
            return
        # LEN field = number of param bytes + 2.
        # Params for SYNC WRITE = ADDR(1) + DATA_LEN(1) + N × (ID(1) + data_len)
        length = 4 + n * (1 + data_len)
        body = bytearray()
        body.append(_BROADCAST_ID)
        body.append(length)
        body.append(_INSTR_SYNC_WRITE)
        body.append(register)
        body.append(data_len)
        for sid, data in servo_data:
            if len(data) != data_len:
                raise ValueError("sync_write data length mismatch")
            body.append(sid)
            body.extend(data)
        body = bytes(body)
        packet = _HEADER + body + bytes([self._checksum(body)])
        self._tx(packet)


class ST3215(Servo):
    """One ST-3215 servo on a shared bus."""

    # Class-level registry of buses so many servos can share one UART.
    _buses = {}

    @classmethod
    def _bus_for(cls, uart_id, tx, rx, baud, dir_pin):
        key = (uart_id, tx, rx, baud)
        if key not in cls._buses:
            cls._buses[key] = _SCServoBus(uart_id, tx, rx, baud, dir_pin)
        return cls._buses[key]

    def __init__(self, servo_id, uart_id=1, tx=17, rx=16,
                 baud=1_000_000, dir_pin=None,
                 min_raw=0, max_raw=4095, range_deg=360):
        self._id = servo_id
        self._bus = self._bus_for(uart_id, tx, rx, baud, dir_pin)
        self._min = min_raw
        self._max = max_raw
        self._range = range_deg
        # Pybricks-consistent: a freshly-constructed servo coasts until
        # its first ``move_to`` (which re-enables torque). Writing 0 —
        # rather than merely not writing — also releases a hold left
        # behind by a previous program on the same power session.
        self._torque_on = False
        self._bus.write(self._id, _REG_TORQUE, bytes([0]))

    def _ensure_torque_on(self):
        """Re-enable torque before a motion command if construction
        (or a future coast) left it disabled. Cached — no redundant
        bus packet on back-to-back moves."""
        if not self._torque_on:
            self._bus.write(self._id, _REG_TORQUE, bytes([1]))
            self._torque_on = True

    def _deg_to_raw(self, angle_deg):
        # Clamp angle and map to raw counts.
        if angle_deg < 0:
            angle_deg = 0
        elif angle_deg > self._range:
            angle_deg = self._range
        return int(self._min + (self._max - self._min) * angle_deg / self._range)

    def _raw_to_deg(self, raw):
        return (raw - self._min) * self._range / (self._max - self._min)

    def move_to(self, angle_deg, speed=None, wait=True):
        estop.check()
        self._ensure_torque_on()
        if speed is not None:
            s = int(speed)
            self._bus.write(self._id, _REG_GOAL_SPEED,
                            bytes([s & 0xFF, (s >> 8) & 0xFF]))
        raw = self._deg_to_raw(angle_deg)
        self._bus.write(self._id, _REG_GOAL_POSITION,
                        bytes([raw & 0xFF, (raw >> 8) & 0xFF]))
        if wait:
            # Poll position until within 2% of target or timeout.
            deadline = time.ticks_ms() + 3000
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                current = self.angle()
                if current is not None and abs(current - angle_deg) < self._range * 0.02:
                    return
                time.sleep_ms(20)

    def angle(self):
        data = self._bus.read(self._id, _REG_PRESENT_POS, 2)
        if data is None:
            return None
        raw = data[0] | (data[1] << 8)
        return self._raw_to_deg(raw)

    def ping(self):
        return self._bus.ping(self._id)


class ST3215Motor(Motor):
    """One ST-3215 in wheel/continuous-rotation mode.

    Implements the openbricks ``Motor`` interface so it drops directly
    into ``DriveBase``. The servo's internal velocity loop handles
    closed-loop speed tracking — we just write the setpoint and read
    the multi-turn accumulated angle.

    ``angle()`` accumulates in software because the servo's
    Present-Position register is a 16-bit signed counter that wraps
    every ~3 turns (4096 counts/rev × ~8 turns to wrap at ±32767).
    Same wrap-correction shape as ``PCNTEncoder``.
    """

    # Class-level registry shared with ``ST3215`` so a position-mode
    # gripper and a wheel-mode wheel on the same physical bus reuse
    # one ``_SCServoBus`` instance.
    _buses = ST3215._buses

    @classmethod
    def _bus_for(cls, uart_id, tx, rx, baud, dir_pin):
        return ST3215._bus_for(uart_id, tx, rx, baud, dir_pin)

    # Stall detection thresholds (bench-tunable class attributes) and
    # the datasheet stall torque used to scale load() into mNm.
    # ST-3215 @ 12 V: ~30 kg·cm ~= 2940 mNm. ST3032Motor overrides.
    STALL_TORQUE_MNM = 2940.0
    STALL_LOAD_PCT   = 80     # % of stall load to call it stalling
    STALL_SPEED_DPS  = 20.0   # and slower than this

    def __init__(self, servo_id, uart_id=1, tx=17, rx=16,
                 baud=1_000_000, dir_pin=None,
                 invert=False,
                 steps_per_dps=_DEFAULT_STEPS_PER_DPS,
                 max_dps=600.0,
                 accel_dps2=1500.0):
        self._id    = servo_id
        self._bus   = self._bus_for(uart_id, tx, rx, baud, dir_pin)
        self._invert = bool(invert)
        self._steps_per_dps = float(steps_per_dps)
        self._max_dps       = float(max_dps)
        self._accel_dps2    = float(accel_dps2)

        # Software multi-turn accumulator state. ``_accum_count`` is the
        # absolute shaft position in motor-frame encoder counts. In
        # wheel/position mode it is rebuilt from present-position reads
        # (the wrap heuristic in ``angle()``); in step mode the present
        # register reads remaining-to-target instead of position, so
        # there we bump ``_accum_count`` by the executed step counts as
        # each ``run_angle`` step parks. ``_accum_initialized`` marks
        # whether a baseline has been taken; ``_last_raw is None`` marks
        # "rebaseline on next read" (after a mode change) WITHOUT
        # discarding the accumulated count.
        self._last_raw    = None
        self._accum_count = 0
        self._accum_initialized = False
        self._zero_offset_count = 0   # set by reset_angle()

        # Cached register state so brake/coast/run_speed avoid redundant
        # bus writes, and so motion commands can transparently restore
        # the mode/torque after a prior ``coast`` or a ``run_angle`` that
        # left the servo in step mode (``then="hold"``).
        self._op_mode    = _MODE_WHEEL
        self._torque_on  = False

        # Whether this servo's angle-limit registers have been zeroed
        # to unlock multi-turn step mode. Done lazily on the first
        # ``run_angle`` (NOT at construction) so a servo used purely as
        # a ``DriveBase`` wheel keeps its stock single-turn limits and
        # its present-position reads keep wrapping cleanly at one rev.
        self._step_limits_zeroed = False

        # State for ``run_angle(wait=False)``. ``None`` means no
        # non-blocking move is in flight; ``done()`` returns True.
        # Layout: dict with keys
        # ``first`` (signed counts of the step currently in flight),
        # ``remaining_counts`` (signed motor-frame counts still to issue
        # as further steps once the current one parks — non-zero only
        # for >7-turn moves), ``tol_counts``, ``then``, and ``started``
        # (whether the present `remaining` register has been seen large
        # enough to confirm the move actually launched, guarding against
        # a stale ~0 read at kickoff). See ``_poll_pending``.
        self._pending = None

        # Switch the servo into wheel/continuous mode and cut torque:
        # Pybricks-consistent, a freshly-constructed motor coasts until
        # its first motion command (every command path re-enables via
        # ``_ensure_torque_on``). Writing 0 — rather than merely not
        # writing — also releases a hold left behind by a previous
        # program on the same power session.
        self._bus.write(self._id, _REG_OP_MODE, bytes([_MODE_WHEEL]))
        self._bus.write(self._id, _REG_TORQUE,  bytes([0]))
        # Hardware acceleration ramp (goal-acc register, unit = 100
        # encoder steps/s²): the SERVO slews every speed/position
        # change at ``accel_dps2``, so direct ``run_speed()`` writes —
        # the line follower, user code — honour the same acceleration
        # default as the DriveBase profile instead of stepping
        # instantly. ``accel_dps2=0`` disables the ramp (register 0 =
        # unlimited, the servo's power-on default). The ramp governs
        # every commanded speed transition, ``brake()`` included
        # (uniform-rule revert of 1.18.1); ``coast()`` and the e-stop
        # cut the torque register, which the ramp does not govern —
        # those two stop instantly.
        self._bus.write(self._id, _REG_GOAL_ACC,
                        bytes([self._encode_goal_acc()]))

    def _encode_goal_acc(self):
        """Goal-acc register value for ``self._accel_dps2``: unit is
        100 encoder steps/s², one byte, 0 = no ramp, capped at 254
        (~2230 °/s² at the stock 4096-count encoder)."""
        acc = int(round(self._accel_dps2 * self._steps_per_dps / 100.0))
        if acc < 0:
            acc = 0
        if acc > 254:
            acc = 254
        return acc

    # --- internal helpers -------------------------------------------------

    def _read_present_pos(self):
        # Present-position is a 12-bit absolute angle within one
        # revolution, range 0..4095 (NOT a free-running multi-turn
        # counter). It wraps to 0 at every full turn — multi-turn
        # tracking is done in software via the wrap heuristic in
        # angle(). Valid only in wheel/position mode; in STEP mode the
        # same register reads remaining-to-target (see
        # ``_read_step_remaining``).
        data = self._bus.read(self._id, _REG_PRESENT_POS, 2)
        if data is None:
            return None
        return (data[0] | (data[1] << 8)) & 0x0FFF

    def _read_step_remaining(self):
        """Read the present-position register interpreted as STEP-mode
        *remaining distance to target*.

        Bench-confirmed (examples/st3032_stepmode_probe.py): in step
        mode (op_mode=3) the present-position register does NOT hold an
        absolute position — it holds the signed number of encoder
        counts still to travel for the current relative step, counting
        down to ~0 (a ±2-count deadband) as the move completes. The
        value is sign-magnitude (bit 15 = direction), e.g. 0x87B1 =
        −1969 counts remaining, 0x1FAE = +8110 remaining, 0x8002 = −2
        (parked). So |remaining| ≤ tol is the move-complete signal.
        """
        data = self._bus.read(self._id, _REG_PRESENT_POS, 2)
        if data is None:
            return None
        raw = data[0] | (data[1] << 8)
        magnitude = raw & 0x7FFF
        return -magnitude if (raw & 0x8000) else magnitude

    def _encode_goal_speed(self, deg_per_s):
        """Compute the 16-bit goal-speed register value for ``deg_per_s``,
        without writing it. Used both by ``run_speed()`` (single write)
        and by ``SyncServoGroup`` (batched broadcast write).
        """
        dps = float(deg_per_s)
        if self._invert:
            dps = -dps
        if dps >  self._max_dps: dps =  self._max_dps
        if dps < -self._max_dps: dps = -self._max_dps
        signed_value = int(dps * self._steps_per_dps)
        magnitude = abs(signed_value)
        if magnitude > 0x7FFF:
            magnitude = 0x7FFF
        v = magnitude
        if signed_value < 0:
            v |= 0x8000   # bit 15 sets direction in sign-magnitude
        return v

    def _write_goal_speed_signed(self, value):
        # Sign-magnitude format: bit 15 of the 16-bit value sets direction.
        magnitude = abs(int(value))
        if magnitude > 0x7FFF:
            magnitude = 0x7FFF
        v = magnitude
        if value < 0:
            v |= 0x8000
        self._bus.write(self._id, _REG_GOAL_SPEED,
                        bytes([v & 0xFF, (v >> 8) & 0xFF]))

    def _ensure_mode(self, mode):
        """Write op_mode only when it differs from our tracked state.
        Saves a bus packet on the common case where the servo is already
        in the desired mode (e.g. ``run_speed`` after another
        ``run_speed``) and keeps the cache in sync after ``run_angle``
        returns with ``then="hold"`` and leaves the servo in step mode.

        A mode change invalidates the present-position delta baseline:
        the register's meaning differs between modes (absolute position
        in wheel/position mode vs remaining-to-target in step mode), so
        force ``angle()`` to rebaseline on its next read rather than
        treat the cross-mode jump as real shaft motion. The accumulated
        count itself is preserved."""
        if self._op_mode != mode:
            self._bus.write(self._id, _REG_OP_MODE, bytes([mode]))
            self._op_mode = mode
            self._last_raw = None

    def _ensure_torque_on(self):
        """Re-enable torque if a prior ``coast`` (or ``then="coast"``)
        left it disabled. No-op otherwise."""
        if not self._torque_on:
            self._bus.write(self._id, _REG_TORQUE, bytes([1]))
            self._torque_on = True

    def _ensure_step_limits(self):
        """Zero the min/max angle-limit registers so the servo will
        accept multi-turn relative moves in step mode.

        Per the FeeTech STS tutorial (§13): step mode is enabled by
        setting *both* angle limits to 0 and op_mode to 3. With the
        limits at their single-turn defaults (0..4095) the servo
        clamps a step move to one revolution; zeroing them unlocks the
        ±7-turn envelope.

        Written once per power session (guarded by an in-memory flag),
        the first time ``run_angle`` runs on this motor — never at
        construction — so a wheel-only motor never has its limits
        touched. These are EEPROM registers, but a single write per
        session is well within endurance.
        """
        if self._step_limits_zeroed:
            return
        self._bus.write(self._id, _REG_MIN_ANGLE, bytes([0, 0]))
        self._bus.write(self._id, _REG_MAX_ANGLE, bytes([0, 0]))
        self._step_limits_zeroed = True

    def _write_step(self, counts):
        """Write one signed relative step to the goal-position register
        (step mode). Direction is carried in bit 15 (sign-magnitude),
        the same encoding the servo uses for goal-speed; the magnitude
        is the number of encoder counts to advance from the current
        commanded position. The servo's position PID drives the move
        and holds at the end."""
        magnitude = abs(int(counts))
        if magnitude > 0x7FFF:
            magnitude = 0x7FFF
        v = magnitude
        if counts < 0:
            v |= 0x8000
        self._bus.write(self._id, _REG_GOAL_POSITION,
                        bytes([v & 0xFF, (v >> 8) & 0xFF]))

    def _abandon_pending(self):
        """Drop any in-flight ``run_angle(wait=False)`` state.

        Called at the start of every motion command — pybricks-style
        "new command supersedes." The new command will overwrite the
        servo's goal_position / op_mode / goal_speed anyway, so all
        we need to do is forget the bookkeeping; the next ``done()``
        call returns ``True``.
        """
        self._pending = None

    def _dispatch_then(self, then):
        """Run the end-of-move register dance for the given ``then``
        mode. Does NOT touch ``_pending`` — caller manages that.
        Used by both the ``run_angle(wait=True)`` finally block and
        the ``done()`` completion path for ``wait=False``.

        At entry the servo is in step mode (op_mode=3) holding the
        move's target position:

        * ``coast`` — cut torque; the wheel free-wheels.
        * ``brake`` — restore wheel mode and write goal_speed=0 so the
          velocity loop actively holds zero rotation rate.
        * ``hold`` — leave the servo in step mode; its position PID is
          already holding the target and resisting rotation, so no
          further write is needed.
        """
        if then == "coast":
            self._bus.write(self._id, _REG_TORQUE, bytes([0]))
            self._torque_on = False
        elif then == "brake":
            self._ensure_mode(_MODE_WHEEL)
            self._write_goal_speed_signed(0)
        # else "hold": step mode already holds the target — nothing to do.

    # --- Motor interface --------------------------------------------------

    def dc(self, duty):
        """Duty-style command, Pybricks ``Motor.dc()`` shape. The
        serial servo has no raw-PWM wheel mode, so duty maps onto
        ``run_speed`` scaled to ``max_dps`` — same behaviour as the
        pre-1.21.0 ``run(power)``."""
        if duty >  100: duty =  100
        if duty < -100: duty = -100
        self.run_speed(self._max_dps * duty / 100.0)

    def speed(self):
        """Measured shaft speed in deg/s from the present-speed
        register (sign-magnitude, bit 15; steps/s scaled by
        ``steps_per_dps``). Returns ``None`` if the bus is silent,
        matching ``angle()``."""
        data = self._bus.read(self._id, _REG_PRESENT_SPEED, 2)
        if data is None:
            return None
        raw = data[0] | (data[1] << 8)
        magnitude = raw & 0x7FFF
        dps = magnitude / self._steps_per_dps
        if raw & 0x8000:
            dps = -dps
        if self._invert:
            dps = -dps
        return dps

    def load(self):
        """Estimated shaft torque in mNm — Pybricks ``Motor.load()``
        shape. The present-load register reports 0.1 %-of-stall units
        (sign in bit 10, per the Feetech SCServo SDK); scaled by the
        model's datasheet stall torque (``STALL_TORQUE_MNM``), so
        treat it as an estimate, not a measurement. ``None`` if the
        bus is silent."""
        data = self._bus.read(self._id, _REG_PRESENT_LOAD, 2)
        if data is None:
            return None
        raw = data[0] | (data[1] << 8)
        magnitude = raw & 0x3FF          # 0..1000 = 0..100 % of stall
        mnm = magnitude * self.STALL_TORQUE_MNM / 1000.0
        if raw & 0x400:
            mnm = -mnm
        if self._invert:
            mnm = -mnm
        return mnm

    def stalled(self):
        """``True`` when the servo is pushing hard (load magnitude at
        least ``STALL_LOAD_PCT`` percent of stall) but barely moving
        (speed magnitude at most ``STALL_SPEED_DPS``) — the Pybricks
        ``Motor.stalled()`` contract, from the servo's own feedback
        registers. Raises ``OSError`` if the bus is silent (a silent
        bus must not read as "not stalled")."""
        data = self._bus.read(self._id, _REG_PRESENT_LOAD, 2)
        spd = self.speed()
        if data is None or spd is None:
            raise OSError("bus silent while reading stall state")
        load_magnitude = (data[0] | (data[1] << 8)) & 0x3FF
        return (load_magnitude >= self.STALL_LOAD_PCT * 10
                and abs(spd) <= self.STALL_SPEED_DPS)

    def run_speed(self, deg_per_s):
        """Set continuous wheel velocity in degrees per second."""
        estop.check()
        if self._native_slot is not None:
            # The slot was attached WITH this motor's invert flag, so
            # pass the user-frame value; the slot applies the sign.
            self._native_sb.servo_run(
                self._native_slot,
                int(float(deg_per_s) * self._steps_per_dps))
            return
        self._abandon_pending()
        self._ensure_mode(_MODE_WHEEL)
        self._ensure_torque_on()
        v = self._encode_goal_speed(deg_per_s)
        self._bus.write(self._id, _REG_GOAL_SPEED,
                        bytes([v & 0xFF, (v >> 8) & 0xFF]))

    def brake(self):
        """Ramp to zero velocity and hold (servo's internal loop).

        Deliberately RAMPED at ``accel_dps2`` like every other speed
        change (user decision, reverting 1.18.1's instant bypass):
        one uniform rule — the acceleration default governs all
        commanded speed transitions, brake included. The SAFETY stop
        is unaffected: the e-stop and ``coast()`` cut the torque
        register, which the ramp does not govern, and remain instant.
        A hard local stop without torque-off is ``accel_dps2=0`` at
        construction.
        """
        if self._native_slot is not None:
            self._native_sb.servo_run(self._native_slot, 0)
            return
        self._abandon_pending()
        self._ensure_mode(_MODE_WHEEL)
        self._ensure_torque_on()
        self._write_goal_speed_signed(0)

    def coast(self):
        """Disable torque — wheel free-wheels."""
        if self._native_slot is not None:
            self._native_sb.servo_coast(self._native_slot)
            return
        self._abandon_pending()
        self._bus.write(self._id, _REG_TORQUE, bytes([0]))
        self._torque_on = False

    # ---- native adoption (1.45.0) -------------------------------------
    #
    # When a DriveBase adopts this motor onto the hard-tick native
    # bus, the machine.UART is released and the wheel-mode subset of
    # the Motor API routes through the C servo slots instead: run /
    # run_speed / dc / brake / stop / coast / angle / reset_angle.
    # Step-mode and feedback-register methods (run_angle, run_target,
    # hold, speed, load, stalled) raise with the roadmap item named —
    # loudly incomplete beats silently wrong.

    _native_slot = None       # class default; instance attr when adopted
    _native_sb = None
    _native_angle_offset = 0.0

    def _adopt_native(self, sb, slot):
        self._native_sb = sb
        self._native_slot = slot
        self._native_angle_offset = 0.0

    def _native_only_error(self, method):
        raise NotImplementedError(
            "%s is not yet available while this motor is adopted by "
            "the native drivebase (step-mode-in-C is the tracked "
            "follow-up); construct the motor without a native "
            "DriveBase to use it" % method)

    def hold(self):
        """Actively hold the current shaft angle so the position PID
        resists rotation. Subsequent ``run_speed`` / ``brake`` /
        ``coast`` calls transparently restore wheel mode.

        Holding never crosses a turn boundary (the shaft is meant to
        stay put), so the mechanism is chosen to avoid disturbing the
        servo's turn model:

        * If this motor has already been switched to multi-turn step
          mode (a prior ``run_angle``), hold with a zero-count step —
          step mode keeps holding its current target.
        * Otherwise (e.g. a ``DriveBase`` wheel that has only ever run
          in velocity mode), hold in single-turn position mode with
          goal=present. This deliberately does NOT zero the angle-limit
          registers, so a wheel motor keeps its stock single-turn
          present-position reads and its odometry stays intact.
        """
        self._abandon_pending()
        if self._step_limits_zeroed:
            self._ensure_mode(_MODE_STEP)
            self._ensure_torque_on()
            # Zero-count step: "stay at the current commanded position".
            self._write_step(0)
            return
        present = self._read_present_pos()
        if present is None:
            return   # bus silent — bail rather than write into the void
        # Anchor goal=present BEFORE the mode flip so the position PID
        # activates already-at-target and can't drift.
        self._bus.write(self._id, _REG_GOAL_POSITION,
                        bytes([present & 0xFF, (present >> 8) & 0xFF]))
        self._ensure_mode(_MODE_POSITION)
        self._ensure_torque_on()

    def done(self):
        """Pybricks-style status check for an in-flight
        ``run_angle(wait=False)`` move. Returns ``True`` if no move
        is in flight (the normal case) or the active move has parked
        (its remaining-to-target register has counted down to within
        tolerance). Returns ``False`` while the move is still running.

        Calling ``done()`` is what advances the move: each call reads
        the step-mode remaining register once. For a move larger than 7
        turns (issued as back-to-back steps), ``done()`` writes the next
        step once the current one parks, and on the final step runs the
        end-of-move ``then=`` dispatch and clears the pending state. So
        polling cadence matters for >7-turn moves — the wheel sits idle
        at each step boundary until the next ``done()`` advances it.
        With a typical ``time.sleep_ms(10)`` poll that's a single-tick
        gap; if you never poll, a multi-step move stalls after the
        first step."""
        if self._pending is None:
            return True
        return self._poll_pending()

    def _poll_pending(self):
        """One iteration of the wait=False state machine. See ``done``."""
        state = self._pending
        rem = self._read_step_remaining()
        if rem is None:
            # Bus glitch — keep waiting, tolerate transient drops.
            return False
        tol = state["tol_counts"]
        if not state["started"]:
            # Guard against a stale ~0 read before the servo has loaded
            # the step: only start watching for completion once the
            # remaining register confirms the move launched.
            if abs(rem) > tol:
                state["started"] = True
            return False
        if abs(rem) > tol:
            return False
        # Current step parked — bank the counts it actually travelled.
        self._advance_accum(state["first"] - rem)
        if state["remaining_counts"] == 0:
            # Whole move complete — run end-of-move dispatch and clear.
            then = state["then"]
            self._pending = None
            self._dispatch_then(then)
            return True
        # >7-turn move: issue the next step.
        step = state["remaining_counts"]
        if step >  _MAX_STEP_COUNTS: step =  _MAX_STEP_COUNTS
        if step < -_MAX_STEP_COUNTS: step = -_MAX_STEP_COUNTS
        self._write_step(step)
        state["remaining_counts"] -= step
        state["first"]   = step
        state["started"] = False
        return False

    def _deg_from_accum(self):
        deg = (self._accum_count - self._zero_offset_count) * 360.0 / _COUNTS_PER_REV
        return -deg if self._invert else deg

    def angle(self):
        """Return shaft angle in degrees, multi-turn accumulated.

        In STEP mode the present-position register reads remaining-to-
        target rather than absolute position, so we can't derive the
        angle from it — return the software accumulator, which
        ``run_angle`` advances by the executed step counts as each step
        parks. In wheel/position mode, rebuild the accumulator from the
        encoder via the wrap heuristic.
        """
        if self._op_mode == _MODE_STEP:
            if not self._accum_initialized:
                return None
            return self._deg_from_accum()

        raw = self._read_present_pos()
        if raw is None:
            return None
        if not self._accum_initialized:
            # First read ever: take the absolute position as the baseline.
            self._accum_count = raw
            self._accum_initialized = True
        elif self._last_raw is None:
            # Rebaseline after a mode change: adopt this read as the new
            # delta reference WITHOUT adding a (cross-mode, meaningless)
            # delta and WITHOUT discarding the accumulated count.
            pass
        else:
            delta = raw - self._last_raw
            # Wrap correction across the 0..4095 boundary (full
            # revolution = 4096 counts). Any single read interval
            # that produced more than half-revolution of motion is
            # treated as a wrap. To avoid mis-correction, the caller
            # must poll fast enough that no single sample period
            # advances more than 2048 counts (half a revolution) —
            # at the ST-3215's max ~360 dps that's once per ~0.5s,
            # but DriveBase polls every scheduler tick (1 kHz) so
            # this is comfortable.
            if delta >  2048:
                delta -= 4096
            elif delta < -2048:
                delta += 4096
            self._accum_count += delta
        self._last_raw = raw
        return self._deg_from_accum()

    def _advance_accum(self, executed_counts):
        """Add ``executed_counts`` (motor-frame) of completed step
        motion to the software accumulator. Used by ``run_angle`` /
        ``done`` because step mode's present register can't be read as
        a position."""
        self._accum_count += int(executed_counts)
        self._accum_initialized = True

    def reset_angle(self, angle=0):
        """Set the current shaft angle to ``angle`` (degrees)."""
        # Drain any pending wrap correction so the offset is taken
        # against an up-to-date accumulator.
        current = self.angle()
        if current is None:
            return
        # Solve for new offset such that future angle() returns ``angle``.
        offset_change_deg = current - float(angle)
        offset_change_count = int(round(offset_change_deg * _COUNTS_PER_REV / 360.0))
        if self._invert:
            offset_change_count = -offset_change_count
        self._zero_offset_count += offset_change_count

    # --- closed-loop position move ----------------------------------------

    def run_angle(self, deg_per_s, target_angle, wait=True,
                  tolerance_deg=0.5, kp=None, poll_ms=None,
                  debug=False, then="coast"):
        """Rotate by ``target_angle`` degrees at up to ``deg_per_s``,
        ending within ``tolerance_deg`` of the target.

        ``target_angle`` is RELATIVE and UNBOUNDED — ``run_angle(200,
        360)`` rotates one full turn forward, ``run_angle(200, 1080)``
        three turns, ``run_angle(200, -540)`` one and a half turns
        back. Direction is the sign of ``target_angle``.

        Implementation: the servo is driven in **step mode** (op_mode=3)
        for the move. In step mode the goal-position register is a
        *signed relative step* — write N counts and the shaft advances
        N counts from where it is, with no single-turn 0/4095 wrap to
        cross (the prerequisite is angle limits = 0, set once on the
        first call by ``_ensure_step_limits``). This is what makes
        moves past 180° / past one full turn work: the older
        single-turn position mode (op_mode=0) clamps to 0..4095 and a
        target across the boundary was executed the *wrong way round*,
        capping real motion at roughly half a turn.

        A single step write covers up to ±7 turns (the STS multi-turn
        envelope); larger moves are issued as back-to-back ±7-turn
        steps, each parking before the next — still no boundary to
        cross. The servo's internal PID handles convergence (≈0.088°
        per encoder count); completion is detected by reading the
        step-mode *remaining-to-target* register and waiting for it to
        count down to ~0 (see ``_read_step_remaining``). The shaft-angle
        accumulator is advanced by the counts actually travelled so
        ``angle()`` / ``reset_angle`` stay correct across the move.

        ``then`` selects the end-state, pybricks-style:

        * ``"coast"`` (default) — cut torque; wheel free-wheels. The
          next ``run_speed`` / ``brake`` / ``run_angle`` transparently
          re-enables torque and restores the mode it needs.
        * ``"brake"`` — restore wheel mode and write goal_speed=0 so
          the servo's velocity loop actively holds zero rotation rate.
        * ``"hold"`` — leave the servo in step mode; its position PID
          is already holding the target and resisting rotation.

        ``wait=False`` kicks off the move and returns immediately
        without blocking. Use it for concurrent multi-motor moves,
        pybricks-style::

            left.run_angle(60, 720, wait=False)
            right.run_angle(60, 720, wait=False)
            while not (left.done() and right.done()):
                time.sleep_ms(10)

        Multi-revolution targets are supported in ``wait=False`` mode
        too. For a move within ±7 turns the whole thing is one step
        write and ``done()`` simply reports convergence; for a larger
        move ``done()`` issues each subsequent ±7-turn step once the
        previous one parks, so you must keep polling. The end-state
        ``then=`` dispatch is deferred until ``done()`` reports the
        final step has converged.

        Any subsequent motion command (``run``, ``run_speed``, ``brake``,
        ``coast``, ``hold``, ``run_angle``) supersedes a pending
        ``wait=False`` move — the new command takes over and the
        pending state is dropped (pybricks "new command wins").

        The legacy ``kp`` / ``poll_ms`` / ``debug`` arguments are
        accepted for back-compat with the velocity-mode implementation
        but no longer apply — the PID lives on the servo, not in Python.
        """
        estop.check()
        if then not in ("coast", "brake", "hold"):
            raise ValueError(
                "then must be 'coast', 'brake', or 'hold' (got %r)" % then)
        if target_angle == 0:
            return
        max_dps = abs(float(deg_per_s))
        if max_dps <= 0:
            return

        # Motor-frame target in encoder counts (``invert`` flips the
        # commanded direction; ``angle()`` already reports user-frame
        # degrees, so done-detection below stays in user frame).
        target_counts = int(round(float(target_angle) *
                                  _COUNTS_PER_REV / 360.0))
        if self._invert:
            target_counts = -target_counts

        # Goal-speed register is unsigned in step mode (direction is in
        # the goal-position sign). Clamp to the per-instance max_dps.
        capped_dps = max_dps if max_dps < self._max_dps else self._max_dps
        speed_steps = int(round(capped_dps * self._steps_per_dps))
        if speed_steps < 1:
            speed_steps = 1
        if speed_steps > 0x7FFF:
            speed_steps = 0x7FFF

        # Completion tolerance in counts (the step register parks within
        # a ~±2-count deadband, so keep a small floor).
        tol_counts = int(round(abs(float(tolerance_deg)) *
                               _COUNTS_PER_REV / 360.0))
        if tol_counts < 3:
            tol_counts = 3

        # New command supersedes any pending wait=False move.
        self._abandon_pending()

        # Bail if the bus is silent rather than command a move we can't
        # track. Probed BEFORE the mode/torque writes so a glitching
        # servo is left exactly as it was — since motors coast at
        # construction, enabling torque first and then bailing would
        # leave a stiff servo with no move commanded. (The value read
        # here is irrelevant; we only care that the servo answers.)
        if self._read_step_remaining() is None:
            return

        self._ensure_torque_on()
        self._ensure_step_limits()       # angle limits = 0 (once)
        self._ensure_mode(_MODE_STEP)    # op_mode = 3
        self._bus.write(self._id, _REG_GOAL_SPEED,
                        bytes([speed_steps & 0xFF,
                               (speed_steps >> 8) & 0xFF]))

        # First step (clamped to the ±7-turn envelope).
        first = target_counts
        if first >  _MAX_STEP_COUNTS: first =  _MAX_STEP_COUNTS
        if first < -_MAX_STEP_COUNTS: first = -_MAX_STEP_COUNTS
        self._write_step(first)
        remaining_counts = target_counts - first

        if not wait:
            self._pending = {
                "first":            first,
                "remaining_counts": remaining_counts,
                "tol_counts":       tol_counts,
                "then":             then,
                "started":          False,
            }
            return

        while True:
            self._advance_accum(self._await_step(first, speed_steps, tol_counts))
            if remaining_counts == 0:
                break
            # Issue the next ±7-turn step.
            first = remaining_counts
            if first >  _MAX_STEP_COUNTS: first =  _MAX_STEP_COUNTS
            if first < -_MAX_STEP_COUNTS: first = -_MAX_STEP_COUNTS
            self._write_step(first)
            remaining_counts -= first

        self._dispatch_then(then)

    def _await_step(self, step, speed_steps, tol_counts):
        """Block until the in-flight step parks (its remaining register
        counts down to within ``tol_counts`` of 0) or a time budget
        expires. Returns the counts actually travelled (``step`` minus
        the final remaining), for the shaft-angle accumulator.

        A ``started`` latch guards against a stale ~0 read at kickoff:
        we only watch for completion once the remaining register has
        first been seen larger than tolerance (the move launched)."""
        # Backstop deadline scales with the move (estimated travel time
        # ×3 for the accel/decel ramp). It only bites on a stall —
        # normal moves return the instant the step parks below tol — so
        # there is no fixed ceiling that could cut a slow multi-turn step
        # short.
        est_ms = int(abs(step) * 1000 / speed_steps + 200)
        deadline = time.ticks_ms() + est_ms * 3
        started = False
        last_rem = step
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            rem = self._read_step_remaining()
            if rem is not None:
                last_rem = rem
                if not started:
                    if abs(rem) > tol_counts:
                        started = True
                elif abs(rem) <= tol_counts:
                    last_rem = rem
                    break
            time.sleep_ms(10)
        return step - last_rem

    # --- ST-3215-specific extras ------------------------------------------

    def ping(self):
        return self._bus.ping(self._id)


class SyncServoGroup:
    """Coordinated multi-servo writes via SCServo SYNC WRITE.

    All servos must share one ``_SCServoBus`` (same UART). Mixed
    servo types (``ST3215``, ``ST3215Motor``, ``ST3032``, ``ST3032Motor``)
    are fine since they all speak the same SCS protocol — SYNC WRITE
    just blasts the same register on all listed IDs.

    Use this whenever you have multiple servos that should apply a
    setpoint at the same packet boundary (drivebase wheels, multi-
    finger gripper) — each servo receives its byte slot of the
    broadcast packet at the same instant, instead of N serialised
    individual writes.

    Example
    -------
    ::

        from openbricks.drivers.st3215 import ST3215Motor, SyncServoGroup

        left  = ST3215Motor(servo_id=1)
        right = ST3215Motor(servo_id=2, invert=True)
        group = SyncServoGroup([left, right])

        # Both wheels start moving at the same packet boundary —
        # one SYNC WRITE instead of two individual writes.
        group.set_goal_speeds([200, 200])
    """

    def __init__(self, servos):
        if not servos:
            raise ValueError("SyncServoGroup needs at least one servo")
        bus = servos[0]._bus
        for s in servos[1:]:
            if s._bus is not bus:
                raise ValueError(
                    "SyncServoGroup: all servos must share one UART bus")
        self._bus    = bus
        self._servos = list(servos)

    def set_goal_speeds(self, speeds_dps):
        """Write goal-speed on every servo in one SYNC WRITE packet.

        ``speeds_dps`` is a list parallel to the servos given at
        construction. Each servo's own ``_encode_goal_speed`` is
        used, so per-servo ``invert`` / ``steps_per_dps`` /
        ``max_dps`` are respected.

        Servos that don't expose ``_encode_goal_speed`` (i.e. the
        position-mode ``ST3215`` class) raise ``TypeError``.
        """
        estop.check()
        if len(speeds_dps) != len(self._servos):
            raise ValueError(
                "speed count (%d) doesn't match servo count (%d)"
                % (len(speeds_dps), len(self._servos)))
        servo_data = []
        for servo, dps in zip(self._servos, speeds_dps):
            encode = getattr(servo, "_encode_goal_speed", None)
            if encode is None:
                raise TypeError(
                    "servo id=%s isn't a wheel-mode servo "
                    "(no _encode_goal_speed method)" % servo._id)
            v = encode(dps)
            servo_data.append(
                (servo._id, bytes([v & 0xFF, (v >> 8) & 0xFF])))
        # Motors coast at construction (and after ``coast()``), so a
        # goal-speed write alone would be silently ignored by a
        # torque-off servo. Restore mode + torque per member first —
        # both are cached, so steady-state group commands still cost
        # exactly one SYNC WRITE packet.
        for servo in self._servos:
            servo._ensure_mode(_MODE_WHEEL)
            servo._ensure_torque_on()
        self._bus.sync_write(_REG_GOAL_SPEED, 2, servo_data)
