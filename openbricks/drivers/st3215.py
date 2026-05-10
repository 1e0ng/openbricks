# SPDX-License-Identifier: MIT
"""
ST-3215 (Waveshare/FeeTech) serial bus servo.

These servos daisy-chain on a half-duplex UART bus: one TX line shared by
host and all servos, each servo addressed by a 1-byte ID. Packet format (as
in the FeeTech SCServo / Dynamixel-style protocol):

    0xFF 0xFF  ID  LEN  INSTR  PARAM...  CHECKSUM

    CHECKSUM = ~(ID + LEN + INSTR + sum(PARAM)) & 0xFF
    LEN      = number of params + 2

Common instructions:
    0x01 PING            ->  probe the servo
    0x02 READ            ->  READ  reg len
    0x03 WRITE           ->  WRITE reg value...

Key registers (ST-3215):
    0x21  Operation Mode — 0 = position, 1 = wheel/continuous
    0x28  Torque Switch  — 1 = enable, 0 = coast
    0x2A  Goal position (low, high) — int16, 0..4095 over ~360°
    0x2E  Goal speed (low, high) — sign-magnitude; in wheel mode this
          is the velocity setpoint (bit 15 of high byte = direction)
    0x38  Present position (low, high) — read only. 12-bit absolute
          angle within one revolution: 0..4095 over 360°. Wraps to 0
          at every full turn; we accumulate multi-turn revolutions in
          software via a wrap heuristic in ``ST3215Wheel.angle``.

Half-duplex wiring: most ST-3215 boards use a single data line driven by a
TX/RX switching circuit, but MicroPython UART pins are usually separate. If
your adapter exposes TX and RX separately, just wire them normally. If you
have a true half-duplex bus, you'll need a driver chip or a direction-enable
GPIO — set ``dir_pin`` below.

Two classes here:

* ``ST3215`` — position-mode (Servo interface), for grippers / lifts /
  sensor turrets. ``move_to(angle)`` blocks until reached.

* ``ST3215Wheel`` — continuous-rotation mode (Motor interface), for
  drivebase wheels. Switches the servo into mode=1 at construction
  and exposes ``run_speed(dps)`` / ``angle()`` / ``brake()`` so it
  drops into ``DriveBase`` the same way ``MG370Motor`` does.

Only a minimal subset of the protocol is implemented here. PR welcome.
"""

import time

from machine import UART, Pin

from openbricks.interfaces import Motor, Servo

_HEADER = b"\xFF\xFF"

_BROADCAST_ID = 0xFE

_INSTR_PING       = 0x01
_INSTR_READ       = 0x02
_INSTR_WRITE      = 0x03
_INSTR_SYNC_WRITE = 0x83

_REG_OP_MODE       = 0x21
_REG_TORQUE        = 0x28
_REG_GOAL_POSITION = 0x2A
_REG_GOAL_SPEED    = 0x2E
_REG_PRESENT_POS   = 0x38

_MODE_POSITION = 0
_MODE_WHEEL    = 1

# Hardware: 4096 encoder counts per output revolution.
_COUNTS_PER_REV = 4096

# Speed register units. The Feetech datasheet uses "step/sec"; one step
# is 360/4096 ≈ 0.0879 deg, so 1 dps ≈ 11.378 step/sec. Exposed as a
# kwarg in case future ST-3215 revisions ship with a different scale.
_DEFAULT_STEPS_PER_DPS = _COUNTS_PER_REV / 360.0   # = 11.378


class _SCServoBus:
    """Shared UART bus. One instance per physical bus; many servos per bus."""

    def __init__(self, uart_id, tx, rx, baud=1_000_000, dir_pin=None):
        self._uart = UART(uart_id, baudrate=baud, tx=tx, rx=rx, timeout=50)
        self._dir = Pin(dir_pin, Pin.OUT, value=0) if dir_pin is not None else None

    def _checksum(self, parts):
        s = 0
        for p in parts:
            s += p
        return (~s) & 0xFF

    def _tx(self, data):
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


class ST3215Wheel(Motor):
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

    def __init__(self, servo_id, uart_id=1, tx=17, rx=16,
                 baud=1_000_000, dir_pin=None,
                 invert=False,
                 steps_per_dps=_DEFAULT_STEPS_PER_DPS,
                 max_dps=600.0):
        self._id    = servo_id
        self._bus   = self._bus_for(uart_id, tx, rx, baud, dir_pin)
        self._invert = bool(invert)
        self._steps_per_dps = float(steps_per_dps)
        self._max_dps       = float(max_dps)

        # Software multi-turn accumulator state.
        self._last_raw    = None      # last present-position read (signed 16-bit)
        self._accum_count = 0         # accumulated counts since reset
        self._zero_offset_count = 0   # set by reset_angle()

        # Switch the servo into wheel/continuous mode.
        self._bus.write(self._id, _REG_OP_MODE, bytes([_MODE_WHEEL]))
        self._bus.write(self._id, _REG_TORQUE,  bytes([1]))

    # --- internal helpers -------------------------------------------------

    def _read_present_pos(self):
        # Present-position is a 12-bit absolute angle within one
        # revolution, range 0..4095 (NOT a free-running multi-turn
        # counter). It wraps to 0 at every full turn — multi-turn
        # tracking is done in software via the wrap heuristic in
        # angle().
        data = self._bus.read(self._id, _REG_PRESENT_POS, 2)
        if data is None:
            return None
        return (data[0] | (data[1] << 8)) & 0x0FFF

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

    # --- Motor interface --------------------------------------------------

    def run(self, power):
        """Open-loop wrapper: power -100..100 → run_speed scaled to max_dps."""
        if power >  100: power =  100
        if power < -100: power = -100
        self.run_speed(self._max_dps * power / 100.0)

    def run_speed(self, deg_per_s):
        """Set continuous wheel velocity in degrees per second."""
        v = self._encode_goal_speed(deg_per_s)
        self._bus.write(self._id, _REG_GOAL_SPEED,
                        bytes([v & 0xFF, (v >> 8) & 0xFF]))

    def brake(self):
        """Hold zero velocity (servo's internal loop actively brakes)."""
        self._write_goal_speed_signed(0)

    def coast(self):
        """Disable torque — wheel free-wheels."""
        self._bus.write(self._id, _REG_TORQUE, bytes([0]))

    def angle(self):
        """Return shaft angle in degrees, multi-turn accumulated."""
        raw = self._read_present_pos()
        if raw is None:
            return None
        if self._last_raw is None:
            # First read: take the absolute position as the baseline.
            self._accum_count = raw
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
        deg = (self._accum_count - self._zero_offset_count) * 360.0 / _COUNTS_PER_REV
        return -deg if self._invert else deg

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
                  tolerance_deg=2.0, kp=2.0, poll_ms=20):
        """Rotate by ``target_angle`` degrees at up to ``deg_per_s``.

        Implementation: Python position-PID on top of the servo's
        internal velocity loop. Each ``poll_ms`` tick we read the
        current angle, compute ``target_velocity = kp × (target -
        current)`` clamped to ``±deg_per_s``, and write that as the
        goal-speed. As the position error shrinks the commanded
        velocity tapers naturally — no separate decel ramp, no
        open-loop "when to brake" decision (which is what makes the
        simpler bang-bang approach overshoot at >50 Hz polling).

        ``target_angle`` is RELATIVE — calling ``run_angle(200, 360)``
        rotates 360° forward from the present position.

        Convergence on a typical bench rig (Feetech ST-3215 over
        1 Mbps UART, 50 Hz outer loop): within ~1–2° of the target
        with the defaults. Tune ``kp`` up for faster convergence
        (risk: chatter near target if poll_ms gets too coarse) or
        ``tolerance_deg`` down for tighter end position (risk: the
        loop can hang if the servo can't actually stop within that
        band — in which case raise ``tolerance_deg`` first, before
        chasing kp).

        ``wait=False`` kicks off motion at the cruise speed in the
        right direction and returns; the caller must poll and brake.
        """
        start = self.angle()
        if start is None:
            return
        target  = start + float(target_angle)
        max_dps = abs(float(deg_per_s))
        tol     = abs(float(tolerance_deg))

        if not wait:
            direction = 1.0 if target_angle >= 0 else -1.0
            self.run_speed(max_dps * direction)
            return

        while True:
            current = self.angle()
            if current is None:
                time.sleep_ms(poll_ms)
                continue
            error = target - current
            if abs(error) < tol:
                break
            target_dps = kp * error
            if target_dps >  max_dps: target_dps =  max_dps
            if target_dps < -max_dps: target_dps = -max_dps
            self.run_speed(target_dps)
            time.sleep_ms(poll_ms)
        self.brake()

    # --- ST-3215-specific extras ------------------------------------------

    def ping(self):
        return self._bus.ping(self._id)


class SyncServoGroup:
    """Coordinated multi-servo writes via SCServo SYNC WRITE.

    All servos must share one ``_SCServoBus`` (same UART). Mixed
    servo types (``ST3215``, ``ST3215Wheel``, future ``ST3032`` etc.)
    are fine since they all speak the same protocol — SYNC WRITE
    just blasts the same register on all listed IDs.

    Use this whenever you have multiple servos that should apply a
    setpoint at the same packet boundary (drivebase wheels, multi-
    finger gripper) — each servo receives its byte slot of the
    broadcast packet at the same instant, instead of N serialised
    individual writes.

    Example
    -------
    ::

        from openbricks.drivers.st3215 import ST3215Wheel, SyncServoGroup

        left  = ST3215Wheel(servo_id=1)
        right = ST3215Wheel(servo_id=2, invert=True)
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
        self._bus.sync_write(_REG_GOAL_SPEED, 2, servo_data)
