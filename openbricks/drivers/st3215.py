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
from collections import namedtuple

from openbricks.interfaces import Motor, Servo
from openbricks import parameters
from openbricks.parameters import Stop, DriveMode

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
# RAM torque cap, 0..1000 = 0..100 % of stall (SMS_STS SDK
# ``TORQUE_LIMIT_L`` = 48). The servo loads it from the EPROM
# max-torque register at power-on; run_until_stalled's duty_limit
# writes it for the duration of the run and restores what it read.
_REG_TORQUE_LIMIT  = 0x30
# The health block (SMS_STS memory table, Feetech STS tutorial):
# present voltage 62 (0.1 V per LSB), temperature 63 (°C), status
# 65 (protection flags), present current 69/70 (6.5 mA per LSB).
# One 9-byte read from 0x3E covers all of them on a Python-owned
# bus; an adopted motor stages them one register at a time.
_REG_PRESENT_VOLTAGE = 0x3E
_REG_PRESENT_TEMP    = 0x3F
_REG_STATUS          = 0x41
_REG_PRESENT_CURRENT = 0x45
_CURRENT_LSB_A = 0.0065
# Status bits, low to high: the protections the datasheet names
# (over-voltage outside 9-14 V, over-heat above 80 C, over-load
# above 80 % of stall for 2 s) plus the SDK's sensor/current/angle
# faults.
_STATUS_FLAGS = ("voltage", "sensor", "temperature", "current",
                 "angle", "overload")

ServoHealth = namedtuple("ServoHealth", ("voltage", "temperature",
                                         "current", "flags", "status"))


def _decode_status(status):
    return tuple(name for bit, name in enumerate(_STATUS_FLAGS)
                 if status & (1 << bit))


def _servo_health(volt_raw, temp_raw, current_raw, status):
    return ServoHealth(volt_raw / 10.0, float(temp_raw),
                       (current_raw & 0x7FFF) * _CURRENT_LSB_A,
                       _decode_status(status), status)


def _read_health(bus, servo_id):
    data = bus.read(servo_id, _REG_PRESENT_VOLTAGE, 9)
    if data is None:
        raise OSError(
            "servo id %s: no reply reading the health registers "
            "(0x3E-0x46) - check power and the servo bus wiring"
            % servo_id)
    return _servo_health(data[0], data[1], data[7] | (data[8] << 8),
                         data[3])

_MODE_POSITION = 0   # single-turn absolute position (0..4095 = 0..360°)
_MODE_WHEEL    = 1   # continuous velocity (wheel, servo-internal loop)
_MODE_PWM      = 2   # wheel OPEN-loop: raw duty, no internal control
_MODE_STEP     = 3   # step servo: goal_position is a SIGNED RELATIVE
                     # step; multi-turn capable (needs angle limits=0)

# Mode-2 duty register (upstream SMS_STS ``WritePwm``: GOAL_TIME, reg
# 44/45). Sign-magnitude with the direction bit at BIT 10, and — like
# the present-LOAD register (bench 2026-08-03) — bit 10 SET means the
# POSITIVE direction on our units, the OPPOSITE of the Feetech SDK's
# convention (bench 2026-08-12: SDK-signed duty drove every wheel
# backwards). Magnitude 0..1000 = 0..100% duty, linear ~0.894 dps
# free-run per unit on the ST-3032. Torque drops on the mode switch
# and must be re-enabled; the goal-SPEED register (0x2E) accepts
# writes in mode 2 but is inert.
_REG_GOAL_TIME = 0x2C
_PWM_SIGN_BIT  = 0x0400

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

# A move is "progressing" if the shaft advanced at least this many
# counts since the last check. Small enough to see a genuinely slow
# move (4 counts ~= 0.35 deg), large enough to ignore encoder jitter
# on a stationary shaft.
_STALL_PROGRESS_COUNTS = 4

# Consecutive failed feedback reads that mean the BUS is dead, not
# the shaft: matches the C drivebase's ST_DB_STALE_FAULT (~200 ms of
# silence at the hard-tick polling rate). A dead bus freezes counts
# exactly like a jam, so every stall path must check this FIRST —
# "the shaft is jammed" advice for an unplugged servo sends the user
# to the wrong part of the robot, and a wiring fault must raise, not
# report-and-continue (only mechanical stalls are survivable).
_DEAD_BUS_STALE = 20


def _native_bus_owns(uart_id):
    """Has the native C bus taken this UART over?

    Asked in C, not remembered in Python: the attached UART
    deliberately survives a program boundary (``reset_runtime`` clears
    the slots but not the hardware), so a fresh program's Python state
    would claim nobody owns pins the IDF driver still holds.
    """
    try:
        from openbricks import _native
        sb = getattr(_native, "st_bus", None)
        uart_num = getattr(sb, "uart_num", None)
        return uart_num is not None and uart_num() == uart_id
    except (ImportError, AttributeError):
        return False


class _SCServoBus:
    """Shared UART bus. One instance per physical bus; many servos per bus."""

    def __init__(self, uart_id, tx, rx, baud=1_000_000, dir_pin=None,
                 verify_writes=True):
        # Every write is confirmed against the servo's status packet
        # (see ``write``). Turn this off only for servos configured
        # with a status-return level that answers reads alone —
        # deliberately, never as a way to quieten an error.
        self.verify_writes = bool(verify_writes)
        # Motors constructed on this bus, so a later DriveBase
        # adoption can migrate them onto native slots.
        self._motors = []
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
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
        buf = b""
        while len(buf) < n and time.ticks_diff(deadline, time.ticks_ms()) > 0:
            chunk = self._uart.read(n - len(buf))
            if chunk:
                buf += chunk
            else:
                time.sleep_ms(1)
        return buf

    def write(self, servo_id, register, data, timeout_ms=20):
        """Write a register and CONFIRM the servo took it.

        The servo answers a write with a status packet
        (``FF FF <id> <len> <err> <chk>``). This used to be read and
        thrown away, which meant a write that never landed — because
        the servo was busy finishing an EEPROM cycle, because a
        connector was loose, because the id was wrong — looked exactly
        like a write that succeeded.

        That is never acceptable, and on this hardware it is actively
        dangerous: goal-speed 0 means MAXIMUM SPEED, so a lost speed
        write doesn't slow a move down, it sends the shaft flying
        (bench 2026-08-04, measured at 697 dps against a commanded
        200). Losses are now raised, never swallowed.

        Broadcasts (id 0xFE) are exempt — the protocol defines no
        reply for them. Set ``verify_writes=False`` on the servo if
        yours are configured with a status-return level that only
        answers reads; that is a deliberate choice, not a default.
        """
        length = len(data) + 3  # register + params + checksum + instr -> LEN = params + 2 conceptually; +1 for register
        params = bytes([register]) + bytes(data)
        body = bytes([servo_id, length, _INSTR_WRITE]) + params
        packet = _HEADER + body + bytes([self._checksum(body)])
        self._tx(packet)
        if servo_id == _BROADCAST_ID or not self.verify_writes:
            return
        resp = self._rx(6, timeout_ms=timeout_ms)
        self._check_write_ack(resp, servo_id, register, data)

    def _check_write_ack(self, resp, servo_id, register, data):
        detail = ("servo id %s register 0x%02X = %s"
                  % (servo_id, register, bytes(data)))
        if len(resp) < 6 or not resp.startswith(_HEADER):
            raise OSError(
                "no acknowledgement for a write to %s (got %d bytes). "
                "The write may never have landed. Check power and the "
                "TX/RX wiring, and that a servo really has that id "
                "(`openbricks servo-id --scan`). If these servos are "
                "configured to answer reads only, construct them with "
                "verify_writes=False." % (detail, len(resp)))
        if resp[2] != servo_id:
            raise OSError(
                "write to %s was acknowledged by servo id %d instead — "
                "two servos are answering to one id, or replies are "
                "arriving out of step" % (detail, resp[2]))
        if self._checksum(resp[2:5]) != resp[5]:
            raise OSError(
                "corrupt acknowledgement for a write to %s — the reply "
                "failed its checksum, so the bus is unreliable (wiring, "
                "termination, or baud mismatch)" % detail)
        if resp[4] != 0:
            # The datasheet names three protections: over-load
            # (>80%% of stall for 2 s), over-voltage (outside 9-14 V)
            # and over-heat (>80 C). It does not publish the bit
            # positions, so report the raw byte rather than invent a
            # decode.
            raise OSError(
                "servo reported error flags 0x%02X on a write to %s — "
                "typically over-load, over-voltage or over-temperature "
                "(reissuing a command clears the overload latch)"
                % (resp[4], detail))

    def read(self, servo_id, register, nbytes):
        length = 4
        params = bytes([register, nbytes])
        body = bytes([servo_id, length, _INSTR_READ]) + params
        packet = _HEADER + body + bytes([self._checksum(body)])
        self._tx(packet)
        # Response: FF FF ID LEN ERR DATA... CHK — verified like a
        # write ACK (sender, checksum), not merely length-checked:
        # reads feed the angle accumulator's wrap heuristic and the
        # step-park detector, so a wrong-sender or corrupt reply can
        # falsely park a step or corrupt multi-turn odometry. A bad
        # reply reads as "no reply" (None) — the callers' retry /
        # stale accounting is the right place for the loss, and every
        # loss is counted there, never absorbed. (Error FLAGS do not
        # invalidate a read: a latched overload doesn't corrupt
        # present-position, and failing every read on a protection
        # flag would look like a dead bus.)
        resp = self._rx(6 + nbytes)
        if len(resp) < 6 + nbytes or not resp.startswith(_HEADER):
            return None
        if resp[2] != servo_id:
            return None
        if self._checksum(resp[2:5 + nbytes]) != resp[5 + nbytes]:
            return None
        return resp[5:5 + nbytes]

    def ping(self, servo_id):
        # A ping must be answered by THE servo asked, with an intact
        # frame — "any 6 bytes" also matched stale residue and other
        # servos' replies, reporting a present servo that wasn't.
        body = bytes([servo_id, 2, _INSTR_PING])
        packet = _HEADER + body + bytes([self._checksum(body)])
        self._tx(packet)
        resp = self._rx(6)
        return (len(resp) == 6 and resp.startswith(_HEADER)
                and resp[2] == servo_id
                and self._checksum(resp[2:5]) == resp[5])

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


    def __init__(self, servo_id, uart_id=1, tx=14, rx=41,
                 baud=1_000_000, dir_pin=None,
                 min_raw=0, max_raw=4095, range_deg=360):
        self._id = servo_id
        if _native_bus_owns(uart_id):
            # The Motor classes take a native slot in this situation;
            # the position-mode servo has no native-slot path (slots
            # configure wheel mode), so opening its own machine.UART
            # here would put two drivers on one wire — the hard
            # tick's replies land in this driver's buffer and get
            # consumed as the wrong packet's answers (bench
            # 2026-08-04: a write to id 4 acknowledged by id 1).
            # Refuse loudly with the remedy instead of corrupting
            # both conversations.
            raise RuntimeError(
                "servo id %s: UART%s is owned by the native bus "
                "driver (a DriveBase adopted motors on it), and the "
                "position-mode ST3215/ST3032 class has no native-"
                "slot path. Put position-mode servos on a second "
                "UART, or use the Motor class (run/run_angle) which "
                "takes a native slot." % (servo_id, uart_id))
        self._bus = self._bus_for(uart_id, tx, rx, baud, dir_pin)
        # Registered so a LATER DriveBase adoption of this UART can
        # refuse with the same remedy instead of silently closing the
        # UART under us (migrate_bus_to_native migrates Motor
        # instances; position-mode servos cannot come across).
        self._bus._servos = getattr(self._bus, "_servos", [])
        self._bus._servos.append(self)
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
        """Move to ``angle_deg`` within the configured position range.

        ``speed`` (raw goal-speed register units) is optional; with
        ``wait=True`` polls until within 2% of target or a 3 s
        timeout."""
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
            deadline = time.ticks_add(time.ticks_ms(), 3000)
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                current = self.angle()
                if current is not None and abs(current - angle_deg) < self._range * 0.02:
                    return
                time.sleep_ms(20)

    def angle(self):
        """Current shaft angle in degrees within the position range,
        or ``None`` if the bus read timed out."""
        data = self._bus.read(self._id, _REG_PRESENT_POS, 2)
        if data is None:
            return None
        raw = data[0] | (data[1] << 8)
        return self._raw_to_deg(raw)

    def health(self):
        """Supply voltage (V), case temperature (°C), supply current
        (A) and the protection flags, as a ``ServoHealth`` tuple —
        see ``ST3215Motor.health``. Raises ``OSError`` if the bus is
        silent."""
        return _read_health(self._bus, self._id)

    def ping(self):
        """``True`` if the servo answers on the bus."""
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

    def __init__(self, servo_id, uart_id=1, tx=14, rx=41,
                 baud=1_000_000, dir_pin=None,
                 invert=False,
                 steps_per_dps=_DEFAULT_STEPS_PER_DPS,
                 max_dps=600.0,
                 accel_dps2=1500.0,
                 raise_on_stall=False,
                 stall_idle_ms=1000):
        self._id    = servo_id
        self._invert = bool(invert)
        self._steps_per_dps = float(steps_per_dps)
        self._max_dps       = float(max_dps)
        self._accel_dps2    = float(accel_dps2)
        # A run_angle that gives up REPORTS by default rather than
        # raising: on a mission robot one stalled task motor should
        # not abort the run. It is loud either way — console, run
        # log, and a False return. Pass raise_on_stall=True to make
        # it fatal instead.
        self._raise_on_stall = bool(raise_on_stall)
        # How long the shaft may sit still before the move is called
        # stuck. Independent of the total budget: a loaded move that
        # keeps inching is fine, a still one is not.
        self._stall_idle_ms = int(stall_idle_ms)
        # The torque cap currently on the wire, in the register's
        # 0..1000 raw units. Assumed full until run_until_stalled's
        # duty_limit writes a lower one; stalled() scales its load
        # threshold by this — under a 30 % cap the load can never
        # reach 80 % of FULL stall, so the unscaled threshold would
        # spin run_until_stalled forever.
        self._duty_limit_raw = 1000
        # ONE BUS, ONE OWNER. If the native C bus already drives this
        # UART — because a DriveBase adopted its wheels onto it — then
        # opening a MicroPython UART here would put two drivers on one
        # wire, and the hard tick's replies would be consumed as the
        # answers to our packets (bench 2026-08-04: a write to id 4
        # acknowledged by id 1). Take a native slot instead; the whole
        # Motor API already routes through slots.
        # Kept whether or not a MicroPython bus is ever opened: when
        # this motor goes straight onto a native slot there is no bus
        # object to recover these from later, and DriveBase adoption
        # still needs them to (idempotently) attach the UART.
        self._uart_id = uart_id
        self._uart_params = (uart_id, tx, rx, baud)
        if _native_bus_owns(uart_id):
            self._bus = None
            self._attach_task_slot()
            return
        self._bus   = self._bus_for(uart_id, tx, rx, baud, dir_pin)
        # Remembered so a later DriveBase adoption can migrate every
        # motor already on this bus (see ``migrate_bus_to_native``) —
        # adoption takes the UART away, and a motor left holding the
        # closed one would go silent.
        self._bus._motors.append(self)

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
        # left the servo in step mode (``then=Stop.HOLD``).
        self._op_mode    = _MODE_WHEEL
        self._torque_on  = False

        # Whether this servo's angle-limit registers have been zeroed
        # to unlock multi-turn step mode. Done lazily on the first
        # ``run_angle`` (NOT at construction) so a servo used purely as
        # a ``DriveBase`` wheel keeps its stock single-turn limits and
        # its present-position reads keep wrapping cleanly at one rev.
        self._step_limits_zeroed = False
        # One warning per motor if the servo won't store goal_acc.
        self._acc_mismatch_warned = False

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
        # program on the same power session. The mode write is
        # read-back verified: its ACK alone does not prove a cold
        # servo applied it.
        self._write_op_mode_verified(_MODE_WHEEL)
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

    # Write-and-read-back rounds before an op-mode change is declared
    # failed, and the pause between rounds. 0x21 is EEPROM-backed: a
    # servo still inside its own power-on init ACKs the write without
    # committing it (bench 2026-08-30 — a cold-started wheel held the
    # old mode at full torque, obeying goal_speed 0 while duty
    # commands hammered the register that mode ignores, and the robot
    # pivoted around it). The pause gives such a servo ~200 ms total
    # to finish booting; a healthy servo passes on round one.
    _MODE_VERIFY_TRIES = 8
    _MODE_VERIFY_PAUSE_MS = 25

    def _write_op_mode_verified(self, mode):
        """Set op_mode and confirm it on the SERVO, not just on the
        wire: write, read the register back, and repeat until it
        matches. The write's ACK proves transport; only the read-back
        proves application. Raises naming the servo and both modes
        when it never sticks."""
        got = None
        for attempt in range(self._MODE_VERIFY_TRIES):
            if attempt:
                time.sleep_ms(self._MODE_VERIFY_PAUSE_MS)
            self._bus.write(self._id, _REG_OP_MODE, bytes([mode]))
            data = self._bus.read(self._id, _REG_OP_MODE, 1)
            if data is not None and data[0] == mode:
                self._op_mode = mode
                return
            got = None if data is None else data[0]
        if got is None:
            raise OSError(
                "servo id %s: op_mode write ACKed but the read-back "
                "got no reply after %d attempts — check power and "
                "the servo bus wiring"
                % (self._id, self._MODE_VERIFY_TRIES))
        raise OSError(
            "servo id %s: op_mode still reads %d after %d verified "
            "writes of %d — the servo ACKs without applying, the "
            "signature of a controller still in its own power-on "
            "init. Give it a second after power-on before starting "
            "the program, then retry." % (self._id, got,
                                          self._MODE_VERIFY_TRIES, mode))

    def _ensure_mode(self, mode):
        """Write op_mode only when it differs from our tracked state.
        Saves a bus packet on the common case where the servo is already
        in the desired mode (e.g. ``run_speed`` after another
        ``run_speed``) and keeps the cache in sync after ``run_angle``
        returns with ``then=Stop.HOLD`` and leaves the servo in step mode.

        A mode change invalidates the present-position delta baseline:
        the register's meaning differs between modes (absolute position
        in wheel/position mode vs remaining-to-target in step mode), so
        force ``angle()`` to rebaseline on its next read rather than
        treat the cross-mode jump as real shaft motion. The accumulated
        count itself is preserved."""
        if self._op_mode != mode:
            self._write_op_mode_verified(mode)
            self._last_raw = None
            # A mode change can drop torque on the servo side (bench
            # 2026-08-12, entering mode 2) — the cache must not claim
            # otherwise, or the next _ensure_torque_on no-ops and the
            # motion command silently does nothing.
            self._torque_on = False

    def _ensure_torque_on(self):
        """Re-enable torque if a prior ``coast`` (or ``then=Stop.COAST``)
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

        These are EEPROM registers, and the guard flag is per
        INSTANCE — so before 1.56.0 every program run rewrote them on
        its first ``run_angle``, even though they were already zero
        from the previous run. Two costs, one of them a real bug:

        * endurance — an EEPROM cycle per program run, forever;
        * an EEPROM operation leaves the servo busy, and the writes
          that follow it (op-mode, goal-acc, goal-speed) are issued
          straight after with no acknowledgement checked. Bench
          2026-08-04: the FIRST move of every run behaved differently
          from every later move, with goal-acc reading 0 afterwards
          despite the constructor writing 171.

        So: read first, and only spend an EEPROM cycle if the limits
        are not already what we need. The common case — a servo that
        has run ``run_angle`` before — becomes two reads and no write
        at all.
        """
        if self._step_limits_zeroed:
            return
        lo = self._bus.read(self._id, _REG_MIN_ANGLE, 2)
        hi = self._bus.read(self._id, _REG_MAX_ANGLE, 2)
        if lo == b"\x00\x00" and hi == b"\x00\x00":
            self._step_limits_zeroed = True
            return                     # already multi-turn capable
        self._bus.write(self._id, _REG_MIN_ANGLE, bytes([0, 0]))
        self._bus.write(self._id, _REG_MAX_ANGLE, bytes([0, 0]))
        # Let the EEPROM cycle finish before anything else is written:
        # packets sent into that window are what went missing.
        time.sleep_ms(20)
        self._step_limits_zeroed = True

    def _write_motion_regs(self, speed_steps):
        """Write the two registers that govern how a step move runs —
        goal-acc and goal-speed — and VERIFY they took.

        ``_SCServoBus.write`` sends the packet and discards the status
        reply without checking it, so a dropped or rejected register
        write is silent. That is tolerable for most registers. It is
        not tolerable for these two, because on a Feetech servo
        **goal-speed 0 means maximum speed**: a lost speed write does
        not make the move slightly wrong, it makes the move run flat
        out. Bench 2026-08-04 measured a first move at ~700 dps
        against a commanded 200, with goal-acc reading 0 despite the
        constructor writing 171.

        Re-asserting goal-acc here (rather than trusting the
        constructor's write to have survived) is what keeps the
        documented uniform-acceleration rule true in step mode.

        The two registers are held to DIFFERENT standards, because
        getting them wrong costs different amounts:

        * goal-speed is verified and a mismatch REFUSES the move.
          Speed 0 means full speed; running anyway risks the shaft.
        * goal-acc is written and checked, but a mismatch only warns
          once. Bench 2026-08-04: an ST-3032 acknowledges the write
          and still reports 0 — the ramp is simply not settable on
          that unit. Refusing every move over an acceleration the
          servo won't store would ground a working robot to enforce a
          preference. Not silent, not fatal.
        """
        acc = self._encode_goal_acc()
        speed = bytes([speed_steps & 0xFF, (speed_steps >> 8) & 0xFF])
        for _ in range(2):
            self._bus.write(self._id, _REG_GOAL_ACC, bytes([acc]))
            self._bus.write(self._id, _REG_GOAL_SPEED, speed)
            got_acc = self._bus.read(self._id, _REG_GOAL_ACC, 1)
            if (got_acc is not None and got_acc[0] != acc
                    and not self._acc_mismatch_warned):
                self._acc_mismatch_warned = True
                print("openbricks: servo id %s stored goal_acc=%d, not "
                      "the %d asked for — this servo does not take an "
                      "acceleration ramp, so moves start and stop "
                      "abruptly. Motion is otherwise unaffected."
                      % (self._id, got_acc[0], acc))
            got_speed = self._bus.read(self._id, _REG_GOAL_SPEED, 2)
            if got_speed is None:
                continue                    # silent bus — retry once
            if (got_speed[0] | (got_speed[1] << 8)) == speed_steps:
                return
        raise OSError(
            "servo id %s would not accept its speed setting: asked "
            "goal_speed=%d, reads back %s. Refusing the move — "
            "goal_speed 0 means FULL SPEED on this servo, so running "
            "it now could send the shaft flying."
            % (self._id, speed_steps,
               None if got_speed is None
               else got_speed[0] | (got_speed[1] << 8)))

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
        if then == Stop.COAST:
            self._bus.write(self._id, _REG_TORQUE, bytes([0]))
            self._torque_on = False
        elif then == Stop.BRAKE:
            self._ensure_mode(_MODE_WHEEL)
            self._ensure_torque_on()
            self._write_goal_speed_signed(0)
        # else Stop.HOLD: step mode already holds the target — nothing to do.

    # --- Motor interface --------------------------------------------------

    def dc(self, duty):
        """True Pybricks ``Motor.dc()``: raw duty, no speed
        regulation. Switches the servo to its open-loop mode (2) and
        writes duty straight to the output stage — speed sags under
        load, exactly like a plain DC motor at fixed voltage.

        On a motor adopted by the native engine the bus belongs to
        the hard tick, so duty still maps onto ``run_speed`` scaled
        to ``max_dps`` there (transitional until the engine grows
        its own duty drive)."""
        estop.check()
        if duty >  100: duty =  100
        if duty < -100: duty = -100
        if self._native_slot is not None:
            self.run_speed(self._max_dps * duty / 100.0)
            return
        self._abandon_pending()
        self._ensure_mode(_MODE_PWM)
        self._ensure_torque_on()
        raw_duty = -duty if self._invert else duty
        raw = int(round(abs(raw_duty) * 10))
        if raw > 1000:
            raw = 1000
        if raw_duty > 0:
            raw |= _PWM_SIGN_BIT   # bit 10 = POSITIVE (load convention)
        self._bus.write(self._id, _REG_GOAL_TIME,
                        bytes([raw & 0xFF, (raw >> 8) & 0xFF]))

    def speed(self):
        """Measured shaft speed in deg/s from the present-speed
        register (sign-magnitude, bit 15; steps/s scaled by
        ``steps_per_dps``). Returns ``None`` if the bus is silent,
        matching ``angle()``. On an adopted motor (native DriveBase)
        the value comes from the hard-tick pump's widened feedback
        read — no extra bus traffic."""
        if self._native_slot is not None:
            steps, _load, fresh = self._native_sb.servo_feedback(
                self._native_slot)
            if not fresh:
                return None
            return steps / self._steps_per_dps
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
        bus is silent. On an adopted motor the value comes from the
        hard-tick pump's widened feedback read (user frame — the
        slot carries this motor's invert)."""
        if self._native_slot is not None:
            _steps, load_raw, fresh = self._native_sb.servo_feedback(
                self._native_slot)
            if not fresh:
                return None
            return load_raw * self.STALL_TORQUE_MNM / 1000.0
        data = self._bus.read(self._id, _REG_PRESENT_LOAD, 2)
        if data is None:
            return None
        raw = data[0] | (data[1] << 8)
        magnitude = raw & 0x3FF          # 0..1000 = 0..100 % of stall
        mnm = magnitude * self.STALL_TORQUE_MNM / 1000.0
        # Bit 10 = POSITIVE direction (bench 2026-08-03, both spin
        # directions — the Feetech SDK's decode reads it inverted and
        # made a forward-driving motor report negative torque).
        if not (raw & 0x400):
            mnm = -mnm
        if self._invert:
            mnm = -mnm
        return mnm

    def health(self):
        """Supply voltage (V), case temperature (°C), supply current
        (A) and the servo's protection flags, as a ``ServoHealth``
        tuple: ``flags`` holds the set names among ``voltage``,
        ``sensor``, ``temperature``, ``current``, ``angle`` and
        ``overload`` (empty when healthy), ``status`` the raw byte.
        From the present-voltage / temperature / status / current
        registers (0.1 V, °C, 6.5 mA per LSB). Raises ``OSError``
        when the bus is silent — a health check that returns nothing
        is the failure it exists to catch. On an adopted motor the
        four reads are staged through the native pump (a few ms —
        for a log line, not a control loop)."""
        if self._native_slot is not None:
            return _servo_health(
                self._reg_read_u16(_REG_PRESENT_VOLTAGE, 1),
                self._reg_read_u16(_REG_PRESENT_TEMP, 1),
                self._reg_read_u16(_REG_PRESENT_CURRENT, 2),
                self._reg_read_u16(_REG_STATUS, 1))
        return _read_health(self._bus, self._id)

    def stalled(self):
        """``True`` when the servo is pushing hard (load magnitude at
        least ``STALL_LOAD_PCT`` percent of stall) but barely moving
        (speed magnitude at most ``STALL_SPEED_DPS``) — the Pybricks
        ``Motor.stalled()`` contract, from the servo's own feedback
        registers. Raises ``OSError`` if the bus is silent (a silent
        bus must not read as "not stalled")."""
        # Threshold in raw 0.1 %-of-stall units, scaled by the active
        # torque cap: at full torque this is STALL_LOAD_PCT * 10
        # (unchanged), under a duty_limit it is the same fraction of
        # what the servo is ALLOWED to push.
        threshold = self.STALL_LOAD_PCT * self._duty_limit_raw // 100
        if self._native_slot is not None:
            steps, load_raw, fresh = self._native_sb.servo_feedback(
                self._native_slot)
            if not fresh:
                raise OSError("bus silent while reading stall state")
            return (abs(load_raw) >= threshold
                    and abs(steps / self._steps_per_dps)
                        <= self.STALL_SPEED_DPS)
        data = self._bus.read(self._id, _REG_PRESENT_LOAD, 2)
        spd = self.speed()
        if data is None or spd is None:
            raise OSError("bus silent while reading stall state")
        load_magnitude = (data[0] | (data[1] << 8)) & 0x3FF
        return (load_magnitude >= threshold
                and abs(spd) <= self.STALL_SPEED_DPS)

    # ---- duty_limit (run_until_stalled's temporary torque cap) ----

    _USER_TXN_TIMEOUT_MS = 500

    def _reg_read_u16(self, reg, nbytes=2):
        """Read a raw register value, adopted or not. On an adopted
        motor the read is staged into the native pump (Python must
        not talk on a natively-owned UART) and polled to its
        verified result; a loss raises, naming servo and register."""
        if self._native_slot is not None:
            if not self._native_sb.servo_user_read(
                    self._native_slot, reg, nbytes):
                raise OSError(
                    "servo id %s: could not stage read of register "
                    "0x%02X (another register transaction is "
                    "unresolved)" % (self._id, reg))
            return self._native_user_wait(reg, "read")
        data = self._bus.read(self._id, reg, nbytes)
        if data is None:
            raise OSError(
                "servo id %s: no reply reading register 0x%02X"
                % (self._id, reg))
        value = data[0]
        if nbytes > 1:
            value |= data[1] << 8
        return value

    def _reg_write_u16(self, reg, value, nbytes=2):
        """Write a raw register value, adopted or not — verified
        either way (the Python bus raises on a lost ACK; the native
        pump retries then latches, surfaced by the poll)."""
        if self._native_slot is not None:
            if not self._native_sb.servo_user_write(
                    self._native_slot, reg, value, nbytes):
                raise OSError(
                    "servo id %s: could not stage write of register "
                    "0x%02X (another register transaction is "
                    "unresolved)" % (self._id, reg))
            self._native_user_wait(reg, "write")
            return
        if nbytes == 1:
            payload = bytes([value & 0xFF])
        else:
            payload = bytes([value & 0xFF, (value >> 8) & 0xFF])
        self._bus.write(self._id, reg, payload)

    def _native_user_wait(self, reg, what):
        import time
        deadline = time.ticks_add(time.ticks_ms(),
                                  self._USER_TXN_TIMEOUT_MS)
        while True:
            status, value = self._native_sb.servo_user_poll(
                self._native_slot)
            if status == 1:
                return value
            if status == -1:
                raise OSError(
                    "servo id %s: %s of register 0x%02X lost on the "
                    "wire after retries — check power and the servo "
                    "bus wiring" % (self._id, what, reg))
            if status == -2:
                raise OSError(
                    "servo id %s: no register transaction pending "
                    "for %s of 0x%02X (internal staging bug)"
                    % (self._id, what, reg))
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                raise OSError(
                    "servo id %s: %s of register 0x%02X timed out "
                    "after %d ms" % (self._id, what, reg,
                                     self._USER_TXN_TIMEOUT_MS))
            time.sleep_ms(2)

    def _duty_limit_push(self, duty_limit):
        """Cap torque at ``duty_limit`` percent for the duration of a
        run_until_stalled. Returns the previous register value so
        ``_duty_limit_pop`` restores exactly what was there — a servo
        with a custom cap keeps it."""
        try:
            duty = float(duty_limit)
        except (TypeError, ValueError):
            raise ValueError(
                "duty_limit must be a number in (0, 100], got %r"
                % (duty_limit,))
        if not 0 < duty <= 100:
            raise ValueError(
                "duty_limit must be in (0, 100] percent, got %r"
                % (duty_limit,))
        raw = max(1, min(1000, int(duty * 10)))
        previous = self._reg_read_u16(_REG_TORQUE_LIMIT)
        self._reg_write_u16(_REG_TORQUE_LIMIT, raw)
        self._duty_limit_raw = raw
        return previous

    def _duty_limit_pop(self, previous):
        self._reg_write_u16(_REG_TORQUE_LIMIT, previous)
        self._duty_limit_raw = previous if previous > 0 else 1000

    def run_speed(self, deg_per_s):
        """Set continuous wheel velocity in degrees per second."""
        estop.check()
        if self._native_slot is not None:
            # The slot was attached WITH this motor's invert flag, so
            # pass the user-frame value; the slot applies the sign.
            self._native_pending = None      # new command wins
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
            self._native_pending = None      # new command wins
            self._native_sb.servo_run(self._native_slot, 0)
            return
        self._abandon_pending()
        self._ensure_mode(_MODE_WHEEL)
        self._ensure_torque_on()
        self._write_goal_speed_signed(0)

    def coast(self):
        """Disable torque — wheel free-wheels."""
        if self._native_slot is not None:
            self._native_pending = None      # new command wins
            self._native_sb.servo_coast(self._native_slot)
            return
        self._abandon_pending()
        self._bus.write(self._id, _REG_TORQUE, bytes([0]))
        self._torque_on = False

    # ---- native adoption (1.45.0; step mode 1.46.0) -------------------
    #
    # When a DriveBase adopts this motor onto the hard-tick native
    # bus, the machine.UART is released and the Motor API routes
    # through the C servo slots instead: run / run_speed / dc / brake
    # / stop / coast / angle / reset_angle (1.45.0), plus run_angle /
    # hold / done via the per-slot position moves in st_move_core
    # (1.46.0). Feedback-register methods (speed, load, stalled)
    # still raise — the pump reads position only; adding per-slot
    # register reads is the tracked follow-up. Loudly incomplete
    # beats silently wrong.

    _native_slot = None       # class default; instance attr when adopted
    _native_sb = None
    _native_angle_offset = 0.0
    _native_pending = None    # then= of an in-flight wait=False move

    def _adopt_into_drivebase(self, right, wheel_diameter_mm,
                              axle_track_mm, imu=None, accel_dps2=400.0,
                              drive=DriveMode.DUTY):
        """DriveBase's adoption hook (polymorphic — the sim's shim
        motors implement their own). Returns the serial-native engine,
        or None when the firmware native bus is absent (then there is
        NO fallback: serial drivebases are native-only by design)."""
        from openbricks import _native
        sbmod = getattr(_native, "st_bus", None)
        if sbmod is None or not hasattr(sbmod, "attach_uart"):
            return None
        from openbricks.robotics.native_drivebase import _SerialNativeEngine
        return _SerialNativeEngine.adopt_motors(
            self, right, wheel_diameter_mm=wheel_diameter_mm,
            axle_track_mm=axle_track_mm, imu=imu, accel_dps2=accel_dps2,
            drive=drive)

    # First free slot wins, whoever asks. Reserving 0/1 for wheels
    # made construction order matter: a script that built its task
    # motors first exhausted the free slots before its DriveBase was
    # reached. The DriveBase now adopts whatever slots its wheels
    # already hold, so no reservation is needed.
    _TASK_SLOTS = (0, 1, 2, 3)

    def _attach_task_slot(self):
        """Claim a native slot for a motor that is not a drivebase
        wheel. Raises rather than silently running unowned."""
        from openbricks import _native
        sb = _native.st_bus
        # The pump is what makes a slot live. A drivebase arms the
        # hard tick in its own constructor; a task motor may be the
        # only thing on the bus, so arm it here too (idempotent).
        try:
            _native.motor_process.hard_tick_selftest()
        except (ImportError, AttributeError):
            pass
        acc = int(self._accel_dps2 * self._steps_per_dps / 100.0)
        acc = 0 if acc < 0 else (254 if acc > 254 else acc)
        slot_of = getattr(sb, "servo_slot_of", None)
        if slot_of is not None:
            held = slot_of(self._id)
            if held >= 0:                 # one servo, one slot
                self._adopt_native(sb, held)
                self._await_slot_odometry(held)
                return
        for slot in self._TASK_SLOTS:
            if sb.servo_attach(slot, self._id, self._invert, acc):
                self._adopt_native(sb, slot)
                self._await_slot_odometry(slot)
                return
        raise RuntimeError(
            "no free native slot for servo id %s: all %d are claimed. "
            "This bus drives at most %d motors — more needs a second "
            "UART." % (self._id, len(self._TASK_SLOTS),
                       len(self._TASK_SLOTS)))

    _SLOT_ODOMETRY_TIMEOUT_MS = 400

    def _await_slot_odometry(self, slot):
        """Block until the slot's first feedback read lands.

        A freshly attached slot has no odometry until the pump's
        round-robin reaches it (config writes go out first), and the
        C layer REFUSES a position move until then — arming one
        against counts=0 would slam the shaft toward a wrong absolute
        target. Without this wait the constructor returned a motor
        that failed its very next ``run_angle`` with "slot odometry is
        not live yet" (bench 2026-08-05, a task motor driven
        immediately after construction).

        Waiting here also means a task motor gets the same
        construction-time liveness check the drivebase wheels get: a
        servo that never answers is a wiring / id / power fault, and
        is reported as one rather than as a puzzling refusal later.
        """
        stats = getattr(self._native_sb, "servo_stats", None)
        if stats is None:
            return
        wstats = getattr(self._native_sb, "servo_write_stats", None)
        deadline = time.ticks_add(time.ticks_ms(),
                                  self._SLOT_ODOMETRY_TIMEOUT_MS)
        while stats(slot)[0] == 0:
            if wstats is not None:
                wfailed, latched = wstats(slot)
                if latched:
                    # The C layer gave up configuring this servo.
                    # Distinguish the two faults that latch: writes
                    # that went unACKed (absent servo — wiring/id/
                    # power) versus writes that were ACKed but whose
                    # op_mode read-back never matched — a servo still
                    # inside its own power-on init acknowledges the
                    # EEPROM-backed mode write without committing it
                    # (bench 2026-08-30), and telling that user to
                    # check the wiring hunts the wrong fault.
                    # Feedback reads never even start for an
                    # unconfigured slot, so without this check the
                    # timeout below would blame "the pump never polled
                    # it". Fail fast and name it.
                    cstate = getattr(self._native_sb,
                                     "servo_config_state", None)
                    if cstate is not None:
                        _f, _l, mismatch, got = cstate(slot)
                        if mismatch:
                            raise OSError(
                                "servo id %s (native slot %d) ACKed "
                                "its configuration but never applied "
                                "it: op_mode reads %d after every "
                                "write was acknowledged. The servo "
                                "was still in its own power-on init "
                                "— give it a second after power-on "
                                "before starting the program, then "
                                "retry." % (self._id, slot, got))
                    raise OSError(
                        "servo id %s (native slot %d): %d configuration "
                        "writes went unacknowledged — the servo never "
                        "ACKed wheel-mode setup. Check its power and "
                        "TX/RX wiring, and that it really has that bus "
                        "id (`openbricks servo-id --scan` lists the "
                        "ids actually answering)."
                        % (self._id, slot, wfailed))
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                ok, failed, stale = stats(slot)
                # "Never asked" and "asked, no answer" are different
                # faults and must not read alike. A silent servo shows
                # failed reads CLIMBING; a pump that never ran shows
                # neither counter moving. Conflating them sent one
                # bench session hunting a wiring problem that was
                # actually a wedged bus (1.57.2).
                if failed == 0:
                    raise OSError(
                        "servo id %s (native slot %d): the bus pump "
                        "never polled it — 0 reads ATTEMPTED, not 0 "
                        "answered. This is not a wiring fault: the "
                        "hard tick is not running, or the bus is stuck "
                        "mid-transaction. Power-cycle the hub; if it "
                        "persists, it is a firmware bug — report the "
                        "counters." % (self._id, slot))
                raise OSError(
                    "servo id %s (native slot %d) is not answering: "
                    "%d replies, %d failed reads — the bus asked and "
                    "got silence. Check the servo's power and TX/RX "
                    "wiring, and that it really has that bus id "
                    "(`openbricks servo-id --scan` lists the ids "
                    "actually answering)." % (self._id, slot, ok, failed))
            time.sleep_ms(10)

    @classmethod
    def migrate_bus_to_native(cls, bus, skip=()):
        """Move every motor still on ``bus`` onto native slots.

        Called by DriveBase adoption right after it takes the UART:
        the wheels become slots 0/1 by their own path, and anything
        else that was sharing the MicroPython bus has to come across
        too or it is left talking into a closed UART."""
        stranded = [s for s in getattr(bus, "_servos", ())]
        if stranded:
            # Position-mode servos CANNOT come across — native slots
            # configure wheel mode. Silently proceeding left them
            # writing into a closed UART (constructed-before-adoption
            # flavour of the two-drivers-one-wire fault). Refuse the
            # adoption with the remedy.
            raise RuntimeError(
                "cannot adopt this UART: position-mode servo id(s) "
                "%s share it, and they have no native-slot path. "
                "Put position-mode servos on a second UART, or use "
                "the Motor class (run/run_angle) for them."
                % ", ".join(str(s._id) for s in stranded))
        for motor in list(getattr(bus, "_motors", ())):
            if motor in skip or motor._native_slot is not None:
                continue
            motor._bus = None
            motor._attach_task_slot()

    def _adopt_native(self, sb, slot):
        self._native_sb = sb
        self._native_slot = slot
        self._native_angle_offset = 0.0
        self._native_pending = None

    @staticmethod
    def _native_move_budget_ms(travel_deg, rate_dps, accel_dps2):
        """Wall-clock stall budget for an adopted run_angle: the
        trapezoid's ideal duration x4 + 1 s (the proven fallback-era
        formula) — only a genuine stall overruns it, because the C
        move's ``done`` requires ARRIVAL, not just profile expiry."""
        travel = abs(float(travel_deg))
        rate = abs(float(rate_dps))
        if rate < 1.0:
            rate = 1.0
        if travel * accel_dps2 >= rate * rate:
            ideal_s = travel / rate + rate / accel_dps2
        else:
            ideal_s = 2.0 * (travel / accel_dps2) ** 0.5
        return int(ideal_s * 4000.0 + 1000.0)

    def _native_dispatch_then(self, then):
        if then == Stop.COAST:
            self._native_sb.servo_coast(self._native_slot)
        elif then == Stop.BRAKE:
            self._native_sb.servo_run(self._native_slot, 0)
        # Stop.HOLD: the C move already parks in a position hold.

    def _native_run_angle(self, deg_per_s, target_angle, wait, then):
        max_dps = abs(float(deg_per_s))
        if target_angle == 0 or max_dps <= 0:
            return
        capped = max_dps if max_dps < self._max_dps else self._max_dps
        accel = self._accel_dps2 if self._accel_dps2 > 0 else 1500.0
        # User-frame delta; the slot carries this motor's invert, same
        # convention as servo_run.
        ok = self._native_sb.servo_move(
            self._native_slot,
            float(target_angle) * _COUNTS_PER_REV / 360.0,
            capped * self._steps_per_dps,
            accel * self._steps_per_dps)
        if not ok:
            raise RuntimeError(
                "run_angle refused: wheel is owned by an in-flight "
                "DriveBase move (stop it first), or slot odometry is "
                "not live yet")
        budget_ms = self._native_move_budget_ms(target_angle, capped,
                                                accel)
        start_counts = self._native_sb.servo_counts(self._native_slot)
        t0 = time.ticks_ms()
        if not wait:
            # ``done()`` carries the same stall/dead-bus watch the
            # blocking path runs below — without it, the documented
            # ``while not m.done():`` polling loop span forever on a
            # jammed or unplugged motor (1.62.0's detection covered
            # only wait=True).
            self._native_pending = {
                "then":         then,
                "t0":           t0,
                "budget_ms":    budget_ms,
                "start_counts": start_counts,
                "target":       float(target_angle),
                "last_counts":  start_counts,
                "last_move_ms": t0,
            }
            return
        # Give up when the shaft STOPS MOVING, not when a fixed budget
        # expires. A move fighting a heavy load is still a move and
        # must not be cut short; a move that has not advanced a count
        # in a second is stuck, whatever the budget says. That also
        # turns a 4-second wait into a 1-second one on a real jam.
        last_counts = start_counts
        last_move_ms = t0
        while not self._native_sb.servo_move_done(self._native_slot):
            estop.check()
            # Bus death first, every pass: it freezes counts exactly
            # like a jam, and waiting out the stall-idle window to
            # then call it "jammed" is the wrong fault named a second
            # too late. This raises within ~200 ms of the silence.
            self._native_check_bus_alive()
            now = time.ticks_ms()
            counts = self._native_sb.servo_counts(self._native_slot)
            if abs(counts - last_counts) >= _STALL_PROGRESS_COUNTS:
                last_counts = counts
                last_move_ms = now
            idle_ms = time.ticks_diff(now, last_move_ms)
            if idle_ms > self._stall_idle_ms or \
                    time.ticks_diff(now, t0) > budget_ms:
                self._native_sb.servo_run(self._native_slot, 0)
                report = self._native_stall_report(
                    budget_ms, start_counts, target_angle,
                    idle_ms if idle_ms > self._stall_idle_ms else None)
                if self._raise_on_stall:
                    raise RuntimeError(report)
                # A MECHANICAL stall should not abort a mission run:
                # report it and let the caller decide. Loud, not
                # fatal — the console sees it, the run log keeps it,
                # and the return value says it. (Bus death is not
                # survivable and raised above.)
                self._report_stall(report)
                return False
            time.sleep_ms(10)
        self._native_dispatch_then(then)
        return True

    def _native_check_bus_alive(self):
        """Raise when the slot's feedback has gone consecutively
        silent — the unplugged/broken-wire fault whose frozen counts
        would otherwise read as a mechanical jam."""
        stats = getattr(self._native_sb, "servo_stats", None)
        if stats is None:
            return
        ok_n, failed, stale = stats(self._native_slot)
        if stale >= _DEAD_BUS_STALE:
            raise OSError(
                "servo id %s (native slot %d) went SILENT mid-move: "
                "%d consecutive failed feedback reads (%d ok, %d "
                "failed total). This is a bus/wiring/power fault, not "
                "a stall — frozen odometry only LOOKS like a jam. "
                "Check the servo's power and TX/RX wiring."
                % (self._id, self._native_slot, stale, ok_n, failed))

    @staticmethod
    def _report_stall(report):
        """Console AND run log. ``log.note`` is file-only by design,
        so a stall that scrolled past on the console is still in the
        log afterwards — and one that happened while nothing was
        watching is not lost."""
        print("openbricks: " + report)
        try:
            from openbricks import log
            log.note("STALL " + report)
        except Exception:
            pass

    def _native_stall_report(self, budget_ms, start_counts,
                             target_angle, idle_ms=None):
        """Say WHICH failure this was, not which three it might be.

        "stalled, blocked, or in overload protection" are different
        faults with different fixes, and the slot has been reporting
        travel, speed and load since 1.50.0 — so read them instead of
        listing possibilities. How far it got separates them:

        * moved ~nothing   -> never started: jammed, or no torque
        * moved partway    -> stalled under load, or the servo's
          overload protection cut in (>80% of stall for 2 s, cleared
          by reissuing a command)
        * moved ~all of it -> it is arriving but not LATCHING, which
          is a tolerance/odometry problem, not a mechanical one
        """
        moved = ((self._native_sb.servo_counts(self._native_slot)
                  - start_counts) * 360.0 / _COUNTS_PER_REV)
        want = float(target_angle)
        frac = abs(moved) / abs(want) if want else 0.0
        if frac < 0.05:
            why = ("it never moved — the shaft is jammed, or torque "
                   "is not reaching it")
        elif frac > 0.9:
            why = ("it travelled essentially the whole way but the "
                   "move never latched as done — suspect arrival "
                   "tolerance or odometry, NOT a mechanical fault")
        else:
            why = ("it stopped partway — stalled under load, or the "
                   "servo's overload protection cut in (it trips "
                   "above ~80% of stall torque held for ~2 s, and "
                   "clears when a new command is issued)")
        detail = ""
        feedback = getattr(self._native_sb, "servo_feedback", None)
        if feedback is not None:
            speed_steps, load_raw, fresh = feedback(self._native_slot)
            if fresh:
                detail = (" At the timeout it reported %.0f deg/s and "
                          "%.0f mNm of load."
                          % (speed_steps / self._steps_per_dps,
                             load_raw * self.STALL_TORQUE_MNM / 1000.0))
        when = ("stopped moving for %d ms" % idle_ms if idle_ms
                else "ran out of its %d ms budget" % budget_ms)
        return ("run_angle(%g deg) on servo id %s gave up — %s: %s. "
                "It moved %.1f deg of the %.1f asked.%s"
                % (want, self._id, when, why, moved, want, detail))

    def hold(self):
        """Actively hold the current shaft angle so the position PID
        resists rotation. Subsequent ``run_speed`` / ``brake`` /
        ``coast`` calls transparently restore wheel mode.

        On an adopted motor (native DriveBase) the hold is a per-slot
        position lock on the hard tick (st_move_core) — it corrects
        disturbances continuously at the bus feedback rate.

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
        estop.check()
        if self._native_slot is not None:
            self._native_pending = None
            if not self._native_sb.servo_hold(self._native_slot):
                raise RuntimeError(
                    "hold refused: wheel is owned by an in-flight "
                    "DriveBase move (stop it first), or slot odometry "
                    "is not live yet")
            return
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
        first step.

        On an adopted motor the C controller advances the move on the
        hard tick; ``done()`` just checks the arrival flag and runs
        the deferred ``then=`` dispatch."""
        if self._native_slot is not None:
            if self._native_pending is None:
                return True
            st = self._native_pending
            if not self._native_sb.servo_move_done(self._native_slot):
                # The same stall/dead-bus watch the blocking path
                # runs: without it, the documented polling loop hangs
                # forever on a jammed or unplugged motor.
                self._native_check_bus_alive()
                now = time.ticks_ms()
                counts = self._native_sb.servo_counts(self._native_slot)
                if abs(counts - st["last_counts"]) >= \
                        _STALL_PROGRESS_COUNTS:
                    st["last_counts"] = counts
                    st["last_move_ms"] = now
                idle_ms = time.ticks_diff(now, st["last_move_ms"])
                if idle_ms > self._stall_idle_ms or \
                        time.ticks_diff(now, st["t0"]) > st["budget_ms"]:
                    self._native_sb.servo_run(self._native_slot, 0)
                    self._native_pending = None
                    report = self._native_stall_report(
                        st["budget_ms"], st["start_counts"],
                        st["target"],
                        idle_ms if idle_ms > self._stall_idle_ms
                        else None)
                    if self._raise_on_stall:
                        raise RuntimeError(report)
                    self._report_stall(report)
                    return True     # move is over; wheel stopped
                return False
            then = st["then"]
            self._native_pending = None
            self._native_dispatch_then(then)
            return True
        if self._pending is None:
            return True
        return self._poll_pending()

    def _poll_pending(self):
        """One iteration of the wait=False state machine. See ``done``."""
        state = self._pending
        now = time.ticks_ms()
        rem = self._read_step_remaining()
        if rem is None:
            # A transient drop is tolerated; PERSISTENT silence is a
            # dead bus, and polling it forever hides the fault.
            if state["silent_since"] is None:
                state["silent_since"] = now
            elif time.ticks_diff(now, state["silent_since"]) > \
                    self._stall_idle_ms:
                self._pending = None
                raise OSError(
                    "servo id %s went SILENT mid-move: no reply to "
                    "%d ms of step-register reads. This is a "
                    "bus/wiring/power fault, not a stall. Check the "
                    "servo's power and TX/RX wiring."
                    % (self._id, self._stall_idle_ms))
            return False
        state["silent_since"] = None
        tol = state["tol_counts"]
        if not state["started"]:
            # Guard against a stale ~0 read before the servo has loaded
            # the step: only start watching for completion once the
            # remaining register confirms the move launched.
            if abs(rem) > tol:
                state["started"] = True
                state["last_rem"] = rem
                state["last_move_ms"] = now
            return False
        if abs(rem) > tol:
            if abs(rem - state["last_rem"]) >= _STALL_PROGRESS_COUNTS:
                state["last_rem"] = rem
                state["last_move_ms"] = now
            elif time.ticks_diff(now, state["last_move_ms"]) > \
                    self._stall_idle_ms:
                # The register stopped counting down: a stall, per the
                # same idle rule as the blocking paths. Report (or
                # raise) and end the move instead of returning False
                # forever.
                self._pending = None
                report = (
                    "run_angle on servo id %s gave up — the step "
                    "stopped counting down for %d ms: stalled under "
                    "load, jammed, or in overload protection (clears "
                    "when a new command is issued). %d counts of this "
                    "step remain." % (self._id, self._stall_idle_ms,
                                      rem))
                if self._raise_on_stall:
                    raise RuntimeError(report)
                self._report_stall(report)
                return True
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
        if self._native_slot is not None:
            # Slot odometry is already multi-turn and already in this
            # motor's frame (the slot carries the invert flag).
            counts = self._native_sb.servo_counts(self._native_slot)
            return counts * 360.0 / _COUNTS_PER_REV \
                - self._native_angle_offset

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
        if self._native_slot is not None:
            counts = self._native_sb.servo_counts(self._native_slot)
            self._native_angle_offset = (
                counts * 360.0 / _COUNTS_PER_REV - float(angle))
            return
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
                  debug=False, then=Stop.COAST):
        """Rotate by ``target_angle`` degrees at up to ``deg_per_s``,
        ending within ``tolerance_deg`` of the target.

        On an adopted motor (native DriveBase) the move runs as a
        per-slot trapezoid position move on the hard tick
        (st_move_core) — same trajectory + arrival semantics as the
        drivebase's own moves; ``tolerance_deg`` is fixed at the
        C core's arrival tolerance there.

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

        * ``Stop.COAST`` (default) — cut torque; wheel free-wheels. The
          next ``run_speed`` / ``brake`` / ``run_angle`` transparently
          re-enables torque and restores the mode it needs.
        * ``Stop.BRAKE`` — restore wheel mode and write goal_speed=0 so
          the servo's velocity loop actively holds zero rotation rate.
        * ``Stop.HOLD`` — leave the servo in step mode; its position PID
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
        parameters.check(Stop, then, "then",
                         allowed=(Stop.COAST, Stop.BRAKE, Stop.HOLD))
        if self._native_slot is not None:
            self._native_pending = None
            return self._native_run_angle(deg_per_s, target_angle,
                                          wait, then)
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

        # A silent bus fails the move LOUDLY rather than commanding
        # one we can't track. Probed BEFORE the mode/torque writes so
        # a glitching servo is left exactly as it was — since motors
        # coast at construction, enabling torque first and then
        # bailing would leave a stiff servo with no move commanded.
        # (The value read is irrelevant; only that the servo answers.
        # Returning quietly here made an unplugged servo look like a
        # completed move.)
        if self._read_step_remaining() is None:
            raise OSError(
                "servo id %s did not answer the pre-move probe — "
                "run_angle cannot track a move it can't read back. "
                "Check the servo's power and TX/RX wiring "
                "(`openbricks servo-id --scan` lists the ids actually "
                "answering)." % self._id)

        self._ensure_torque_on()
        self._ensure_step_limits()       # angle limits = 0 (once)
        self._ensure_mode(_MODE_STEP)    # op_mode = 3
        self._write_motion_regs(speed_steps)

        # First step (clamped to the ±7-turn envelope).
        first = target_counts
        if first >  _MAX_STEP_COUNTS: first =  _MAX_STEP_COUNTS
        if first < -_MAX_STEP_COUNTS: first = -_MAX_STEP_COUNTS
        self._write_step(first)
        remaining_counts = target_counts - first

        if not wait:
            now = time.ticks_ms()
            self._pending = {
                "first":            first,
                "remaining_counts": remaining_counts,
                "tol_counts":       tol_counts,
                "then":             then,
                "started":          False,
                # Stall/dead-bus watch state (see _poll_pending) —
                # without it the documented ``while not m.done():``
                # loop polls a jammed or unplugged motor forever.
                "last_rem":         first,
                "last_move_ms":     now,
                "silent_since":     None,
            }
            return

        while True:
            travelled, parked = self._await_step(first, speed_steps,
                                                 tol_counts)
            self._advance_accum(travelled)
            if not parked:
                # The step never parked within its budget. This used
                # to fall through and return True — a stalled motor
                # reporting a fully successful move. Same contract as
                # the adopted path now: raise if asked, else report
                # loudly and say False.
                report = (
                    "run_angle(%g deg) on servo id %s gave up — a "
                    "step never parked within its time budget: "
                    "stalled under load, jammed, or in overload "
                    "protection (clears when a new command is "
                    "issued). This step travelled %.1f deg of the "
                    "%.1f commanded."
                    % (float(target_angle), self._id,
                       travelled * 360.0 / _COUNTS_PER_REV,
                       first * 360.0 / _COUNTS_PER_REV))
                if self._raise_on_stall:
                    raise RuntimeError(report)
                self._report_stall(report)
                return False
            if remaining_counts == 0:
                break
            # Issue the next ±7-turn step.
            first = remaining_counts
            if first >  _MAX_STEP_COUNTS: first =  _MAX_STEP_COUNTS
            if first < -_MAX_STEP_COUNTS: first = -_MAX_STEP_COUNTS
            self._write_step(first)
            remaining_counts -= first

        self._dispatch_then(then)
        return True

    def _await_step(self, step, speed_steps, tol_counts):
        """Block until the in-flight step parks (its remaining register
        counts down to within ``tol_counts`` of 0) or a time budget
        expires. Returns ``(travelled_counts, parked)`` — the counts
        actually covered (``step`` minus the final remaining) for the
        shaft-angle accumulator, and whether the step really parked
        (False = the budget expired on a stall; the caller reports).

        A ``started`` latch guards against a stale ~0 read at kickoff:
        we only watch for completion once the remaining register has
        first been seen larger than tolerance (the move launched)."""
        # Backstop deadline scales with the move (estimated travel time
        # ×3 for the accel/decel ramp). It only bites on a stall —
        # normal moves return the instant the step parks below tol — so
        # there is no fixed ceiling that could cut a slow multi-turn step
        # short.
        est_ms = int(abs(step) * 1000 / speed_steps + 200)
        deadline = time.ticks_add(time.ticks_ms(), est_ms * 3)
        started = False
        last_rem = step
        parked = False
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            rem = self._read_step_remaining()
            if rem is not None:
                last_rem = rem
                if not started:
                    if abs(rem) > tol_counts:
                        started = True
                elif abs(rem) <= tol_counts:
                    last_rem = rem
                    parked = True
                    break
            time.sleep_ms(10)
        return step - last_rem, parked

    # --- ST-3215-specific extras ------------------------------------------

    def ping(self):
        """``True`` if the servo answers on the bus."""
        return self._bus.ping(self._id)


class SyncServoGroup:
    """Coordinated multi-servo writes via SCServo SYNC WRITE.

    All servos must share one ``_SCServoBus`` (same UART). Mixed
    servo types (``ST3215``, ``ST3215Motor``, ``ST3032``, ``ST3032Motor``)
    are fine since they all speak the same SCS protocol — SYNC WRITE
    just blasts the same register on all listed IDs.

    Use this whenever you have multiple servos that should apply a
    setpoint at the same packet boundary (a multi-finger gripper, a
    multi-axis arm) — each servo receives its byte slot of the
    broadcast packet at the same instant, instead of N serialised
    individual writes.

    NOT for drivebase wheels: use ``DriveBase.move_wheels(left,
    right)`` instead. It gives the same one-packet guarantee, and a
    SyncServoGroup cannot drive adopted wheels at all — a DriveBase
    hands their UART to the native bus driver, so the MicroPython
    bus this class writes through is closed.

    Example
    -------
    ::

        from openbricks.drivers.st3215 import ST3215Motor, SyncServoGroup

        thumb = ST3215Motor(servo_id=1)
        index = ST3215Motor(servo_id=2)
        group = SyncServoGroup([thumb, index])

        # Both fingers start moving at the same packet boundary —
        # one SYNC WRITE instead of two individual writes.
        group.set_goal_speeds([200, 200])
    """

    @staticmethod
    def _refuse_native(servos):
        """A servo on a native slot cannot be commanded from here.

        Checked at construction AND before every write, because
        adoption can happen in between: build the group first and a
        DriveBase adopts the same motors a line later, and the group
        is left holding wheels it can no longer safely reach. That
        ordering used to pass construction and then write into a
        contended bus — silently, which is the whole failure this
        guard exists to end. Refusing in one place and not the other
        made the error depend on run order (fine the first time,
        refused the second), which is worse than either.
        """
        for s in servos:
            slot = getattr(s, "_native_slot", None)
            if slot is None:
                continue
            raise RuntimeError(
                "servo id %s is driven by the native bus (slot %d), "
                "so a SyncServoGroup cannot command it: this class "
                "writes through the MicroPython UART, which the "
                "native driver now owns — the two collide on the "
                "wire and each reads the other's replies. For "
                "drivebase wheels use ``DriveBase.move_wheels(left, "
                "right)``, which gives the same one-packet guarantee "
                "from inside the engine; for a task motor drive it "
                "directly (``motor.run_speed(...)``)." % (s._id, slot))

    def __init__(self, servos):
        if not servos:
            raise ValueError("SyncServoGroup needs at least one servo")
        self._refuse_native(servos)
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
        self._refuse_native(self._servos)
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
