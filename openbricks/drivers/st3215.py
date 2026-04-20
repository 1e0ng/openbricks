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
    0x2A  Goal position (low, high) — int16, 0..4095 over ~360°
    0x2E  Running speed (low, high)
    0x38  Present position (low, high) — read only

Half-duplex wiring: most ST-3215 boards use a single data line driven by a
TX/RX switching circuit, but MicroPython UART pins are usually separate. If
your adapter exposes TX and RX separately, just wire them normally. If you
have a true half-duplex bus, you'll need a driver chip or a direction-enable
GPIO — set ``dir_pin`` below.

Only a minimal subset of the protocol is implemented here. PR welcome.
"""

import time

from machine import UART, Pin

from openbricks.interfaces import Servo

_HEADER = b"\xFF\xFF"

_INSTR_PING  = 0x01
_INSTR_READ  = 0x02
_INSTR_WRITE = 0x03

_REG_GOAL_POSITION = 0x2A
_REG_GOAL_SPEED    = 0x2E
_REG_PRESENT_POS   = 0x38


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
