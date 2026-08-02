# SPDX-License-Identifier: MIT
"""
TI TCA9548A 8-channel I2C multiplexer / switch.

Some I2C sensors have a fixed address with no way to change it — the
TCS34725 colour sensor is stuck at ``0x29``, for instance — so you
cannot put two of them on one bus without an address collision. The
TCA9548A sits between the host and the sensors and fans one bus out to
eight electrically-isolated channels (SD0/SC0 .. SD7/SC7); each channel
can host its own copy of the same-address device.

The mux itself answers at ``0x70`` by default (``0x70``..``0x77`` via
the A0/A1/A2 address pins). Channel selection is a single-byte write to
that address: bit *N* enables channel *N*. Multiple bits enable several
channels at once; ``0x00`` disables all of them.

The drop-in path is ``mux[n]``: it returns an object that quacks like a
``machine.I2C`` bus but transparently selects channel ``n`` before every
operation, so existing drivers construct against it unchanged::

    from machine import I2C, Pin
    from openbricks.drivers.tca9548a import TCA9548A
    from openbricks.drivers.tcs34725 import TCS34725

    i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400_000)
    mux = TCA9548A(i2c)
    left  = TCS34725(mux[0])   # 0x29 on channel 0
    right = TCS34725(mux[1])   # 0x29 on channel 1

Reference: TCA9548A datasheet (Texas Instruments), section 8.3.
"""

_DEFAULT_ADDR = 0x70
_NUM_CHANNELS = 8


class TCA9548A:
    """8-channel I2C multiplexer for same-address sensors.

    Index it like a list: ``mux[n]`` returns an I2C-compatible handle
    for channel *n* (0-7) that selects the channel transparently
    before every transaction — pass it to any driver in place of the
    real ``machine.I2C``.

    Args:
        i2c: the upstream ``machine.I2C`` bus the mux sits on.
        address: mux's own I2C address, 0x70-0x77 via A0/A1/A2 straps
            (default 0x70).
    """

    def __init__(self, i2c, address=_DEFAULT_ADDR):
        self._i2c = i2c
        self._addr = address
        # Cache the last control byte written so repeated access to the
        # same channel doesn't re-issue the select. ``None`` means
        # "unknown" — the first select always writes.
        self._selected = None

    def select(self, channel):
        """Enable exactly ``channel`` (0..7), disabling the rest."""
        mask = 1 << _check_channel(channel)
        if mask != self._selected:
            # Unknown until the write SUCCEEDS. If it raises, the
            # mux's real register state is anybody's guess (the write
            # may have half-happened, or the mux may have reset) — a
            # cache still claiming the old channel would make the next
            # select skip its write and silently talk to the wrong
            # channel.
            self._selected = None
            self._i2c.writeto(self._addr, bytes([mask]))
            self._selected = mask

    def disable(self):
        """Disable every channel (control byte ``0x00``)."""
        if self._selected != 0:
            self._i2c.writeto(self._addr, b"\x00")
            self._selected = 0

    def __getitem__(self, channel):
        """Return a ``machine.I2C``-like proxy bound to ``channel``."""
        return _MuxChannel(self, _check_channel(channel))


def _check_channel(channel):
    if not 0 <= channel < _NUM_CHANNELS:
        raise ValueError(
            "channel must be 0..%d, got %r" % (_NUM_CHANNELS - 1, channel))
    return channel


class _MuxChannel:
    """A view of one mux channel that quacks like ``machine.I2C``.

    Every bus method selects the channel on the parent mux first, then
    delegates to the real I2C bus, so a driver that only ever touches
    this proxy addresses its own channel's device transparently.
    """

    def __init__(self, mux, channel):
        self._mux = mux
        self._channel = channel

    def _select(self):
        try:
            self._mux.select(self._channel)
        except OSError as e:
            # The MUX itself didn't ACK — a different failure from the
            # device behind it not answering, and the message must say
            # so (a bare ENODEV made the two indistinguishable on the
            # bench; a whole session went to telling them apart).
            raise OSError(
                e.args[0] if e.args else 19,
                "mux at 0x%02x did not ACK selecting channel %d"
                % (self._mux._addr, self._channel))

    def _no_ack(self, e, addr):
        """Re-raise a device-side OSError with the channel + address
        that failed. Same errno; the extra text is what turns
        ``[Errno 19] ENODEV`` into a named sensor."""
        raise OSError(
            e.args[0] if e.args else 19,
            "no ACK from device 0x%02x on mux channel %d"
            % (addr, self._channel))

    def readfrom_mem(self, addr, reg, nbytes, *args, **kwargs):
        self._select()
        try:
            return self._mux._i2c.readfrom_mem(
                addr, reg, nbytes, *args, **kwargs)
        except OSError as e:
            self._no_ack(e, addr)

    def writeto_mem(self, addr, reg, data, *args, **kwargs):
        self._select()
        try:
            return self._mux._i2c.writeto_mem(
                addr, reg, data, *args, **kwargs)
        except OSError as e:
            self._no_ack(e, addr)

    def readfrom(self, addr, nbytes, *args, **kwargs):
        self._select()
        try:
            return self._mux._i2c.readfrom(addr, nbytes, *args, **kwargs)
        except OSError as e:
            self._no_ack(e, addr)

    def writeto(self, addr, buf, *args, **kwargs):
        self._select()
        try:
            return self._mux._i2c.writeto(addr, buf, *args, **kwargs)
        except OSError as e:
            self._no_ack(e, addr)

    def scan(self):
        self._select()
        return self._mux._i2c.scan()
