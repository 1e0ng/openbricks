# SPDX-License-Identifier: MIT
"""
Nordic UART Service bridge for the MicroPython REPL over BLE.

Vendored from upstream MicroPython's
``examples/bluetooth/ble_uart_peripheral.py`` (the ``BLEUART``
class) and ``ble_uart_repl.py`` (the ``BLEUARTStream`` dupterm
adapter). Both files are MIT-licensed under MicroPython's project
LICENSE. The one structural change is the advertising payload —
upstream advertises only the device name + appearance; we add the
NUS 128-bit service UUID so clients filtering by service can find
us. Everything else (connection-set tracking, append-mode rx
buffer, scheduled-flush write batching, ``io.IOBase`` inheritance)
is the upstream pattern, unmodified.

Why vendored: writing this from scratch using only the docs (the
1.0.0-1.1.1 history) produced a long parade of "invisible until
hardware" bugs — each fix required a hardware reflash + paste
diagnostic round. Starting from upstream's working example would
have caught all of them at once.

Public surface (what ``openbricks.bluetooth.apply_persisted_state``
calls):

* ``start()`` — bring up the NUS bridge. Idempotent.
* ``stop()``  — tear down. Idempotent.
* ``is_running()`` — query.
"""

import struct

try:
    import bluetooth
    import io
    import os
    from micropython import const
    _IOBASE = io.IOBase
except ImportError:
    # Desktop tests install fakes via ``tests._fakes_ble``; the
    # module-level imports still need to succeed at import time on
    # CPython. ``const`` is a MicroPython optimisation — pass-through
    # works on CPython.
    import io
    import os
    _IOBASE = io.IOBase
    const = lambda x: x  # noqa: E731

try:
    import micropython
    _SCHEDULE = micropython.schedule
except ImportError:
    # Desktop tests have no scheduler; flush synchronously.
    def _SCHEDULE(fn, arg):
        fn(arg)


# ---- BLE constants ----------------------------------------------------

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

_FLAG_WRITE  = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_MP_STREAM_POLL    = const(3)
_MP_STREAM_POLL_RD = const(0x0001)


# ---- NUS service UUIDs (the de-facto BLE serial ones) ----------------

# Stored as strings so tests can compare without instantiating
# ``bluetooth.UUID`` (which on the test fakes wraps the string).
_UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
_UART_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # hub → client
_UART_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # client → hub


# ---- Advertising payload helper --------------------------------------
#
# Vendored from ``examples/bluetooth/ble_advertising.py``,
# specialised to our exact needs (flags + name + 128-bit UUID).

_ADV_TYPE_FLAGS            = const(0x01)
_ADV_TYPE_NAME             = const(0x09)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)
_ADV_MAX_PAYLOAD           = const(31)


def _uuid_bytes_le(uuid_str):
    """Convert a 128-bit UUID string to little-endian bytes (the wire
    format BLE advertising expects)."""
    hex_ = uuid_str.replace("-", "")
    b = bytes.fromhex(hex_)
    return bytes(reversed(b))


def _advertising_payload(name, service_uuid_str):
    """Build a BLE advertising payload: flags + name + 128-bit
    service UUID. Raises if it overflows 31 bytes (the BLE-LE
    advertising max)."""
    payload = bytearray()
    def _append(adv_type, value):
        payload.extend(struct.pack("BB", len(value) + 1, adv_type) + value)
    # 0x06 = LE general discoverable + BR/EDR not supported.
    _append(_ADV_TYPE_FLAGS, struct.pack("B", 0x06))
    if name:
        name_bytes = name.encode() if isinstance(name, str) else name
        _append(_ADV_TYPE_NAME, name_bytes[:29])  # leave room for headers
    _append(_ADV_TYPE_UUID128_COMPLETE, _uuid_bytes_le(service_uuid_str))
    if len(payload) > _ADV_MAX_PAYLOAD:
        raise ValueError("advertising payload too large (%d bytes)" % len(payload))
    return bytes(payload)


# ---- _BLEUART: NUS service registration + connection tracking --------
#
# Mirror-image of upstream's ``BLEUART`` class. The handler-callback
# pattern (``self._handler``) lets ``_BLEUARTStream`` (below) get
# notified on rx without us having to call into dupterm from the
# IRQ handler — the stream wraps ``_on_rx`` and that's where the
# ``os.dupterm_notify`` poke lives.

class _BLEUART:
    def __init__(self, ble, name, rxbuf=100):
        self._ble = ble
        self._ble.irq(self._irq)
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services(
            ((bluetooth.UUID(_UART_SERVICE_UUID), (
                (bluetooth.UUID(_UART_TX_UUID), _FLAG_NOTIFY),
                (bluetooth.UUID(_UART_RX_UUID), _FLAG_WRITE),
            )),)
        )
        # Append-mode rx buffer: back-to-back writes from the central
        # accumulate instead of overwriting. Without this, a quick
        # "Ctrl-C Ctrl-A" from openbricks-dev run loses one of the two.
        self._ble.gatts_set_buffer(self._rx_handle, rxbuf, True)
        self._connections = set()
        self._rx_buffer = bytearray()
        self._handler = None
        self._payload = _advertising_payload(name=name, service_uuid_str=_UART_SERVICE_UUID)
        self._advertise()

    def irq(self, handler):
        """Set the rx-arrived handler. Called from ``_BLEUARTStream``
        to install the ``os.dupterm_notify`` poke."""
        self._handler = handler

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            # Keep accepting new connections.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._rx_handle:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)
                if self._handler:
                    self._handler()

    def any(self):
        return len(self._rx_buffer)

    def read(self, sz=None):
        if not sz:
            sz = len(self._rx_buffer)
        result = bytes(self._rx_buffer[0:sz])
        self._rx_buffer = self._rx_buffer[sz:]
        return result

    def write(self, data):
        for conn_handle in self._connections:
            try:
                self._ble.gatts_notify(conn_handle, self._tx_handle, data)
            except OSError:
                # Peer went away mid-notify; ignore — disconnect IRQ
                # will tidy up.
                pass

    def close(self):
        for conn_handle in list(self._connections):
            try:
                self._ble.gap_disconnect(conn_handle)
            except OSError:
                pass
        self._connections.clear()
        self._stop_advertising()

    def _advertise(self, interval_us=100_000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def _stop_advertising(self):
        # ``gap_advertise(None)`` stops advertising in MicroPython.
        try:
            self._ble.gap_advertise(None)
        except (TypeError, OSError):
            pass


# ---- _BLEUARTStream: dupterm-compatible stream wrapper ---------------
#
# Vendored from upstream's ``BLEUARTStream``. ``io.IOBase`` is what
# gives the type the C-level stream-protocol slot — without that
# inheritance, ``os.dupterm()`` raises ``stream operation not
# supported``.
#
# Write-side batching: ``write()`` queues bytes into ``_tx_buf`` and
# schedules a ``_flush`` callback via ``micropython.schedule``. The
# flush sends up to 100 bytes per pass and re-schedules if more
# remain. This keeps gatts_notify off the IRQ-handler hot path AND
# avoids one notify per byte (which BLE link-layer rate limits would
# choke on).

class _BLEUARTStream(_IOBASE):
    def __init__(self, uart):
        self._uart = uart
        self._tx_buf = bytearray()
        self._scheduled = False
        self._uart.irq(self._on_rx)

    def _on_rx(self):
        # Wake dupterm so it drains uart's rx buffer into stdin
        # immediately. Without this, a Ctrl-C arriving over BLE sits
        # in the buffer until the REPL happens to poll — which can
        # be never, if user code is busy-looping.
        if hasattr(os, "dupterm_notify"):
            os.dupterm_notify(None)

    def read(self, sz=None):
        return self._uart.read(sz)

    def readinto(self, buf):
        avail = self._uart.read(len(buf))
        if not avail:
            return None
        for i in range(len(avail)):
            buf[i] = avail[i]
        return len(avail)

    def ioctl(self, op, arg):
        if op == _MP_STREAM_POLL:
            if self._uart.any():
                return _MP_STREAM_POLL_RD
        return 0

    def _flush(self, _arg):
        self._scheduled = False
        if not self._tx_buf:
            return
        data = bytes(self._tx_buf[0:100])
        self._tx_buf = self._tx_buf[100:]
        self._uart.write(data)
        if self._tx_buf and not self._scheduled:
            _SCHEDULE(self._flush, None)
            self._scheduled = True

    def write(self, buf):
        self._tx_buf += buf
        if not self._scheduled:
            _SCHEDULE(self._flush, None)
            self._scheduled = True


# ---- Public API ------------------------------------------------------

# Singleton bridge state. ``_state["bridge"]`` is the active
# ``_BLEUART`` (kept under that key for backwards-compat with tests
# that introspected the bridge before 1.2.0). ``_state["stream"]``
# is the ``_BLEUARTStream`` installed in dupterm.
_state = {"bridge": None, "stream": None}


def is_running():
    return _state["bridge"] is not None


def start():
    """Bring up the NUS bridge. Idempotent — second call is a no-op
    if already running.

    Caller must have ``ble.active(True)`` before calling. Hub name
    comes from ``openbricks.HUB_NAME`` (NVS-backed); we refuse to
    advertise without one.
    """
    if _state["bridge"] is not None:
        return
    import openbricks
    name = openbricks.HUB_NAME
    if name is None:
        raise RuntimeError(
            "hub name unset; flash with `openbricks flash --name NAME ...`"
        )
    ble = bluetooth.BLE()
    if not ble.active():
        raise RuntimeError(
            "BLE is not active; call openbricks.bluetooth.set_enabled(True) "
            "before ble_repl.start()"
        )
    uart = _BLEUART(ble, name=name)
    stream = _BLEUARTStream(uart)
    _install_dupterm(stream)
    _state["bridge"] = uart
    _state["stream"] = stream


def stop():
    """Tear down the NUS bridge. Idempotent."""
    if _state["bridge"] is None:
        return
    _install_dupterm(None)
    _state["bridge"].close()
    _state["bridge"] = None
    _state["stream"] = None


def _install_dupterm(stream):
    """Install/clear the dupterm stream. Indirection point so tests
    can monkey-patch this helper instead of touching ``os.dupterm``
    directly (``os`` is a frozen module on MicroPython and rejects
    ``setattr``)."""
    if hasattr(os, "dupterm"):
        os.dupterm(stream)
