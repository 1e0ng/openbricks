# SPDX-License-Identifier: MIT
"""
Nordic UART Service bridge for the MicroPython REPL over BLE.

Registers the de-facto Bluetooth-LE serial UART service (Nordic UART,
aka NUS) and plumbs its two characteristics through ``os.dupterm`` so
whatever the REPL reads/writes also flows over BLE. This is what makes
``openbricks-dev run`` / ``stop`` work — the client tool speaks the same
cooked REPL a serial-attached user would see.

Wiring:

* Advertising payload carries the NUS 128-bit service UUID and the
  baked-in GAP name (``openbricks.HUB_NAME``), so clients filtering by
  service can discover us and pick the right hub by name.
* ``_UART_RX_CHAR`` (client → hub, WRITE) — bytes written here land in
  the REPL's input buffer, as if typed on a keyboard.
* ``_UART_TX_CHAR`` (hub → client, NOTIFY) — whatever the REPL prints
  is chunked to the negotiated MTU and notified out.
* ``start()`` is called from ``openbricks.bluetooth`` right after the
  stack goes active; ``stop()`` tears down on deactivate.

Only one IRQ handler can be registered per ``bluetooth.BLE()`` instance.
This module currently owns that handler — if another feature needs to
co-exist (e.g. a GATT service for streaming telemetry), we'll need a
small dispatcher. Not blocking today.
"""

try:
    import bluetooth
    import io
    from micropython import const
    _IOBASE = io.IOBase
except ImportError:
    # Desktop tests install fakes via ``tests._fakes_ble``; the module
    # level imports still need to succeed at import time. ``const`` is
    # a MicroPython optimisation — on CPython, a pass-through works.
    # ``io.IOBase`` exists on CPython too; use it as-is so subclassing
    # behaves the same on host tests.
    import io
    _IOBASE = io.IOBase
    const = lambda x: x  # noqa: E731


_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)
_IRQ_MTU_EXCHANGED      = const(21)

_ADV_TYPE_FLAGS         = const(0x01)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)
_ADV_TYPE_NAME_COMPLETE = const(0x09)

_ADV_FLAG_LE_GENERAL_DISC = const(0x06)  # LE general discoverable + BR/EDR not supported

# Requested MTU. The peer may negotiate lower. Larger MTU = fewer
# notifications per REPL print, which matters when the user program
# prints long tracebacks.
_REQ_MTU = const(247)

# Standard Nordic UART Service UUIDs. These are what every commodity
# BLE UART tool — Nordic's nRF Connect, Adafruit's BluefruitLE,
# mpremote's experimental BLE transport, bleak examples — expects.
_UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
_UART_TX_UUID      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  # hub → client
_UART_RX_UUID      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  # client → hub


_state = {"bridge": None}  # module singleton; None ⇔ not running


def _install_dupterm(stream):
    """Route REPL I/O through ``stream``. Firmware calls ``os.dupterm``;
    tests monkey-patch this function to observe installations without
    touching the ``os`` module (which on MicroPython is a frozen type
    and rejects ``setattr``)."""
    import os
    if hasattr(os, "dupterm"):
        os.dupterm(stream)


class _Bridge(_IOBASE):
    """Holds the live BLE state, plus a stream interface for ``dupterm``.

    Subclasses ``io.IOBase`` so MicroPython attaches the C-level
    stream-protocol slot to the type. ``os.dupterm()``'s C-side check
    (``mp_get_stream_raise(... READ | WRITE | IOCTL)``) requires the
    type to *have* the protocol slot — pure Python classes without an
    ``IOBase`` ancestor fail the check with ``OSError: stream
    operation not supported`` even when the right Python methods are
    present. v1.0.7 added ``read``/``ioctl`` to the methods but didn't
    fix the inheritance; v1.0.8 does.
    """

    def __init__(self, ble):
        self._ble = ble
        self._tx_handle = None
        self._rx_handle = None
        self._rx_buffer = bytearray()
        self._conn_handle = None
        # ATT default payload is MTU - 3. We start conservative and let
        # _IRQ_MTU_EXCHANGED bump us up.
        self._mtu_payload = 20

    # ---- lifecycle ----

    def register_services(self):
        tx = (bluetooth.UUID(_UART_TX_UUID), bluetooth.FLAG_NOTIFY)
        rx = (bluetooth.UUID(_UART_RX_UUID),
              bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE)
        service = (bluetooth.UUID(_UART_SERVICE_UUID), (tx, rx))
        ((self._tx_handle, self._rx_handle),) = \
            self._ble.gatts_register_services((service,))

    # ---- IRQ ----

    def on_irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _addr_type, _addr = data
            self._conn_handle = conn_handle
        elif event == _IRQ_CENTRAL_DISCONNECT:
            self._conn_handle = None
            # Keep accepting new connections so a second ``openbricks-dev
            # run`` can dial back in without a hub reboot.
            _advertise(self._ble)
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if value_handle == self._rx_handle:
                self._rx_buffer += self._ble.gatts_read(self._rx_handle)
        elif event == _IRQ_MTU_EXCHANGED:
            _conn_handle, mtu = data
            self._mtu_payload = mtu - 3

    # ---- dupterm stream protocol ----
    #
    # ``os.dupterm()`` in modern MicroPython requires the stream object
    # to expose all four of ``read`` / ``readinto`` / ``write`` /
    # ``ioctl``. The C-side check is at ``mp_get_stream_raise(...,
    # MP_STREAM_OP_READ | MP_STREAM_OP_WRITE | MP_STREAM_OP_IOCTL)`` —
    # missing any one of them raises ``OSError: stream operation not
    # supported``. Pre-1.0.7 we only had readinto+write, which used to
    # be enough on older MP but breaks on the v1.27+ MP we vendor.

    def read(self, sz=None):
        """Read up to ``sz`` bytes from the BLE rx buffer (or all if
        ``sz`` is None). Returns ``b''`` when nothing is buffered."""
        rx = self._rx_buffer
        if not rx:
            return b""
        if sz is None or sz >= len(rx):
            result = bytes(rx)
            self._rx_buffer = bytearray()
        else:
            result = bytes(rx[:sz])
            self._rx_buffer = bytearray(rx[sz:])
        return result

    def readinto(self, buf):
        """Called by the REPL when it needs stdin. Returns ``None`` when
        no bytes are buffered (polls again later), else the byte count.

        MicroPython's bytearray doesn't support slice deletion, so we
        rebuild the tail into a fresh bytearray rather than ``del rx[:n]``.
        """
        rx = self._rx_buffer
        if not rx:
            return None
        n = min(len(buf), len(rx))
        buf[:n] = rx[:n]
        self._rx_buffer = bytearray(rx[n:])
        return n

    def ioctl(self, op, arg):
        """Stream-protocol ioctl. The REPL polls this to ask "is there
        data to read?" — return ``MP_STREAM_POLL_RD`` (0x01) when
        ``self._rx_buffer`` is non-empty, else 0. Other ops are
        ignored (return 0)."""
        # MP_STREAM_POLL = 3, MP_STREAM_POLL_RD = 0x01.
        # Hard-coded so we don't import a constant module on every poll.
        if op == 3:
            if self._rx_buffer:
                return 0x01
        return 0

    def write(self, buf):
        """Called by the REPL to emit stdout/stderr. Fan out over one or
        more BLE notifications depending on the MTU. Returns bytes sent;
        callers (including dupterm) rely on this being a truthy int."""
        if self._conn_handle is None:
            return 0
        mv = memoryview(buf)
        total = len(mv)
        sent = 0
        step = max(1, self._mtu_payload)
        while sent < total:
            chunk = bytes(mv[sent:sent + step])
            try:
                self._ble.gatts_notify(self._conn_handle, self._tx_handle, chunk)
            except OSError:
                # Peer went away mid-print. Don't propagate; the next
                # _IRQ_CENTRAL_DISCONNECT will clean up.
                break
            sent += len(chunk)
        return sent

    def close(self):
        # dupterm calls close() when it's unhooked. Nothing to free here.
        pass


def _advertise(ble, name=None):
    """Start advertising with the NUS service UUID and the GAP name.

    The name comes from ``openbricks.HUB_NAME`` — set by
    ``openbricks-dev flash`` into NVS and read fresh on each call so an
    in-field rename surfaces at the next disconnect/reconnect.
    """
    if name is None:
        import openbricks
        name = openbricks.HUB_NAME
    if name is None:
        # ``bluetooth.set_enabled`` guards against this, but belt-and-
        # braces for direct _advertise callers.
        raise RuntimeError("hub name unset; flash with openbricks-dev flash --name NAME")
    name_bytes = name.encode()[:29]
    payload = bytes([
        # Flags: LE general discoverable, classic BR/EDR unsupported.
        2, _ADV_TYPE_FLAGS, _ADV_FLAG_LE_GENERAL_DISC,
        # Complete local name.
        len(name_bytes) + 1, _ADV_TYPE_NAME_COMPLETE,
    ]) + name_bytes
    # 128-bit service UUID, little-endian byte order per the spec.
    svc = _uuid_bytes_le(_UART_SERVICE_UUID)
    payload += bytes([len(svc) + 1, _ADV_TYPE_UUID128_COMPLETE]) + svc
    # 100 ms interval — fast enough for "I just flashed, where's the
    # hub" discovery, low enough not to batter radio power budget.
    ble.gap_advertise(100_000, payload)


def _uuid_bytes_le(uuid_str):
    """Pack a dashed UUID string into 16 bytes, little-endian.

    Bluetooth service UUIDs on the wire are little-endian. ``int(..., 16)``
    gives big-endian so we reverse.
    """
    hex_ = uuid_str.replace("-", "")
    b = bytes.fromhex(hex_)
    return bytes(reversed(b))


def start():
    """Bring up the NUS bridge. Call after ``bluetooth.BLE().active(True)``.

    Idempotent: calling twice is a no-op, matching the ``set_enabled(True);
    set_enabled(True)`` pattern that main.py users might write.
    """
    if _state["bridge"] is not None:
        return
    ble = bluetooth.BLE()
    # BLE must already be active; asking gatts_register_services on an
    # inactive stack errors with OSError on ESP32.
    if not ble.active():
        raise RuntimeError(
            "BLE is not active; call openbricks.bluetooth.set_enabled(True) "
            "before ble_repl.start()"
        )
    bridge = _Bridge(ble)
    bridge.register_services()
    ble.irq(bridge.on_irq)
    try:
        ble.config(mtu=_REQ_MTU)
    except (OSError, ValueError):
        # Older MP builds may reject mtu in config(). Not fatal — 20-byte
        # chunks still work, just slower for big prints.
        pass
    _install_dupterm(bridge)
    _advertise(ble)
    _state["bridge"] = bridge


def stop():
    """Tear the bridge back down. Safe to call when not running."""
    bridge = _state["bridge"]
    if bridge is None:
        return
    _install_dupterm(None)
    ble = bluetooth.BLE()
    try:
        ble.gap_advertise(None)
    except OSError:
        pass
    # Clear the IRQ handler so a fresh start() can install its own
    # without racing on stale closure state.
    try:
        ble.irq(None)
    except (OSError, TypeError):
        pass
    _state["bridge"] = None


def is_running():
    return _state["bridge"] is not None
