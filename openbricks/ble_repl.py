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

try:
    import time as _TIME
    _TICKS = _TIME.ticks_ms
except (ImportError, AttributeError):
    import time as _TIME
    def _TICKS():
        return int(_TIME.monotonic() * 1000)
# Wrap-safe on MicroPython (ticks_diff); a plain difference where the
# monotonic fallback above is the clock.
_TICKS_DIFF = getattr(_TIME, "ticks_diff", None) or (lambda a, b: a - b)


# ---- Host activity (3.7.0) --------------------------------------------
#
# Every byte a central writes to the RX characteristic stamps
# ``_last_rx_ms``; ``host_active()`` says whether one landed within
# HOST_ACTIVE_MS. The status LED's transfer indicator (purple, fast)
# rides this: an ``openbricks upload`` / ``run`` paste is a stream of
# such writes, and a one-second hold-off bridges the pauses between
# the CLI's staging steps. No protocol change, so every CLI version
# lights it — and a stray REPL keystroke costs one second of purple,
# which is the honest answer to "is the host talking to my hub".
HOST_ACTIVE_MS = 1000
_last_rx_ms = None


def host_active(window_ms=HOST_ACTIVE_MS):
    """True while a BLE central has written to the hub within
    ``window_ms`` — an upload, a run's staging paste, a keystroke."""
    if _last_rx_ms is None:
        return False
    return _TICKS_DIFF(_TICKS(), _last_rx_ms) < window_ms


# ---- In-memory event log (1.2.1) -------------------------------------
#
# 1.2.0 worked in unit tests but produced 0 notify packets on hardware
# (host BLE-connected with MTU 256, 30 s wait, nothing came back).
# Three possible causes: CENTRAL_CONNECT IRQ not firing, GATTS_WRITE
# IRQ not firing, gatts_notify silently dropping. They look identical
# from the host. Logging IRQs + write attempts to memory (NOT through
# print/dupterm — that recurses through this module's own write path)
# tells us which one. Inspect via:
#
#   mpremote connect /dev/cu.usbmodem... exec \
#       'from openbricks import ble_repl; ble_repl.dump_log()'
#
# Cheap enough at 500 entries × ~40 bytes ≈ 20 KB to keep enabled.

_LOG = []
_LOG_MAX = 500
# Ring write index, meaningful once the log is full. Single-element
# list so scheduled/IRQ contexts mutate in place without ``global``.
_LOG_NEXT = [0]


def _log(tag, *args):
    """Record a timestamped event, keeping the LAST ``_LOG_MAX``.

    This used to stop recording once full ("first 500 events since
    boot"), which made it useless for exactly its purpose: a BLE
    failure minutes after boot left no trace because chatty connect
    traffic had already filled the log in the first minute. A ring
    keeps the most recent window instead. Failures inside logging are
    swallowed — a diagnostic must never take down the hot path."""
    try:
        entry = (_TICKS(), tag, args)
        if len(_LOG) < _LOG_MAX:
            _LOG.append(entry)
        else:
            i = _LOG_NEXT[0]
            _LOG[i] = entry
            _LOG_NEXT[0] = (i + 1) % _LOG_MAX
    except MemoryError:
        pass


def log_entries():
    """The recorded events, oldest first (unwraps the ring)."""
    n = len(_LOG)
    if n < _LOG_MAX:
        return list(_LOG)
    start = _LOG_NEXT[0]
    return [_LOG[(start + k) % n] for k in range(n)]


def dump_log():
    """Print the in-memory BLE event log over USB-Serial-JTAG.

    Why a helper rather than ``print(_LOG)``: ``print`` routes through
    dupterm which routes through this module's stream, so a long log
    print adds entries to itself while running. Iterating + printing
    item by item is bounded; the bytes added during the print sit in
    _tx_buf until later and don't grow the log.
    """
    entries = log_entries()
    wrapped = " (older events dropped)" if len(_LOG) == _LOG_MAX else ""
    print("ble_repl event log (last %d%s):" % (len(entries), wrapped))
    for entry in entries:
        t, tag, args = entry
        print("  %d %s %s" % (t, tag, args))


def clear_log():
    _LOG[:] = []
    _LOG_NEXT[0] = 0


# ---- BLE constants ----------------------------------------------------

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

_FLAG_WRITE             = const(0x0008)
_FLAG_NOTIFY            = const(0x0010)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)

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

# GATT rx buffer for the NUS RX characteristic. MUST be >= 2x the
# firmware's ``MICROPY_REPL_STDIN_BUFFER_MAX`` (boards/*/mpconfigboard.h):
# raw-paste flow control lets the host have TWO advertised windows
# (= one buffer max) in flight before it waits for an ack, and NimBLE
# silently drops writes once this buffer is full — the chip then
# compiles a fragment of the pasted program.
#
# 1.32.0 shipped exactly that bug: the window went 128 -> 2048
# (in-flight 4096) while this stayed at 512, sized for the OLD
# window ("2 windows + headroom"). Bench symptom: the staged
# receiver produced empty stdout AND empty stderr — a program
# truncated to nothing — and ``openbricks run`` failed with "hub did
# not confirm the staged chunk". ``tests/test_board_config.py``
# ::BleRxBufferTests pins the relationship in both directions so the
# two constants can never drift apart again.
_RX_BUFFER_BYTES = 8192

# Consumed-prefix threshold for ``_BLEUART.read``'s compaction. Reads
# advance an index instead of re-slicing (see that method), and the
# buffer is only rebuilt once this many bytes have been consumed —
# so each byte is copied at most once per this many reads instead of
# once per read.
_RX_COMPACT_AT = 1024


class _BLEUART:
    def __init__(self, ble, name, rxbuf=_RX_BUFFER_BYTES):
        self._ble = ble
        self._ble.irq(self._irq)
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services(
            ((bluetooth.UUID(_UART_SERVICE_UUID), (
                (bluetooth.UUID(_UART_TX_UUID), _FLAG_NOTIFY),
                # RX needs BOTH flags. The host-side openbricks
                # uses ``write_gatt_char(..., response=False)`` for
                # the streaming-REPL hot path; bleak / CoreBluetooth
                # silently drop that write if WRITE_NO_RESPONSE isn't
                # advertised on the characteristic. WRITE alone leaves
                # the chip reading 0 GATTS_WRITE IRQs even after a
                # successful BLE connect — which is exactly what 1.2.x
                # produced on hardware (see _LOG entries: CONNECT +
                # MTU exchange, then no irq(3,) entries).
                (bluetooth.UUID(_UART_RX_UUID),
                 _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE),
            )),)
        )
        # Append-mode rx buffer: back-to-back writes from the central
        # accumulate instead of overwriting. Without this, a quick
        # "Ctrl-C Ctrl-A" from openbricks run loses one of the two.
        #
        # Size: see ``_RX_BUFFER_BYTES`` above — it tracks the
        # raw-paste window, and undersizing it silently truncates
        # pasted programs (upstream BLEUART's 100 lost two thirds of
        # the bootstrap; our own 512 lost everything once 1.32.0
        # raised the window).
        self._ble.gatts_set_buffer(self._rx_handle, rxbuf, True)
        self._connections = set()
        self._rx_buffer = bytearray()
        self._rx_pos = 0
        self._handler = None
        self._payload = _advertising_payload(name=name, service_uuid_str=_UART_SERVICE_UUID)
        _log("init", self._tx_handle, self._rx_handle, name)
        self._advertise()

    def irq(self, handler):
        """Set the rx-arrived handler. Called from ``_BLEUARTStream``
        to install the ``os.dupterm_notify`` poke."""
        self._handler = handler

    def _irq(self, event, data):
        # Log EVERY event — including ones we don't expect — so we can
        # tell "CONNECT never fired" from "CONNECT fired but with weird
        # data" from "some other event we silently dropped".
        global _last_rx_ms
        _log("irq", event)
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            _log("CONNECT", conn_handle, len(self._connections))
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            _log("DISCONNECT", conn_handle, len(self._connections))
            # Keep accepting new connections.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            in_set = conn_handle in self._connections
            if in_set and value_handle == self._rx_handle:
                buf = self._ble.gatts_read(self._rx_handle)
                self._rx_buffer += buf
                _last_rx_ms = _TICKS()
                _log("GATTS_WRITE_OK", conn_handle, len(buf), bytes(buf[:8]))
                if self._handler:
                    self._handler()
            else:
                _log("GATTS_WRITE_DROP", conn_handle, value_handle, in_set, self._rx_handle)

    def any(self):
        return len(self._rx_buffer) - self._rx_pos

    def read(self, sz=None):
        """Drain up to ``sz`` bytes. O(sz) with amortised compaction.

        THE HOT PATH, and it is called ONCE PER BYTE: MicroPython's
        dupterm pulls stdin a byte at a time
        (``extmod/os_dupterm.c`` calls ``readinto`` with a 1-byte
        buffer), and the raw-paste tokenizer pulls each pasted byte
        through ``mp_hal_stdin_rx_chr``. The original implementation
        did ``self._rx_buffer = self._rx_buffer[sz:]`` here, which
        REALLOCATES AND COPIES the whole remaining buffer on every
        single byte — O(n^2) time plus one GC allocation per byte,
        on a heap that PSRAM made large (and therefore slow to
        collect).

        That is the real staging bottleneck, and it is why raising
        the raw-paste window made things WORSE rather than faster
        (1.32.0/1.32.1): a bigger window means more bytes buffered,
        and the per-byte cost grows with the buffer. Bench maths at
        the stock window put ~80% of each 269 ms window round trip
        on-chip, not on the radio.

        Reading via an index makes each byte O(1); the buffer is
        compacted only when the consumed prefix is worth reclaiming,
        and reset outright whenever it drains empty (the common
        case between pastes).
        """
        avail = len(self._rx_buffer) - self._rx_pos
        if avail <= 0:
            if self._rx_pos:
                self._rx_buffer = bytearray()
                self._rx_pos = 0
            return b""
        if not sz or sz > avail:
            sz = avail
        start = self._rx_pos
        self._rx_pos = start + sz
        result = bytes(self._rx_buffer[start:self._rx_pos])
        if self._rx_pos >= len(self._rx_buffer):
            # Fully drained: drop the whole allocation.
            self._rx_buffer = bytearray()
            self._rx_pos = 0
        elif self._rx_pos >= _RX_COMPACT_AT:
            # Reclaim the consumed prefix. Amortised: each byte is
            # copied at most once per _RX_COMPACT_AT bytes read.
            self._rx_buffer = self._rx_buffer[self._rx_pos:]
            self._rx_pos = 0
        return result

    def write(self, data):
        """Notify ``data`` to every connected central.

        Returns ``True`` when the bytes are handled — delivered to at
        least one central, or deliberately dropped because nobody is
        connected — and ``False`` when every notify failed (NimBLE
        raises ``OSError`` when its msys buffer pool is exhausted,
        which a fast dump like ``openbricks log`` reliably provokes).
        The caller keeps the bytes and retries on ``False``; treating
        that error as fire-and-forget silently lost one TX chunk per
        exhaustion, which read back as missing / seemingly duplicated
        log lines on the host (the 1.11.0 ``log`` corruption)."""
        if not self._connections:
            _log("write_no_conn", len(data))
            return True
        ok = False
        for conn_handle in self._connections:
            try:
                self._ble.gatts_notify(conn_handle, self._tx_handle, data)
                ok = True
                _log("notify_ok", conn_handle, len(data))
            except OSError as e:
                # Buffer exhaustion, or the peer went away mid-notify
                # (the disconnect IRQ will tidy the latter up).
                _log("notify_err", conn_handle, len(data), str(e))
        return ok

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
        _log("advertise", interval_us, len(self._payload))

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
        self._uart.irq(self._on_rx)

    def _on_rx(self):
        # Wake dupterm so it drains uart's rx buffer into stdin
        # immediately. Without this, a Ctrl-C arriving over BLE sits
        # in the buffer until the REPL happens to poll — which can
        # be never, if user code is busy-looping.
        if hasattr(os, "dupterm_notify"):
            os.dupterm_notify(None)

    # Exception armor on every dupterm-facing method. MicroPython
    # DEACTIVATES a dupterm stream whose read()/write() raises
    # (extmod/os_dupterm.c: mp_os_deactivate, "dupterm: Exception in
    # ... method, deactivating") — the message goes to the USB
    # console, so over BLE the symptom is a hub that connects, logs
    # every host byte, and never sends another notify. The stop
    # button's injected KeyboardInterrupt raises at an arbitrary
    # main-thread bytecode — including inside these methods during a
    # print storm — so a stop press could silently kill the BLE
    # console until reboot (the 1.12.0 bench wedge). Swallowing here
    # is safe: eating one injection costs one 300 ms launcher retry
    # (the e-stop latch already stopped the robot), while a
    # deactivated dupterm is permanent.

    def read(self, sz=None):
        try:
            return self._uart.read(sz)
        except BaseException:
            return None

    def readinto(self, buf):
        try:
            avail = self._uart.read(len(buf))
            if not avail:
                return None
            for i in range(len(avail)):
                buf[i] = avail[i]
            return len(avail)
        except BaseException:
            return None

    def ioctl(self, op, arg):
        try:
            if op == _MP_STREAM_POLL:
                if self._uart.any():
                    return _MP_STREAM_POLL_RD
            return 0
        except BaseException:
            return 0

    # No "already scheduled" flag — deliberately. The stop button
    # injects KeyboardInterrupt at an arbitrary bytecode boundary of
    # whatever the main thread is executing, and scheduled callbacks
    # run on the main thread: with a flag, an interrupt landing at
    # ``_flush``'s entry (before the flag-clearing statement) unwound
    # the callback with the flag stranded True, and every later
    # ``write()`` believed a flush was pending — BLE TX dead until
    # reboot, while advertising and connections still worked (the
    # 1.10.4 bug: "hub connects but never notifies after a button
    # stop"). Scheduling per write is idempotent instead: duplicate
    # flushes no-op on an empty buffer, a full scheduler queue just
    # defers to the next write, and an interrupt unwinding ``_flush``
    # anywhere leaves no state to strand — the next write (the
    # KeyboardInterrupt traceback itself, usually) re-schedules.

    # TX buffer hard cap. Bytes are retained across notify failures
    # (see ``_flush``), so a persistently jammed link plus a chatty
    # program could otherwise grow ``_tx_buf`` without bound. When the
    # cap is hit the OLDEST bytes are dropped — for a console stream
    # the tail is the valuable part.
    _TX_MAX = 8192

    def _flush(self, _arg):
        if not self._tx_buf:
            return
        # Peek-then-trim: the chunk leaves the buffer only after the
        # notify reports success. The previous trim-before-write order
        # lost the chunk whenever gatts_notify raised (NimBLE buffer
        # exhaustion under a sustained dump) — at-least-once beats
        # at-most-once here, because a lost chunk can be the raw-REPL
        # ``\x04`` terminator, which hangs the host until its timeout.
        # The residual race — an injected KeyboardInterrupt landing
        # between the successful write and the trim — re-sends at most
        # one 100-byte chunk, only during a button stop.
        data = bytes(self._tx_buf[0:100])
        if not self._uart.write(data):
            # Notify failed; keep the bytes. Deliberately NOT
            # re-scheduling here: an immediate retry can spin the
            # scheduler drain loop while the stack's buffers are
            # still full. The retry is paced instead — the next
            # ``write()`` or the launcher's ``pump_tx`` tick.
            return
        self._tx_buf = self._tx_buf[100:]
        if self._tx_buf:
            self._schedule_flush()

    def _schedule_flush(self):
        try:
            _SCHEDULE(self._flush, None)
        except RuntimeError:
            # Scheduler queue full. The bytes stay in ``_tx_buf``;
            # the next ``write()`` or ``pump_tx`` re-requests.
            pass

    def write(self, buf):
        # Same armor rationale as read/readinto/ioctl above: an
        # exception escaping write() (stop-button KeyboardInterrupt
        # landing mid-body, MemoryError growing the buffer) makes
        # MicroPython deactivate the dupterm slot permanently. Worst
        # case of swallowing: this chunk is lost or the flush waits
        # for the pump tick.
        try:
            was_empty = not self._tx_buf
            self._tx_buf += buf
            overflow = len(self._tx_buf) - self._TX_MAX
            if overflow > 0:
                self._tx_buf = self._tx_buf[overflow:]
                _log("tx_overflow", overflow)
            # Schedule only on the empty->non-empty transition. A
            # non-empty buffer means a flush chain is (normally)
            # already re-scheduling itself, and if that chain died the
            # launcher pump revives it within one poll tick. The old
            # schedule-per-write flooded the 8-deep scheduler queue
            # during print storms, and machine.Timer callbacks share
            # that queue: a full queue silently DROPS timer ticks —
            # including the button watcher, which is how a stop press
            # went completely unseen (no 'button pressed' breadcrumb).
            # The decision derives from buffer content, not a flag, so
            # an interrupt unwinding anywhere strands nothing.
            if was_empty:
                self._schedule_flush()
        except BaseException:
            pass
        return len(buf)


# ---- Public API ------------------------------------------------------

# Singleton bridge state. ``_state["bridge"]`` is the active
# ``_BLEUART`` (kept under that key for backwards-compat with tests
# that introspected the bridge before 1.2.0). ``_state["stream"]``
# is the ``_BLEUARTStream`` installed in dupterm.
_state = {"bridge": None, "stream": None}


def is_running():
    return _state["bridge"] is not None


def pump_tx():
    """Re-arm the TX flush if bytes are sitting in the buffer.

    Liveness backstop, called from the launcher's poll tick. A flush
    chain dies in two ways: ``micropython.schedule``'s queue was full
    at re-schedule time, or a notify failed and the retry is
    deliberately paced (see ``_flush``). In both cases the buffered
    bytes wait for the *next* ``write()`` — which never comes when the
    writer already finished. That was the ``openbricks log``
    end-of-dump stall: the file content and the raw-REPL terminator
    sat in ``_tx_buf`` forever while the host timed out. The pump
    gives the chain a fresh start at tick cadence; it is idempotent
    and no-ops when the buffer is empty or the bridge is down."""
    stream = _state["stream"]
    if stream is not None and stream._tx_buf:
        stream._schedule_flush()


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
