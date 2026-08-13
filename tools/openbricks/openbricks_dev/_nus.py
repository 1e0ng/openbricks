# SPDX-License-Identifier: MIT
"""
Shared NUS (Nordic UART Service) client helper for ``openbricks``.

Thin wrapper around ``bleak`` that:

* Scans for a hub advertising the openbricks NUS service by its GAP
  name (the string baked into NVS at ``openbricks flash`` time).
* Opens a :class:`NUSLink` context that exposes two async primitives:
  ``write(bytes)`` pushes bytes into the hub's REPL stdin;
  ``read(timeout)`` yields whatever stdout/stderr the hub has notified
  since the last read.

Callers are the ``run`` and ``stop`` subcommands (and ``upload`` in
PR 4). Keeping this out of each subcommand module avoids bleak-setup
boilerplate duplication and gives us one place to tweak retry/timeout
policy.
"""

import asyncio
import sys
import time


# Hub-side advertises these — keep in sync with ``openbricks/ble_repl.py``.
UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX_UUID      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # hub → client (notify)
UART_RX_UUID      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # client → hub (write)


class NUSError(Exception):
    """Raised on BLE discovery / connect / I/O problems."""


def _print_packet(data, connected_at):
    """Print one received notify packet to stderr in --debug mode.

    Format: ``[+0.123s] rx 12 bytes: 01 02 ... | "ascii"`` — relative
    timestamp from connect, length, hex, and an ascii column with
    non-printables shown as ``.``.
    """
    t = time.monotonic() - connected_at if connected_at else 0.0
    hex_ = " ".join("%02x" % b for b in data)
    ascii_ = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
    print("[+%.3fs] rx %d bytes: %s | %r" % (t, len(data), hex_, ascii_),
          file=sys.stderr)


async def _find_by_name(name, timeout):
    """Return a ``BLEDevice`` advertising ``name`` within ``timeout`` seconds.

    Bleak's ``find_device_by_name`` is the idiomatic one-liner, but its
    matching is exact; we want the same semantics here (two openbricks
    hubs named "RobotA" and "RobotAA" should not alias).
    """
    try:
        from bleak import BleakScanner
    except ImportError as e:
        raise NUSError("bleak is not installed — run: pip install bleak") from e
    device = await BleakScanner.find_device_by_name(name, timeout=timeout)
    if device is None:
        raise NUSError(
            "no hub named %r found in %.1f s — check it's powered, "
            "BLE is enabled, and the flashed name matches" % (name, timeout))
    return device


class NUSLink:
    """Async context manager owning one live NUS connection.

    Typical use::

        async with NUSLink.connect("RobotA") as link:
            await link.write(b"print('hello')\\r\\n")
            chunk = await link.read(timeout=1.0)

    The read side buffers notifications internally; ``read`` pops
    whatever has arrived and optionally blocks up to ``timeout`` for
    the first byte. Subsequent bytes that arrive within the same
    callback cycle are coalesced.
    """

    def __init__(self, client, debug=False):
        self._client = client
        self._rx = bytearray()
        self._rx_event = asyncio.Event()
        # Diagnostic counters surfaced in timeout errors and --debug
        # output. ``connect()`` overwrites _connected_at; ``_on_notify``
        # bumps the rest. Used by ``stats()``.
        self._debug = debug
        self._connected_at = None
        self._notify_count = 0
        self._byte_count   = 0
        self._last_byte_at = None

    @classmethod
    async def connect(cls, name, scan_timeout=5.0, debug=False):
        """Scan + connect + subscribe. Raises :class:`NUSError` on any failure.

        ``debug=True`` makes the link print every notify packet's
        timestamp + hex + ascii to stderr — useful when you need to
        see whether the hub is sending anything at all (vs. silently
        dropping our writes) — plus a one-line per-stage timing
        breakdown (scan / connect / subscribe), the phases that
        dominate ``openbricks run``'s startup latency. The same
        numbers live in ``link.timings`` for tools and tests."""
        try:
            from bleak import BleakClient
        except ImportError as e:
            raise NUSError("bleak is not installed — run: pip install bleak") from e

        t0 = time.monotonic()
        device = await _find_by_name(name, scan_timeout)
        t_scan = time.monotonic()
        client = BleakClient(device)
        try:
            await client.connect()
        except Exception as e:
            raise NUSError("failed to connect to %r: %s" % (name, e)) from e
        t_conn = time.monotonic()

        link = cls(client, debug=debug)
        link._connected_at = t_conn

        def _on_notify(_char, data):
            link._notify_count += 1
            link._byte_count   += len(data)
            link._last_byte_at  = time.monotonic()
            if link._debug:
                _print_packet(data, link._connected_at)
            link._rx += data
            link._rx_event.set()

        try:
            await client.start_notify(UART_TX_UUID, _on_notify)
        except Exception as e:
            # Clean up the half-open connection before bailing.
            try:
                await client.disconnect()
            except Exception:
                pass
            raise NUSError(
                "failed to subscribe to TX characteristic (is this an "
                "openbricks hub with BLE REPL enabled?): %s" % e) from e

        t_sub = time.monotonic()
        link.timings = {
            "scan":      t_scan - t0,
            "connect":   t_conn - t_scan,
            "subscribe": t_sub - t_conn,
        }
        if debug:
            mtu = getattr(client, "mtu_size", None)
            print("[debug] connected, mtu=%s" % mtu, file=sys.stderr)
            print("[debug] timing: scan %.2fs  connect %.2fs  "
                  "subscribe %.2fs  (BLE total %.2fs)"
                  % (link.timings["scan"], link.timings["connect"],
                     link.timings["subscribe"], t_sub - t0),
                  file=sys.stderr)
        return link

    def stats(self):
        """Return a dict of diagnostic counters for timeout error
        formatting. Safe to call after disconnect."""
        now = time.monotonic()
        last_ago = None if self._last_byte_at is None else now - self._last_byte_at
        try:
            connected = self._client.is_connected
        except Exception:
            connected = "unknown"
        return {
            "connected":     connected,
            "notify_count":  self._notify_count,
            "byte_count":    self._byte_count,
            "last_byte_ago": last_ago,
            "uptime":        None if self._connected_at is None else now - self._connected_at,
        }

    async def __aenter__(self):
        # ``connect`` is already an async classmethod; we don't need to
        # do extra work on enter, but the ``async with`` form is nicer.
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.close()

    async def write(self, data):
        """Send ``data`` over the RX characteristic (client → hub).

        WRITE_NO_RESPONSE = no per-packet ACK; the fastest path for
        streaming REPL input, and what the hub's NUS RX characteristic
        advertises alongside WRITE.

        We chunk by the negotiated write size OURSELVES. The previous
        comment here claimed "bleak handles that internally" — it does
        NOT: neither the CoreBluetooth nor the BlueZ backend splits a
        write, and bleak's own API documents ``data`` as bounded by
        ``max_write_without_response_size``. Anything larger is at the
        mercy of the platform, and on this path failures are SILENT in
        both directions: an ATT Write Command has no error response by
        spec (NimBLE's ``ble_att_rx_extended`` discards the return for
        command opcodes), and MicroPython truncates a full
        characteristic buffer while still returning success
        (``modbluetooth_nimble.c``, with an upstream TODO admitting
        it). So an oversized write can vanish with no error at any
        layer — exactly the failure class that cost 1.31.0-1.32.1.
        """
        try:
            limit = getattr(self._client, "mtu_size", 0) or 23
            limit = max(20, limit - 3)          # ATT header
            mwwr = getattr(self._client, "max_write_without_response_size",
                           None)
            if isinstance(mwwr, int) and mwwr > 0:
                limit = min(limit, mwwr)
            mv = memoryview(data)
            for off in range(0, len(mv), limit):
                await self._client.write_gatt_char(
                    UART_RX_UUID, mv[off:off + limit], response=False)
        except Exception as e:
            raise NUSError("write failed: %s" % e) from e

    async def read(self, timeout=None):
        """Return any bytes received since the last ``read``, waiting up
        to ``timeout`` seconds for the first byte. ``timeout=None`` waits
        forever; ``timeout=0`` is a non-blocking drain.
        """
        if not self._rx:
            try:
                if timeout == 0:
                    # Non-blocking: the event may be set from a pending
                    # notification on the same loop iteration.
                    if not self._rx_event.is_set():
                        return b""
                else:
                    await asyncio.wait_for(self._rx_event.wait(), timeout=timeout)
            except asyncio.TimeoutError:
                return b""
        data = bytes(self._rx)
        self._rx.clear()
        self._rx_event.clear()
        return data

    async def close(self):
        # Teardown must be BOUNDED and cancellation-proof: under a
        # notification flood a stop_notify can stall (wait_for caps
        # each step), and the SIGINT routing delivers Ctrl-C as task
        # cancellation — BaseException so a cancellation mid-close
        # cannot skip the disconnect. (A RAW KeyboardInterrupt
        # inside a task is re-raised through the loop by asyncio
        # itself and no guard here could contain it — that crash
        # class is prevented upstream by run.py's signal routing,
        # macOS "Python quit unexpectedly", bench 2026-08-13.)
        try:
            await asyncio.wait_for(
                self._client.stop_notify(UART_TX_UUID), 3.0)
        except BaseException:
            pass
        try:
            await asyncio.wait_for(self._client.disconnect(), 4.0)
        except BaseException:
            pass
