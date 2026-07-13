# SPDX-License-Identifier: MIT
"""Tests for ``openbricks_dev._nus`` — the BLE NUS transport every
subcommand rides on — against an injected fake bleak."""

import asyncio
import io
import sys
import unittest

from openbricks_dev import _nus
from openbricks_dev._nus import NUSError, NUSLink


class _FakeBleakClient:
    """Scripted BleakClient: records calls, captures the notify
    callback so tests can inject hub->host packets, and raises on
    demand at each step."""

    def __init__(self, device):
        self.device = device
        self.calls = []
        self.notify_cb = None
        self.is_connected = True
        self.mtu_size = 256
        self.fail_connect = None
        self.fail_start_notify = None
        self.fail_write = None

    async def connect(self):
        self.calls.append("connect")
        if self.fail_connect:
            raise self.fail_connect

    async def disconnect(self):
        self.calls.append("disconnect")

    async def start_notify(self, uuid, cb):
        self.calls.append(("start_notify", uuid))
        if self.fail_start_notify:
            raise self.fail_start_notify
        self.notify_cb = cb

    async def stop_notify(self, uuid):
        self.calls.append(("stop_notify", uuid))

    async def write_gatt_char(self, uuid, data, response=True):
        self.calls.append(("write", uuid, bytes(data), response))
        if self.fail_write:
            raise self.fail_write


class _FakeBleakModule:
    """Injected as ``sys.modules['bleak']``. Class-level state scripts
    one connect() flow."""

    device = "fake-device"
    client = None            # set per-test

    class BleakScanner:
        @staticmethod
        async def find_device_by_name(name, timeout=None):
            _FakeBleakModule.last_scan = (name, timeout)
            return _FakeBleakModule.device

    class BleakClient:
        def __new__(cls, device):
            _FakeBleakModule.client = _FakeBleakClient(device)
            return _FakeBleakModule.client


class _BleakInjection(unittest.TestCase):
    """setUp/tearDown that swaps a fake ``bleak`` into sys.modules."""

    def setUp(self):
        self._had = "bleak" in sys.modules
        self._prev = sys.modules.get("bleak")
        sys.modules["bleak"] = _FakeBleakModule
        _FakeBleakModule.device = "fake-device"
        _FakeBleakModule.client = None

    def tearDown(self):
        if self._had:
            sys.modules["bleak"] = self._prev
        else:
            del sys.modules["bleak"]


class PrintPacketTests(unittest.TestCase):
    def test_hex_and_ascii_columns(self):
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            _nus._print_packet(b"A\x01", connected_at=None)
        finally:
            sys.stderr = orig
        line = err.getvalue()
        self.assertIn("rx 2 bytes", line)
        self.assertIn("41 01", line)
        self.assertIn("'A.'", line)   # non-printable rendered as dot


class ConnectTests(_BleakInjection):
    def test_happy_path_subscribes_and_wires_notify(self):
        link = asyncio.run(NUSLink.connect("RobotA", scan_timeout=2.0))
        client = _FakeBleakModule.client
        self.assertEqual(_FakeBleakModule.last_scan, ("RobotA", 2.0))
        self.assertIn("connect", client.calls)
        self.assertIn(("start_notify", _nus.UART_TX_UUID), client.calls)
        # Hub sends a packet: counters bump and read() returns it.
        client.notify_cb(None, b"hello")
        self.assertEqual(asyncio.run(link.read(timeout=0.1)), b"hello")
        s = link.stats()
        self.assertEqual(s["notify_count"], 1)
        self.assertEqual(s["byte_count"], 5)
        self.assertTrue(s["uptime"] >= 0)
        self.assertTrue(s["last_byte_ago"] >= 0)

    def test_device_not_found_raises(self):
        _FakeBleakModule.device = None
        try:
            asyncio.run(NUSLink.connect("Ghost", scan_timeout=0.5))
        except NUSError as e:
            self.assertIn("no hub named 'Ghost'", str(e))
        else:
            self.fail("expected NUSError")

    def test_connect_failure_raises(self):
        class _Failing(_FakeBleakModule):
            class BleakClient:
                def __new__(cls, device):
                    c = _FakeBleakClient(device)
                    c.fail_connect = OSError("radio off")
                    _FakeBleakModule.client = c
                    return c
        sys.modules["bleak"] = _Failing
        try:
            asyncio.run(NUSLink.connect("RobotA"))
        except NUSError as e:
            self.assertIn("failed to connect", str(e))
        else:
            self.fail("expected NUSError")

    def test_subscribe_failure_disconnects_and_raises(self):
        class _Failing(_FakeBleakModule):
            class BleakClient:
                def __new__(cls, device):
                    c = _FakeBleakClient(device)
                    c.fail_start_notify = OSError("no such char")
                    _FakeBleakModule.client = c
                    return c
        sys.modules["bleak"] = _Failing
        try:
            asyncio.run(NUSLink.connect("RobotA"))
        except NUSError as e:
            self.assertIn("failed to subscribe", str(e))
        else:
            self.fail("expected NUSError")
        # The half-open connection must have been torn down.
        self.assertIn("disconnect", _FakeBleakModule.client.calls)

    def test_subscribe_failure_with_sticky_disconnect_still_typed(self):
        # Cleanup disconnect ALSO failing must not mask the NUSError.
        class _Failing(_FakeBleakModule):
            class BleakClient:
                def __new__(cls, device):
                    c = _FakeBleakClient(device)
                    c.fail_start_notify = OSError("no such char")

                    async def _bad_disconnect():
                        raise OSError("gone")
                    c.disconnect = _bad_disconnect
                    _FakeBleakModule.client = c
                    return c
        sys.modules["bleak"] = _Failing
        with self.assertRaises(NUSError):
            asyncio.run(NUSLink.connect("RobotA"))

    def test_debug_prints_connect_line_and_packets(self):
        err = io.StringIO()
        orig, sys.stderr = sys.stderr, err
        try:
            link = asyncio.run(NUSLink.connect("RobotA", debug=True))
            _FakeBleakModule.client.notify_cb(None, b"\x04")
        finally:
            sys.stderr = orig
        out = err.getvalue()
        self.assertIn("[debug] connected, mtu=256", out)
        self.assertIn("rx 1 bytes", out)
        self.assertEqual(asyncio.run(link.read(timeout=0)), b"\x04")


class LinkIOTests(unittest.TestCase):
    def _link(self):
        client = _FakeBleakClient("dev")
        return NUSLink(client), client

    def test_write_uses_rx_char_without_response(self):
        link, client = self._link()
        asyncio.run(link.write(b"abc"))
        self.assertEqual(
            client.calls, [("write", _nus.UART_RX_UUID, b"abc", False)])

    def test_write_failure_raises_nus_error(self):
        link, client = self._link()
        client.fail_write = OSError("gatt gone")
        try:
            asyncio.run(link.write(b"abc"))
        except NUSError as e:
            self.assertIn("write failed", str(e))
        else:
            self.fail("expected NUSError")

    def test_read_returns_buffered_and_clears(self):
        link, _ = self._link()
        link._rx += b"data"
        link._rx_event.set()
        self.assertEqual(asyncio.run(link.read(timeout=0)), b"data")
        self.assertEqual(asyncio.run(link.read(timeout=0)), b"")

    def test_read_nonblocking_empty(self):
        link, _ = self._link()
        self.assertEqual(asyncio.run(link.read(timeout=0)), b"")

    def test_read_timeout_returns_empty(self):
        link, _ = self._link()
        self.assertEqual(asyncio.run(link.read(timeout=0.01)), b"")

    def test_read_wakes_on_late_arrival(self):
        link, _ = self._link()

        async def _feed_then_read():
            async def _feed():
                await asyncio.sleep(0.01)
                link._rx += b"late"
                link._rx_event.set()
            feeder = asyncio.ensure_future(_feed())
            data = await link.read(timeout=1.0)
            await feeder
            return data
        self.assertEqual(asyncio.run(_feed_then_read()), b"late")

    def test_stats_fresh_link(self):
        link, _ = self._link()
        s = link.stats()
        self.assertEqual(s["notify_count"], 0)
        self.assertIsNone(s["last_byte_ago"])
        self.assertIsNone(s["uptime"])
        self.assertTrue(s["connected"])

    def test_stats_survives_broken_client(self):
        class _Broken:
            @property
            def is_connected(self):
                raise OSError("gone")
        link = NUSLink(_Broken())
        self.assertEqual(link.stats()["connected"], "unknown")

    def test_close_swallows_teardown_errors(self):
        class _Sticky(_FakeBleakClient):
            async def stop_notify(self, uuid):
                raise OSError("already gone")

            async def disconnect(self):
                raise OSError("already gone")
        link = NUSLink(_Sticky("dev"))
        asyncio.run(link.close())   # must not raise

    def test_async_with_closes(self):
        link, client = self._link()

        async def _use():
            async with link:
                pass
        asyncio.run(_use())
        self.assertIn(("stop_notify", _nus.UART_TX_UUID), client.calls)
        self.assertIn("disconnect", client.calls)


class MissingBleakTests(unittest.TestCase):
    def setUp(self):
        self._had = "bleak" in sys.modules
        self._prev = sys.modules.get("bleak")
        sys.modules["bleak"] = None   # import bleak -> ImportError

    def tearDown(self):
        if self._had:
            sys.modules["bleak"] = self._prev
        else:
            del sys.modules["bleak"]

    def test_find_reports_install_hint(self):
        try:
            asyncio.run(_nus._find_by_name("x", 0.1))
        except NUSError as e:
            self.assertIn("pip install bleak", str(e))
        else:
            self.fail("expected NUSError")

    def test_connect_reports_install_hint(self):
        try:
            asyncio.run(NUSLink.connect("x"))
        except NUSError as e:
            self.assertIn("pip install bleak", str(e))
        else:
            self.fail("expected NUSError")


if __name__ == "__main__":
    unittest.main()
