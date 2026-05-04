# SPDX-License-Identifier: MIT
"""Tests for openbricks.ble_repl — NUS bridge + dupterm plumbing."""

import tests._fakes       # noqa: F401
import tests._fakes_ble   # noqa: F401  (installs fake esp32 + bluetooth + os.dupterm)

import os as _real_os
import unittest

from tests._fakes_ble import (
    _FakeBLE,
    _FakeNVS,
    _FakeOsDupterm,
    _FakeBluetoothModule,
)
import openbricks
from openbricks import ble_repl


def _set_hub_name(name):
    _FakeNVS(openbricks._HUB_NAME_NVS_NAMESPACE).set_blob(
        openbricks._HUB_NAME_NVS_KEY, name.encode())


def _activate_ble():
    """Take BLE from cold → active without going through bluetooth.set_enabled
    (which would also start the REPL bridge itself)."""
    import bluetooth
    bluetooth.BLE().active(True)


class StartStopTests(unittest.TestCase):
    def setUp(self):
        _FakeBLE._reset_for_test()
        _FakeNVS._reset_for_test()
        _FakeOsDupterm._reset_for_test()
        # ble_repl is a singleton — clear any lingering bridge from a
        # previous test.
        if ble_repl.is_running():
            ble_repl.stop()
        _set_hub_name("TestHub")

    def test_start_registers_nus_service_and_installs_dupterm(self):
        _activate_ble()
        ble_repl.start()

        self.assertTrue(ble_repl.is_running())
        # Exactly one service registered, with two characteristics (TX, RX).
        self.assertEqual(len(_FakeBLE._service_registrations), 1)
        svc_uuid, chars = _FakeBLE._service_registrations[0]
        self.assertEqual(svc_uuid.value, ble_repl._UART_SERVICE_UUID)
        self.assertEqual(len(chars), 2)
        self.assertEqual(chars[0][0].value, ble_repl._UART_TX_UUID)
        self.assertEqual(chars[1][0].value, ble_repl._UART_RX_UUID)
        # Flags: TX is NOTIFY, RX is WRITE | WRITE_NO_RESPONSE.
        self.assertEqual(chars[0][1], _FakeBluetoothModule.FLAG_NOTIFY)
        self.assertEqual(
            chars[1][1],
            _FakeBluetoothModule.FLAG_WRITE | _FakeBluetoothModule.FLAG_WRITE_NO_RESPONSE,
        )
        # dupterm has the bridge stream.
        self.assertIsNotNone(_FakeOsDupterm.installed_stream)
        # Advertising started, interval 100 ms, payload includes the name.
        self.assertEqual(_FakeBLE._adv_interval_us, 100_000)
        self.assertIn(b"TestHub", _FakeBLE._adv_payload)

    def test_start_when_ble_inactive_raises(self):
        # BLE stays inactive on purpose.
        with self.assertRaises(RuntimeError):
            ble_repl.start()

    def test_start_is_idempotent(self):
        _activate_ble()
        ble_repl.start()
        first_bridge = ble_repl._state["bridge"]
        ble_repl.start()
        self.assertIs(ble_repl._state["bridge"], first_bridge)

    def test_stop_clears_dupterm_and_stops_advertising(self):
        _activate_ble()
        ble_repl.start()
        ble_repl.stop()

        self.assertFalse(ble_repl.is_running())
        self.assertIsNone(_FakeOsDupterm.installed_stream)
        # gap_advertise(None) clears the payload.
        self.assertIsNone(_FakeBLE._adv_payload)

    def test_stop_when_not_running_is_safe(self):
        # Should not raise when there's no bridge to tear down.
        ble_repl.stop()
        self.assertFalse(ble_repl.is_running())


class AdvertPayloadTests(unittest.TestCase):
    """Payload must carry the NUS 128-bit service UUID and the GAP name
    so ``openbricks-dev list`` (service-UUID filter) and
    ``openbricks-dev run -n NAME`` (name filter) both work."""

    def setUp(self):
        _FakeBLE._reset_for_test()
        _FakeNVS._reset_for_test()
        _FakeOsDupterm._reset_for_test()
        if ble_repl.is_running():
            ble_repl.stop()

    def test_payload_contains_service_uuid_bytes_le(self):
        _set_hub_name("H")
        _activate_ble()
        ble_repl.start()

        uuid_le = ble_repl._uuid_bytes_le(ble_repl._UART_SERVICE_UUID)
        self.assertEqual(len(uuid_le), 16)
        self.assertIn(uuid_le, _FakeBLE._adv_payload)

    def test_payload_contains_name(self):
        _set_hub_name("RobotA")
        _activate_ble()
        ble_repl.start()
        self.assertIn(b"RobotA", _FakeBLE._adv_payload)


class StreamBridgeTests(unittest.TestCase):
    def setUp(self):
        _FakeBLE._reset_for_test()
        _FakeNVS._reset_for_test()
        _FakeOsDupterm._reset_for_test()
        if ble_repl.is_running():
            ble_repl.stop()
        _set_hub_name("TestHub")
        _activate_ble()
        ble_repl.start()
        self.bridge = ble_repl._state["bridge"]
        self.rx_handle = self.bridge._rx_handle
        self.tx_handle = self.bridge._tx_handle

    def test_bridge_subclasses_io_iobase(self):
        # ``os.dupterm()``'s C-side check at ``mp_get_stream_raise``
        # requires the OBJECT'S TYPE to have the stream-protocol slot
        # (the ``mp_stream_p_t`` of function pointers). Python classes
        # inherit this slot from their first base; pure-Python classes
        # without a stream-aware ancestor fail the check with
        # ``OSError: stream operation not supported`` — even if all
        # the right methods (``read``/``readinto``/``write``/``ioctl``)
        # are defined as Python methods. ``io.IOBase`` has the slot
        # pre-installed with a Python-method-dispatching adapter.
        # 1.0.7 added the methods but didn't fix inheritance; 1.0.8
        # makes ``_Bridge`` extend ``io.IOBase``.
        import io
        self.assertIsInstance(self.bridge, io.IOBase,
                              "_Bridge must inherit from io.IOBase so "
                              "MicroPython attaches the stream-protocol "
                              "slot to the type")

    def test_read_returns_buffered_bytes(self):
        # ``read(sz)`` complements ``readinto`` for the dupterm stream
        # protocol — modern MicroPython requires both. Pre-1.0.7 we
        # only had readinto, which made os.dupterm raise OSError.
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"abcdef")
        self.assertEqual(self.bridge.read(3), b"abc")
        self.assertEqual(self.bridge.read(), b"def")
        self.assertEqual(self.bridge.read(), b"")

    def test_ioctl_reports_readable_when_buffered(self):
        # ``os.dupterm`` polls the stream via ``ioctl(MP_STREAM_POLL=3,
        # ...)`` and expects ``MP_STREAM_POLL_RD = 0x01`` when there's
        # data to read. Without this, the REPL never sees BLE input.
        _MP_STREAM_POLL = 3
        _MP_STREAM_POLL_RD = 0x01
        # Empty buffer → no readable bit set.
        self.assertEqual(self.bridge.ioctl(_MP_STREAM_POLL, None), 0)
        # Buffer has data → readable.
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"x")
        self.assertEqual(self.bridge.ioctl(_MP_STREAM_POLL, None),
                         _MP_STREAM_POLL_RD)
        # Other ops return 0 (we don't support them).
        self.assertEqual(self.bridge.ioctl(99, None), 0)

    def test_rx_irq_calls_dupterm_notify(self):
        # ``os.dupterm`` runs the REPL on its own schedule; without an
        # explicit ``dupterm_notify`` poke from the BLE rx-IRQ handler,
        # bytes sit in ``_rx_buffer`` until the REPL eventually polls
        # — but a Ctrl-C the host sent gets stuck in the buffer for
        # too long, breaking ``openbricks run -n NAME`` (it sends
        # Ctrl-C to interrupt user code, then expects the raw-REPL
        # banner; with the poke missing, it times out).
        #
        # The upstream ``examples/bluetooth/ble_uart_repl.py`` does
        # the same poke in its rx callback. Pin: when the central
        # writes to the rx characteristic, our IRQ handler calls
        # ``os.dupterm_notify``.
        import os
        notify_calls = []
        orig = getattr(os, "dupterm_notify", None)
        os.dupterm_notify = lambda arg: notify_calls.append(arg)
        try:
            _FakeBLE._simulate_central_write(1, self.rx_handle, b"x")
        finally:
            if orig is None:
                del os.dupterm_notify
            else:
                os.dupterm_notify = orig
        self.assertEqual(len(notify_calls), 1,
                         "rx IRQ must call os.dupterm_notify(None) "
                         "exactly once per write — without it, BLE "
                         "input never reaches the REPL")

    def test_central_write_fills_rx_buffer(self):
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"hello")
        buf = bytearray(32)
        n = self.bridge.readinto(buf)
        self.assertEqual(n, 5)
        self.assertEqual(bytes(buf[:n]), b"hello")

    def test_readinto_returns_none_when_empty(self):
        buf = bytearray(32)
        self.assertIsNone(self.bridge.readinto(buf))

    def test_multiple_writes_coalesce(self):
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"foo")
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"bar")
        buf = bytearray(32)
        n = self.bridge.readinto(buf)
        self.assertEqual(bytes(buf[:n]), b"foobar")

    def test_write_emits_gatts_notify_when_connected(self):
        # Simulate the central-connect IRQ so the bridge has a conn_handle.
        _FakeBLE._fire_irq(1, (42, 0, b"\x00" * 6))  # _IRQ_CENTRAL_CONNECT
        self.bridge.write(b"hi")

        self.assertTrue(_FakeBLE._notify_log)
        conn, handle, data = _FakeBLE._notify_log[-1]
        self.assertEqual(conn, 42)
        self.assertEqual(handle, self.tx_handle)
        self.assertEqual(data, b"hi")

    def test_write_without_connection_is_noop(self):
        # No central ever connected.
        n = self.bridge.write(b"ignored")
        self.assertEqual(n, 0)
        self.assertEqual(_FakeBLE._notify_log, [])

    def test_large_write_chunks_to_mtu(self):
        _FakeBLE._fire_irq(1, (42, 0, b"\x00" * 6))  # connect
        # Start with the default 20-byte payload cap.
        payload = b"A" * 55
        self.bridge.write(payload)

        # Expect ceil(55 / 20) = 3 notifications, first two 20 bytes, last 15.
        sizes = [len(chunk) for (_c, _h, chunk) in _FakeBLE._notify_log]
        self.assertEqual(sizes, [20, 20, 15])
        # Concatenation round-trips.
        joined = b"".join(chunk for (_c, _h, chunk) in _FakeBLE._notify_log)
        self.assertEqual(joined, payload)

    def test_mtu_irq_enlarges_chunk_size(self):
        _FakeBLE._fire_irq(1, (42, 0, b"\x00" * 6))  # connect
        _FakeBLE._fire_irq(21, (42, 100))            # _IRQ_MTU_EXCHANGED mtu=100
        # Now payload cap is 100 - 3 = 97 bytes.
        payload = b"B" * 97
        _FakeBLE._notify_log.clear()
        self.bridge.write(payload)
        self.assertEqual(len(_FakeBLE._notify_log), 1)

    def test_disconnect_resumes_advertising(self):
        _FakeBLE._fire_irq(1, (42, 0, b"\x00" * 6))  # connect
        # Wipe the advertising payload to simulate the "connected →
        # advertising paused" state.
        _FakeBLE._adv_payload = None
        _FakeBLE._fire_irq(2, (42, 0, b"\x00" * 6))  # _IRQ_CENTRAL_DISCONNECT
        self.assertIsNotNone(_FakeBLE._adv_payload)


class BluetoothIntegrationTests(unittest.TestCase):
    """``openbricks.bluetooth.set_enabled(True)`` should start the REPL
    bridge; ``set_enabled(False)`` should stop it."""

    def setUp(self):
        _FakeBLE._reset_for_test()
        _FakeNVS._reset_for_test()
        _FakeOsDupterm._reset_for_test()
        if ble_repl.is_running():
            ble_repl.stop()
        _set_hub_name("TestHub")

    def test_set_enabled_true_starts_bridge(self):
        from openbricks import bluetooth
        bluetooth.set_enabled(True)
        self.assertTrue(ble_repl.is_running())

    def test_set_enabled_false_stops_bridge(self):
        from openbricks import bluetooth
        bluetooth.set_enabled(True)
        self.assertTrue(ble_repl.is_running())
        bluetooth.set_enabled(False)
        self.assertFalse(ble_repl.is_running())

    def test_apply_persisted_state_starts_bridge_when_on(self):
        from openbricks import bluetooth
        bluetooth.apply_persisted_state()
        self.assertTrue(ble_repl.is_running())


if __name__ == "__main__":
    unittest.main()
