# SPDX-License-Identifier: MIT
"""Tests for openbricks.ble_repl — vendored upstream BLEUART + dupterm.

The bridge is now a near-verbatim port of upstream MicroPython's
``examples/bluetooth/ble_uart_peripheral.py`` + ``ble_uart_repl.py``;
tests focus on the public surface and the openbricks-specific bits
(advertising payload, NVS hub-name, idempotent start/stop)."""

import tests._fakes       # noqa: F401
import tests._fakes_ble   # noqa: F401  (installs fake esp32 + bluetooth + os.dupterm)

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
        if ble_repl.is_running():
            ble_repl.stop()
        _set_hub_name("TestHub")

    def test_start_registers_nus_service_and_installs_dupterm(self):
        _activate_ble()
        ble_repl.start()

        self.assertTrue(ble_repl.is_running())
        # One service registered, two characteristics (TX, RX).
        self.assertEqual(len(_FakeBLE._service_registrations), 1)
        svc_uuid, chars = _FakeBLE._service_registrations[0]
        self.assertEqual(svc_uuid.value, ble_repl._UART_SERVICE_UUID)
        self.assertEqual(len(chars), 2)
        self.assertEqual(chars[0][0].value, ble_repl._UART_TX_UUID)
        self.assertEqual(chars[1][0].value, ble_repl._UART_RX_UUID)
        # TX = NOTIFY, RX = WRITE (matching upstream BLEUART exactly).
        self.assertEqual(chars[0][1], _FakeBluetoothModule.FLAG_NOTIFY)
        # RX must declare BOTH WRITE and WRITE_NO_RESPONSE — bleak
        # uses write-no-response for the streaming-REPL hot path
        # (response=False), and CoreBluetooth silently drops it
        # against a characteristic that only advertises WRITE.
        rx_flags = chars[1][1]
        self.assertTrue(rx_flags & _FakeBluetoothModule.FLAG_WRITE,
                        "RX must include FLAG_WRITE")
        self.assertTrue(rx_flags & _FakeBluetoothModule.FLAG_WRITE_NO_RESPONSE,
                        "RX must include FLAG_WRITE_NO_RESPONSE")
        # dupterm has the stream installed.
        self.assertIsNotNone(_FakeOsDupterm.installed_stream)
        # Advertising started, interval 100 ms, payload includes the name.
        self.assertEqual(_FakeBLE._adv_interval_us, 100_000)
        self.assertIn(b"TestHub", _FakeBLE._adv_payload)

    def test_start_calls_gatts_set_buffer_with_append_mode(self):
        # Append mode is critical: without it, back-to-back writes
        # from the central overwrite each other in the GATTS layer
        # (e.g. Ctrl-C followed by Ctrl-A loses one of the two).
        # Pre-1.2.0 we never called gatts_set_buffer at all.
        _activate_ble()
        ble_repl.start()
        rx_handle = ble_repl._state["bridge"]._rx_handle
        size, append = _FakeBLE._gatts_buffer_settings[rx_handle]
        self.assertTrue(append,
                        "rx buffer must be in append mode so back-to-back "
                        "writes accumulate instead of overwriting")
        self.assertGreater(size, 0)

    def test_start_when_no_hub_name_raises(self):
        _FakeNVS._reset_for_test()  # clear name
        _activate_ble()
        with self.assertRaises(RuntimeError):
            ble_repl.start()

    def test_start_when_ble_inactive_raises(self):
        with self.assertRaises(RuntimeError):
            ble_repl.start()

    def test_start_is_idempotent(self):
        _activate_ble()
        ble_repl.start()
        first = ble_repl._state["bridge"]
        ble_repl.start()
        self.assertIs(ble_repl._state["bridge"], first)

    def test_stop_clears_dupterm_and_stops_advertising(self):
        _activate_ble()
        ble_repl.start()
        ble_repl.stop()

        self.assertFalse(ble_repl.is_running())
        self.assertIsNone(_FakeOsDupterm.installed_stream)
        self.assertIsNone(_FakeBLE._adv_payload)

    def test_stop_when_not_running_is_safe(self):
        ble_repl.stop()
        self.assertFalse(ble_repl.is_running())


class AdvertPayloadTests(unittest.TestCase):
    """Payload must carry the NUS 128-bit service UUID and the GAP
    name so ``openbricks list`` (UUID filter) and ``openbricks run
    -n NAME`` (name filter) both work."""

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


class UartRxTxTests(unittest.TestCase):
    """The vendored ``_BLEUART`` rx/tx pipe — appends incoming writes
    to its buffer (gated on the conn_handle being in
    ``_connections``), notifies all connected centrals on write."""

    def setUp(self):
        _FakeBLE._reset_for_test()
        _FakeNVS._reset_for_test()
        _FakeOsDupterm._reset_for_test()
        if ble_repl.is_running():
            ble_repl.stop()
        _set_hub_name("TestHub")
        _activate_ble()
        ble_repl.start()
        self.uart = ble_repl._state["bridge"]
        self.rx_handle = self.uart._rx_handle
        self.tx_handle = self.uart._tx_handle

    def test_central_write_fills_rx_buffer(self):
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"hello")
        self.assertEqual(self.uart.read(), b"hello")

    def test_unknown_conn_writes_are_ignored(self):
        # The bridge tracks connections in a set — a write from a
        # conn_handle that never fired CONNECT shouldn't reach the
        # rx buffer. ``_simulate_central_write`` auto-fires CONNECT
        # for new conns, so to test "unknown conn", we have to
        # bypass it and fire GATTS_WRITE directly.
        _FakeBLE._char_values[self.rx_handle] = b"sneak"
        _FakeBLE._fire_irq(3, (999, self.rx_handle))   # unknown conn=999
        self.assertEqual(self.uart.read(), b"")

    def test_multiple_writes_accumulate(self):
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"foo")
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"bar")
        self.assertEqual(self.uart.read(), b"foobar")

    def test_uart_write_notifies_all_connected(self):
        # Two centrals connected → write goes to both via gatts_notify.
        _FakeBLE._fire_irq(1, (42, 0, b"\x00" * 6))
        _FakeBLE._fire_irq(1, (43, 0, b"\x00" * 6))
        self.uart.write(b"hi")
        recipients = sorted(c for (c, _h, _d) in _FakeBLE._notify_log)
        self.assertEqual(recipients, [42, 43])

    def test_uart_write_without_connection_is_silent(self):
        self.uart.write(b"ignored")
        self.assertEqual(_FakeBLE._notify_log, [])

    def test_disconnect_resumes_advertising(self):
        _FakeBLE._fire_irq(1, (42, 0, b"\x00" * 6))
        _FakeBLE._adv_payload = None
        _FakeBLE._fire_irq(2, (42, 0, b"\x00" * 6))  # _IRQ_CENTRAL_DISCONNECT
        self.assertIsNotNone(_FakeBLE._adv_payload)


class StreamProtocolTests(unittest.TestCase):
    """The dupterm-facing ``_BLEUARTStream`` — io.IOBase subclass with
    the four methods MicroPython's stream protocol checks for."""

    def setUp(self):
        _FakeBLE._reset_for_test()
        _FakeNVS._reset_for_test()
        _FakeOsDupterm._reset_for_test()
        if ble_repl.is_running():
            ble_repl.stop()
        _set_hub_name("TestHub")
        _activate_ble()
        ble_repl.start()
        self.uart   = ble_repl._state["bridge"]
        self.stream = ble_repl._state["stream"]
        self.rx_handle = self.uart._rx_handle

    def test_stream_subclasses_io_iobase(self):
        # io.IOBase inheritance is what gives the type the C-level
        # stream-protocol slot. Without it, os.dupterm raises
        # ``OSError: stream operation not supported``.
        import io
        self.assertIsInstance(self.stream, io.IOBase)

    def test_readinto_returns_none_on_empty(self):
        # Returning ``b""`` here would be interpreted as EOF by
        # MicroPython's dupterm and permanently deactivate the
        # stream. Upstream returns None.
        buf = bytearray(8)
        self.assertIsNone(self.stream.readinto(buf))

    def test_readinto_drains_buffer(self):
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"abcdef")
        buf = bytearray(3)
        n = self.stream.readinto(buf)
        self.assertEqual(n, 3)
        self.assertEqual(bytes(buf), b"abc")
        # Remaining bytes still readable.
        buf2 = bytearray(8)
        n2 = self.stream.readinto(buf2)
        self.assertEqual(n2, 3)
        self.assertEqual(bytes(buf2[:n2]), b"def")

    def test_ioctl_reports_readable_when_buffered(self):
        # MP_STREAM_POLL = 3, MP_STREAM_POLL_RD = 0x01.
        self.assertEqual(self.stream.ioctl(3, None), 0)
        _FakeBLE._simulate_central_write(1, self.rx_handle, b"x")
        self.assertEqual(self.stream.ioctl(3, None), 0x01)

    def test_rx_handler_is_wired(self):
        # _BLEUARTStream.__init__ registers _on_rx with the uart, so
        # an rx event triggers os.dupterm_notify (when available).
        # Just check the handler is set + callable — bound methods
        # don't compare via ``is`` (each access creates a fresh
        # wrapper) and MicroPython unix port doesn't expose
        # ``__func__``.
        self.assertIsNotNone(self.uart._handler)
        self.assertTrue(callable(self.uart._handler))


class _RecordingUart:
    """Bare uart stand-in for ``_BLEUARTStream`` TX tests: records
    every write; can raise once to simulate a KeyboardInterrupt
    landing mid-flush."""

    def __init__(self):
        self.writes = []
        self.raise_once = None
        self._handler = None

    def irq(self, handler):
        self._handler = handler

    def write(self, data):
        if self.raise_once is not None:
            exc, self.raise_once = self.raise_once, None
            raise exc
        self.writes.append(bytes(data))

    def any(self):
        return 0

    def read(self, sz=None):
        return b""


class TxSchedulingTests(unittest.TestCase):
    """The TX flush path must survive the stop button.

    The stop button injects KeyboardInterrupt at an arbitrary bytecode
    of whatever the main thread runs — including the scheduled
    ``_flush`` callback itself. The old flag-based batching stranded
    ``_scheduled=True`` when the interrupt unwound ``_flush`` at entry,
    after which no write ever scheduled a flush again: the hub kept
    advertising and accepting connections but never sent a notify
    (the "cannot reconnect after button stop" bug)."""

    def setUp(self):
        self._orig_schedule = ble_repl._SCHEDULE
        self.queue = []
        self.fail_schedule_with = None
        this = self

        def fake_schedule(fn, arg):
            if this.fail_schedule_with is not None:
                raise this.fail_schedule_with
            this.queue.append((fn, arg))
        ble_repl._SCHEDULE = fake_schedule
        self.uart = _RecordingUart()
        self.stream = ble_repl._BLEUARTStream(self.uart)

    def tearDown(self):
        ble_repl._SCHEDULE = self._orig_schedule

    def _run_queued(self):
        while self.queue:
            fn, arg = self.queue.pop(0)
            fn(arg)

    def test_flush_lost_to_interrupt_does_not_wedge_tx(self):
        self.stream.write(b"hello")
        # Simulate the injected KeyboardInterrupt unwinding the queued
        # callback before it executed a single statement: the flush
        # simply never runs.
        del self.queue[:]
        self.stream.write(b" world")
        self.assertTrue(
            len(self.queue) >= 1,
            "write() after a lost flush must schedule a new one — "
            "the flag-based code wedged TX here")
        self._run_queued()
        self.assertEqual(b"".join(self.uart.writes), b"hello world")

    def test_interrupt_mid_flush_recovers(self):
        # KeyboardInterrupt inside the flush body (during the notify)
        # loses at most that one chunk; the next write must revive TX.
        self.stream.write(b"a" * 150)
        self.uart.raise_once = KeyboardInterrupt()
        try:
            self._run_queued()
        except KeyboardInterrupt:
            pass
        del self.queue[:]
        self.stream.write(b"tail")
        self._run_queued()
        joined = b"".join(self.uart.writes)
        self.assertTrue(joined.endswith(b"tail"),
                        "TX never recovered after mid-flush interrupt: %r"
                        % joined)

    def test_schedule_queue_full_is_tolerated(self):
        self.fail_schedule_with = RuntimeError("schedule queue full")
        self.stream.write(b"abc")   # must not raise out of print()
        self.assertEqual(self.uart.writes, [])
        self.fail_schedule_with = None
        self.stream.write(b"def")
        self._run_queued()
        self.assertEqual(b"".join(self.uart.writes), b"abcdef")

    def test_flush_chunks_at_100_bytes(self):
        self.stream.write(b"x" * 250)
        self._run_queued()
        self.assertEqual(b"".join(self.uart.writes), b"x" * 250)
        for chunk in self.uart.writes:
            self.assertTrue(len(chunk) <= 100)


class BluetoothIntegrationTests(unittest.TestCase):
    """``openbricks.bluetooth.set_enabled(True)`` should start the
    REPL bridge; ``set_enabled(False)`` should stop it."""

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
