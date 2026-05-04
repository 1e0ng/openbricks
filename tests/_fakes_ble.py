# SPDX-License-Identifier: MIT
"""
Optional ``esp32.NVS`` + ``bluetooth.BLE`` fakes for tests exercising
``openbricks.bluetooth`` and ``openbricks.ble_repl``.

Kept separate from ``tests/_fakes`` so modules that don't touch BLE /
NVS don't pay the memory cost on MicroPython's tight unix heap (same
pattern as ``tests/_fakes_pcnt.py`` / ``tests/_fakes_ssd1306.py``).
Import alongside ``tests._fakes`` only from the BLE-related tests.

The fakes are deliberately thin — just enough shape to let
``openbricks.bluetooth`` + ``openbricks.ble_repl`` run through their
happy paths and for tests to observe state (GAP name, registered
services, handles written by gatts_notify, etc.). Idempotence and
singleton semantics match the real MicroPython modules.
"""

import sys


# ---- esp32.NVS ----

class _FakeNVS:
    # One dict per namespace, shared across all _FakeNVS instances so
    # a fresh ``esp32.NVS("openbricks")`` sees previously-committed keys.
    _STORE = {}

    def __init__(self, namespace):
        self.namespace = namespace
        _FakeNVS._STORE.setdefault(namespace, {})

    def get_i32(self, key):
        ns = _FakeNVS._STORE[self.namespace]
        if key not in ns:
            raise OSError("NVS key not found: %r" % key)
        value = ns[key]
        if not isinstance(value, int):
            raise OSError("NVS key %r is not an int32" % key)
        return value

    def set_i32(self, key, value):
        _FakeNVS._STORE[self.namespace][key] = int(value)

    def get_blob(self, key, buffer):
        """Mirror MP's ``NVS.get_blob(key, buffer) -> length`` signature."""
        ns = _FakeNVS._STORE[self.namespace]
        if key not in ns:
            raise OSError("NVS key not found: %r" % key)
        value = ns[key]
        if not isinstance(value, (bytes, bytearray)):
            raise OSError("NVS key %r is not a blob" % key)
        n = len(value)
        if n > len(buffer):
            raise OSError("buffer too small for NVS blob %r" % key)
        buffer[:n] = value
        return n

    def set_blob(self, key, value):
        _FakeNVS._STORE[self.namespace][key] = bytes(value)

    def commit(self):
        pass

    @classmethod
    def _reset_for_test(cls):
        cls._STORE = {}


class _FakeEsp32Module:
    NVS = _FakeNVS


# ---- bluetooth.BLE ----

class _FakeUUID:
    """Minimal stand-in for ``bluetooth.UUID``; stores the string form."""

    def __init__(self, uuid_str):
        self.value = uuid_str

    def __bytes__(self):
        # Real ``bluetooth.UUID`` returns the little-endian byte layout
        # used over the wire; our ble_repl code calls ``bytes(uuid)``
        # for advertising payload composition.
        hex_ = self.value.replace("-", "")
        return bytes(reversed(bytes.fromhex(hex_)))

    def __repr__(self):
        return "UUID(%r)" % self.value


class _FakeBLE:
    # Singleton — real ``bluetooth.BLE()`` returns the same singleton
    # across calls, and tests want to observe state set via any instance.
    _INSTANCE = None
    _active = False
    _gap_name = None

    # gatts state: next handle we'll return, and the stored value per handle.
    _next_handle = 1
    _service_registrations = []   # list[tuple(service_uuid, [(char_uuid, flags), ...])]
    _registered_handles   = []    # list[tuple of tuples] — mirrors return of gatts_register_services
    _char_values          = {}    # handle → bytes

    _mtu = None
    _irq_handler = None

    # Advertising state.
    _adv_interval_us = None
    _adv_payload     = None

    # Notify history so tests can assert what was sent over BLE.
    _notify_log = []  # list[(conn_handle, value_handle, bytes)]

    # gatts_set_buffer settings: handle → (size, append).
    _gatts_buffer_settings = {}

    # gap_disconnect call log.
    _disconnect_log = []

    # Track which conn_handles have fired CENTRAL_CONNECT so test
    # helpers can simulate writes from "already-connected" centrals
    # without re-firing connect each time.
    _connected_centrals = set()

    def __new__(cls):
        if cls._INSTANCE is None:
            cls._INSTANCE = super().__new__(cls)
        return cls._INSTANCE

    # ---- activation ----

    def active(self, enable=None):
        if enable is None:
            return _FakeBLE._active
        _FakeBLE._active = bool(enable)
        return None

    # ---- config ----

    def config(self, *args, **kwargs):
        """Support the setter form ``config(key=value, ...)`` and the
        getter form ``config("key")`` that real MP BLE exposes.
        """
        if kwargs:
            if "gap_name" in kwargs:
                _FakeBLE._gap_name = kwargs["gap_name"]
            if "mtu" in kwargs:
                _FakeBLE._mtu = int(kwargs["mtu"])
            return None
        if args:
            key = args[0]
            if key == "gap_name":
                return _FakeBLE._gap_name
            if key == "mtu":
                return _FakeBLE._mtu
        return None

    # ---- IRQ ----

    def irq(self, handler):
        _FakeBLE._irq_handler = handler

    # ---- GATT server ----

    def gatts_register_services(self, services):
        """Real signature: ``services`` is a tuple of (uuid, chars) where
        ``chars`` is a tuple of (uuid, flags). Returns a tuple of tuples
        of value-handles — one outer tuple per service, one inner per
        characteristic.

        We hand out monotonic integer handles starting from 1 so tests
        can predict them.
        """
        all_handles = []
        for service in services:
            service_uuid, chars = service
            char_handles = []
            for char in chars:
                handle = _FakeBLE._next_handle
                _FakeBLE._next_handle += 1
                char_handles.append(handle)
                _FakeBLE._char_values[handle] = b""
            _FakeBLE._service_registrations.append((service_uuid, list(chars)))
            all_handles.append(tuple(char_handles))
        result = tuple(all_handles)
        _FakeBLE._registered_handles = list(all_handles)
        return result

    def gatts_read(self, handle):
        return _FakeBLE._char_values.get(handle, b"")

    def gatts_write(self, handle, data):
        _FakeBLE._char_values[handle] = bytes(data)

    def gatts_notify(self, conn_handle, value_handle, data):
        _FakeBLE._notify_log.append((conn_handle, value_handle, bytes(data)))

    def gatts_set_buffer(self, value_handle, size, append=False):
        # Real MP just sets internal RX buffer geometry. Fake records
        # so a regression test can pin "we asked for append mode".
        _FakeBLE._gatts_buffer_settings[value_handle] = (size, append)

    # ---- GAP ----

    def gap_advertise(self, interval_us, adv_data=None):
        _FakeBLE._adv_interval_us = interval_us
        _FakeBLE._adv_payload = None if adv_data is None else bytes(adv_data)

    def gap_disconnect(self, conn_handle):
        # Just record that we asked. Real MP terminates the BLE link.
        _FakeBLE._disconnect_log.append(conn_handle)

    # ---- test helpers ----

    @classmethod
    def _reset_for_test(cls):
        cls._INSTANCE = None
        cls._active = False
        cls._gap_name = None
        cls._next_handle = 1
        cls._service_registrations = []
        cls._registered_handles = []
        cls._char_values = {}
        cls._mtu = None
        cls._irq_handler = None
        cls._adv_interval_us = None
        cls._adv_payload = None
        cls._notify_log = []
        cls._gatts_buffer_settings = {}
        cls._disconnect_log = []
        cls._connected_centrals = set()

    # ---- helpers for ble_repl IRQ simulation ----

    @classmethod
    def _fire_irq(cls, event, data):
        """Test helper: call whichever handler ble_repl has registered."""
        if cls._irq_handler is not None:
            cls._irq_handler(event, data)

    @classmethod
    def _simulate_central_write(cls, conn_handle, value_handle, data):
        """Write ``data`` to the given char (as a BLE central would),
        then fire the GATTS_WRITE IRQ so the bridge picks it up.

        Auto-fires a CENTRAL_CONNECT IRQ first if this conn_handle
        hasn't been seen — the post-1.2.0 bridge tracks connections
        in a set and ignores writes from unknown peers, so tests
        that skipped the connect step would otherwise see no buffer
        updates.
        """
        if conn_handle not in cls._connected_centrals:
            cls._fire_irq(1, (conn_handle, 0, b"\x00" * 6))  # _IRQ_CENTRAL_CONNECT
            cls._connected_centrals.add(conn_handle)
        cls._char_values[value_handle] = bytes(data)
        cls._fire_irq(3, (conn_handle, value_handle))  # _IRQ_GATTS_WRITE = 3

    @classmethod
    def _simulate_central_disconnect(cls, conn_handle):
        cls._connected_centrals.discard(conn_handle)
        cls._fire_irq(2, (conn_handle, 0, b"\x00" * 6))  # _IRQ_CENTRAL_DISCONNECT


class _FakeBluetoothModule:
    BLE = _FakeBLE
    UUID = _FakeUUID
    # Real module exports integer flag bits; values match the MP docs.
    FLAG_READ             = 0x0002
    FLAG_WRITE            = 0x0008
    FLAG_NOTIFY           = 0x0010
    FLAG_WRITE_NO_RESPONSE = 0x0004


# ---- dupterm recording ----
#
# ``openbricks.ble_repl`` routes REPL I/O through ``os.dupterm`` in
# production. Neither CPython nor unix MicroPython exposes a usable
# ``os.dupterm`` (CPython lacks the symbol entirely; MP unix omits it
# from its ``os`` port). ble_repl indirects through its own
# ``_install_dupterm`` helper; tests swap that helper out for this
# recorder so we can assert installations without poking the real
# ``os`` module (which MP freezes and refuses ``setattr``).

class _FakeOsDupterm:
    installed_stream = None

    @classmethod
    def install(cls, stream):
        cls.installed_stream = stream

    @classmethod
    def _reset_for_test(cls):
        cls.installed_stream = None


def _install():
    """Install every fake into ``sys.modules`` (and ble_repl's dupterm
    hook). Idempotent."""
    if sys.modules.get("esp32") is not _FakeEsp32Module:
        sys.modules["esp32"] = _FakeEsp32Module
    sys.modules["bluetooth"] = _FakeBluetoothModule
    # Redirect the bridge's dupterm hook to our recorder. Do this lazily
    # — if ble_repl hasn't been imported yet, the fakes file is usually
    # imported first and ble_repl will find our hook when it loads.
    try:
        from openbricks import ble_repl
        ble_repl._install_dupterm = _FakeOsDupterm.install
    except ImportError:
        pass


_install()
