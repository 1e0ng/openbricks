# SPDX-License-Identifier: MIT
"""
Optional ``esp32.NVS`` + ``bluetooth.BLE`` fakes for tests exercising
``openbricks.bluetooth``.

Kept separate from ``tests/_fakes`` so modules that don't touch BLE /
NVS don't pay the memory cost on MicroPython's tight unix heap (same
pattern as ``tests/_fakes_pcnt.py`` / ``tests/_fakes_ssd1306.py``).
Import alongside ``tests._fakes`` only from the BLE-related tests.

Both fakes are in-memory: ``NVS`` stores int values per (namespace, key),
``BLE`` records whether ``active(True|False)`` has been called most
recently. Tests can reset state via the module-level helpers.
"""

import sys


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


class _FakeBLE:
    # Singleton — real ``bluetooth.BLE()`` returns the same singleton
    # across calls, and tests want to see the active state set via any
    # instance.
    _INSTANCE = None
    _active   = False
    _gap_name = None

    def __new__(cls):
        if cls._INSTANCE is None:
            cls._INSTANCE = super().__new__(cls)
        return cls._INSTANCE

    def active(self, enable=None):
        if enable is None:
            return _FakeBLE._active
        _FakeBLE._active = bool(enable)
        return None

    def config(self, **kwargs):
        """Mirror MP's ``BLE.config(gap_name=...)`` setter + getter."""
        if "gap_name" in kwargs:
            _FakeBLE._gap_name = kwargs["gap_name"]
            return None
        # Treat positional single-string arg as a getter key (MP's real
        # API supports this too, but our tests don't exercise it).
        return None

    @classmethod
    def _reset_for_test(cls):
        cls._INSTANCE = None
        cls._active   = False
        cls._gap_name = None


class _FakeBluetoothModule:
    BLE = _FakeBLE


def _install():
    """Install both fakes into ``sys.modules``. Idempotent."""
    if sys.modules.get("esp32") is not _FakeEsp32Module:
        sys.modules["esp32"] = _FakeEsp32Module
    sys.modules["bluetooth"] = _FakeBluetoothModule


_install()
