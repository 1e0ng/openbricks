# SPDX-License-Identifier: MIT
"""
Persistent Bluetooth on/off state for the hub.

The firmware ships with BLE compiled in (so ``mpremote``-over-BLE
code transfer works). At runtime we let the user toggle the advertising
stack on and off; the choice is persisted in NVS so reboots don't lose
the state. Default (no key in NVS yet) is **on**.

Typical usage from ``main.py`` right after the firmware boots:

    from openbricks import bluetooth
    bluetooth.apply_persisted_state()

Then toggle programmatically (or from the hub's button — see
``openbricks.hub.ESP32DevkitHub`` once that integration lands):

    bluetooth.toggle()                # flip current state, persist, apply
    bluetooth.set_enabled(False)      # explicit off

``esp32.NVS`` and ``bluetooth.BLE`` are imported lazily so desktop
tests running under unix MicroPython don't need to have the firmware-
only modules present — they install fakes in ``tests/_fakes_ble.py``.
"""

_NAMESPACE = "openbricks"
_KEY       = "ble_enabled"


def is_enabled():
    """Return the persisted flag (``True`` if no value has ever been written)."""
    try:
        import esp32
        nvs = esp32.NVS(_NAMESPACE)
        return bool(nvs.get_i32(_KEY))
    except OSError:
        # Key not set yet — default to enabled.
        return True


def set_enabled(enabled):
    """Persist ``enabled`` and apply it to the BLE stack immediately."""
    import esp32
    import bluetooth
    nvs = esp32.NVS(_NAMESPACE)
    nvs.set_i32(_KEY, 1 if enabled else 0)
    nvs.commit()
    bluetooth.BLE().active(bool(enabled))


def toggle():
    """Flip the current state. Returns the new (post-toggle) value."""
    new_state = not is_enabled()
    set_enabled(new_state)
    return new_state


def apply_persisted_state():
    """Read the persisted flag and apply it to the BLE stack.

    Call this at boot (e.g. top of ``main.py``) so the BLE radio reflects
    whatever the user last chose before the reboot — or comes up enabled
    on the very first boot.
    """
    import bluetooth
    bluetooth.BLE().active(is_enabled())
