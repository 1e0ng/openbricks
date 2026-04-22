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

Activating BLE requires a hub name (``openbricks.HUB_NAME``) — flash
the image with ``scripts/flash_firmware.py --name NAME`` first. We
refuse to advertise with no name rather than defaulting to a shared
value, since two hubs with the same advertising name can't be
individually addressed.

``esp32.NVS`` and ``bluetooth.BLE`` are imported lazily so desktop
tests running under unix MicroPython don't need to have the firmware-
only modules present — they install fakes in ``tests/_fakes_ble.py``.
"""

_NAMESPACE = "openbricks"
_KEY       = "ble_enabled"


class HubNameNotSetError(RuntimeError):
    """Raised when the firmware was flashed without ``--name NAME``."""


def _require_hub_name():
    import openbricks
    name = openbricks.HUB_NAME
    if name is None:
        raise HubNameNotSetError(
            "hub name not set — reflash with "
            "scripts/flash_firmware.py --name NAME --port /dev/ttyUSB0"
        )
    return name


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
    """Persist ``enabled`` and apply it to the BLE stack immediately.

    Raises :class:`HubNameNotSetError` when turning BLE on with no hub
    name flashed. Turning BLE off is always allowed — a nameless hub
    can still be silenced.

    When enabling, also starts the NUS REPL bridge (``openbricks.ble_repl``)
    so ``openbricks-dev run`` / ``stop`` can push scripts over BLE. When
    disabling, the bridge is torn down first so dupterm isn't left
    pointing at an inactive stack.
    """
    import esp32
    import bluetooth
    from openbricks import ble_repl
    if enabled:
        name = _require_hub_name()
    nvs = esp32.NVS(_NAMESPACE)
    nvs.set_i32(_KEY, 1 if enabled else 0)
    nvs.commit()
    ble = bluetooth.BLE()
    if not enabled:
        # Tear down the REPL bridge before we kill the radio so dupterm
        # doesn't end up writing into a dead stack.
        ble_repl.stop()
    if enabled:
        # gap_name must be set before active(True); config() raises if
        # the stack is already active.
        ble.config(gap_name=name)
    ble.active(bool(enabled))
    if enabled:
        # Start the REPL bridge *after* active(True) — gatts_register_services
        # errors otherwise.
        ble_repl.start()


def toggle():
    """Flip the current state. Returns the new (post-toggle) value."""
    new_state = not is_enabled()
    set_enabled(new_state)
    return new_state


def apply_persisted_state():
    """Read the persisted flag and apply it to the BLE stack.

    Call this at boot (e.g. top of ``main.py``) so the BLE radio reflects
    whatever the user last chose before the reboot — or comes up enabled
    on the very first boot. Raises :class:`HubNameNotSetError` when the
    persisted state is enabled but no hub name was flashed.
    """
    import bluetooth
    from openbricks import ble_repl
    enabled = is_enabled()
    if enabled:
        name = _require_hub_name()
    ble = bluetooth.BLE()
    if not enabled:
        ble_repl.stop()
    if enabled:
        ble.config(gap_name=name)
    ble.active(enabled)
    if enabled:
        ble_repl.start()
