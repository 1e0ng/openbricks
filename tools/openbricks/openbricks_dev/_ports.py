# SPDX-License-Identifier: MIT
"""Shared USB-serial port autodetection.

Both ``flash`` (the hub's bridge) and ``servo-id`` (a URT-2-style
half-duplex adapter) talk to a USB serial device whose port name the
user shouldn't have to hunt down. The same vendor-ID filter serves
both: Espressif's native USB-Serial-JTAG plus the bridge chips
soldered onto essentially every commodity dev board and adapter.

Exactly one candidate is required. Both callers write to hardware
(flash is destructive; servo-id rewrites servo EEPROM), and with two
ports connected — say the hub AND the servo adapter — grabbing
whichever enumerated first is how the wrong device gets written.
Refusing to guess is the contract, not a limitation.
"""

USB_SERIAL_VIDS = {
    0x303A: "Espressif native USB",
    0x10C4: "CP210x bridge",
    0x1A86: "CH340/CH910x bridge",
    0x0403: "FTDI bridge",
}


def autodetect_port(err_cls, refusing_because):
    """The single connected USB-serial candidate's device path.

    Raises ``err_cls`` (the caller's own error type, so it surfaces
    through the caller's normal error path) when pyserial is missing,
    no candidate is connected, or more than one is —
    ``refusing_because`` names the caller's stake in not guessing.
    """
    try:
        from serial.tools import list_ports
    except ImportError:
        raise err_cls(
            "--port not given and pyserial is unavailable for "
            "auto-detection — pip install pyserial (or pass --port)")
    candidates = [p for p in list_ports.comports()
                  if p.vid in USB_SERIAL_VIDS]
    if not candidates:
        raise err_cls(
            "--port not given and no connected USB serial device "
            "found (looked for %s). Plug it in, or pass --port "
            "explicitly." % ", ".join(sorted(USB_SERIAL_VIDS.values())))
    if len(candidates) > 1:
        listing = "\n".join(
            "  %s  (%s)" % (c.device, USB_SERIAL_VIDS.get(c.vid, "?"))
            for c in candidates)
        raise err_cls(
            "--port not given and %d candidate ports are connected — "
            "%s, refusing to guess:\n%s"
            % (len(candidates), refusing_because, listing))
    port = candidates[0].device
    print("auto-detected port %s (%s)"
          % (port, USB_SERIAL_VIDS.get(candidates[0].vid, "?")))
    return port
