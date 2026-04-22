# SPDX-License-Identifier: MIT
"""
``openbricks-dev list`` — scan for BLE devices in range and print them.

Uses ``bleak`` so it works on macOS, Linux (BlueZ), and Windows without
platform-specific tooling. Output is sorted by RSSI so the closest /
strongest hub sits at the top — the one you're most likely trying to
flash code to.

We don't filter by service UUID yet: a hub that hasn't had its name
written (or is still in the pre-``apply_persisted_state`` boot window)
won't advertise with any openbricks-specific marker. Showing every
named device in range and letting the human pick is more forgiving than
a tight filter that silently hides the hub you just plugged in. Once
the BLE REPL lands (PR 3), ``list`` can grow a ``--openbricks-only``
flag that filters by the NUS service UUID.
"""

import asyncio


class ScanError(Exception):
    """Raised when the BLE scan itself fails (e.g. adapter not present)."""


async def _discover(timeout):
    """Run one BleakScanner pass. Kept out of ``run`` so tests can
    monkey-patch this function instead of the whole bleak import."""
    try:
        from bleak import BleakScanner
    except ImportError as e:
        raise ScanError(
            "bleak is not installed — run: pip install bleak"
        ) from e
    try:
        devices = await BleakScanner.discover(
            timeout=timeout,
            return_adv=True,
        )
    except Exception as e:
        raise ScanError("BLE scan failed: %s" % e) from e
    # bleak 0.22+ returns {address: (BLEDevice, AdvertisementData)}. On
    # older versions it returns a list of BLEDevice — handle both.
    if isinstance(devices, dict):
        return [(dev, adv) for dev, adv in devices.values()]
    return [(dev, None) for dev in devices]


def _format_row(dev, adv, show_all):
    name = dev.name or (adv.local_name if adv else None)
    if not name and not show_all:
        return None
    # RSSI can live on either the BLEDevice (pre-0.22) or the
    # AdvertisementData (0.22+). Prefer the newer location.
    rssi = getattr(adv, "rssi", None) if adv else None
    if rssi is None:
        rssi = getattr(dev, "rssi", None)
    rssi_str = ("%4d dBm" % rssi) if rssi is not None else "   ? dBm"
    name_str = name if name else "(no name)"
    return "%s  %-20s  %s" % (rssi_str, name_str, dev.address)


def run(args):
    """Subcommand entry. ``args`` is an argparse ``Namespace``."""
    print("scanning %.1f s ..." % args.timeout)
    try:
        devices = asyncio.run(_discover(args.timeout))
    except ScanError:
        raise
    except Exception as e:
        # asyncio.run wraps some errors; surface the original message.
        raise ScanError("scan failed: %s" % e) from e

    # Sort by RSSI strongest-first. Devices without RSSI sink to bottom.
    def _rssi_key(entry):
        dev, adv = entry
        rssi = getattr(adv, "rssi", None) if adv else None
        if rssi is None:
            rssi = getattr(dev, "rssi", None)
        # None → -inf so it sorts to the end under reverse=True.
        return rssi if rssi is not None else -10_000

    devices.sort(key=_rssi_key, reverse=True)

    print("  RSSI    NAME                  ADDRESS")
    print("  ------  --------------------  ---------------------------")
    shown = 0
    for dev, adv in devices:
        row = _format_row(dev, adv, args.all)
        if row is None:
            continue
        print("  " + row)
        shown += 1
    if shown == 0:
        if args.all:
            print("  (no BLE devices found)")
        else:
            print("  (no named BLE devices found; try --all)")
    return 0
