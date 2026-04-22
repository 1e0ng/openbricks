# SPDX-License-Identifier: MIT
"""Argument parsing + subcommand dispatch for ``openbricks-dev``.

Subcommands mirror the pybricks-dev workflow so the UX is familiar:

    openbricks-dev flash --name NAME --port PORT --firmware FW
    openbricks-dev list [--timeout SEC]
    openbricks-dev run      -n NAME SCRIPT  (PR 3)
    openbricks-dev download -n NAME SCRIPT  (PR 4)
    openbricks-dev stop     -n NAME         (PR 3)

Each subcommand's real work lives in its own module so tests can import
and call it directly without going through argparse.
"""

import argparse
import sys


def _build_parser():
    parser = argparse.ArgumentParser(
        prog="openbricks-dev",
        description="Host-side CLI for flashing and running code on "
                    "openbricks hubs.",
    )
    sub = parser.add_subparsers(dest="command", metavar="COMMAND")
    sub.required = True

    # ---- flash ----
    p_flash = sub.add_parser(
        "flash",
        help="Flash firmware onto a hub and bake in its BLE name.",
        description="Flash a firmware image onto a hub (via esptool) and "
                    "write the hub's BLE advertising name into NVS (via "
                    "mpremote). --name is mandatory so every hub gets a "
                    "unique identifier — two hubs with the same name "
                    "can't be individually addressed over BLE.",
    )
    p_flash.add_argument(
        "--name", required=True,
        help="Hub identifier for BLE (required, <=20 chars recommended).",
    )
    p_flash.add_argument(
        "--port", required=True,
        help="Serial port (/dev/ttyUSB0, /dev/cu.usbserial-*, COM5 ...).",
    )
    p_flash.add_argument(
        "--firmware", required=True,
        help="Path to firmware.bin produced by scripts/build_firmware.sh.",
    )
    p_flash.add_argument(
        "--chip", default="auto",
        help="esptool --chip value (esp32, esp32s3, auto). Default: auto.",
    )
    p_flash.add_argument(
        "--baud", default="460800",
        help="esptool flash baud rate. Default: 460800.",
    )
    p_flash.add_argument(
        "--skip-erase", action="store_true",
        help="Skip erase_flash (faster dev loop; leaves stale NVS keys).",
    )

    # ---- list ----
    p_list = sub.add_parser(
        "list",
        help="Scan for openbricks hubs in BLE range.",
        description="Run a BLE scan and print every device found, sorted "
                    "by RSSI (strongest first). Unnamed devices are shown "
                    "with a placeholder so you can still spot a hub whose "
                    "name wasn't flashed.",
    )
    p_list.add_argument(
        "--timeout", type=float, default=5.0,
        help="Scan duration in seconds. Default: 5.0.",
    )
    p_list.add_argument(
        "--all", action="store_true",
        help="Show every BLE device, not just those with names. Useful "
             "when debugging a hub that came up without a flashed name.",
    )

    return parser


def main(argv=None):
    """Entry point. ``argv`` defaults to ``sys.argv[1:]`` for tests."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    try:
        if args.command == "flash":
            from openbricks_dev import flash
            return flash.run(args)
        if args.command == "list":
            from openbricks_dev import scan
            return scan.run(args)
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    except Exception as e:
        # Subcommand modules raise their own typed errors (FlashError,
        # ScanError); we uniformly surface them as "error: <msg>\n" and
        # exit non-zero, matching CLI convention.
        print("error: %s" % e, file=sys.stderr)
        return 1
    # argparse `required=True` guarantees we never land here.
    parser.error("unknown command: %r" % args.command)
