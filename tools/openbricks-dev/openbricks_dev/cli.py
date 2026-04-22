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

    # ---- run ----
    p_run = sub.add_parser(
        "run",
        help="Push a Python script to a hub over BLE and stream output.",
        description="Connect to the named hub over BLE, push SCRIPT to its "
                    "REPL (via paste mode), and stream stdout/stderr back "
                    "to this terminal until the script finishes. Ctrl-C "
                    "interrupts the remote program.",
    )
    p_run.add_argument(
        "-n", "--name", required=True,
        help="Hub name baked in at flash time (``openbricks-dev flash --name``).",
    )
    p_run.add_argument(
        "script", metavar="SCRIPT",
        help="Path to the local Python script to run on the hub.",
    )
    p_run.add_argument(
        "--scan-timeout", type=float, default=5.0,
        help="How long to scan for the named hub before giving up. Default: 5.0 s.",
    )

    # ---- download ----
    p_download = sub.add_parser(
        "download",
        help="Persist a Python script to a hub so it runs at every boot.",
        description="Upload SCRIPT to the hub's filesystem (default path "
                    "``/main.py``). MicroPython runs it on every power-up. "
                    "A tiny boot-safety preamble is prepended so BLE + REPL "
                    "come up automatically — otherwise a crash in user code "
                    "would leave the hub unreachable.",
    )
    p_download.add_argument(
        "-n", "--name", required=True,
        help="Hub name baked in at flash time.",
    )
    p_download.add_argument(
        "script", metavar="SCRIPT",
        help="Path to the local Python script to persist.",
    )
    p_download.add_argument(
        "--path", default="/main.py",
        help="Destination path on the hub's filesystem. Default: /main.py.",
    )
    p_download.add_argument(
        "--no-reset", action="store_true",
        help="Don't reboot the hub after writing. Default is to reset so "
             "the new script runs immediately.",
    )
    p_download.add_argument(
        "--scan-timeout", type=float, default=5.0,
        help="BLE scan timeout. Default: 5.0 s.",
    )

    # ---- stop ----
    p_stop = sub.add_parser(
        "stop",
        help="Send Ctrl-C to a hub running a program.",
        description="Connect to the named hub over BLE and send a single "
                    "Ctrl-C, which MicroPython surfaces as KeyboardInterrupt. "
                    "Use when a long-running ``openbricks-dev run`` has "
                    "already ended and you just want the hub to idle again.",
    )
    p_stop.add_argument(
        "-n", "--name", required=True,
        help="Hub name.",
    )
    p_stop.add_argument(
        "--scan-timeout", type=float, default=5.0,
        help="BLE scan timeout. Default: 5.0 s.",
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
        if args.command == "run":
            from openbricks_dev import run as run_mod
            return run_mod.run(args)
        if args.command == "download":
            from openbricks_dev import download as download_mod
            return download_mod.run(args)
        if args.command == "stop":
            from openbricks_dev import stop as stop_mod
            return stop_mod.run(args)
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
