# SPDX-License-Identifier: MIT
"""Argument parsing + subcommand dispatch for ``openbricks``.

Subcommands mirror the pybricksdev workflow so the UX is familiar,
plus a ``sim`` passthrough that forwards to the MuJoCo-backed
simulator when the ``[sim]`` extra is installed:

    openbricks flash --name NAME --port PORT --firmware FW
    openbricks list [--timeout SEC]
    openbricks run    -n NAME SCRIPT
    openbricks upload -n NAME SCRIPT
    openbricks stop   -n NAME
    openbricks log    -n NAME [--list | --run N]
    openbricks sim …  (requires ``pip install openbricks[sim]``)

Python module name stays ``openbricks_dev`` to avoid colliding
with the firmware-side ``openbricks`` package on the hub, which is
also imported on the host by the sim's driver shim.
"""

import argparse
import sys

import asyncio

from openbricks_dev import __version__


def _build_parser():
    parser = argparse.ArgumentParser(
        prog="openbricks",
        description="Host-side CLI for flashing and running code on "
                    "openbricks hubs, plus a MuJoCo-backed simulator "
                    "(``openbricks sim …``).",
    )
    parser.add_argument(
        "--version", action="version",
        version="openbricks {}".format(__version__),
        help="Print the openbricks package version and exit.",
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
        "--port", default=None,
        help="Serial port (/dev/ttyUSB0, /dev/cu.usbserial-XXXX, COM5 "
             "...). Omit to auto-detect — works when exactly ONE ESP "
             "device is connected (Espressif native USB or a CP210x/"
             "CH340/FTDI bridge).",
    )
    p_flash.add_argument(
        "--firmware", default=None,
        help="Path to firmware.bin produced by scripts/build_firmware.sh "
             "or downloaded from the Releases page. Omit to download "
             "the newest release automatically for the detected chip "
             "(cached under ~/.cache/openbricks/firmware).",
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
        "--with-qtr-init", action="store_true",
        help="After flashing, store a starter QTR line-sensor "
             "calibration at /qtr.cal (recorded on the reference "
             "bench, default pins 1-10) so line-follow examples work "
             "out of the box. Re-run examples/qtr_calibrate.py for a "
             "calibration measured on your own mat and lighting.",
    )
    p_flash.add_argument(
        "--skip-erase", action="store_true",
        help="Skip erase_flash (faster dev loop; leaves stale NVS keys).",
    )
    p_flash.add_argument(
        "--yes", action="store_true",
        help="Skip the confirmation prompt when the target firmware "
             "is the same version as (or older than) the current one.",
    )
    p_flash.add_argument(
        "--verbose", "-v", action="store_true",
        help="Echo every subprocess command line (mpremote/esptool) "
             "and cache paths. Default output is step-level only.",
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
        help="Hub name baked in at flash time (``openbricks flash --name``).",
    )
    p_run.add_argument(
        "script", metavar="SCRIPT", nargs="?",
        help="Path to the local Python script to run on the hub. "
             "Mutually exclusive with -c.",
    )
    p_run.add_argument(
        "-c", "--code", metavar="CODE",
        dest="inline_code",
        # Long form is ``--code`` rather than ``--command`` because the
        # subparsers reserve ``args.command`` for the subcommand name
        # ("flash", "run", "list", ...). The short ``-c`` stays for
        # familiarity with ``python -c``.
        help="Inline Python code to run on the hub (analogous to "
             "``python -c CODE``). Useful for quick diagnostics — "
             "e.g. ``openbricks run -n ls -c 'import openbricks; "
             "print(openbricks.__version__)'``. Mutually exclusive "
             "with the SCRIPT positional.",
    )
    p_run.add_argument(
        "--scan-timeout", type=float, default=5.0,
        help="How long to scan for the named hub before giving up. Default: 5.0 s.",
    )
    p_run.add_argument(
        "--debug", action="store_true",
        help="Print every BLE notify packet (timestamp + hex + ascii) "
             "to stderr as it arrives. Use to diagnose 'timed out reading "
             "from hub' errors — tells you whether the hub is sending "
             "anything at all.",
    )

    # ---- upload ----
    p_upload = sub.add_parser(
        "upload",
        help="Stage a Python script on a hub; the user launches it with the hub button.",
        description="Upload SCRIPT to the hub's filesystem (default path "
                    "``/program.py``). The uploaded code does NOT run "
                    "automatically — the hub's frozen main.py watches "
                    "the hub button and exec's the staged script on each "
                    "short press. Second short-press stops a running "
                    "program. (Pybricks calls this same operation "
                    "``download`` from the hub's perspective; we name "
                    "by direction-of-data-travel — bytes flow *up* to "
                    "the hub.)",
    )
    p_upload.add_argument(
        "-n", "--name", required=True,
        help="Hub name baked in at flash time.",
    )
    p_upload.add_argument(
        "script", metavar="SCRIPT",
        help="Path to the local Python script to stage.",
    )
    p_upload.add_argument(
        "--path", default=None,
        help="Destination path on the hub's filesystem; the file is "
             "staged VERBATIM there (no compilation) for custom boot "
             "flows. Default: compile with mpy-cross and stage "
             "/program.mpy (which the frozen launcher runs; older "
             "firmware gets the source at /program.py).",
    )
    p_upload.add_argument(
        "--scan-timeout", type=float, default=5.0,
        help="BLE scan timeout. Default: 5.0 s.",
    )

    # ---- stop ----
    p_stop = sub.add_parser(
        "stop",
        help="Send Ctrl-C to a hub running a program.",
        description="Connect to the named hub over BLE and send a single "
                    "Ctrl-C, which MicroPython surfaces as KeyboardInterrupt. "
                    "Use when a long-running ``openbricks run`` has "
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

    # ---- log ----
    p_log = sub.add_parser(
        "log",
        help="Pull a script-run log file off a hub.",
        description="Every program executed via the launcher (button "
                    "press OR ``openbricks run``) gets its stdout / "
                    "stderr tee'd to a flash file under "
                    "``/openbricks_logs/``. Ten rotating slots are "
                    "kept. With no flags this prints "
                    "the most recent run; ``--list`` shows the index; "
                    "``--run N`` selects a specific slot. Useful for "
                    "post-mortem on an untethered run where no live "
                    "console was attached.",
    )
    p_log.add_argument(
        "-n", "--name", required=True,
        help="Hub name baked in at flash time.",
    )
    p_log.add_argument(
        "--list", action="store_true",
        help="List the available run indices + their on-flash size, "
             "instead of dumping a run's contents.",
    )
    p_log.add_argument(
        "--run", type=int, default=None,
        help="Specific run index to dump. Defaults to the most recent.",
    )
    p_log.add_argument(
        "--scan-timeout", type=float, default=5.0,
        help="BLE scan timeout. Default: 5.0 s.",
    )

    # ---- servo-id ----
    p_servo = sub.add_parser(
        "servo-id",
        help="Assign a Feetech SCS/STS servo's bus ID over a USB "
             "serial adapter.",
        description="Scans the bus (IDs 0..253), rewrites the servo's "
                    "EEPROM ID register, and verifies the result. "
                    "With several servos attached, --old-id is "
                    "required so the tool never guesses which one to "
                    "re-ID. Wire the servo to a USB half-duplex "
                    "adapter (e.g. the URT-2 board) — this talks "
                    "directly to the adapter's serial port, no hub "
                    "involved.",
    )
    p_servo.add_argument(
        "new_id", type=int, nargs="?", default=None,
        help="Bus ID to assign (0..253). Omit with --scan.",
    )
    p_servo.add_argument(
        "-p", "--port", default=None,
        help="Serial port of the USB adapter, e.g. "
             "/dev/cu.usbmodem123. Omitted (and no -n): "
             "auto-detected when exactly one USB serial device is "
             "connected.",
    )
    p_servo.add_argument(
        "-n", "--name", default=None,
        help="Hub name: run the scan/re-ID THROUGH THE HUB over BLE "
             "instead of a USB adapter — the servo stays wired to "
             "the robot. Mutually exclusive with -p.",
    )
    p_servo.add_argument(
        "--tx", type=int, default=14,
        help="Hub path only: servo-bus TX pin (default 14).",
    )
    p_servo.add_argument(
        "--rx", type=int, default=41,
        help="Hub path only: servo-bus RX pin (default 41).",
    )
    p_servo.add_argument(
        "--scan-timeout", type=float, default=5.0,
        help="Hub path only: BLE scan timeout. Default: 5.0 s.",
    )
    p_servo.add_argument(
        "--scan", action="store_true",
        help="Just list the IDs that answer on the bus; change "
             "nothing.",
    )
    p_servo.add_argument(
        "--old-id", type=int, default=None,
        help="Current ID of the servo to re-ID. Required when more "
             "than one servo is attached; otherwise auto-detected.",
    )
    p_servo.add_argument(
        "--baudrate", type=int, default=1_000_000,
        help="Bus baudrate. Default: 1000000 (Feetech factory).",
    )
    p_servo.add_argument(
        "--timeout", type=float, default=0.02,
        help="Per-ping serial read timeout in seconds. Default: 0.02 "
             "(a full 254-ID scan takes ~5 s).",
    )

    # ---- paste-probe (measure the hub's raw-paste burst limit) ----
    p_probe = sub.add_parser(
        "paste-probe",
        help="Measure the largest raw-paste burst this hub survives.",
        description="Pastes padded no-op programs of increasing size "
                    "through the real raw-paste path and reports the "
                    "largest that completes, plus how each failure "
                    "presents (truncated vs hung). Use before changing "
                    "the firmware's MICROPY_REPL_STDIN_BUFFER_MAX: two "
                    "windows may be in flight at once, so that setting "
                    "is only safe at or below the measured limit.",
    )
    p_probe.add_argument("-n", "--name", required=True,
                         help="Hub BLE name.")
    p_probe.add_argument("--scan-timeout", type=float, default=5.0,
                         help="BLE scan timeout in seconds. Default: 5.")
    p_probe.add_argument("--max", type=int, default=8192,
                         help="Largest size to try, bytes. Default: 8192.")
    p_probe.add_argument("--timeout", type=float, default=15.0,
                         help="Per-size timeout in seconds. Default: 15.")

    # ---- docs (offline documentation reader) ----
    p_docs = sub.add_parser(
        "docs",
        aliases=["doc"],
        help="Read the documentation offline (opens your browser).",
        description="Opens the full manual in your browser — the "
                    "same Sphinx build as docs.openbricks.dev, API "
                    "reference included, bundled and served from "
                    "disk so no internet is needed. Pass a topic to "
                    "jump straight to that page.",
    )
    p_docs.add_argument(
        "topic", nargs="?", default=None,
        help="Page to open (e.g. install, hardware, robotics). "
             "Guides and API pages both work. Omit for the index.",
    )

    # ---- sim (passthrough to openbricks_sim.cli) ----
    #
    # Argparse-wise this is a stub: the real grammar lives in
    # ``openbricks_sim.cli``. ``main()`` short-circuits before
    # ``parse_args`` runs when ``argv[0] == "sim"`` — see
    # ``_dispatch_sim``. Registered here only so it shows up in
    # ``openbricks --help``.
    sub.add_parser(
        "sim",
        help="Run a sim subcommand (preview, run; "
             "requires ``pip install openbricks[sim]``).",
        description="Forwards all remaining arguments to the "
                    "MuJoCo-backed simulator's CLI. Use ``openbricks "
                    "sim --help`` to see the sim's own subcommand list.",
        add_help=False,   # let openbricks_sim handle --help itself
    )

    return parser


def _dispatch_sim(remaining_argv):
    """``openbricks sim <args>`` → ``openbricks_sim.cli.main(args)``.

    We bypass argparse for this so the sim CLI's grammar lives in one
    place. If ``openbricks_sim`` isn't importable (the user installed
    ``openbricks`` without the ``[sim]`` extra), print a hint instead
    of an ImportError traceback.
    """
    try:
        from openbricks_sim.cli import main as sim_main
    except ImportError:
        print(
            "error: ``openbricks sim`` requires the simulator extra.\n"
            "       install it with:  pip install openbricks[sim]",
            file=sys.stderr)
        return 1
    return sim_main(remaining_argv)


def main(argv=None):
    """Entry point. ``argv`` defaults to ``sys.argv[1:]`` for tests."""
    if argv is None:
        argv = sys.argv[1:]

    # ``openbricks sim …`` short-circuits argparse and forwards the
    # remaining argv to openbricks_sim's CLI. See ``_dispatch_sim``.
    if argv and argv[0] == "sim":
        return _dispatch_sim(argv[1:])

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
        if args.command == "upload":
            from openbricks_dev import upload as upload_mod
            return upload_mod.run(args)
        if args.command == "stop":
            from openbricks_dev import stop as stop_mod
            return stop_mod.run(args)
        if args.command == "log":
            from openbricks_dev import log as log_mod
            return log_mod.run(args)
        if args.command == "servo-id":
            from openbricks_dev import servo_id as servo_id_mod
            return servo_id_mod.run(args)
        if args.command == "paste-probe":
            from openbricks_dev import pasteprobe
            return pasteprobe.run(args)
        if args.command in ("docs", "doc"):
            from openbricks_dev import docs as docs_mod
            return docs_mod.run(args)
    except KeyboardInterrupt:
        print("\naborted.", file=sys.stderr)
        return 130
    except asyncio.CancelledError:
        # The run command routes Ctrl-C through task cancellation
        # (raw KeyboardInterrupt inside bleak teardown crashes the
        # interpreter under a notification flood) — a cancellation
        # reaching here IS the user's Ctrl-C, already cleaned up.
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
