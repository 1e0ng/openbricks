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
        "--path", default="/program.py",
        help="Destination path on the hub's filesystem. Default: "
             "/program.py (which the frozen launcher reads).",
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

    # ---- log ----
    p_log = sub.add_parser(
        "log",
        help="Pull a script-run log file off a hub.",
        description="Every program executed via the launcher (button "
                    "press OR ``openbricks-dev run``) gets its stdout / "
                    "stderr tee'd to a flash file under "
                    "``/openbricks_logs/``. Three rotating slots are "
                    "kept (run_0..run_2). With no flags this prints "
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
