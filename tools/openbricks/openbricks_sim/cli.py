# SPDX-License-Identifier: MIT
"""
Top-level CLI for ``openbricks-sim``.

Subcommands:

  * ``preview`` — open a world + the default chassis in MuJoCo's
    viewer so the user can orbit the scene with the mouse.
  * ``run`` — execute a Python script with a pre-constructed
    :class:`SimRobot` exposed in its globals (``robot``). The script
    can drive ``robot.drivebase`` / ``robot.left`` / ``robot.right``
    and step the sim via ``robot.run_for`` / ``robot.run_until``.

The driver-shim monkey-patch (replacing ``openbricks.drivers.*`` so
firmware-targeting code runs unchanged) lands in Phase C3 — for now,
``run`` requires the script to import from ``openbricks_sim``.
"""

import argparse
import runpy
import sys

from openbricks_sim.chassis import ChassisSpec


_BUILTIN_WORLDS = {
    # Shorthand aliases for the worlds shipped in the repo. The
    # runtime can still take a full path to any MJCF.
    "empty":     None,  # <- generated standalone chassis preview
    "wro-2026-elementary": "worlds/wro_2026_elementary_robot_rockstars/world.xml",
    "wro-2026-junior":     "worlds/wro_2026_junior_heritage_heroes/world.xml",
    "wro-2026-senior":     "worlds/wro_2026_senior_mosaic_masters/world.xml",
    # Small practice scenes for learning / iteration. See
    # ``worlds/<name>/README.md`` for the layout + suggested missions.
    "practice-zones":      "worlds/practice_zones/world.xml",
    "practice-walls":      "worlds/practice_walls/world.xml",
}


def _resolve_world(arg: str):
    """Map an alias or path to an on-disk MJCF. None ⇒ standalone."""
    if arg in _BUILTIN_WORLDS:
        rel = _BUILTIN_WORLDS[arg]
        if rel is None:
            return None
        # Aliases are package-relative — the worlds directory ships
        # inside ``openbricks_sim/`` so the wheel bundles them, and
        # ``Path(__file__).parent`` resolves to the installed package
        # root regardless of how the user installed (pip, pipx,
        # editable, sdist-compile).
        from pathlib import Path
        pkg_dir = Path(__file__).resolve().parent
        candidate = pkg_dir / rel
        if candidate.is_file():
            return str(candidate)
        # Fallback: treat the arg as a plain path, let load_world error.
        return arg
    return arg


def _maybe_randomize(model, data, args):
    """Apply per-round randomization if the world has a spec.

    Skips silently for worlds without a spec (e.g. ``empty``,
    ``practice-*``, raw paths) — randomization is opt-in per
    world. Prints the chosen layout to stderr so the user knows
    what was permuted, mirroring the way a WRO judge announces
    each round's randomization at the table."""
    from openbricks_sim import randomization
    try:
        layout = randomization.randomize(
            model, data, world=args.world, seed=args.seed)
    except KeyError:
        return  # no spec — skip silently
    seed_str = "default" if args.seed is None else str(args.seed)
    print("[randomize] {} (seed={}):".format(args.world, seed_str),
          file=sys.stderr)
    for body, slot_label in sorted(layout.items()):
        print("  {} -> {}".format(body, slot_label), file=sys.stderr)


def cmd_preview(args):
    from openbricks_sim import chassis as chassis_mod
    from openbricks_sim.world import load_world

    spec = ChassisSpec(pos_x=args.x, pos_y=args.y)

    world_path = _resolve_world(args.world)
    if world_path is None:
        # Standalone chassis on a checker floor.
        xml = chassis_mod.standalone_mjcf(spec)
        import mujoco
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
    else:
        model, data, _ = load_world(world_path, chassis_spec=spec)

    _maybe_randomize(model, data, args)

    if args.headless:
        import mujoco
        steps = int(args.duration / model.opt.timestep)
        for _ in range(steps):
            mujoco.mj_step(model, data)
        print("headless preview: simulated %.2f s (%d steps)" %
              (data.time, steps))
        return 0

    # Interactive: ``mujoco.viewer.launch`` (blocking). The blocking
    # variant runs MuJoCo's own time loop on the calling thread, so it
    # doesn't need the main-thread handoff that ``launch_passive``
    # requires on macOS. Plain Python — no mjpython wrapper, works
    # cross-platform identically.
    try:
        import mujoco.viewer
    except ImportError:
        print("error: mujoco.viewer not available (install a newer mujoco "
              "wheel, or pass --headless)", file=sys.stderr)
        return 1

    mujoco.viewer.launch(model, data)
    return 0


def cmd_run(args):
    """Execute ``args.script`` with a SimRobot pre-built in its globals.

    By default the driver shim is installed, so a script that imports
    ``openbricks.drivers.*`` + ``machine`` runs unchanged from the
    firmware path. ``--no-shim`` skips the shim install — handy for
    scripts that want to drive ``robot`` / ``drivebase`` directly with
    the openbricks_sim API.
    """
    from openbricks_sim.robot import SimRobot
    from openbricks_sim import shim

    spec  = ChassisSpec(pos_x=args.x, pos_y=args.y)
    robot = SimRobot(world=args.world, chassis_spec=spec)
    _maybe_randomize(robot.model, robot.data, args)

    if not args.no_shim:
        shim.install(robot.runtime)

    init_globals = {
        "robot":   robot,
        # Convenience aliases so openbricks_sim-style scripts (and
        # ``--no-shim`` runs) can grab the common handles without
        # digging through SimRobot.
        "drivebase": robot.drivebase,
        "left":      robot.left,
        "right":     robot.right,
    }
    try:
        try:
            runpy.run_path(args.script, init_globals=init_globals,
                           run_name="__main__")
        except SystemExit as e:
            if args.viewer:
                # Even if the user script called sys.exit, hold the
                # viewer so the user can inspect the final state.
                robot.run_viewer()
            return e.code if isinstance(e.code, int) else 0
        if args.viewer:
            robot.run_viewer()
        return 0
    finally:
        if not args.no_shim:
            shim.uninstall()


def _build_parser():
    parser = argparse.ArgumentParser(
        prog="openbricks-sim",
        description="MuJoCo-backed simulator for openbricks firmware.",
    )
    sub = parser.add_subparsers(dest="command", metavar="COMMAND")
    sub.required = True

    p_preview = sub.add_parser(
        "preview",
        help="Load a world + the default chassis in MuJoCo's viewer.",
        description="Loads the named world (alias or path), splices the "
                    "default openbricks-sim chassis in, and hands off "
                    "to mujoco.viewer for interactive inspection. Pass "
                    "``--headless`` to step N seconds of physics without "
                    "opening a window (useful for CI smoke tests).",
    )
    p_preview.add_argument(
        "--world", default="empty",
        help="World alias or path. Aliases: empty, wro-2026-elementary, "
             "wro-2026-junior, wro-2026-senior. Default: empty.",
    )
    p_preview.add_argument("--x", type=float, default=0.0,
                           help="Chassis spawn x (m).")
    p_preview.add_argument("--y", type=float, default=0.0,
                           help="Chassis spawn y (m).")
    p_preview.add_argument("--headless", action="store_true",
                           help="Skip the viewer; step ``--duration`` "
                                "seconds and exit.")
    p_preview.add_argument("--duration", type=float, default=2.0,
                           help="Headless step duration in seconds. "
                                "Default: 2.0.")
    p_preview.add_argument("--seed", type=int, default=None,
                           help="Randomization seed for per-round prop "
                                "permutation. Set to a different integer "
                                "each round to mimic the WRO judge's "
                                "pre-round randomization; same seed "
                                "produces same layout. Has no effect on "
                                "worlds without a randomization spec.")

    p_run = sub.add_parser(
        "run",
        help="Execute a user script against the sim.",
        description="Loads the named world + the default chassis, "
                    "constructs a ``SimRobot`` over them, and execs the "
                    "script with ``robot`` (plus ``drivebase``, "
                    "``left``, ``right`` aliases) in its globals. The "
                    "script drives the sim by calling robot.run_for / "
                    "run_until between actions.",
    )
    p_run.add_argument("script",
                       help="Path to the Python script to execute.")
    p_run.add_argument("--world", default="empty",
                       help="World alias or path (same set as preview).")
    p_run.add_argument("--x", type=float, default=0.0,
                       help="Chassis spawn x (m).")
    p_run.add_argument("--y", type=float, default=0.0,
                       help="Chassis spawn y (m).")
    p_run.add_argument("--viewer", action="store_true",
                       help="Drop into the MuJoCo viewer after the "
                            "script returns so you can orbit the "
                            "final scene.")
    p_run.add_argument("--no-shim", action="store_true",
                       help="Skip installing the driver shim. The "
                            "default behaviour ``install``s shims for "
                            "``machine`` + ``openbricks._native`` and "
                            "patches ``time.sleep_ms`` to advance the "
                            "sim — disable when your script uses the "
                            "openbricks_sim API directly.")
    p_run.add_argument("--seed", type=int, default=None,
                       help="Randomization seed (same semantics as "
                            "``preview --seed``).")

    return parser


def main(argv=None):
    parser = _build_parser()
    args = parser.parse_args(argv)
    if args.command == "preview":
        return cmd_preview(args)
    if args.command == "run":
        return cmd_run(args)
    parser.error("unknown command: %r" % args.command)


if __name__ == "__main__":
    sys.exit(main())
