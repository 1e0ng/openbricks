# Simulator

The `[sim]` extra ships a MuJoCo-backed physics simulator, so you can
develop robot programs without a hub on the desk:

```console
$ pipx install 'openbricks[sim]'
$ openbricks sim run examples/full_robot.py --viewer
```

The sim runs the **same script you'd push to the hub** — a driver shim
maps the `openbricks` API onto simulated motors and sensors, so
`ST3032Motor`, `DriveBase`, color sensors, and distance sensors behave
like their hardware counterparts.

## Commands

```console
$ openbricks sim preview [--world WORLD] [--x X] [--y Y] [--headless] [--duration S] [--seed N]
```

Loads the named world (an alias or a path to an MJCF file), splices in
the default chassis, and opens the MuJoCo viewer so you can inspect the
scene. `--headless` steps the physics for `--duration` seconds without
opening a window — useful as a smoke test.

```console
$ openbricks sim run SCRIPT [--world WORLD] [--x X] [--y Y] [--viewer] [--no-shim] [--seed N]
```

Loads the world plus the default chassis and executes `SCRIPT` against
the simulated robot. `--viewer` opens the interactive MuJoCo window;
without it the sim runs headless (CI-friendly). `--seed` makes
randomized worlds reproducible.

Run `openbricks sim --help` for the full, always-current option list.

## Notes

- The sim needs the `[sim]` extra (`mujoco`, `numpy`). Without it,
  `openbricks sim …` prints an install hint instead of crashing.
- Firmware-only users never need the simulator — it's strictly
  host-side tooling.
