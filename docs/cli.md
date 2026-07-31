---
myst:
  html_meta:
    description: "The openbricks command-line tool: flash firmware, list hubs over BLE, run and upload programs, stop them, and pull logs — with port and firmware auto-detection."
---

# Command-line tool

`pipx install openbricks` installs one console script, `openbricks`,
which mirrors the `pybricksdev` workflow: flash firmware over USB, then
run / upload / stop programs and pull logs over BLE. With the `[sim]`
extra installed, `openbricks sim …` forwards to the
{doc}`MuJoCo-backed simulator <simulator>`.

A typical session:

```console
$ openbricks flash --name RobotA     # port, chip and newest firmware auto-detected
$ openbricks list                    # hubs in BLE range
$ openbricks run -n RobotA main.py   # push + stream output
$ openbricks upload -n RobotA main.py  # stage; start it with the hub button
$ openbricks stop -n RobotA          # Ctrl-C a running program
$ openbricks log -n RobotA           # dump the most recent run log
$ openbricks docs hardware           # read this manual's guides offline
```

## Reference

The reference below is generated from the CLI's own argument parser, so
it always matches the installed version.

```{eval-rst}
.. argparse::
   :module: openbricks_dev.cli
   :func: _build_parser
   :prog: openbricks
```
