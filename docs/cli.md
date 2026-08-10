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
$ openbricks docs hardware           # open this manual offline in your browser
```

## Firmware versions and provenance

`openbricks flash` first reports the firmware already on the chip —
version plus an `(official)` / `(customized)` suffix — before it
looks up the newest release. Flashing the **same version** again, or
an **older** one, asks for confirmation first; pass `--yes` to skip
the prompt in scripts.

The default output is step-level (probe, download, erase, write,
hub name, marker, reboot); pass `--verbose` / `-v` to also echo
every underlying `mpremote` / `esptool` command line and the
firmware cache paths — useful when reporting a flash problem.

Every firmware image published by CI is signed (Ed25519), and the
CLI ships the matching public key. An image whose `.bin.sig`
verifies is labeled `(official)`; anything else — a self-built
image, a missing or wrong signature — is `(customized)`. Customized
firmware flashes normally: the suffix is provenance, not a gate.
After each flash the verdict is stored on the hub, which is how the
next `openbricks flash` labels the current firmware.

The suffix follows the version everywhere it reaches you: the
`firmware 1.79.0 (official)` banner at the top of every
`openbricks run`, the `started:` header line in every run log
(`openbricks log`), and the flash preflight above. On the hub,
`openbricks.firmware_label()` returns the same string.

## Reference

The reference below is generated from the CLI's own argument parser, so
it always matches the installed version.

```{eval-rst}
.. argparse::
   :module: openbricks_dev.cli
   :func: _build_parser
   :prog: openbricks
```
