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

## Programs are compiled on the host

Since 1.92.0, `openbricks run` and `openbricks upload` cross-compile
your script with `mpy-cross` **before** connecting, and stage
compiled bytecode instead of source (like Pybricks). Three things
get better:

- **syntax errors surface in milliseconds**, on your terminal,
  naming your file and line and quoting the offending source line —
  no BLE scan, no connect, no upload round-trip;
- **programs start faster**: the hub loads bytecode directly and
  skips its on-device parse/compile step;
- **tracebacks name your real file and line** (`File "square.py",
  line 12`) instead of `File "<string>"`.

No flags, nothing to configure. Firmware older than 1.92.0 can't run
compiled programs, so the CLI probes the hub's version in-session and
sends plain source instead — announced on stderr, never silently.
`upload --path` (custom boot flows) always stages your file verbatim,
uncompiled, at the path you give.

`run` is an **upload-then-run** (since 2.7.0, deliberately
different from Pybricks): it stages your script at the button's
`/program.mpy` before executing it, so even a run that fails midway
leaves the program on the hub — press the start button to rerun it,
no BLE round trip needed. The flip side: running a calibration or a
one-shot diagnostic replaces the button's program too, so re-upload
your mission after such tools (`upload` alone stages without
running).

`flash --with-qtr-init` additionally stores a starter QTR
line-sensor calibration at `/qtr.cal` (recorded on the reference
bench, default pins 1-10), so the line-follow examples work on a
fresh hub out of the box. Heights, mats and lighting differ — run
`examples/qtr_calibrate.py` once for a calibration measured on your
own rig.

## One upload at a time

`openbricks run` and `openbricks upload` push a program through the
same raw-paste channel, and the operating system shares a single BLE
link between every process that connects to the same hub (macOS,
Linux and Windows all multiplex). Two transfers started from two
terminals used to interleave their bytes on the hub's REPL — a
corrupted program, or both terminals reading each other's output —
and the hub, which sees one connection, could not tell them apart.
Since 3.10.0 the host refuses at once, before any scan:

```
$ openbricks run -n RobotA main.py
error: an upload is ongoing (another openbricks run/upload is transferring to 'RobotA'; wait for it to finish)
```

The guard is a per-hub OS file lock held for the transfer: `upload`
holds it until its confirmation returns, `run` releases it the moment
the program is staged and started, so a second `run` while the first
is only streaming output supersedes it the way a button press would.
A CLI that crashes or is killed mid-transfer leaves nothing behind —
the kernel releases the lock with the process. Other hubs are
unaffected, and so is `openbricks stop`.

## Reference

The reference below is generated from the CLI's own argument parser, so
it always matches the installed version.

```{eval-rst}
.. argparse::
   :module: openbricks_dev.cli
   :func: _build_parser
   :prog: openbricks
```

`openbricks sim …` forwards to the simulator's own parser:

```{eval-rst}
.. argparse::
   :module: openbricks_sim.cli
   :func: _build_parser
   :prog: openbricks sim
```
