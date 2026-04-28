# openbricks (host CLI + sim)

Host-side tooling for openbricks hubs — same UX as `pybricksdev`, built on commodity Python tooling (`bleak`, `esptool`, `mpremote`). One package, one console script: `openbricks flash / list / run / upload / stop / log`. The MuJoCo-backed simulator is built in too — `openbricks sim …` opens the sim's CLI when the `[sim]` extra is installed.

## Install

Recommended — with [`pipx`](https://pipx.pypa.io/) so the CLI lands in an isolated venv and avoids the "externally managed environment" error on modern macOS / Linux distros:

```
pipx install openbricks            # CLI only (lightweight; bleak / esptool / mpremote)
pipx install 'openbricks[sim]'     # CLI + MuJoCo physics simulator
```

Plain `pip` works too:

```
pip install openbricks
pip install 'openbricks[sim]'
```

For development against a repo checkout (editable install):

```
pip install -e 'tools/openbricks[sim]'
```

`[sim]` adds `mujoco` (~50 MB, native OpenGL) and `numpy` — most users (flash + run + log) don't need it. Without `[sim]`, `openbricks sim …` prints a helpful "pip install openbricks[sim]" hint instead of crashing.

> **Note (0.10.x):** the package currently ships sdist-only. `pip install openbricks` compiles the bundled native extension (`openbricks_sim._native`) on first install, so a C compiler + Python headers are required (`gcc` / `clang` on Linux/macOS, MSVC on Windows). Manylinux wheels via cibuildwheel are a follow-up — when those land, fresh installs will be faster and toolchain-free.

## Commands

### `flash` — program a hub

```
openbricks flash \
    --name RobotA \
    --port /dev/tty.usbserial-XXXX \
    --firmware native/micropython/ports/esp32/build-openbricks_esp32s3/firmware.bin
```

Writes the firmware image and bakes the hub's BLE advertising name into NVS. `--name` is mandatory: two hubs with the same name can't be individually addressed over BLE.

Port syntax: `/dev/ttyUSB0` (Linux), `/dev/cu.usbserial-*` (macOS), `COM5` (Windows).

Useful flags: `--chip esp32s3`, `--baud 921600`, `--skip-erase` (faster dev loop, keeps stale NVS).

### `list` — scan for hubs

```
openbricks list [--timeout 5.0] [--all]
```

Runs a BLE scan and prints every named device found, sorted by RSSI (strongest first). `--all` includes unnamed devices too.

### `run` — stage and launch; button stops it; client exits on stop

```
openbricks run -n RobotA examples/hello.py
```

Stages the script to `/program.py` (same target as `upload`) and triggers the hub's launcher to execute it immediately. Output streams back to your terminal in real time.

- **Button stop.** Pressing the hub button while the program runs raises `KeyboardInterrupt` via the same launcher path `upload`+button uses. The client sees the clean "stopped by button press" line and exits.
- **Program completion.** When the program finishes (or raises), the client disconnects and exits — same as `pybricks-dev run`.
- **Script persists.** Because `run` stages to `/program.py`, the hub can re-run the last program via a button press without another upload. `upload` and `run` differ only in whether the client auto-launches after upload.

Stderr (e.g. exception tracebacks) arrives after stdout and is surfaced with a blank-line separator. No paste-mode `===` echo — raw-paste mode is clean.

### `stop` — interrupt the running program

```
openbricks stop -n RobotA
```

Sends a single Ctrl-C byte over the NUS REPL bridge, which MicroPython surfaces as `KeyboardInterrupt`. Useful when `openbricks run` has already exited but the hub's still chewing on a long-running user program.

### `upload` — stage a script; hub button launches it

```
openbricks upload -n RobotA examples/wander.py
```

Writes the script to `/program.py` on the hub. **The code does not run automatically.** Place your robot, press the **program button** (GPIO 4), and the program starts. Press again to stop it mid-run.

(Pybricks calls this same operation `download` from the hub's "download to me" perspective. We name by direction-of-data-travel — bytes flow *up* to the hub, hence `upload`.)

This works because the firmware ships a frozen `main.py` that:

1. Brings BLE + REPL bridge up immediately (so `openbricks run` / `upload` / `stop` are always reachable, even when no program is running).
2. Instantiates the board's Hub, which wires the **BLE-toggle button** (short-press on GPIO 5) with LED feedback.
3. Watches the **program button** (GPIO 4) via `openbricks.launcher.run()` — a short-press runs `/program.py`, a second short-press raises `KeyboardInterrupt` in the running program.

Two separate pins (4 for program, 5 for BLE toggle), each handled by short-press only — no duration-based dispatch.

Pass `--path /alt.py` to stage at a different filename (if you've written your own `main.py` that reads from there).

## Tests

```
cd tools/openbricks-dev
PYTHONPATH=. python -m unittest discover -s tests -t .
```

No real hardware needed — `esptool`, `mpremote`, and `bleak` are mocked.
