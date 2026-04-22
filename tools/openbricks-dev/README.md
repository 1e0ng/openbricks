# openbricks-dev

Host-side CLI for openbricks hubs — the same UX as `pybricks-dev`, built on commodity Python tooling (`bleak`, `esptool`, `mpremote`). One tool for flashing, scanning, and (coming) running code over BLE.

## Install

```
pip install -e tools/openbricks-dev
```

This pulls in `bleak` (cross-platform BLE), `esptool`, and `mpremote`.

## Commands

### `flash` — program a hub

```
openbricks-dev flash \
    --name RobotA \
    --port /dev/tty.usbserial-XXXX \
    --firmware native/micropython/ports/esp32/build-openbricks_esp32s3/firmware.bin
```

Writes the firmware image and bakes the hub's BLE advertising name into NVS. `--name` is mandatory: two hubs with the same name can't be individually addressed over BLE.

Port syntax: `/dev/ttyUSB0` (Linux), `/dev/cu.usbserial-*` (macOS), `COM5` (Windows).

Useful flags: `--chip esp32s3`, `--baud 921600`, `--skip-erase` (faster dev loop, keeps stale NVS).

### `list` — scan for hubs

```
openbricks-dev list [--timeout 5.0] [--all]
```

Runs a BLE scan and prints every named device found, sorted by RSSI (strongest first). `--all` includes unnamed devices too.

### `run` — push a script and stream output

```
openbricks-dev run -n RobotA examples/hello.py
```

Connects to the named hub over BLE, uploads the script through MicroPython's raw-paste protocol (same one `mpremote` uses), and streams stdout/stderr back to your terminal while it executes. Ctrl-C interrupts the remote program. Transient — nothing is written to the hub's flash; for that use `download` (PR 4).

The script is not echoed back to the local terminal, and stderr (e.g. exception tracebacks) is surfaced separately after stdout completes.

### `stop` — interrupt the running program

```
openbricks-dev stop -n RobotA
```

Sends a single Ctrl-C byte over the NUS REPL bridge, which MicroPython surfaces as `KeyboardInterrupt`. Useful when `openbricks-dev run` has already exited but the hub's still chewing on a long-running user program.

### coming in PR 4

- `download -n NAME script.py` — write a script to the hub's flash; runs at boot.

## Tests

```
cd tools/openbricks-dev
PYTHONPATH=. python -m unittest discover -s tests -t .
```

No real hardware needed — `esptool`, `mpremote`, and `bleak` are mocked.
