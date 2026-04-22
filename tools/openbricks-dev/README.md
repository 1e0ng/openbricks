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

### coming in PR 3 / PR 4

- `run -n NAME script.py` — push a script to the hub and exec it transiently.
- `download -n NAME script.py` — write a script to the hub's flash; runs at boot.
- `stop -n NAME` — interrupt the running program.

## Tests

```
cd tools/openbricks-dev
PYTHONPATH=. python -m unittest discover -s tests -t .
```

No real hardware needed — `esptool`, `mpremote`, and `bleak` are mocked.
