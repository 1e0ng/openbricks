# Building the firmware

openbricks is a custom MicroPython firmware. Users flash the resulting image to their MCU; the openbricks Python package is frozen into the image and the motor-control hot path (scheduler, trajectory, observer, servo, drivebase) lives in a compiled C extension.

See `native/README.md` for the directory layout.

## One-time setup

### 1. Initialise all submodules

```
git submodule update --init --recursive
```

The MicroPython source tracks a master commit under `native/micropython/`. Bumps should be deliberate and tested on real hardware — `motor_process.c` and the other native modules assume a specific MP ABI. The current pin is documented in `.gitmodules` / `native/micropython` HEAD.

### 2. Install the target toolchain

#### ESP32 / ESP32-S3 (current targets)

Install **ESP-IDF v5.5.4** (or newer in the 5.5 line). MicroPython master supports ESP-IDF 5.3 through 5.5; we recommend 5.5.4 because that's what CI builds against and what the maintainer tests on hardware.

```
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.4 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.5
cd esp-idf-v5.5 && ./install.sh esp32,esp32s3
source export.sh          # sets $IDF_PATH; needs to happen in every shell
```

Verify:

```
echo $IDF_PATH       # /Users/<you>/esp/esp-idf-v5.5
idf.py --version     # ESP-IDF v5.5.4
```

> **Note.** If you're running an older MicroPython pin (e.g. v1.28.0), you'll need ESP-IDF v5.2.x — v1.28 pre-dates the MP commits that add 5.4+ support. Check `git -C native/micropython log --oneline -1` first.

## Building

From the repo root:

```
./scripts/build_firmware.sh esp32      # original ESP32 (Xtensa LX6)
./scripts/build_firmware.sh esp32s3    # ESP32-S3  (Xtensa LX7, native USB)
```

The script checks `native/micropython` is populated and `$IDF_PATH` is set, then builds `mpy-cross` once and produces the image for the selected target with `BOARD=openbricks_<target>` and `USER_C_MODULES=$(pwd)/native/user_c_modules`.

Output tree: `native/micropython/ports/esp32/build-openbricks_<target>/`

| File | Purpose |
|---|---|
| `firmware.bin` | complete flash image (everything below combined) |
| `bootloader/bootloader.bin` | second-stage bootloader |
| `partition_table/partition-table.bin` | partition map |
| `micropython.bin` | application partition only |

## Flashing

Use `esptool.py`. The combined `firmware.bin` is all you need — **just pay attention to the offset, it differs by chip**. Always `erase_flash` first on a new chip so stale bytes from a prior flash attempt can't linger.

```
# Classic ESP32 — bootloader at 0x1000
esptool.py --chip esp32 --port /dev/tty.usbserial-XXXX erase_flash
esptool.py --chip esp32 --port /dev/tty.usbserial-XXXX --baud 460800 \
    write_flash -z 0x1000 openbricks-esp32-firmware.bin

# ESP32-S3 — bootloader at 0x0
esptool.py --chip esp32s3 --port /dev/tty.usbmodemXXXX erase_flash
esptool.py --chip esp32s3 --port /dev/tty.usbmodemXXXX --baud 460800 \
    write_flash -z 0x0 openbricks-esp32s3-firmware.bin
```

> **Common footgun.** Flashing the S3 `firmware.bin` at `0x1000` (classic-ESP32 offset) leaves `0x0..0xFFF` untouched; the S3 ROM boots from `0x0` and fails with `Invalid image block, can't boot`. Always match the offset to the chip.

Or, if you built with `idf.py`:

```
cd native/micropython/ports/esp32/build-openbricks_esp32
idf.py -p /dev/tty.usbserial-XXXX flash monitor
```

## Verifying the image at the REPL

Connect over the USB UART (e.g. `mpremote connect /dev/tty.usbserial-XXXX`) and at the REPL:

```python
from openbricks._native import motor_process, Servo, TrapezoidalProfile, Observer, DriveBase
motor_process.is_running()    # False initially; True once a motor attaches
```

The openbricks Python package is frozen into the image, so `import openbricks.*` works without copying any files. The first time a closed-loop motor calls `run_speed`, it registers its control step with the scheduler and the 1 kHz timer ISR comes online.

## Running tests

Tests exercise the real C module under the unix MicroPython binary — the same build as firmware, minus the ESP32 port:

```
make -C native/micropython/mpy-cross -j
make -C native/micropython/ports/unix \
    VARIANT=standard \
    USER_C_MODULES=$(pwd)/native/user_c_modules -j
./native/micropython/ports/unix/build-standard/micropython tests/run.py
```

The runner spawns one MP subprocess per test module for state isolation. Expected result: all 10 modules green, 85 tests.

## CI

GitHub Actions runs two jobs on every push / PR (see `.github/workflows/ci.yaml`):

- **`test`** — builds the unix MP binary with the `_openbricks_native` user_c_module and runs `tests/run.py`. No ESP-IDF needed.
- **`firmware`** — a matrix job (targets: `esp32`, `esp32s3`) that builds each image inside the `espressif/idf:v5.5.4` container, then uploads `firmware.bin` + bootloader + partition-table for each target as a workflow artifact.

Successful PRs produce a flashable image downloadable from the Actions run.

## Troubleshooting

**"IDF_PATH is not set"** — source `export.sh` in the shell that runs the build script. Each new shell needs it.

**Build fails at `modbluetooth_nimble.c`** — BLE is disabled in `boards/openbricks_esp32/mpconfigboard.h` + `sdkconfig.board`; if you're re-enabling it, also un-disable in the `mpconfigboard.cmake` `SDKCONFIG_DEFAULTS` list.

**Build fails at `network_wlan.c` with a `_Static_assert` about `WIFI_AUTH_MAX`** — ESP-IDF / MP version mismatch. Either bump ESP-IDF into the supported range (5.3–5.5 for current MP master) or bump the MP submodule.

**Submodule is empty** — `git submodule update --init --recursive`.

**Link error "undefined reference to `mp_register_module__openbricks_native`"** — the user_c_module isn't being built into the image. Check `./scripts/build_firmware.sh` is passing `USER_C_MODULES=$(pwd)/native/user_c_modules` to `idf.py`.
