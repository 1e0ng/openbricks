# Building the firmware

openbricks is a custom MicroPython firmware. Users flash the resulting image to their MCU; the openbricks Python package is frozen into the image, and the motor-control hot path lives in a compiled C extension.

This doc covers bringing up a build environment and producing an image. See `native/README.md` for the directory layout.

## One-time setup

### 1. Initialise the MicroPython submodule

```
git submodule update --init --recursive native/micropython
```

The submodule pins the MicroPython version openbricks is known to build against. Don't upgrade it without also testing the full suite on hardware.

### 2. Install the target toolchain

#### ESP32 (current target)

Install **ESP-IDF 5.x**. The simplest path is through Espressif's installer:

```
# macOS
mkdir -p ~/esp && cd ~/esp
git clone -b v5.2.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32
source export.sh          # sets $IDF_PATH; needs to happen in every shell
```

Verify:

```
echo $IDF_PATH       # should print the esp-idf path
idf.py --version     # should print ESP-IDF v5.x
```

#### RP2040

Planned for M5. Not supported yet.

## Building

From the repo root:

```
./scripts/build_firmware.sh esp32
```

The script:

1. Checks that `native/micropython` is populated.
2. Checks that `$IDF_PATH` is set (fails fast with a helpful message if not).
3. Builds `mpy-cross` once (cross-compiler for `.mpy` bytecode).
4. Builds the ESP32 firmware with `BOARD=openbricks_esp32` and `USER_C_MODULES=native/user_c_modules/openbricks/micropython.cmake`.

Output: `native/micropython/ports/esp32/build-openbricks_esp32/firmware.bin`.

## Flashing

Use `esptool.py` directly or the wrapper `idf.py` provides:

```
esptool.py --chip esp32 --port /dev/tty.usbserial-XXXX --baud 460800 \
    write_flash -z 0x1000 \
    native/micropython/ports/esp32/build-openbricks_esp32/firmware.bin
```

Or, if you built with `idf.py`:

```
idf.py -p /dev/tty.usbserial-XXXX flash monitor
```

## Verifying the image at the REPL

Connect over the USB UART (e.g. `mpremote connect /dev/tty.usbserial-XXXX`) and at the REPL:

```python
from openbricks._native import motor_process
motor_process.is_running()    # True — scheduler auto-started by first JGB37 motor
```

The first time any `JGB37Motor` is constructed and told to `run_speed`, it registers its control step and the scheduler comes alive. `motor_process.reset()` stops the timer and clears subscribers for debugging.

## Troubleshooting

**"IDF_PATH is not set"** — source the ESP-IDF `export.sh` in the shell that runs the build script. Each new shell needs it.

**Submodule is empty** — `git submodule update --init --recursive native/micropython`.

**`user_c_modules` not picked up** — check that `boards/openbricks_esp32/mpconfigboard.cmake` sets `USER_C_MODULES` to the path of `native/user_c_modules/openbricks/micropython.cmake`. The build script also passes it explicitly.

**Link error "undefined reference to `mp_register_module__openbricks_native`"** — the user_c_module isn't being built into the image. Re-check the CMake paths in `micropython.cmake` and that `boards/openbricks_esp32/mpconfigboard.cmake` wires it in.
