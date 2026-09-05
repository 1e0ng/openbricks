---
myst:
  html_meta:
    description: "Install the openbricks host tools with pipx, download prebuilt firmware, and flash your ESP32 hub with a single command: openbricks flash --name."
---

# Installation

Two things get installed: the **host tooling** on your computer (a CLI
that flashes firmware and talks to hubs over BLE) and the **firmware**
on the hub itself.

## 1. Install the host tooling

```console
$ pipx install 'openbricks[sim]'   # CLI + MuJoCo simulator
```

or, lighter:

```console
$ pipx install openbricks          # CLI only (flash / run / log)
```

`pip install` works too; `pipx` is recommended on modern macOS / Linux
to avoid the "externally managed environment" error. The package is on
PyPI: <https://pypi.org/project/openbricks/>.

The `[sim]` extra adds `mujoco` (~50 MB, native OpenGL) and `numpy`.
If you only want to flash and run code on real hardware, skip it —
`openbricks sim …` will print an install hint instead of crashing.

## 2. Get a firmware image

Grab a prebuilt image from the
[Releases page](https://github.com/1e0ng/openbricks/releases):

- `openbricks-esp32s3-firmware-<version>.bin` — ESP32-S3 boards (primary target)
- `openbricks-esp32-firmware-<version>.bin` — classic ESP32 boards

(You can also {doc}`build the firmware from source <build>`.)

## 3. Flash the hub

Connect the board over USB, then flash and name the hub in one step:

```console
$ openbricks flash --name RobotA
```

That's the whole command: the serial port is auto-detected (when
exactly one ESP device is connected), the chip type is probed, and
the newest matching firmware release is downloaded automatically
(cached under `~/.cache/openbricks/firmware`).

- `--name` is the BLE advertising identifier you'll use later with
  `openbricks run -n …`; pick a unique one per hub.
- With several serial devices connected, pass `--port` explicitly
  (`/dev/ttyUSB0` on Linux, `/dev/cu.usbserial-*` on macOS, `COM5`
  on Windows).
- To flash a specific/downloaded image instead of the newest
  release, pass `--firmware path/to/firmware.bin`.

Skip this step entirely if you only want to run code in the
{doc}`simulator <simulator>`.

## 4. Run your first program

```console
$ openbricks list                  # find your hub over BLE
$ openbricks run -n RobotA main.py # push a script and stream its output
```

See {doc}`the CLI reference <cli>` for every command, and
{doc}`examples` for programs to start from.

## Troubleshooting

- **Hub doesn't show up in `openbricks list`** — BLE may be toggled
  off. Short-press the Bluetooth button (GPIO 38 on the S3, GPIO 5 on the classic ESP32); on the
  ESP32-S3 the onboard LED turns blue when BLE is on, yellow when off.
  While a program is running the LED flashes that colour at 5 Hz
  instead of holding it solid — a flashing LED means "robot running",
  not a different BLE state. While a host tool is sending to the hub
  (`openbricks upload`, or `run` staging its program) it flashes
  **purple** twice as fast, then hands back. A brief flash is the
  press acknowledgment, shown the moment a program-button press is
  recognized: **red** for the press that starts a run, **green**
  for the press that stops one.
  See {class}`openbricks.hub.ESP32S3DevkitHub`.
- **Serial port permission errors on Linux** — add yourself to the
  `dialout` group (`sudo usermod -aG dialout $USER`) and re-login.
- **Flash succeeds but the program doesn't start** — check the run log
  with `openbricks log -n RobotA` for the traceback of the last run.
