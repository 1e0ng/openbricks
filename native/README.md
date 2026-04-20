# native

Everything needed to build openbricks as a custom MicroPython firmware image.

```
native/
├── boards/
│   └── openbricks_esp32/       # ESP32 board definition + frozen manifest
├── user_c_modules/
│   └── openbricks/              # C extensions (motor scheduler, later: observer / trajectory)
└── micropython/                 # git submodule — pinned MicroPython source (not checked in)
```

**The MicroPython source is a git submodule.** After cloning openbricks, run:

```
git submodule update --init --recursive native/micropython
```

See `docs/build.md` for how to install the ESP-IDF toolchain and build the image with `scripts/build_firmware.sh`.

## What's here today

- `user_c_modules/openbricks/motor_process.c` — the motor scheduler. A `machine.Timer`-driven periodic tick that iterates a Python callback list. Exposes `openbricks._native.motor_process` with `register` / `unregister` / `start` / `stop` / `tick` / `is_running` / `configure` / `reset`.
- `boards/openbricks_esp32/` — ESP32 devkit board definition with the openbricks Python package frozen into the image.

## What's coming

Everything in [`docs/architecture.md`](../docs/architecture.md) under M2 and beyond: observer, trajectory planner, per-motor servo state machine, hub primitives, second platform (RP2040).
