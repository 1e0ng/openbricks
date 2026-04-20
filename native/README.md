# native

Everything needed to build openbricks as a custom MicroPython firmware image.

```
native/
├── boards/
│   └── openbricks_esp32/       # ESP32 board: mpconfigboard.{cmake,h}, sdkconfig.board, manifest.py
├── user_c_modules/
│   └── openbricks/              # C extensions — the openbricks hot path
└── micropython/                 # git submodule, MicroPython source
```

**The MicroPython source is a git submodule.** After cloning openbricks, run:

```
git submodule update --init --recursive
```

See `docs/build.md` for ESP-IDF install + build steps. CI (GitHub Actions) builds the same image on every push and publishes it to [Releases](../../releases).

## What's here today

All pbio-parity control code, in C, baked into the firmware image:

- **`motor_process.c`** — 1 kHz scheduler singleton (`machine.Timer` ISR drives it; sibling C modules subscribe via a fast function-pointer path). Exposes `openbricks._native.motor_process`.
- **`trajectory.c`** — trapezoidal speed-profile sampler. Exposes `openbricks._native.TrapezoidalProfile`.
- **`observer.c`** — 2-state α-β position/velocity observer. Exposes `openbricks._native.Observer`.
- **`servo.c`** — per-motor state machine: observer feedback + trajectory setpoint + feedforward-P velocity loop. Exposes `openbricks._native.Servo`.
- **`drivebase.c`** — 2-DOF coupled (forward, heading) controller writing per-servo setpoints each tick. Exposes `openbricks._native.DriveBase`.
- **`openbricks_module.c`** — `MP_REGISTER_MODULE(_openbricks_native, …)` glue.

`boards/openbricks_esp32/` is an ESP32 devkit board with the whole `openbricks/` Python package frozen into the image.

## What's coming

Everything in [`docs/architecture.md`](../docs/architecture.md) from M4 onward: distance + IR-remote sensor drivers, hub abstraction (battery / LED / buttons), a second platform (RP2040), 1.0 polish + release.
