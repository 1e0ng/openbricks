# native

Everything needed to build openbricks as a custom MicroPython firmware image.

```
native/
├── boards/
│   ├── openbricks_esp32/        # ESP32 (Xtensa LX6) board definition
│   └── openbricks_esp32s3/      # ESP32-S3 (Xtensa LX7, native USB)
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

`boards/openbricks_esp32/` and `boards/openbricks_esp32s3/` each freeze the whole `openbricks/` Python package into a dedicated image. They share `native/user_c_modules/openbricks/` — the C module builds for both Xtensa cores unchanged.

## What's coming

M4 (hub abstraction — status LED, user button — plus the SSD1306 OLED driver) is on `main`. Remaining: M5 — 1.0 polish + release (version bump, CHANGELOG, docs audit).
