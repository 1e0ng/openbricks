# Architecture

A short tour of how `openbricks` is organized and why. If you've read Pybricks'
`pbio` codebase, a lot of this will look familiar — the layering is borrowed
directly, and for the same reason: openbricks ships as a **custom MicroPython
firmware**, not a library you install on top of stock MicroPython. Pybricks does
exactly this for LEGO hubs; we do it for commodity MCUs.

Owning the firmware shapes several decisions:

- Background control loops (`MotorProcess`) can run always-on off a hardware
  timer — nobody else is contending for that peripheral.
- Platform selection means picking which firmware image to flash, not
  runtime-dispatching between adapters.
- Hot control code can be compiled in as a native C extension later without
  a separate install step.
- We can extend or add `machine`-level primitives (custom timers, a hub
  abstraction) because we build the `machine` module.

## Four layers

```
┌─────────────────────────────────────────────────────────┐
│  User code     (main.py, robotics.DriveBase, …)          │
├─────────────────────────────────────────────────────────┤
│  Abstract interfaces   (Motor, Servo, IMU, ColorSensor)  │
├─────────────────────────────────────────────────────────┤
│  Concrete drivers      (l298n, jgb37_520, bno055, …)     │
├─────────────────────────────────────────────────────────┤
│  MicroPython HAL       (machine.Pin, I2C, UART, PWM)     │
├─────────────────────────────────────────────────────────┤
│  openbricks firmware image — custom MicroPython build    │
│  for this specific MCU, with all the above baked in      │
└─────────────────────────────────────────────────────────┘
```

The two middle layers are what make this different from "a pile of MicroPython
scripts." Interfaces (`openbricks/interfaces.py`) define the contract each
family of component obeys; drivers implement that contract; everything above
the interface line depends only on interfaces, not on specific chips. That's
why swapping an L298N motor for an ST-3215 servo only changes the driver
you instantiate — the `DriveBase` class asks for "a `Motor`" and doesn't
know or care what's underneath.

This is the same split Pybricks has: `pbio/include/pbio/*.h` is the interface,
`pbio/src/*.c` is the library, `pbio/drv/*` is the driver layer. We take the
same approach in C — `native/user_c_modules/openbricks/` holds the hot control
code that runs at the scheduler tick rate. Targeted pbio-parity on control
quality is the reason that code is C and not Python.

## Pybricks-parity control, in C

All four of the big pbio control-quality items are ported and shipped in
`native/user_c_modules/openbricks/`. Each corresponds to a pbio source
file and keeps its structure close — `pbio` is MIT-licensed so the ports
are direct where they can be.

1. **State observer** (pbio `observer.c`) — our `observer.c` is a
   two-state α-β filter. Less capable than pbio's full model-based
   observer (no motor model, no PWM coupling, no current/flux
   estimation) but a ~60× variance reduction over raw
   finite-differencing for little code. Upgrading to a model-based
   observer is later roadmap work.
2. **Trajectory planning** (pbio `trajectory.c`) — our `trajectory.c`
   computes trapezoidal (and triangular fall-through) speed profiles
   with explicit accel / cruise / decel phases. `servo.run_target()`
   and `DriveBase.straight()` / `.turn()` sample it each tick.
3. **Cooperative multitasking** (pbio `motor_process.c` + `os.c`) —
   our `motor_process.c`. Always-on 1 kHz tick off a `machine.Timer`
   ISR. Native subscribers (`Servo`, `DriveBase`) register via a fast
   C-function-pointer path (~1 µs/tick); Python callables are still
   accepted on a slower dispatch path for user extensibility.
4. **Drivebase coupling** (pbio `drivebase.c`) — our `drivebase.c`
   runs two coupled controllers in (sum, diff) coordinates with
   position feedback on both. Exit criterion: asymmetric-friction
   test (one wheel at 0.9× commanded speed) keeps heading error
   under 5% of forward distance — the pure-Kp M1 fallback fails it.

## What's next

M1 / M2 / M3 (scheduler, observer, trajectory, 2-DOF drivebase, all in C)
are landed. The remaining roadmap:

- **M4** — `hub` abstraction (status LED, user button) plus an SSD1306
  OLED driver (wired via I2C, used alongside the hub rather than through
  it). Already landed on `main`; ESP32 + ESP32-S3 firmware images ship as
  separate binaries from the same codebase, so the dual-platform build
  seam is already validated.
- **M5** — 1.0 polish + release with per-platform firmware images
  published through GitHub Releases (already wired up in
  `.github/workflows/ci.yaml` for the main branch).

Upgrading the α-β observer to a pbio-style model-based observer (with
voltage/current coupling and a motor model) is later work — a precision
lift we pick up once we can measure real hardware performance.
