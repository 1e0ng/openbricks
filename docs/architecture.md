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

## Host tooling

Everything above describes what runs on the hub. There's a parallel
host-side surface — a single PyPI package called `openbricks` that
ships:

- A console CLI: `openbricks flash | list | run | upload | stop |
  log` for hub interaction over BLE / USB. See
  `tools/openbricks/openbricks_dev/`.
- A MuJoCo-backed simulator: `openbricks sim {preview, run}` opens a
  physics sim with the same C control cores as the firmware
  (`*_core.c` files compile into both targets, so the sim's hot-path
  math is byte-identical). Lives under `tools/openbricks/openbricks_sim/`.
  Optional via `pip install openbricks[sim]`.
- A driver shim that lets `from openbricks.drivers.jgb37_520 import
  JGB37Motor` (and BNO055 / TCS34725 / HC-SR04 / VL53L0X / VL53L1X)
  run unchanged in MuJoCo — `openbricks sim run main.py` installs
  no-op `machine` fakes and replaces the I2C driver classes with
  sim-aware versions.
- Per-run log capture on the hub: every program execution tee'd to
  `/openbricks_logs/run_N.log` (3 rotating slots, 64 KB each).
  `openbricks log -n NAME` reads them back over BLE — useful for
  untethered runs where no live console was attached.

The Python module names on the host are deliberately split
(`openbricks_dev` for the CLI, `openbricks_sim` for the sim) so they
don't shadow the firmware-side `openbricks` package, which is
sometimes imported on the host by the sim's driver shim.

## Status

All foundational milestones are landed. Roadmap items completed:

- **M1** — always-on 1 kHz scheduler in C (`motor_process.c`).
- **M2** — observer + trajectory + servo state machine, all in C.
- **M3** — 2-DOF coupled drivebase in C, with optional gyro-feedback
  (`use_gyro(True)`) for slip-immune heading via an attached IMU.
- **M4** — `hub` abstraction (status LED, user button) + SSD1306 OLED.
  ESP32 + ESP32-S3 firmware images both build from the same codebase.
- **M5** — per-platform firmware images auto-published on every
  push to `main` (rolling `latest`) and on `v*` tags (versioned).

Sim phases (host-side): A (chassis + worlds) → B (shared C cores) →
C (runtime + driver shim) → D (sensors + scenario reset / scoring)
all landed. Phase E1 — pixel-accurate colour-sensor texture
sampling — landed via CPU-side sampling: the sensor reads
``model.tex_data`` directly, computes UV from the geom-local hit
point, and indexes the texel. No offscreen GL context, no platform
divergence, works on macOS / Linux / Windows. Originally scoped as
"Linux EGL headless rendering" but the EGL machinery is only needed
for scenes with shadows / lighting / overlays over the textured
plane — the WRO use case is a flat printed mat where the texture IS
the answer.

Remaining in Phase E: broader worlds library, more example
walkthroughs. EGL offscreen rendering would unlock simulation of
scenes more complex than a printed mat (e.g. coloured 3D obstacles
that cast shadows onto the colour sensor's view); not yet
prioritised.

Upgrading the α-β observer to a pbio-style model-based observer
(voltage/current coupling + motor model) is on the longer-term list —
a precision lift we pick up once we have real hardware to measure
against.
