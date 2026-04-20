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
  abstraction, battery monitoring) because we build the `machine` module.

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
why swapping an L298N motor for an ST-3215 servo needs only a config change —
the `DriveBase` class asks for "a `Motor`" and doesn't know or care what's
underneath.

This is the same split Pybricks has: `pbio/include/pbio/*.h` is the interface,
`pbio/src/*.c` is the library, `pbio/drv/*` is the driver layer. We take the
same approach in C — `native/user_c_modules/openbricks/` holds the hot control
code that runs at the scheduler tick rate. Targeted pbio-parity on control
quality is the reason that code is C and not Python.

## Where Pybricks puts the real work (and where we are catching up)

Pybricks' `pbio` library does four things well that this project does only
minimally today. Each is a planned milestone:

1. **State observer** (`pbio/src/observer.c`) — a Kalman-flavored model of
   the motor that estimates speed, current, and torque from voltage
   commands and encoder readings. We currently just finite-difference the
   encoder, which is noisy.
2. **Trajectory planning** (`pbio/src/trajectory.c`) — trapezoidal speed
   profiles with explicit acceleration, cruise, and deceleration phases.
   We set a constant speed and stop when we're there.
3. **Cooperative multitasking** (`pbio/src/motor_process.c`, `os.c`) — a
   1 kHz motor tick scheduled off a timer interrupt, independent of the
   user program. **Landed for openbricks in M1** (`native/user_c_modules/openbricks/motor_process.c`):
   the scheduler runs always-on at 1 kHz via `machine.Timer` and dispatches
   tick callbacks via direct function-pointer calls for native subscribers
   (`Servo` today; observer / trajectory / drivebase in M2–M3). Python-callable
   subscribers are still supported on a slower path for user extensibility.
4. **Drivebase coupling** (`pbio/src/drivebase.c`) — the two wheels are
   controlled as one 2-DOF system, not two independent 1-DOF motors, which
   matters a lot for straight-line accuracy. We approximate this with a
   small correction term and it mostly works on flat floors.

The scheduler landed in C in M1 (the `_openbricks_native` user_c_module,
baked into the firmware image via `native/boards/openbricks_esp32/`); the
remaining three items follow the same model. `pbio` is MIT-licensed, so we
can port observer / trajectory / drivebase line-for-line with attribution —
no clean-room implementation needed.

## The configuration layer

The config layer (`openbricks/config.py`) turns a JSON description of the
robot into instantiated objects. The design goals:

- **Users edit one file.** Hardware changes shouldn't require code changes.
- **Drivers are plug-in.** A new driver is one file in `drivers/` plus one
  line in `_DRIVER_REGISTRY` (or a runtime `register_driver()` call from
  user code for out-of-tree drivers).
- **Buses are shared.** Multiple sensors on the same I2C bus should share
  one `machine.I2C` handle, not fight for the peripheral.
- **No magic.** The JSON maps 1:1 to Python kwargs. If a driver accepts
  `invert=True`, you write `"invert": true` in JSON. No schema DSL.

The registry pattern here mirrors Pybricks' `MP_REGISTER_MODULE` for
MicroPython modules: each driver self-registers with a human name, and the
loader looks it up by that name.

## What's next

- **M2:** port pbio's `observer.c` + `trajectory.c` to C. The existing
  native `Servo` gets upgraded to sample the trapezoidal profile and use
  the observer estimate instead of finite-differencing the encoder.
  Delivers items 1 and 2 above.
- **M3:** port pbio's `drivebase.c` — the 2-DOF coupled controller that
  keeps straight-line accuracy under asymmetric friction (item 4).
- **M4:** distance sensor and IR-remote interfaces (pure Python — they're
  not on the hot path).
- **M5:** `hub` abstraction (battery, LED, buttons) and a second firmware
  image (RP2040) to validate the build seam.
- **M6:** 1.0 polish + release with per-platform firmware images.
