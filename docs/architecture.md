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
`pbio/src/*.c` is the library, `pbio/drv/*` is the driver layer. We collapse
the library layer into the interface layer for now because MicroPython-level
code is fast enough at 100 Hz, and the full pbio-equivalent C implementation
is on the roadmap.

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
   user program. **Landed for openbricks in M1:** `MotorProcess` runs
   always-on at 100 Hz via `machine.Timer`, with the same "motors
   subscribe a control step, the process iterates them each beat" shape.
4. **Drivebase coupling** (`pbio/src/drivebase.c`) — the two wheels are
   controlled as one 2-DOF system, not two independent 1-DOF motors, which
   matters a lot for straight-line accuracy. We approximate this with a
   small correction term and it mostly works on flat floors.

The natural endpoint is a C module baked into the firmware image that ports
`pbio`'s observer + trajectory code. Because openbricks ships its own
MicroPython firmware (just like Pybricks), this is a straightforward
`user_c_modules` addition rather than an external dependency the user has to
install. `pbio` is MIT-licensed, so the port is legally painless too.

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

Near-term additions to prioritize:

- Trapezoidal trajectory planner and Kalman-style state observer (items 1–2
  above), feeding into the existing `MotorProcess` tick.
- True 2-DOF coupled drivebase controller (item 4) on top of the planner +
  observer.
- Distance sensor interface + driver for HC-SR04 and VL53L0X.
- IR remote receiver driver (any NEC-protocol IR module).
- A `hub` abstraction (battery, status LED, buttons) — now concretely in
  reach because we ship the firmware image and can add native hub
  primitives rather than rely on whatever GPIO happens to be available.
- Firmware build scripts that pin the MicroPython version, freeze the
  openbricks modules, and produce per-platform images (ESP32 first).

Further out, and the point at which this project earns the comparison to
Pybricks: a C-extension port of `pbio`'s observer + trajectory, compiled
into the firmware via `user_c_modules`.
