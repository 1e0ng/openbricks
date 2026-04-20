# Architecture

A short tour of how `openbricks` is organized and why. If you've read Pybricks'
`pbio` codebase, a lot of this will look familiar — the layering is borrowed
directly. What's different is that we replace pbio's C-level real-time library
with MicroPython drivers, and we add a configuration layer so the user never
has to touch the driver code for common hardware.

## Three layers

```
┌─────────────────────────────────────────────────────────┐
│  User code     (main.py, robotics.DriveBase, …)          │
├─────────────────────────────────────────────────────────┤
│  Abstract interfaces   (Motor, Servo, IMU, ColorSensor)  │
├─────────────────────────────────────────────────────────┤
│  Concrete drivers      (l298n, jgb37_520, bno055, …)     │
├─────────────────────────────────────────────────────────┤
│  MicroPython HAL       (machine.Pin, I2C, UART, PWM)     │
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
`pbio/src/*.c` is the library, `pbio/drv/*` is the driver layer. We just
collapse the library layer into the interface layer for now because
MicroPython-level code is too slow for Pybricks-quality control loops anyway.

## Where Pybricks puts the real work (and where we don't, yet)

Pybricks' `pbio` library does four things well that this project does only
minimally:

1. **State observer** (`pbio/src/observer.c`) — a Kalman-flavored model of
   the motor that estimates speed, current, and torque from voltage
   commands and encoder readings. We currently just finite-difference the
   encoder, which is noisy.
2. **Trajectory planning** (`pbio/src/trajectory.c`) — trapezoidal speed
   profiles with explicit acceleration, cruise, and deceleration phases.
   We set a constant speed and stop when we're there.
3. **Cooperative multitasking** (`pbio/src/motor_process.c`, `os.c`) — a
   1 kHz motor tick scheduled off a timer interrupt, independent of the
   user program. In MicroPython we'd build this on `machine.Timer` or
   `asyncio`; right now control happens in user-driven loops.
4. **Drivebase coupling** (`pbio/src/drivebase.c`) — the two wheels are
   controlled as one 2-DOF system, not two independent 1-DOF motors, which
   matters a lot for straight-line accuracy. We approximate this with a
   small correction term and it mostly works on flat floors.

The first three are where you'd put effort if you want production-quality
control. A reasonable path is to keep the MicroPython API surface identical,
and move `jgb37_520.py`'s control loop into a C extension module (ESP32
supports these via `user_c_modules`). At that point the project looks a lot
like Pybricks, just with commodity parts.

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

- Distance sensor interface + driver for HC-SR04 and VL53L0X.
- IR remote receiver driver (any NEC-protocol IR module).
- A proper `MotorProcess` background task using `machine.Timer` so control
  loops run regardless of what the user program is doing.
- A `hub` abstraction (battery, status LED, buttons) so user code reads the
  same whether you're on an ESP32 devkit or a custom board.

Further out, and the point at which this project earns the comparison to
Pybricks: a C module (compiled into a custom MicroPython firmware) that
ports `pbio`'s observer + trajectory code. Because `pbio` is MIT-licensed,
this is legally straightforward.
