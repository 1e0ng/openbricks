---
myst:
  html_meta:
    description: "openbricks is a Pybricks-style MicroPython firmware for open hardware: commodity ESP32 boards, off-the-shelf motors and sensors, and a clean Python robotics API."
---

# openbricks documentation

**openbricks** is a Pybricks-style MicroPython firmware for **open
hardware** — commodity MCUs, commodity motors, commodity sensors.

*Prefer offline reading? This documentation is also available as a
single* **[PDF download](https://docs.openbricks.dev/openbricks-docs.pdf)**,
*rebuilt on every deploy.*

Like Pybricks, openbricks is a *firmware you flash to an MCU*, not a
library you `pip install` on top of stock MicroPython. The firmware owns
the runtime: the 1 kHz motor scheduler, trajectory planner, state
observer, and drivebase controller run as native C code inside the
image, and the three-layer Python API (drivers → interfaces → robotics)
is what `import openbricks` gives you out of the box.

```python
from openbricks.drivers.st3032 import ST3032Motor
from openbricks.robotics import DriveBase

left = ST3032Motor(servo_id=1, tx=14, rx=6)
right = ST3032Motor(servo_id=2, tx=14, rx=6, invert=True)

db = DriveBase(left, right, wheel_diameter_mm=56, axle_track_mm=114)
db.straight(500)   # mm
db.turn(90)        # degrees
```

- **Firmware images**: ESP32-S3 (primary) and classic ESP32, prebuilt on
  every [release](https://github.com/1e0ng/openbricks/releases).
- **Host tooling**: `pipx install openbricks` gets you the
  {doc}`openbricks CLI <cli>` (flash / run / upload / stop / log over
  BLE) and, with the `[sim]` extra, a MuJoCo-backed {doc}`simulator
  <simulator>`.
- **Source**: [github.com/1e0ng/openbricks](https://github.com/1e0ng/openbricks), MIT licensed.

```{toctree}
:caption: Getting started
:maxdepth: 1

install
hardware
cli
simulator
examples
```

```{toctree}
:caption: API reference
:maxdepth: 2

api/index
```

```{toctree}
:caption: Internals
:maxdepth: 1

architecture
build
```
