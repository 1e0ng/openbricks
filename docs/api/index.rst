API reference
=============

Everything below ships frozen into the firmware image — ``import
openbricks`` on the hub gives you all of it. The pages are generated
from the package's docstrings.

The API is layered:

- **Drivers** (``openbricks.drivers.*``) — one module per supported
  component. Construct these with your wiring's pins / IDs.
- **Interfaces** (``openbricks.interfaces``,
  ``openbricks.distance``) — the abstract contracts drivers implement.
  Higher layers depend only on these, which is what makes the system
  plug-and-play.
- **Robotics** (``openbricks.robotics``) — building blocks composed
  from interfaces, like the two-wheel ``DriveBase``.
- **Hub & runtime** — board-level peripherals, BLE, program launching,
  and log capture.

.. toctree::
   :maxdepth: 2

   robotics
   drivers_motors
   drivers_sensors
   drivers_misc
   interfaces
   hub
   tools
   runtime
