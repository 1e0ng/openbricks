# Contributing

Thanks for your interest. This is an early-stage project — architectural
feedback and new drivers are both welcome.

## Adding a new driver

1. Pick the interface your component fits: `Motor`, `Servo`, `IMU`,
   `ColorSensor` (or add a new one in `openbricks/interfaces.py` if none
   fit).
2. Create a new file in `openbricks/drivers/` — one file per chip family.
3. Subclass the interface and implement every method. Raising
   `NotImplementedError` on methods your hardware genuinely can't do is
   acceptable.
4. Register the driver in `openbricks/config.py`'s `_DRIVER_REGISTRY`.
5. Add a short example to `examples/`.
6. If the chip needs calibration or has surprising quirks, add a section
   to `docs/hardware.md`.

## Style

- Follow PEP 8 loosely. The code targets MicroPython, which doesn't have
  f-strings on every port — prefer `"{}".format(...)` for portability.
- No heap allocations inside ISR callbacks. MicroPython's GC may be
  disabled in interrupt context.
- Keep drivers free of `asyncio` for now. MicroPython's `asyncio` support
  varies across ports; a synchronous API is the common denominator.

## Testing

Unit tests live in `tests/` and use plain `unittest`. They don't require
hardware — drivers should be testable by injecting fake `I2C` / `UART` /
`Pin` objects. See `tests/test_l298n.py` for the pattern.

Run tests on the desktop with CPython:

    python -m unittest discover -s tests

## Licensing

By contributing you agree your contributions will be released under the
MIT license (see `LICENSE`).
