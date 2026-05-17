# SPDX-License-Identifier: MIT
"""
ST-3032 (Feetech STS3032) serial bus servo.

Smaller sibling of the ST-3215 — same SCS protocol, same 4096-count
12-bit magnetic encoder, same operating modes (0 = position,
1 = wheel), same default 1 Mbps bus. The only differences are
mechanical / electrical.

Full datasheet: ``docs/datasheets/feetech_sts3032.pdf`` (model
ST-3032-C062, Edition A/0 2025-11-30). Headline specs at typical
12 V operation, all ±10% per Feetech:

    Working voltage      9–14 V (typ 12 V)        ST-3215 typ 6–12.6 V
    No-load speed        0.067 s/60° = 148 RPM = 888 °/s
                         (driver default ``max_dps=600`` is below
                         this — instantiate with ``max_dps=900`` if
                         you want to chase no-load top speed)
    Stall torque         10 kg·cm (139 oz·in)     ST-3215 ~30 kg·cm
    Stall current        1.6 A
    Rated torque         3.3 kg·cm (1/3 of stall)
    Rated current        500 mA
    No-load current      100 mA
    Idle current         20 mA
    Kt                   6.3 kg·cm/A
    Size                 23 × 12 × 27.5 mm        ST-3215 larger
    Weight               20.6 g
    Gear                 1/205, coreless motor
    Backlash             ≤ 1.0°
    Over-temp shutdown   80 °C
    Over-current         > 80 % stall for 2 s
    Max position update  1 ms

Because the wire protocol is identical, ``ST3032`` and ``ST3032Motor``
are thin marker subclasses of ``ST3215`` / ``ST3215Motor`` with no
behavioural override. Use them when your bench actually carries an
ST-3032 — the typed name documents the hardware and gives us a place
to specialise defaults later (lower max_dps, torque caps) if hardware
testing surfaces a real difference.

ST-3032 and ST-3215 instances on the same UART share the bus
registry, so mixing them on one daisy chain is fine — each servo is
addressed by its 1-byte ID regardless of model.

⚠ Voltage rail: an ST-3032 will brown out below ~9 V. Don't share a
6 V bus with ST-3215s; budget a separate 12 V rail.
"""

from openbricks.drivers.st3215 import ST3215, ST3215Motor


class ST3032(ST3215):
    """One ST-3032 in position-servo mode (Servo interface).

    Behaves identically to ``ST3215`` — see that class for the
    full API. Subclassed only to let user code spell the actual
    hardware in place.
    """


class ST3032Motor(ST3215Motor):
    """One ST-3032 in wheel/continuous-rotation mode (Motor interface).

    Behaves identically to ``ST3215Motor`` — see that class for
    ``run_speed`` / ``angle`` / ``run_angle`` semantics. Subclassed
    only to let user code spell the actual hardware in place.
    """
