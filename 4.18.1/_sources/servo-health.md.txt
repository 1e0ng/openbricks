---
myst:
  html_meta:
    description: "Read an ST-3032/ST-3215 servo's voltage, temperature, current and protection flags; characterise a servo on a two-servo dynamometer; soak-test it before a competition."
---

# Servo health & characterisation

Every Feetech STS servo reports its own supply voltage, case
temperature, supply current and a byte of protection flags, next to
the position/speed/load block the drivers already read. Since 3.3.0
that block is one call away, and two bundled programs turn it into
the two questions a team actually asks about a servo: *what does
this particular unit do*, and *is it still doing it*.

## `health()`

```python
from openbricks.drivers.st3032 import ST3032Motor

m = ST3032Motor(servo_id=3, uart_id=1, tx=14, rx=41)
h = m.health()
print(h.voltage, "V", h.temperature, "C", h.current, "A", h.flags)
```

`health()` returns a `ServoHealth` tuple:

| Field | Meaning |
|-------|---------|
| `voltage` | supply rail at the servo, volts (0.1 V resolution) |
| `temperature` | case temperature, °C |
| `current` | supply current, amps (6.5 mA resolution) |
| `flags` | the protection flags currently set, as names — `voltage`, `sensor`, `temperature`, `current`, `angle`, `overload`; empty when healthy |
| `status` | the raw status byte |

It works on position servos (`ST3215` / `ST3032`) and wheel-mode
motors alike, adopted by a `DriveBase` or on their own bus. It
**raises `OSError`** when the bus is silent rather than returning
nothing — a health check that can't reach the servo is the failure
it exists to catch. On an adopted motor the four registers are staged
through the native bus pump, which takes a few milliseconds: read it
for a log line, not inside a control loop.

The ST-3032's own protections, from its datasheet, are the numbers
to read the fields against: it cuts torque above **80 °C**, flags
**over-voltage** outside 9–14 V, and latches **overload** after 2 s
above 80 % of stall (re-issuing a command clears it). Nominal current
is 100 mA free-running, 500 mA at rated load, 1.6 A stalled.

## Dynamometer: what does *this* servo do?

`examples/st3032_dyno.py` characterises one servo against a second
one, the way actuator labs do it with a load motor — minus the load
cells. Couple two ST-3032s horn to horn with a rigid coupler and bolt
the pair down. The servo under test drives at a fixed duty; the load
servo opposes it with a duty that steps up from 0 to 30 %; at each
step the program averages the driven servo's speed and supply
current. The current above the unloaded baseline, times the motor
constant (6.3 kg·cm/A, `ST3032Motor.KT_MNM_PER_A`), is the shaft
torque, so the sweep yields a torque–speed line, and a DC motor
behind a gearbox droops linearly — the fit's intercepts are the
unit's **no-load speed** and **stall torque** at that duty, scaled
to 100 % in the summary. A gentle reversal against the held load
servo at the end measures the pair's combined **gear play**.

```text
load_duty_pct,speed_dps,current_a,torque_mnm
0,262.4,0.118,0.0
5,251.0,0.151,20.4
...
# --- dyno summary ---
servo 1 at 30 % duty: no-load 263 dps, stall 226 mNm (2.30 kg.cm), droop -1.16 dps per mNm
scaled to 100 % duty: stall ~753 mNm, datasheet 980; no-load ~877 dps, datasheet 888
combined gear play of the pair: 0.95 deg (datasheet <= 1.0 per servo)
```

Run it on every servo you own and keep the summary lines. The
numbers are not lab-grade — the torque rides on a datasheet
constant, and the stall figure is an extrapolation from a 30 % sweep
(the bench cap; raise `DUTY` and `LOAD_STEPS` only with the pair
bolted down) — but they are *consistent*, which is what matters:
the unit whose no-load speed or stall sits well off its siblings is
the one to keep out of the drive pair, and a pair that measures
alike will track alike under `DriveBase`. The simulator's servos are
ideal velocity loops, so there is nothing to feed these numbers
into; they are for choosing and comparing hardware.

## Soak test: is it still good?

`examples/st3032_soak_test.py` runs one servo through ±90° swings
for an hour (`DURATION_MIN`) and every `REPORT_EVERY` cycles logs
`health()` plus the servo's gear play — one target approached from
below and from above under hold, the difference between the two rest
angles. The summary is the drift from the first report to the last:

```text
# --- soak summary ---
1780 cycles: temperature +14 C (peak 46 C, shutdown at 80), current +0.021 A, play +0.08 deg, supply dipped to 11.4 V
```

What to look for: a temperature that keeps climbing instead of
levelling off, play that grows through the run, current that rises
at the same speed and load, or any protection flag at all. Each is a
servo to retire — or a wiring/battery problem to fix — before it
shows up on the mat. Feetech's own life test is 50 000 cycles at a
fifth of stall torque; an hour is a couple of thousand, enough to
expose a unit that is already going.
