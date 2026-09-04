# SPDX-License-Identifier: MIT
"""ST-3032 dynamometer — measure what YOUR servo does, unit by unit.

Couple two ST-3032s horn to horn (a rigid coupler; the pair bolted
down). The servo under test drives at a fixed duty while the second
servo loads it with an opposing duty in steps; the speed and supply
current of the servo under test give a torque-speed line through the
motor constant (torque = Kt x current above no-load), and a gentle
reversal against the held load servo gives the pair's combined gear
play. Run it on every spare and keep the summary lines: the unit
whose no-load speed or stall estimate sits well off the others is
the one to leave out of the drive pair.

    openbricks upload -n <hub> examples/st3032_dyno.py
    openbricks log -n <hub>

The sweep stops at 30 % duty on both servos; the fitted line is
extrapolated to stall (a DC motor behind a gearbox droops linearly).
"""

import time

from openbricks.drivers.st3032 import ST3032Motor

UART_ID, TX, RX = 1, 14, 41
DUT_ID, LOAD_ID = 1, 2
DUTY = 30
LOAD_STEPS = (0, 5, 10, 15, 20, 25, 30)
SETTLE_MS = 800
SAMPLES = 10
PLAY_DUTY = 8


# --- dyno summary ---
def torque_mnm(current_a, no_load_current_a, kt_mnm_per_a):
    """Shaft torque from supply current above the no-load current,
    through the motor constant (ST-3032 datasheet: 6.3 kg·cm/A)."""
    extra = current_a - no_load_current_a
    if extra < 0.0:
        extra = 0.0
    return extra * kt_mnm_per_a


def fit_torque_speed(points):
    """Least-squares line through (torque_mnm, speed_dps) points:
    (no_load_dps, stall_mnm, slope_dps_per_mnm) — the speed at zero
    torque, the torque at zero speed, and the droop between."""
    n = len(points)
    mean_t = sum(t for t, _ in points) / n
    mean_s = sum(s for _, s in points) / n
    sxx = sum((t - mean_t) ** 2 for t, _ in points)
    if sxx == 0.0:
        raise ValueError("torque did not change across the sweep - "
                         "is the load servo coupled and powered?")
    slope = sum((t - mean_t) * (s - mean_s) for t, s in points) / sxx
    no_load = mean_s - slope * mean_t
    if slope >= 0.0:
        raise ValueError("speed rose with torque - the load servo is "
                         "helping, not opposing; flip its sign")
    return (no_load, -no_load / slope, slope)


def play_deg(plus_angles, minus_angles):
    """Combined gear play of the coupled pair: the mean rest angle
    after pushing gently one way minus the mean after pushing the
    other way."""
    return abs(sum(plus_angles) / len(plus_angles)
               - sum(minus_angles) / len(minus_angles))
# --- end dyno summary ---


dut = ST3032Motor(servo_id=DUT_ID, uart_id=UART_ID, tx=TX, rx=RX)
load = ST3032Motor(servo_id=LOAD_ID, uart_id=UART_ID, tx=TX, rx=RX)

print("load_duty_pct,speed_dps,current_a,torque_mnm")
points = []
no_load_current = None
for step in LOAD_STEPS:
    dut.dc(DUTY)
    load.dc(-step)
    time.sleep_ms(SETTLE_MS)
    speed = 0.0
    current = 0.0
    for _ in range(SAMPLES):
        s = dut.speed()
        if s is None:
            raise OSError("servo %d went silent mid-sweep" % DUT_ID)
        speed += s
        current += dut.health().current
        time.sleep_ms(20)
    speed /= SAMPLES
    current /= SAMPLES
    if no_load_current is None:
        no_load_current = current
    torque = torque_mnm(current, no_load_current,
                        ST3032Motor.KT_MNM_PER_A)
    points.append((torque, abs(speed)))
    print("%d,%.1f,%.3f,%.1f" % (step, speed, current, torque))
dut.coast()
load.coast()
time.sleep_ms(500)

load.hold()
plus = []
minus = []
for _ in range(3):
    dut.dc(PLAY_DUTY)
    time.sleep_ms(400)
    a = dut.angle()
    dut.dc(-PLAY_DUTY)
    time.sleep_ms(400)
    b = dut.angle()
    if a is None or b is None:
        raise OSError("servo %d went silent during the play check"
                      % DUT_ID)
    plus.append(a)
    minus.append(b)
dut.coast()
load.coast()

no_load, stall, slope = fit_torque_speed(points)
print("# --- dyno summary ---")
print("servo %d at %d %% duty: no-load %.0f dps, stall %.0f mNm "
      "(%.2f kg.cm), droop %.3f dps per mNm"
      % (DUT_ID, DUTY, no_load, stall, stall / 98.07, slope))
print("scaled to 100 %% duty: stall ~%.0f mNm, datasheet %.0f; "
      "no-load ~%.0f dps, datasheet %.0f"
      % (stall * 100.0 / DUTY, ST3032Motor.STALL_TORQUE_MNM,
         no_load * 100.0 / DUTY, ST3032Motor.max_dps_default()))
print("combined gear play of the pair: %.2f deg (datasheet <= 1.0 "
      "per servo)" % play_deg(plus, minus))
