# SPDX-License-Identifier: MIT
"""IMU warmup probe — measure the gyro turn-on transient.

Keep the robot perfectly still and run twice: button-started within
seconds of a cold power-on, then again after a minute of idle:

    openbricks upload -n <hub> examples/imu_warmup_probe.py
    openbricks log -n <hub>

A cold trace whose gyro-z starts high and settles while the warm
trace stays flat is the turn-on transient; ``calibrated=1`` on the
first row means an NVS-seeded bias is passing warmup checks at t=0.
"""

import time

from openbricks.drivers.icm45686 import ICM45686

SCK, MOSI, MISO, CS = 12, 13, 11, 17


# --- settle summary ---
def gyro_z_settle(samples):
    """Mean gyro-z over the first and the last second of ``samples``
    ((t_ms, dps) tuples in time order) — the difference is the
    transient that was still decaying while the probe ran."""
    t0 = samples[0][0]
    t1 = samples[-1][0]
    early = [dps for t, dps in samples if t - t0 < 1000]
    late = [dps for t, dps in samples if t1 - t < 1000]
    return (sum(early) / len(early), sum(late) / len(late))
# --- end settle summary ---


imu = ICM45686(sck=SCK, mosi=MOSI, miso=MISO, cs=CS)

print("t_ms,gyro_z_dps,heading_deg,calibrated")
samples = []
start = time.ticks_ms()
while True:
    t = time.ticks_diff(time.ticks_ms(), start)
    if t >= 30_000:
        break
    dps = imu.gyro()[2]
    samples.append((t, dps))
    print("%d,%.4f,%.3f,%d" % (t, dps, imu.heading(), imu.calibrated()))
    time.sleep_ms(100)

early, late = gyro_z_settle(samples)
print("gyro-z mean: first second %+.4f dps, last second %+.4f dps, "
      "settle %+.4f dps" % (early, late, early - late))
print("heading moved %+.3f deg over 30 s of stillness" % imu.heading())
