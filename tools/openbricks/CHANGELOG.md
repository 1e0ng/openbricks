# openbricks — host-tooling changelog

Versions the unified `openbricks` PyPI package (CLI + MuJoCo sim).
Firmware versions are tracked separately on the `v*` tag namespace.

## 1.28.0 — `openbricks docs`: read the guides offline

New `openbricks docs [topic]` subcommand (alias: `doc`) prints the
documentation guides in the terminal — no internet, browser, or
repo checkout needed. The hand-written guide pages (install,
hardware, cli, simulator, examples, architecture, build, index)
are synced from `docs/` into the wheel at build time; MyST
front-matter and inline roles are cleaned for terminal reading;
output pages through `$PAGER`/`less` on a TTY and prints plainly
when piped. With no topic, lists what's available. The generated
API reference isn't bundled — its offline equivalent is Python's
own `help()`, and the full manual (HTML + PDF) stays at
docs.openbricks.dev. A wheel-bundling test guards the pages the
same way the sim worlds are guarded.

## 1.27.0 — WS2812 / WS2812B RGB LED strip driver

New `openbricks.drivers.ws2812.WS2812` for addressable RGB LED
modules — the common WS2812B ×8 stick, rings, and strips — on a
single data GPIO, wrapping MicroPython's built-in `neopixel`
protocol driver with the ergonomics user code wants: global
`brightness` scaling applied at `show()` (colors read back
unscaled; 0.2 indoor default, same convention as the hub status
LED), buffered item assignment with one-transaction `show()` for
flicker-free animation frames, and immediate `fill()` / `clear()`
conveniences. Reserved-pin validation via `openbricks.pins` like
every other driver. Ships with `examples/ws2812_test.py` (solid
colors → running dot → brightness ramp). Firmware reflash required
to get the new frozen module.

## 1.26.0 — BNO055 drops the magnetometer: IMU fusion mode

Bench (2026-07-25): a gyro'd square REPORTED only +1.8° of heading
drift while the robot's body visibly ended much further off — the
controller faithfully steered a corrupted measurement to its
target. Static tests were clean (heading rock-steady at one spot,
motors on or off), which isolates the corruption to NDOF mode's
magnetometer blend: fused heading follows the LOCAL magnetic
field, and motor magnets/load currents plus steel in floors and
furniture bend that field from place to place, silently eating
real rotation as the robot translates.

The driver now engages the chip's IMU fusion mode (accel + gyro,
no magnetometer) instead of NDOF. Semantics change: ``heading()``
zero is the orientation at driver construction, not magnetic
north — which is all any consumer here ever needed (DriveBase's
absolute frame baselines at ``use_gyro(True)`` regardless), and
it's the same trade Pybricks-class drivebases make. Firmware
reflash required.

## 1.25.0 — gyro mode holds an absolute heading target (Pybricks-style)

Hardware verification of 1.24.0 measured ~+7° of drift over one
gyro'd square where encoder mode drifted -4.3° — the gyro pass was
WORSE than the thing it replaces. Cause: every move re-baselined its
heading reference at its own start, so each turn's termination
overshoot (~1.5-2° of poll latency + coast momentum) was forgiven
and accumulated forever.

`use_gyro(True)` now maintains a persistent ABSOLUTE heading
target, like Pybricks: enabling it makes the current pose the
reference, `turn(a)` advances the target by `a`, and every
subsequent move steers the measured heading toward the target —
so overshoot banked by one turn is pulled back by the next move,
and a turn that arrives already past its target ends immediately
instead of rotating another lap. The measured heading is
accumulated continuously (per-tick deltas wrapped ±180), so
multi-turn totals and the BNO055's ±180 boundary are handled.

Applies uniformly to both paths — the serial-bus fallback loop and
the native C controller (core keeps the target in `turn_hold`
across moves; bindings baseline once at the enable transition, not
per move). Also fixes a units bug in the fallback gyro-turn decel
(body-degrees were fed to a wheel-degree profile).

Sim verification: the gyro square example closes to +0.0° (was
+0.2°), and `wander_with_gyro` (native path) ends 0.0095° from its
start after four squares (was 0.11°). Firmware reflash required.

## 1.24.0 — BREAKING: turn/heading sign now matches Pybricks (positive = right/clockwise)

Through 1.23.2, openbricks used its own CCW-positive convention:
``DriveBase.turn(90)`` meant turn LEFT. Pybricks documents the
opposite — "Positive angles and turn rates mean turning right …
positive means clockwise" — and per the project's Pybricks-parity
contract the whole system now adopts it:

* ``DriveBase.turn(angle)`` / ``drive(speed, turn_rate)`` — positive
  now turns RIGHT (clockwise viewed from above), on both the native
  C path and the serial-bus fallback. **Negate the angle in any
  existing script that relied on positive-turns-left.**
* ``BNO055.heading()`` / ``euler()[0]`` — CW-positive, which is what
  the chip natively reports (compass convention), so 1.23.2's
  driver-boundary negation is reverted; heading is once again the
  raw fused value wrapped to [-180, 180].
* ``SimIMU.heading()`` — negated (MuJoCo world yaw is math-convention
  CCW) so the sim reports the same sign as real hardware.
  ``robot.chassis_pose()`` deliberately keeps math-convention world
  yaw — it's a world-frame pose API with no Pybricks counterpart.
* The C core's turn mapping and body→wheel-diff conversion flipped
  to match; every consumer routes through the shared core, so the
  firmware, the CPython sim extension, and the shim all agree.

Every sign site flipped in one coordinated change and the sign
contract is now pinned by paired tests (``test_bno055`` on firmware,
``test_imu`` in the sim) that reference each other. Verified in sim:
a gyro'd 4×(straight+turn) square closes to ~0.1° drift.

## 1.23.2 — firmware: BNO055 heading sign fixed (CW → CCW-positive)

Real-hardware testing of 1.23.0's fallback-path gyro support caught
a sign bug in the BNO055 driver, live on the bench as tens of
degrees of runaway heading drift per rep in a gyro'd square that
should have closed to ~0: the chip's fused Euler heading is compass
convention — Bosch datasheet Table 3-13, "turning clockwise
increases values", regardless of the Windows/Android UNIT_SEL
format bit — while everything that consumes ``heading()``
(``DriveBase``'s native gyro tick and the serial-bus fallback
alike) assumes openbricks' CCW-positive "positive = left"
convention, same as the sim's ``SimIMU`` (which is why the gyro
path closed fine in the sim and ran away only on hardware). The
driver now negates once at the boundary; ``heading()`` and
``euler()``'s heading component are CCW-positive in [-180, 180].
Closed-loop heading correction was positive feedback before this —
firmware reflash required.

Also: the gyro examples now carry the reference chassis's real
servo-side mapping (left wheel = ID 2 with ``invert=True``, right =
ID 1).

## 1.23.1 — sim: ST-3032's real speed ceiling, gyro example updated

`ShimST3032Motor` (the sim drop-in for the firmware's `ST3032Motor`)
was a bare marker subclass of `ShimST3215Motor` and inherited its
600 dps `max_dps` default verbatim — the same bug the firmware
driver fixed in 1.16.1 (ST-3215's protective clamp is well under
the ST-3032's actual 888 °/s no-load speed), just recurring on the
sim side. A default-constructed `ST3032Motor` in a sim script now
clamps at 888, matching real hardware.

`examples/wander_with_gyro.py` now uses `ST3032Motor` (serial-bus,
fallback DriveBase path) instead of `JGB37Motor` (encoder DC motor,
native path) as its motor — demonstrating 1.23.0's new fallback-path
gyro support specifically, since that's the path that couldn't do
gyro feedback before this release.

## 1.23.0 — `DriveBase.use_gyro()` now works on serial-bus servos too

`use_gyro(True)` previously required the native (encoder-servo)
controller and raised `RuntimeError` for anything else — including
ST-3215/ST-3032 drivebases, which only ever run the pure-Python
open-loop fallback since serial-bus servos don't subscribe to the
1 kHz `motor_process` scheduler. The fallback now reads the IMU too:

* `straight()`'s heading-hold correction is sourced from the gyro
  instead of the encoder differential when `use_gyro(True)` — slip
  on one wheel no longer fools the correction term, since the IMU
  measures actual body rotation regardless of what the encoders say.
* `turn()` now *terminates* on measured body rotation reaching the
  target angle, not on encoder-estimated wheel travel — the more
  consequential half of slip-immunity for turns specifically, where
  wheel spin/skid on carpet is common.
* `use_gyro(True)` without an attached `imu=` now raises `ValueError`
  ("no imu attached...") on both paths — previously the fallback
  path raised a different `RuntimeError` for a different reason
  (no native controller at all), independent of whether an IMU was
  even passed in.

No behavior change when `imu=` isn't provided, or `use_gyro` is left
at its default `False`.

## 1.22.2 — run/upload fast again: one round trip per script

1.19.2's chunked staging (the fragmented-heap upload fix) traded
speed for reliability: every 512-byte piece of the script was its
own raw-paste round trip, each paying BLE's connection-interval
latency — a typical 8 KB script took ~16 round trips where it used
to take 1. Since 1.20.0 gave the S3 8 MB of PSRAM, the fragmentation
that forced the small chunk size is no longer a practical concern
on current boards: the chunk size now equals ``_MAX_SCRIPT_BYTES``
(64 KB), so any script within the supported size limit stages in a
single round trip again. The chunking loop stays in place as a
safety net if the size limit is ever raised past a single
comfortably-sized buffer.

## 1.22.1 — chip probe fixed for esptool v5

1.22.0's chip identification silently failed on every esptool v5
install (bench: "could not identify the connected chip"): v5 prints
a column-padded ``Chip type:          ESP32-S3`` where v4 printed
``Chip is ESP32-S3``, and the parser only knew the v4 phrasing. Both
formats parse now, the test fakes carry the REAL v5.3.1 output, and
an unparseable probe includes esptool's last output line in the
warning so the next format change diagnoses itself.

## 1.22.0 — `openbricks flash` with no arguments but the name

``--port`` and ``--firmware`` are now optional:

* **Port auto-detection** — with exactly ONE ESP device connected
  (Espressif native USB or a CP210x/CH340/FTDI bridge, matched by
  USB vendor ID so a modem can't be grabbed), the port is found
  automatically. Zero or several candidates refuse with a listing —
  flashing is destructive, so we never guess.
* **Chip identification** — the bootloader is probed (``chip-id``)
  and variant names canonicalized (an ESP32-D0WD is an ``esp32``,
  an ESP32-S3 is an ``esp32s3``); the result feeds esptool's
  ``--chip`` and the new safety check below.
* **Firmware auto-download** — ``--firmware`` omitted downloads the
  newest GitHub release's image for the detected chip (cached in
  ``~/.cache/openbricks/firmware``, size-verified).
* **Image/chip mismatch guard** — the merged image's bootloader
  header records the chip it was built for; flashing an esp32 image
  onto an S3 (a silent no-boot brick) now dies BEFORE the erase.

``openbricks flash --name ls`` is now the whole command.

## 1.21.1 — wheels slimmed 17.3 MB → ~4 MB (PyPI storage limit)

The ``cli/v1.21.0`` publish died mid-matrix on PyPI's 10 GB
project-storage limit: 43 releases × ~277 MB, almost all of it
three lossless WRO mat textures (16.9 MB) duplicated into every
wheel. The mats are now 75 dpi (0.34 mm/px — the TCS34725 sampling
spot still spans ~9 px, sim colour-sampling tests unchanged) and
256-colour palette-quantized: 17 MB → 3.1 MB. The regen script
bakes both steps in, and a wheel-size budget test fails any regen
that reinflates them. Note: 1.21.0 is only partially on PyPI
(cp310/cp311); 1.21.1 supersedes it.

## 1.21.0 — Pybricks Prime Motor API parity

**BREAKING: ``motor.run()`` changed meaning.** It now follows
Pybricks — ``run(speed)`` in degrees per second, closed loop. The
old percent-power command is ``dc(duty)``, exactly Pybricks' split.
A script still calling ``run(30)`` for power now gets 30 deg/s
(slow, not dangerous) on closed-loop motors, or
``NotImplementedError`` on open-loop drivers. Sweep your scripts:
``run(power)`` → ``dc(power)``.

New, per the Pybricks Motor API:

* ``speed()`` — measured deg/s (serial servos: present-speed
  register; encoder motors: the native α-β observer via a new
  ``measured_dps`` C binding; sim: physics joint velocity).
* ``load()`` — estimated shaft torque in mNm from the servo's load
  register scaled by the model's datasheet stall torque (ST-3215
  2940 mNm, ST-3032 980 mNm).
* ``stalled()`` — loaded ≥ 80 % of stall and slower than 20 deg/s
  (class-attr thresholds, bench-tunable).
* ``run_time(speed, ms, then, wait)``, ``run_target(speed,
  target_angle, then, wait)`` (absolute, ``reset_angle`` frame) and
  ``run_until_stalled(speed, then)`` — concrete on the Motor base,
  built from the primitives, ``then`` ∈ hold/brake/coast/none
  (Pybricks defaults).
* Sim: ``dc()`` (sustained duty through the DC-motor model),
  ``run()``, ``speed()``; ``load``/``stalled`` raise (no load model).

Additional openbricks methods remain as aliases: ``coast()``,
``run_speed()``, ``run_angle`` (relative), non-blocking ``done()``.
Not implemented (documented): ``track_target``, the ``control``
settings object, ``duty_limit=`` on ``run_until_stalled``, and
``wait=False`` on ``run_time``.

## 1.20.2 — Motor.stop() (Pybricks semantics) + line_follow fixes

``Motor.stop()`` joins the interface with Pybricks Prime semantics
— stop and spin freely, coasting to rest by friction — as a
concrete method delegating to ``coast()``, so every driver (ST,
MG370, JGB37, L298N) and the sim's ``SimMotor`` get it uniformly;
``examples/line_follow.py`` ends its run with ``stop()`` as
written. Also alongside the example's simplification (endless
follow loop, 400 dps cruise): ``MAX_DPS`` was left at 400 ==
CRUISE_DPS, pinning the outer wheel at cruise and halving the
steering authority — retuned to 800 (under the ST-3032's 888
ceiling) with a headroom regression test so cap == cruise can't
come back silently. The unused ``DEBUG`` flag is gone.

## 1.20.1 — TCS34725 defaults: gain=16, integration_ms=2.4

The low-latency configuration the line follower proved out is now
the driver default: 2.4 ms integration (one cycle, the chip
minimum) with gain=16 to keep the signal budget healthy. A bare
``TCS34725(i2c)`` now behaves like the explicitly configured
line-follow sensors. Note for existing sketches using bare
constructors: normalized ``ambient()`` reads ~4× higher than under
the old 24 ms/gain-4 defaults (gain ×4, full-scale ÷10) — retune
thresholds accordingly (``line_align.py``'s ``LINE_AMBIENT`` moved
5 → 20 in the same change). Explicit arguments are unaffected.

## 1.20.0 — ESP32-S3: the 8 MB PSRAM is finally in the heap

Through 1.19.x the S3 board config listed only
``sdkconfig.spiram_oct`` — which selects the octal PSRAM *mode* but
never sets ``CONFIG_SPIRAM=y`` — so PSRAM was silently off and the
GC heap was internal-RAM only (~234 KB; the fragmentation behind
1.19.2's upload abort). The config now includes upstream's
``spiram_sx`` enable fragment before the octal override, matching
the ESP32_GENERIC_S3 SPIRAM_OCT variant: on an N16R8 the GC
auto-split heap grows into the 8 MB on demand. Boards without
working PSRAM still boot (``CONFIG_SPIRAM_IGNORE_NOTFOUND``), just
without the extra memory. Drift-guarded by
``tests/test_board_config.py``. After flashing, verify with
``gc.mem_free()`` — expect megabytes.

## 1.19.2 — run/upload survive a fragmented hub heap (chunked staging)

``openbricks run examples/line_follow.py`` died with "hub aborted
the upload": the hub had 177 KB free but a max contiguous hole of
5.2 KB, and the one-shot raw-paste of the 9.4 KB bootstrap needs
one contiguous, doubling buffer. MicroPython's GC never compacts,
so a long-lived heap will always fragment eventually — the upload
path now stages ``/program.py`` in 512-byte chunks (each a ~1-2 KB
paste, bounded even for worst-case binary) and then pastes a small
fixed-size runner, so peak hub RAM no longer scales with script
size. ``openbricks upload`` uses the same staging and cross-checks
the on-hub file size after. Hub-side staging errors (e.g. a full
filesystem) surface with their tracebacks instead of aborting
silently.

## 1.19.1 — brake() ramps again: one uniform acceleration rule

Reverts 1.18.1's instant-brake bypass by user decision: the
acceleration default now governs every commanded speed transition —
launches, speed changes, and ``brake()`` alike (at 1500 deg/s² a
brake from 200 dps settles in ~0.13 s). The safety story is
unchanged: the stop button / e-stop and ``coast()`` cut the torque
register, which the ramp does not govern, and remain instant.
``accel_dps2=0`` at construction still gives hard local stops.

## 1.19.0 — default acceleration retuned 360 → 1500 deg/s²

Pybricks parity: 1500 deg/s² is what a Pybricks DriveBase runs on a
SPIKE Prime hub (motors default 2000, DriveBase scales ×3/4). Now
that the serial-bus motors ramp too (1.18.0) and stops stay instant
(1.18.1), the snappier default applies uniformly: DriveBase moves,
direct ``run_speed()``, everything. All EIGHT value homes move
together (goal-acc register now encodes 171). Top speed untouched,
as always; the short-move threshold shrinks back to ~v²/accel ≈
27 mm at the 200 dps default.

## 1.18.1 — brake() is instant again: the ramp governs motion, not stops

1.18.0's servo-side ramp also slewed ``brake()`` — a 0.5+ s
roll-down from cruise, inconsistent with the encoder-motor drivers
and Pybricks, where ``brake()`` is immediate. The rule is now
uniform across every motor type: launches and speed changes ramp at
``accel_dps2``; ``brake()``, ``coast()`` and the e-stop stop
instantly. (``brake()`` brackets its zero-speed write with a
ramp-off/ramp-restore pair, so the following launch still slews.)

## 1.18.0 — serial-bus motors honour the acceleration default too

The acceleration retunes never touched direct ``run_speed()`` on
ST-3215/ST-3032 — the serial drivers had no acceleration home at
all, so speed commands stepped instantly (bench: "I intend it to be
affected"). The drivers now program the servo's own goal-acc
register (0x29, hardware ramp inside the servo) from a new
``accel_dps2`` parameter, default 360 — so the line follower and any
direct motor code launch at the same default as DriveBase moves.
``accel_dps2=0`` restores the old instant behaviour. The drift-guard
test now pins EIGHT homes. Safety note: the ramp also gentles
``brake()``, but the e-stop path cuts the torque register, which the
ramp does not govern — stop-button kills stay instant.

## 1.17.0 — default acceleration retuned 720 → 360 deg/s²

Gentler launch/brake ramps by default. As before, acceleration only
shapes the ramps — top speed still comes from ``straight_speed`` /
``turn_rate`` alone (the short-move footnote grows: below ~v²/accel
≈ 111 mm at the 200 dps default, a straight ends before touching
cruise). All six value homes move together, pinned by the
drift-guard tests.

## 1.16.1 — ST-3032 no longer speed-capped at 600 °/s by the driver

"Top speed is capped" on the bench, and it was: ``ST3032Motor``
inherited the ST-3215's protective ``max_dps=600`` default, and the
driver clamps every goal-speed write to it — 288 °/s below the
ST-3032's own datasheet no-load speed (888 °/s at 12 V). The
subclass now defaults ``max_dps`` to 888, so speed requests up to
spec reach the wire; an explicit ``max_dps=...`` still wins, and the
ST-3215's default is unchanged. (Under load the servo tops out
around 2/3 of no-load — that part is physics, not the driver.)

## 1.16.0 — default acceleration retuned 1000 → 720 deg/s²

Back to the original pre-1.13.0 value after bench driving at 1000.
The default only shapes the launch/brake ramps — it never caps top
speed: the profile commands min(cruise, accel·t, √(2·accel·remaining)),
so cruise still comes from ``straight_speed`` / ``turn_rate`` alone.
(Physics footnote: a move shorter than ~v²/accel ends before the ramp
reaches cruise; at 720 with the 200 dps default that's ~28 mm.)
All six value homes move together (Python fallback, native C
drivebase + servo, both encoder-motor drivers, simulator), pinned by
the drift-guard tests.

## 1.15.5 — the start press's whole lifecycle is consumed

1.15.4's press-DOWN starts left the finger on the button while the
program came up, and the bench event ring caught the same physical
press echoing into the run through both detectors:

* its debounced level confirmation arrived one tick later with the
  run already up and read as a mid-run stop press — the newborn run
  died at ~55 ms (start-latch, then press-down('running') +
  stop-fire, three times in one capture);
* a long-held press's release chatter edges would land past the
  400 ms grace window and fire latch-stops — the original "starts
  on press, stops on release" bug, back for long holds.

A counter-dispatched press is now tracked while physically down
(a mid-hold flicker can't detach it; DEBOUNCE_TICKS of released
samples end it): its level confirmation is consumed (ring:
`press-down start-press-consumed`), its release chatter is ignored
by the PCNT latch for 200 ms after the consumed release — including
back at idle after a short run the press outlived, where the
chatter used to dispatch a phantom restart. Stops stay covered
throughout by the debounced level path: a new press necessarily
begins after a confirmed release, which ends every one of these
windows.

## 1.15.4 — no tap too fast: counter-driven starts + verified idle restore

The 1.15.3 debounce traded the chatter bug for a fast-tap bug: a
press had to span two 50 ms polls to be believed, so crisp taps
were coin flips and the crispest fell between polls entirely
(bench: "press too fast → doesn't start", with an empty event
ring). Fixes on both ends of the pipeline:

* Firmware: at idle, the PCNT hardware edge counter — which sees
  every tap in silicon — is now the START trigger. Starts fire at
  press-DOWN (faster than the old release dispatch), no tap is too
  fast, chatter clusters dispatch once, the post-stop lockout still
  applies, and the start press's own release is consumed even when
  the program is already running by then. The level path demotes to
  state tracking (and remains the fallback where PCNT is
  unavailable). Every `_request_start` branch now records a ring
  event — a dead press can never again vanish without a fingerprint.
* CLI: the post-session idle-loop restore is verified instead of
  fire-and-forget — the tools wait for the hub's "Press button to
  run" banner and retry up to 3×; a restore lost in the disconnect
  race used to park the hub with a dead idle loop and silently dead
  button starts until a power-cycle.

## 1.15.3 — start-press chatter no longer kills the newborn run

Bench event-ring capture of "pressed start 4 times, only the 4th
worked": the start press's release chatter re-closed the contact and
stopped the run it had just started — once as a phantom
press-down 54 ms after start (level polling), twice as PCNT falling
edges 100/180 ms after start (the hardware press latch doing its job
on edges that weren't presses). Two defences, one per detector:

* Debounce: a button level change must hold for two consecutive
  50 ms polls before the launcher believes it. Chatter flickers are
  ~10-20 ms; a real press costs one extra poll of latency.
* Start grace: for 400 ms after a run starts, hardware-latch edges
  are consumed as the start press's own chatter instead of fired as
  stops. A real second press can't physically arrive that fast, and
  the debounced level path covers the window regardless (test-pinned:
  no stop-dead zone).

Post-stop start lockout widened 400 → 500 ms to account for the
debounce delaying a re-contact press's dispatch. Firmware-only
change (lockstep release).

## 1.15.2 — gyro-guided moves work after the robot's first rotation

A sim-based IMU verification caught a long-standing native
drivebase bug: with `use_gyro` on, a move's origin was snapshotted
from the ENCODER diff while the control tick read the move-relative
IMU override — so any accumulated encoder diff (i.e. any robot that
had ever turned before) became a permanent phantom heading error the
controller chased forever. A gyro'd `turn(90)` after one encoder
turn rotated ~180° and never stopped. The core now frames gyro
moves in the override's own coordinates (slot reset + zero snapshot
at move start).

Sim side: `SimDriveBase` gained the IMU heading feed
(`attach_imu()` + the per-tick push the shim previously owned), and
`set_use_gyro(True)` without a feed is now a loud error instead of
silently steering on a stale override. Regression tests on both
sides pin the fix: the firmware test fails 681-dps-of-phantom-error
style on the old core; the sim test drives a gyro turn after an
encoder turn in real physics and asserts it lands ~90° and stops.

## 1.15.1 — TCS34725 accepts fractional integration times

`TCS34725(integration_ms=2.4)` (the chip minimum, used by the
line-follow example for low-latency PID) crashed at construction:
the first-cycle settle called MicroPython's `time.sleep_ms` with a
float. Fixed; the test fake's `sleep_ms` is now int-strict like the
real MicroPython so this class of bug fails in CI instead of on the
bench. No host-tooling changes (lockstep release).

## 1.15.0 — one version for firmware and host tooling

Firmware and the `openbricks` PyPI package now share a single
version number, bumped together by `scripts/bump-version.py X.Y.Z`
and released as a pair (`v<version>` + `cli/v<version>`). PyPI jumps
from 0.14.0 straight to 1.15.0 to align with the firmware line.
No functional changes in this release — the version on your hub and
the version of your CLI now mean the same thing, and a mismatch is
worth noticing (tests pin the two source files equal).

## 0.14.0 — default acceleration retuned to 1000 deg/s²

Bench verdict on 1440: too aggressive as a default. All six
definition sites (firmware Python fallback, native C drivebase and
servo, both encoder-motor drivers, and the simulator) move to 1000
wheel-deg/s²; the drift-guard test keeps them pinned equal. Explicit
`settings(acceleration=...)` / `accel` arguments unaffected. Matches
firmware 1.14.0. Well within the sim's traction headroom (its DC
motor model tracks up to ~12,000 deg/s² before slipping).

## 0.13.0 — default acceleration 720 → 1440 deg/s²; sim physics fixed

The default acceleration doubles to 1440 wheel-deg/s² everywhere:
`DriveBase` ramps (Python fallback and native C), encoder-motor
`run_target`, the MG370/JGB37-520 driver constructors, and the
simulator. Explicit `settings(acceleration=...)` / `accel`
arguments are unaffected. A drift-guard test pins all six
definition sites to the same value.

Unifying the sim required fixing two long-standing physics bugs
(issue #234) that had made its chassis tracking loose at ANY
acceleration (a 100 mm straight used to land at ~245 mm):

* The wheel/caster z-offset formulas buried the wheels 20 mm into
  the floor — the robot rode on contact penetration-recovery forces
  instead of rolling.
* Motor power mapped linearly to torque with no back-EMF, so the
  servo core's feed-forward (calibrated for real DC motors) applied
  ~4x the traction limit as permanent torque, slipping the wheels
  and limit-cycling the control loop. `SimMotor` now models a
  linear DC motor (`torque = T_stall*(duty − ω/ω_rated)`, stall
  bounded below the traction limit) whose equilibrium matches the
  core's feed-forward exactly.

After the fixes a 100 mm straight lands at 98.7 mm with <0.2 mm of
wheel slip at both 720 and 1440 deg/s², and `turn(90)` lands at
89.1°.

## 0.12.0 — `servo-id`: assign Feetech servo bus IDs from the host

New subcommand speaking the Feetech SCS/STS protocol directly
through a USB half-duplex adapter (e.g. the URT-2 board):

    openbricks servo-id -p /dev/cu.usbmodem123 3        # set ID -> 3
    openbricks servo-id -p /dev/cu.usbmodem123 --scan   # who's there?

Safer than the usual one-shot scripts: the whole bus is scanned
first and, with more than one servo attached, the tool demands
`--old-id` instead of re-ID'ing whichever servo answers first; the
result is verified (new ID answers, old ID silent) or the command
fails loudly. Declares pyserial as a direct dependency (it already
arrived transitively via esptool).

## 0.11.0 — timestamped run logs + RTC sync on connect

Every hub log line now carries a timestamp. The hub (firmware
1.12.0+) stores a raw int64 UTC Unix-epoch-milliseconds prefix per
line — no formatting, no timezone on the device — and `openbricks
log` renders it as `[YYYY-MM-DD HH:MM:SS.mmm]` in the host's local
timezone at display.

The ESP32 has no NTP and its RTC powers up at 2000-01-01, so the CLI
now syncs the hub RTC from the host clock (in UTC) on every connect
— `run`, `upload`, and `log` all prepend the sync. Runs started
after any connect carry real wall-clock stamps; a hub that never saw
a connect since power-up stamps from 2000-01-01, which is
self-diagnosing.

Compatibility: logs written by older firmware have no stamps and
pass through unchanged; new-firmware logs read with an older CLI
show the raw epoch numbers.

## 0.10.24 — `log` works again (missed in the 0.10.23 signature change)

Every `openbricks log` invocation died with
`_stream_output() missing 1 required positional argument: 'out'`
before reading a byte: 0.10.23 added a `link` parameter to
`_stream_output` for timeout diagnostics and updated the `run` and
`upload` call sites, but not `log`. The test suite missed it because
the log dispatch tests stubbed `_stream_output` with the same stale
2-arg signature. The stubs now bind against the real helpers'
signatures, so an arity drift in any of the shared raw-REPL helpers
fails the suite instead of the user.

## 0.10.23 — BLE raw-REPL handshake retries instead of one-shot

`openbricks run` / `upload` / `log` sent a single Ctrl-C + Ctrl-A and
then waited up to 30 s for the raw-REPL banner. The Ctrl-C is
delivered on the hub as an injected KeyboardInterrupt, which raises
in whatever main-thread frame is executing — a scheduled callback
(BLE TX flush, button poll) can eat it, the same disease the stop
button had. One eaten interrupt cost a 30 s hang and a failed
connect with `notify_count=0`.

The handshake now retries: Ctrl-C + Ctrl-A every 4 s, up to 6
attempts (both are idempotent at the REPL). The timeout error names
the attempt count so a genuinely wedged hub is still diagnosable.

## 0.10.22 — `flash` writes the classic ESP32 at the right offset

`openbricks flash` hardcoded write offset `0x0`, which is correct for
the ESP32-S3 but wrong for the classic ESP32: MicroPython's merged
`firmware.bin` for that chip starts at flash address `0x1000` (the
chip's bootloader offset), so writing it at `0x0` put the bootloader
where the ROM never looks and the board failed to boot with
`Invalid image block, can't boot`.

The offset is now derived from the image itself — the partition table
always lives at flash `0x8000`, so its magic bytes sit at file offset
`0x8000 - base`, which pins the base the image was built for (`0x1000`
classic, `0x0` S3). No reliance on `--chip`, which defaults to `auto`.
An image matching neither layout (or both) aborts the flash *before*
the erase step rather than guessing.

Also wires the previously-orphaned `tests/test_st3032.py` into the
firmware test runner and the CPython CI job — it was in the tree but
executed by nothing.

## 0.10.21 — BLE tools restore the hub's idle loop on exit

`openbricks run` / `upload` / `log` all get their REPL by Ctrl-C'ing
whatever the hub was doing — usually the frozen main.py's
`launcher.run()` idle loop — and used to leave the hub parked at the
REPL on exit. Firmware 1.9.0 made main-thread idle-loop draining the
path where the button stop works; parked at the REPL, a button press
falls back to the degraded schedule-exec start (stop button
unavailable for that run, with a printed warning).

Each tool now sends a fire-and-forget `launcher.run()` raw-REPL exec
before disconnecting, so the hub returns to the same state the frozen
main.py boots into: button press starts /program.py in the main
thread, stop button works. The next tool invocation Ctrl-Cs the
restored loop exactly like it always Ctrl-C'd the boot one.

## 0.10.20 — serial-bus servos (ST-3032 / ST-3215) run in the sim

`openbricks sim run` now covers serial-bus drivebases. The shim
replaces `ST3215Motor` / `ST3032Motor` at the class level (they talk
UART directly, like the I2C drivers), binding each constructed motor
to the next chassis wheel slot. The openbricks `DriveBase` wrapper
then runs its serial-bus fallback loop — the same code path as
hardware — against MuJoCo.

Each shim motor emulates the servo's internal wheel-mode controller
with a per-tick P velocity loop on the exact MuJoCo joint velocity
(`data.qvel`). It is deliberately not routed through `SimMotor`'s
count-based servo core: integer encoder-count quantisation at 1 kHz
makes that observer's velocity estimate swing thousands of dps around
a ~250 dps wheel, and the resulting bang-bang torque never accumulates
the wheel rotation the fallback loop's position check waits for.

Semantics: `run_speed` / `run` / `run_angle(wait=)` / `done` /
`brake` / `hold` / `coast` / `angle` / `reset_angle` / `ping`, with
`max_dps` honoured. `invert=` and bus identity (`servo_id`, pins,
baud) are ignored as wiring concerns — the sim chassis defines both
wheel hinges on the same axis, and slot binding is by construction
order (first = left). Wheel rotation tracks the command exactly
(88.2° landed vs 88.1° target in the drivebase test); millimetre
travel reflects the model's wheel size, since the fallback path
doesn't resize the sim chassis the way the native `ShimDriveBase`
path does.

## 0.10.13 — `openbricks flash` uses esptool v5 commands when available

`openbricks flash` invoked `esptool.py write_flash` and `erase_flash`
— the esptool v4 forms. esptool v5 (out for ~12 months) renamed
the binary to `esptool` and the commands to kebab-case
(`write-flash`, `erase-flash`); the legacy forms still work but
print deprecation warnings on every flash:

```
Warning: DEPRECATED: 'esptool.py' is deprecated. Please use 'esptool' instead.
Warning: Deprecated: Command 'write_flash' is deprecated. Use 'write-flash' instead.
```

`flash.py` now runtime-detects which esptool form is on PATH and
picks command spelling accordingly: `esptool` + kebab-case if v5
is installed, falling back to `esptool.py` + underscore on v4.
Output is clean of deprecation warnings on v5 installs.

Why not bump the dependency floor? esptool 5+ requires Python ≥ 3.10
and openbricks supports ≥ 3.9; pinning v5 would break wheel builds
for Python 3.9 users. Runtime detection lets both paths coexist.

## 0.10.12 — Junior + Senior randomization tightened to mat-truth

Closes the two TODOs left after 0.10.10's per-round randomization
introduction (PR #105). Both are user-visible behaviour changes
for `openbricks sim --world wro-2026-junior` and
`--world wro-2026-senior`.

### Junior — slot coordinates now extracted from the mat

The four "black squares at the lower end of the field" used as
randomization slots had estimated coordinates (`y=-0.45`, round
x). PR #109 replaces them with positions extracted from the
high-res Junior mat artwork via `scripts/extract-wro-slot-coords.py`
(same flow used for Elementary in 0.10.10):

```
world (-0.1052, -0.5055) m   <- slot_1
world (+0.0267, -0.5055) m   <- slot_2
world (+0.1586, -0.5055) m   <- slot_3
world (+0.2904, -0.5055) m   <- slot_4
```

~0.13 m spacing — the same pattern as Elementary's note-start
squares.

### Senior — randomization now covers all 4 cement colours

0.10.10 wired Senior randomization for the yellow cement group
only (10 elements). 0.10.12 expands to all 4 colours
(yellow + blue + green + white = 40 elements). The rules require
each colour permutes within its own storage area independently,
so internally the spec moved from a single `_RandomizationSpec`
per world to a tuple of specs (one per colour group), driven by
one shared seeded RNG so `seed=N` still pins the entire
40-element layout.

`_SPECS` type changed from `Dict[str, _RandomizationSpec]` to
`Dict[str, Tuple[_RandomizationSpec, ...]]`. The public
`randomize(...)` signature is unchanged; the return dict simply
grows from 10 → 40 entries for Senior.

### Compatibility

No public API changes. Junior layouts produced by
`randomize(seed=N)` differ from 0.10.11 because the slot
coordinates moved; Senior layouts differ because three new
colour groups were added. Tests pinning specific (x, y) values
will need updating — the layout dict structure is the same.

## 0.10.11 — fix unusable wheels + WRO 2026 prop library complete

**Critical fix.** Wheels 0.10.7 → 0.10.10 on PyPI shipped with
**zero** `props/*.ldr` files in the package data. Every WRO world
on `pip install openbricks` raised:

```
WorldLoadError: lego_prop 'clef' references missing .ldr file
'.../openbricks_sim/worlds/wro_2026_elementary_robot_rockstars/props/clef.ldr'
```

…regardless of which `--world` you asked for, because every WRO
world references at least one `<lego_prop ldr="props/*.ldr"/>`
placeholder. Same shape of bug as the missing-worlds bug fixed
in 0.10.6; the F2 phase (PR #98 onwards) introduced `.ldr` files
without a matching `package-data` entry and the regression
slipped past until packaging-test coverage caught it during the
F4.1 wheel-build verification.

Fix: extend `[tool.setuptools.package-data]` to include
`worlds/*/props/*.ldr` and `worlds/*/*.stl`, with two new
regression tests in `test_wheel_bundles_worlds` to pin both.
**Anyone on 0.10.7 → 0.10.10 should upgrade to 0.10.11.**

### What's new (was previously stuck on PyPI)

The complete WRO 2026 RoboMission prop library now actually
loads. Every prop in all three age-group worlds is built from
LDraw assemblies (one `<lego_prop>` per visible LEGO model in the
official Building Instructions PDFs):

* **Elementary "Robot Rockstars"** — clef, two cables, three
  instruments + microphone, six notes, amplifier + two speakers
  (PRs #98 → #102).
* **Junior "Heritage Heroes"** — visitors, artefacts, towers,
  dirt, barrier, parrot (PR #103).
* **Senior "Mosaic Masters"** — 72 prop instances total: 3 tools,
  24 mosaic tiles (6 × 4 colours), 40 cement elements
  (10 × 4 colours), 4 barriers (2 colour schemes), plus the
  3D-printed mosaic frame mesh from WRO's own STL (PRs #104,
  #106, #107).

`scripts/extract-wro-slot-coords.py` produces randomization slot
coordinates for Junior + Senior the same way Elementary's were
extracted in 0.10.10 (pixel-inspection of the high-res mat).
F3.J wires Junior randomization (5 artefacts × 4 slots,
choose-N-of-M); F3.S wires Senior (10 cement positions per colour
within their respective target areas).

### Compatibility

No API changes. Worlds that worked on 0.10.6 still work; worlds
that *would* have worked on 0.10.7 → 0.10.10 if the wheel weren't
broken now actually do.

## 0.10.10 — Phase F3: per-round randomization (Elementary)

Per the WRO General Rules glossary ("Robot Round" definition):
"Before the round starts with the first team but after all robots
are placed on the robot parking, randomizations to game fields (if
any) are done." That's the part of the challenge that forces
robots to perceive the field rather than rely on hardcoded
positions. 0.10.10 brings the same to openbricks-sim:

```
openbricks sim preview --world wro-2026-elementary --seed 42
openbricks sim run     --world wro-2026-elementary --seed 7  main.py
```

Each integer seed produces a deterministic permutation; same seed
twice gives the same layout. Without `--seed`, randomization is
non-deterministic per run.

The Elementary spec (the only one wired in 0.10.10): the four
notes `black`, `white`, `yellow`, `blue` are permuted across four
fixed light-green start squares at the upper end of the mat. Per
the Game Rules PDF p7: "Four of the notes (black, white, yellow,
blue) are randomly placed on the four light-green squares at the
upper end of the game field. The positions of the green and red
note are not random." So 4! = 24 distinct layouts; the red and
green notes stay at fixed positions.

The four slot coordinates were extracted from the actual mat
artwork (the high-res `mat.png` shipped in 0.10.8), not estimated.
`scripts/extract-wro-slot-coords.py` finds connected components
of pixels matching a target colour in a strip of the mat and
prints centroids in mat-local world coordinates. Re-run when
WRO updates the mat artwork. For the Elementary mat with target
RGB (144, 208, 112) and tolerance 32, the script identifies four
clusters of identical size (35,532 px each, ≈32×32 mm) at:

```
world (+0.0499, +0.4881) m   <- slot_1
world (+0.1818, +0.4881) m   <- slot_2
world (+0.5775, +0.4881) m   <- slot_3
world (+0.7094, +0.4881) m   <- slot_4
```

Implementation surface in `openbricks_sim.randomization`:

```python
from openbricks_sim import randomization
layout = randomization.randomize(model, data,
                                 world="wro-2026-elementary",
                                 seed=N)
# {"note_black": "slot_2", "note_white": "slot_4", ...}
```

For each note it:
- locates the body's freejoint and writes `(x, y)` directly into
  `data.qpos` at the joint's qpos address;
- preserves the body's resting Z from `model.body_pos` (the value
  authored in the world XML), so the mixed-shape notes
  (sphere/box/cylinder of differing heights) all sit on the mat
  correctly after randomization;
- zeroes the freejoint's `qvel` so the body doesn't carry any
  pre-randomization motion;
- calls `mj_forward` once at the end to refresh `xpos` /
  `cam_xpos` / sensor reads.

Test pins in `test_randomization.py` (7 tests):
- determinism (same seed → same layout);
- 4! = 24 distinct layouts achievable (200 seeds, 24 unique
  results);
- every randomized note lands at one of the 4 spec slots;
- no two notes ever share a slot;
- the fixed `note_red` / `note_green` don't move;
- per-note Z is preserved through randomization;
- unknown world raises `KeyError` (Junior + Senior today; spec
  KeyError is how the CLI knows to skip the print).

Junior and Senior randomization specs land in follow-up PRs once
their Game Rules PDFs are read into specs. The mechanism is
ready; the `_SPECS` dict just needs another entry.

## 0.10.9 — fix: drop mjpython, use blocking `mujoco.viewer.launch`

User-reported on 0.10.8: `openbricks sim preview --world
wro-2026-elementary` on macOS 26.4.1 (Tahoe) with mujoco 3.8 hangs
in mjpython's `_mjpython_init` with the kernel spamming "Task
policy set failed: 4 ((os/kern) invalid argument)" at ~4 Hz. The
hang reproduces even when invoking `mjpython -m openbricks_sim
preview ...` directly, with no openbricks helper involved.

Root cause: macOS recently tightened `task_policy_set()`
permissions for QoS class promotion, and the version of mjpython
bundled with mujoco 3.8 hasn't caught up. mjpython's GLFW
main-thread handoff never completes, so the viewer thread sits
on `threading.wait()` forever.

The fix sidesteps mjpython entirely. MuJoCo also exposes
`mujoco.viewer.launch(model, data)` — the blocking variant —
which runs MuJoCo's own time loop on the calling thread instead
of returning control to Python. The blocking variant doesn't need
the main-thread handoff that `launch_passive` requires on macOS,
so it works on this user's macOS 26 with plain Python (no
mjpython wrapper). User confirmed `mujoco.viewer.launch(model,
data)` opens a window cleanly on their box.

The 0.10.7 mjpython auto-relaunch helper, the 0.10.7 viewer
manual-stepping loop in `cmd_preview`, and the corresponding
manual-stepping loop in `SimRobot.run_viewer` are all removed.
`cmd_preview` now just calls `mujoco.viewer.launch(model, data)`.
`SimRobot.run_viewer` does the same. The `until` callback param
on `run_viewer` is kept for backwards-compat in the signature
but is now ignored — blocking `launch` doesn't return control
until the user closes the window, so a Python-side predicate
couldn't end it early. Scripts that need predicate-driven
termination should use `run_for` / `run_until` instead.

Tests removed: the four `RelaunchUnderMjpythonTests` from
`test_sim_cli.py` are gone (they covered the now-deleted helper).
Tests on `cmd_preview --headless` and on `run --viewer` still
gate the surrounding behaviour; the actual GUI-launch is
manually exercised on macOS / Linux / Windows.

Net code change is a simplification: ~70 lines deleted, ~5 added.
The 0.10.7 + 0.10.9 (closed PR #94) work was in the wrong
direction; this is the right fix.

## 0.10.8 — Phase F1: high-fidelity WRO 2026 mat textures (150 dpi)

The three WRO 2026 mats (Elementary, Junior, Senior) were shipped
as 2048×991 PNGs — about 22 dpi for a 2362×1143 mm mat. That left
zone boundaries soft and made the Phase E1 colour-sensor
texture-sampling pipeline read averaged colours instead of crisp
printed pixels.

0.10.8 replaces them with 13949×6750 PNGs rasterized at 150 dpi
(0.169 mm/pixel — ~7× finer than the TCS34725's physical sampling
spot, so the chassis colour sensor reads actual printed pixels).
Source is the official "Game Mat Printing File" PDF that WRO
publishes — vector PDF, 2362×1143 mm at 1:1, rendered with
`pdftoppm -r 150`.

Wheel size goes from ~3 MB to ~17 MB because the three mats add
8.6 + 4.1 + 5.1 = ~18 MB of PNG. That's the cost of pixel-level
fidelity. If you don't need the WRO worlds at all and want a
slim install, the future direction is on-demand asset download
via `openbricks sim assets fetch <world>` — not in this release;
defer until someone asks.

The mats are not committed as the source PDFs (license + size).
`scripts/regen-wro-mat-textures.sh` re-fetches the PDFs from
`wro-association.org` and re-rasterizes — run it whenever WRO
publishes a mat-fix mid-season, or when adjusting DPI.

## 0.10.7 — fix: auto-relaunch under mjpython on macOS for the GUI viewer

`openbricks sim preview --world wro-2026-elementary` on macOS
crashed with a Python stack trace ending in:

    RuntimeError: `launch_passive` requires that the Python script
    be run under `mjpython` on macOS

MuJoCo's interactive viewer needs to control the main thread for
OpenGL on macOS — Python's REPL doesn't give it that, hence the
`mjpython` wrapper that ships alongside `mujoco`. Without an opt-in,
every macOS user hit the crash.

The fix is a small re-exec helper. When the sim CLI on macOS is
asked to open the GUI viewer (`preview` without `--headless`, or
`run --viewer`), it locates the venv's bundled `mjpython` and
`os.execv`s into it before any model loading happens. Inside
mjpython, the viewer call works. No-op on Linux / Windows / when
already under mjpython / when mjpython isn't in the venv.

Workarounds that worked previously:

```
openbricks sim preview --world wro-2026-elementary --headless --duration 5
~/.local/pipx/venvs/openbricks/bin/mjpython -m openbricks_sim preview --world wro-2026-elementary
```

…still work, of course. After 0.10.7, the bare invocation also
works on macOS.

Tests in `test_sim_cli.py::RelaunchUnderMjpythonTests` pin: no-op on
Linux, no-op when already under mjpython, no-op when mjpython is
missing from the venv (let upstream raise), and re-exec strips the
leading `sim` keyword from `sys.argv` before forwarding (the CLI
sees `sim preview ...` because `openbricks` dispatched it; the sim's
own argv parser sees `preview ...`).

## 0.10.6 — fix: bundle world XMLs into the wheel

Every wheel published 0.10.3 → 0.10.5 shipped without the WRO and
practice-world MJCFs. End users hit
`WorldLoadError: world file not found: wro-2026-elementary` even
though `openbricks sim --help` listed the alias.

Root cause: the `worlds/` directory lived at
`tools/openbricks/worlds/` — outside the `openbricks_sim` Python
package — so `[tool.setuptools.packages.find]` (which only catches
`*.py` files inside the package) didn't pick it up. `MANIFEST.in`'s
`recursive-include worlds *.xml` seeded the *sdist* but not the
wheel. CI's smoke-tests passed because they ran from an editable
install (`pip install -e ".[sim,dev]"`), where the worlds exist
next to the package on the developer's disk.

The fix:

  * `worlds/` moved inside the package: now at
    `tools/openbricks/openbricks_sim/worlds/` (10 files: 5 worlds
    × `world.xml` + per-world `README.md` + the 3 WRO `mat.png`
    textures).
  * `[tool.setuptools.package-data]` added to `pyproject.toml` so
    the XMLs / PNGs / MDs ride along into the wheel.
  * `_resolve_world()` in both `cli.py` and `robot.py` now uses
    `Path(__file__).parent` (the installed package root) instead
    of `parent.parent` (the now-non-existent sibling sit).
  * `MANIFEST.in` no longer needs the `recursive-include worlds`
    line — package-data covers both sdist and wheel.

Regression test in `tests/test_wheel_bundles_worlds.py` builds a
fresh wheel and asserts `world.xml` for every alias plus `mat.png`
for the textured WRO worlds. Same gating shape as
`test_sdist_build.py`.

If you installed 0.10.3, 0.10.4, or 0.10.5, `pip install --upgrade
openbricks[sim]` to 0.10.6 — the alias resolution will start
working without any code changes on your end. (Also: 0.10.5 is the
last release with this bug; we'd recommend `pip install
'openbricks[sim]==0.10.6'` to avoid resolver pickup of the broken
versions.)

## 0.10.5 — sim: practice-zones and practice-walls learning worlds

Two small worlds for users iterating on sim code without wrestling
with a full WRO mat:

  * **practice-zones** — 1.5 × 1.5 m white floor with red / green /
    blue 150 × 150 mm slabs in a triangle around the chassis spawn.
    Pair with the colour sensor and `robot.chassis_in_box(...)` for
    "drive to the X zone" missions. The Phase E1 surface dispatch
    routes the colour-sensor raycast on the slabs' solid `rgba` to
    the material-rgba path, so reads are deterministic.
  * **practice-walls** — 2 × 2 m grey floor with three perpendicular
    walls forming a corridor open on +X. Pair with the distance
    sensor (HC-SR04 / VL53L0X / VL53L1X) for obstacle-avoidance
    practice.

Both registered in the CLI + `SimRobot` world-alias maps as
`practice-zones` and `practice-walls`. Each ships with a `README.md`
documenting layout + suggested missions.

```
openbricks sim preview --world practice-zones
openbricks sim run --world practice-walls my_avoidance.py
```

CI smoke-tests both worlds headlessly on every push, alongside the
existing three WRO worlds.

The 0.10.4 release was prepared but not tagged; this version
collapses Phase E1 + practice-worlds into a single PyPI release.
The 0.10.4 changelog entry below describes the colour-sensor
texture-sampling work that 0.10.5 also includes.

## 0.10.4 — sim Phase E1: pixel-accurate colour-sensor texture sampling

`SimColorSensor` now samples the actual texture pixel at the chassis
position when driving over a textured plane (the WRO-mat use case),
instead of returning the material's flat tint. The 0.10.x sensor
saw a single colour everywhere on a printed mat; 0.10.4 reads the
real printed pattern.

Implementation is CPU-side: the sensor reads `model.tex_data`
directly, computes UV from the geom-local hit point, and indexes
the texel. No GL context, no offscreen rendering, no platform
divergence — works on macOS / Linux / Windows identically. The
phase was originally scoped as "Linux EGL headless rendering," but
EGL is only required for scenes with shadows / lighting / overlays;
for a flat printed mat the texel itself IS the answer.

Surface-colour resolution in `SimColorSensor` now dispatches on
what the ray hit:

  * Textured plane geom — sample the texture at the (u, v) of the
    world hit point (the WRO mat path).
  * Untextured material — material's `rgba` (the previous behaviour
    on solid-colour floors, kept unchanged).
  * No material — geom's own `rgba`.

Six new tests in `test_color_sensor.py::TexturedPlaneSamplingTests`
pin the four-quadrant checker mapping and `texrepeat` re-tiling.
All 246 existing host tests untouched.

Future work — full MuJoCo offscreen rendering with `MUJOCO_GL=egl`
on Linux — only becomes relevant when sim scenes grow more complex
than a printed mat (3D coloured obstacles casting shadows on the
sensor's view, translucent overlays). Not yet prioritised.

## 0.10.3 — manylinux + macOS + Windows binary wheels

`pip install openbricks` now downloads a prebuilt binary wheel
instead of recompiling `openbricks_sim._native` on every machine.
The 0.10.2 sdist-only fallback stays in place for platforms outside
the matrix below.

Wheel matrix:

| Platform | Tag | Built on |
|---|---|---|
| Linux x86_64 | `manylinux_2_28_x86_64` | `ubuntu-latest` via cibuildwheel |
| macOS Intel + Apple Silicon | `macosx_*_universal2` | `macos-latest` |
| Windows AMD64 | `win_amd64` | `windows-latest` |

Each platform ships wheels for CPython 3.9 / 3.10 / 3.11 / 3.12 / 3.13.
Skipped (no asks; revisit when requested): musllinux (Alpine),
linux aarch64, linux i686, win32, PyPy.

CI gate: the `build-openbricks-wheels` job runs on every push / PR,
so wheel-build breakage is caught at review time rather than at
release time. Same pattern as the existing `build-openbricks-sdist`
regression check (`tests/test_sdist_build.py`).

## 0.10.2 — sdist-only publish (PyPI manylinux requirement)

The 0.10.1 publish failed at the upload step:

    400 Bad Request: Binary wheel
    'openbricks-0.10.1-cp311-cp311-linux_x86_64.whl' has an
    unsupported platform tag 'linux_x86_64'.

PyPI requires Linux wheels to be tagged ``manylinux*``, built inside
the ``quay.io/pypa/manylinux2014`` Docker image (or via
``cibuildwheel``). The CI's ``python -m build`` on a stock Ubuntu
runner produces ``linux_x86_64`` wheels, which PyPI rejects.

Quick fix: publish ``--sdist`` only. Users ``pip install openbricks``
get the source distribution and compile the native extension on
first install (slower, but gcc + python headers are typical on
Linux/macOS dev machines). Multi-platform manylinux wheels are a
follow-up.

No functional changes vs. 0.10.0 / 0.10.1 — same code, just shippable.

## 0.10.1 — sdist build fix (no functional changes)

Fixes a build failure that prevented 0.10.0 from publishing:
`python -m build` runs `setup.py` inside an isolated environment
that can't reach `../../native/user_c_modules/openbricks/`. The
sdist now bundles the synced cores via `MANIFEST.in`, and
`_sync_cores()` falls through cleanly when the upstream isn't
available (sdist build context). Regression test in
`tests/test_sdist_build.py` builds a fresh sdist and asserts every
shared core is inside.

If you saw a 0.10.0 install fail, install 0.10.1 instead — it's the
same code, just shippable.

## 0.10.0 — unification

The host CLI (`openbricks-dev`) and the MuJoCo simulator
(`openbricks-sim`) are now a single `openbricks` PyPI package with
optional `[sim]` extra. One install, one console script, every
subcommand under `openbricks`.

### New install commands

```
pip install openbricks            # CLI only (lightweight)
pip install 'openbricks[sim]'     # CLI + MuJoCo physics
```

### Breaking changes

- **Console script renamed.** `openbricks-dev <subcmd>` →
  `openbricks <subcmd>`. The old binary is gone in 0.10.0.
- **Sim subcommand.** `openbricks-sim <subcmd>` → `openbricks sim
  <subcmd>`. Same arguments, dispatched via passthrough to the sim's
  CLI when the `[sim]` extra is installed.
- **PyPI distribution names.** Users on `openbricks-dev` and
  `openbricks-sim` should `pip uninstall openbricks-dev openbricks-sim`
  and `pip install 'openbricks[sim]'`.
- **Release-tag namespace.** `openbricks-dev/v*` → `openbricks/v*`.
  Firmware tags `v*` are unchanged.

### What did NOT change

- The Python module names: `openbricks_dev` and `openbricks_sim`
  stay as they are. The firmware-side `openbricks` package on the
  hub (drivers, robotics, native cores) is also imported on the host
  by the sim's driver shim — collapsing the host CLI's module name
  to `openbricks` would shadow it.
- Subcommand argument grammars: `flash` / `list` / `run` / `upload` /
  `stop` / `log` / `sim {preview,run}` accept the same flags as before.
- The codecov flag names (`openbricks-dev`, `openbricks-sim`) — the
  CI workflow uploads coverage under both flags from the unified job
  so the dashboard split stays meaningful.

### Migration guide

```bash
# Uninstall the legacy split packages.
pip uninstall openbricks-dev openbricks-sim

# Install the unified package. Drop the [sim] if you only need flash + run.
pip install 'openbricks[sim]'

# Update your shell aliases / scripts:
#   openbricks-dev flash …  →  openbricks flash …
#   openbricks-sim run …    →  openbricks sim run …
```

The legacy `openbricks-dev` PyPI project is frozen at 0.9.x — its
last release will carry a deprecation notice in the description. No
new versions will be published under that name.
