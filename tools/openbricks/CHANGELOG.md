# openbricks — host-tooling changelog

Versions the unified `openbricks` PyPI package (CLI + MuJoCo sim).
Firmware versions are tracked separately on the `v*` tag namespace.

## 1.66.2 — QTR refuses non-ADC pins by name; final bench pin map

Wiring the bench exposed the gap: `pins.check` knows the S3's GPIO
map but not its ANALOG map, so five channels soldered to GPIO 38-42
(no ADC at all on the S3) would have failed at runtime with a bare
`ValueError` from `machine.ADC`. The driver now refuses at
construction, naming the pin and the rule: ADC1 = GPIO 1-10 only
(GPIO 11-20 is ADC2 — radio-shared and errata-flaky, called out
separately; classic ESP32: GPIO 32-39).

Examples move to the final bench map — ADC1 also hosts the
start/stop button (GPIO 4) and servo-bus RX (GPIO 6), so the line
cluster is GPIO 1,2,3,7,8,9,10 and the branch flag takes GPIO 5,
with the BLE button moving to a non-ADC pin (GPIO 38).

## 1.66.1 — QTR examples match the real bench wiring; QTRChannel

The bench array is wired as TWO instruments, and the driver now
models that: QTRX channels 15..9 (adjacent, 4 mm pitch) are the
line cluster on ADC1 GPIO 1,2,3,4,5,7,8 (GPIO 6 belongs to the
servo bus), and channel 1 — far right — is a branch/marker flag on
GPIO 9. New `QTRChannel`: one element with the array's
calibrate/read contract plus `value()`/`dark()`, kept OUT of the
steering centroid on purpose — a marker under a flag element must
not yank the position. `position()` on a single element raises and
points there.

With a ~20 mm line on the 24 mm 7-channel window, normal following
already darkens ~5 elements, so the follower's intersection
threshold is ALL 7 — and the probe now prints the branch flag next
to the bars.

## 1.66.0 — QTR/QTRX reflectance-array driver

New `openbricks.drivers.qtr.QTRArray` for Pololu QTR/QTRX analog
arrays (bench: QTRX-HD-15A, 9 channels on ADC1). Unlike a pair of
colour sensors, the array reports a CONTINUOUS line position — a
weighted centroid in millimetres — so a follower steers on a real
analog error instead of edge-crossings, and a lost line still leaves
a recovery direction (`last_side`). `dark_count` is the
intersection/stop-bar signal.

Calibration is mandatory and loud: reading before `calibrate()`
raises (an uncalibrated centroid is a plausible-looking wrong
number), and a channel that never sees line/mat contrast during the
sweep is named by index — that is the unwired one.

`examples/qtr_probe.py` is the bring-up tool (bar display, position,
dark count — verifies wiring and left/right orientation before any
control), and `examples/qtr_line_follow.py` is the array follower:
PD on millimetre error, intersection stop, spin-in-place
calibration, lost-line recovery.

## 1.65.2 — review cleanups: CLI, sim, and lock hygiene

* `openbricks docs` extraction is atomic (scratch dir + rename): an
  interrupted first extraction used to leave a half-manual that
  every later invocation reused forever, because `index.html` — the
  only completeness check — sits at entry 54 of 64 in the archive.
  File URLs are built with `pathlib.as_uri()`, fixing backslash
  `file://` URLs on Windows.
* Sim: a physically blocked wheel's `run_angle` now reports and
  continues after a firmware-style budget instead of hanging the
  suite until CI's timeout; a colour sensor on an unmapped mux
  channel raises instead of silently impersonating the centre
  camera; `shim.uninstall()` evicts `openbricks.*` modules imported
  under the fake `machine`, closing an order-dependent test poison;
  the line-follow suite can no longer leak an installed shim when
  its setUp fails.
* Firmware: every deadline uses wrap-safe `time.ticks_add` (raw
  `ticks_ms() + n` misbehaves at the 2^30 ms wrap, ~12 days of
  uptime); every `st_bus` binding hoists MicroPython conversions
  outside the bus spinlock (a raising conversion longjmped past the
  release and wedged core 0's timer task until power-cycle);
  `db_straight`/`db_turn` on an unconfigured drivebase raise instead
  of indexing `st_moves[-1]`, and `db_config` validates its slot
  pair.
* The offline-docs drift check also compares the docs/ source
  fingerprint recorded at build time, so EDITING a page without
  rebuilding the bundle now fails CI (the page-set check only caught
  additions).

## 1.65.1 — six edges from the review, closed

* `drive()` now supersedes an in-flight move like every other motion
  verb — it was the one that didn't, so a still-running `straight()`
  overwrote its speeds ~1000×/s and a later `done()` poll dispatched
  the stale move's end-state on top. On serial wheels it also ships
  both setpoints in one sync-write now.
* A failed DriveBase adoption (dead wheel, slot exhaustion — designed
  raises) no longer strands the motors: the MicroPython bus it had
  already taken is restored, so plain motor commands still work after
  catching the error.
* Position-mode `ST3215`/`ST3032` servos are refused loudly on a
  UART the native driver owns — both directions (constructed after
  adoption, or present before it) — instead of silently joining the
  two-drivers-one-wire fault or being left talking into a closed
  UART. The remedy (second UART, or the Motor class) is in the error.
* Per-slot `run_angle`/`hold` gets the drivebase's dead-feedback
  guard in C: a wheel whose feedback dies mid-move is halted at the
  same ~200 ms threshold, instead of ramping open-loop on frozen
  odometry until Python's ~1 s stall detector noticed.
* `read()` and `ping()` verify sender and checksum like write ACKs —
  a wrong-sender or corrupt reply read as data could falsely park a
  step or corrupt multi-turn odometry; "any 6 bytes" made ping
  report a present servo that wasn't.
* Fault evidence has the right lifetime: a button stop no longer
  erases a latched dead-wheel diagnosis (post-mortem `db_fault()`
  read healthy), and a program boundary now does clear it (the next
  program's first read reported the previous run's fault).

## 1.65.0 — a dead bus is never called a jam, and polling finds stalls

Firmware-side; the host package rides the lockstep version.

1.62.0 taught `run_angle` to report which failure a stall was — but a
servo that goes bus-silent freezes its odometry exactly like a jam,
so the reporter told users "the shaft is jammed, or torque is not
reaching it" about an unplugged wire, then carried on. Every stall
path now checks the feedback counters FIRST: consecutive silence
(~200 ms, the C fault latch's own threshold) raises a wiring-fault
error naming the servo, regardless of `raise_on_stall` — a wiring
fault is not survivable, only mechanical stalls are.

`run_angle(wait=False)` had no detection at all: the documented
`while not m.done():` loop polled a jammed or unplugged motor
forever. `done()` now runs the same progress-idle watch as the
blocking path on both the adopted and MicroPython-bus paths —
stall → stop the wheel, report (or raise), and end the move; bus
silence → raise.

And two silent losses on the MicroPython-bus path are loud now: a
servo that fails the pre-move probe raises instead of returning as
if the move completed, and a step that never parks reports a stall
and returns False instead of returning True — a stalled motor no
longer reports a fully successful move.
## 1.64.4 — Ctrl-C stops the robot verifiably, and says so

1.61.2 made Ctrl-C forward an interrupt to the hub — but exactly one,
unverified, with every failure swallowed. The hub can eat a single
injected interrupt (a scheduled callback catches it — the same
disease the raw-REPL entry retries 6× for), the send itself can fail
with the robot out of BLE range, and a second Ctrl-C abandoned the
whole stop mid-flight (`KeyboardInterrupt` is a `BaseException`; the
`except Exception` guards never saw it). In all three cases the CLI
printed "aborted." and exited — indistinguishable from a successful
stop, while the robot kept driving.

The stop is now the same verified, retried primitive the connection
path trusts: re-enter the raw REPL and wait for its banner, which is
proof the program died. A second (or third) Ctrl-C restarts the stop
instead of killing it. The outcome is printed either way — "robot
stopped." or a warning naming what failed and what to do ("press its
hub button or cut power"). The idle-loop restore gets the same
treatment: absorbed repeat interrupts, and a named warning ("button
may not start programs until power-cycled") instead of a silent
`pass`. Dropping the old post-interrupt drain also removes a path
that could hang the exit for 30 s mis-framing the output stream.
## 1.64.3 — the sim bus stops lying about pose, slots, and motor count

Three fidelity gaps between the emulated `st_bus` and the firmware
one, all sim-side (the firmware is unchanged beyond the lockstep
version):

**Pose.** The shim fed bridge odometry to the drivebase controller
only while the drivebase was WRITING; the firmware syncs it every
hard tick. So `straight(50)` after two seconds of `move_wheels()`
armed against the pose from before the yielded stretch and drove the
chassis **backward 142.6 mm** — verified by execution, now a pinned
regression test. `RawDriveBase` grew the firmware's yielded-tick
half (`sync()`); the shim calls it every tick and at every arm.

**Slots.** `servo_attach` never marked a slot in use, so the
engine's first-free-slot loop handed BOTH wheels slot 0. Occupied
slots now refuse and `servo_slot_of` exists, like the firmware.

**Motor count.** The sim refused a third motor; the firmware bus has
four slots and the bench robot is four ST-3032s on one UART. The
third and fourth constructed motors now bind kinematic task-motor
slots — no MuJoCo body, the shaft integrates its commanded speed —
so a real four-servo `main.py` constructs and runs end-to-end.
Encoder motors still get only the two physical slots, wheels must be
the first two constructed (the sim binds by construction order, not
servo id), and adopting a kinematic stand-in into a DriveBase is
refused loudly.

## 1.64.1 — stop-then-run drives; a coasted motor goes cold

Two coast-path firmware bugs, no host-side changes beyond the
lockstep version.

`stop()` followed quickly by `run()` left the motor limp with a live
goal speed: the pending torque-off (staged, not yet on the wire)
survived the new speed command — `set_speed` only staged torque when
the WIRE said off, and the pump ships torque first. The drivebase
never noticed (it re-stages every tick); one-shot task-motor calls
had no second chance. A pending coast is now superseded.

And `stop()` (coast) left the old target speed in the slot, which
the pump's heat heuristic reads as "commanded to keep turning" — so
a coasted-and-parked task motor kept a full share of the feedback
rotation forever, re-creating the odometry-bandwidth regression
1.59.x fixed, for coast but not brake. Coast now parks the slot.

## 1.64.0 — the C bus verifies every single-servo write ACK

Firmware-side fix; the host package rides the lockstep version and
its sim shim gains the matching `servo_write_stats` surface.

The native bus collected each write's 6-byte status reply but, when
it never arrived, counted the write OK — justified by a comment
claiming the Python driver "discards the status without caring",
which has been false since 1.56.0. A lost config write could mark a
servo "configured" while it was still in position mode, silently
receiving speed sync-writes. The servo's error flags (overload,
over-heat, EEPROM lock) were never examined at all.

Now: a status that never comes is a TIMEOUT; a status carrying error
flags fails the write (`SERVO_ERR`); a wrong-sender or corrupt
status was already refused. Config steps advance only on a verified
ACK — a lost write reissues the same register, and a servo that
fails 8 in a row is latched `config_failed` so its retries stop
hogging the bus. The latch feeds the drivebase fault path (an
unplugged wheel faults immediately — its stale counter can never
climb because unconfigured slots are never polled) and construction
errors now name the loss: "N configuration writes went
unacknowledged", instead of misblaming the pump.

## 1.63.0 — `openbricks docs` shows the whole manual, offline

The CLI shipped the nine hand-written guides and re-rendered them
with a markdown library, so everything generated from docstrings —
the entire API reference — was simply absent. Both output paths had
the hole: the terminal renderer and the browser view shared the same
bundled markdown, and the browser one never opened the website
despite its own footer pointing there.

It now ships the SAME Sphinx build as docs.openbricks.dev as a
single archive and opens that. Parity is structural: same sources,
same extensions, same autodoc, so the two cannot drift.

**304 KB**, from 19 MB of raw Sphinx output. Most of that was never
real content — ~6 MB of `.doctrees` build cache, and the theme
shipping Lato and FontAwesome in five formats across two directories.
`docs/_static/custom.css` now sets a system font stack, so the fonts
are dropped at the source rather than deleted afterwards and the
build produces exactly what ships. Only the typeface differs from
the website; layout, structure and every cross-reference are
identical.

Verified genuinely offline: no remote stylesheets, scripts or fonts,
pinned by a test. The remote URLs that remain are ordinary links you
click (GitHub, python.org intersphinx), not resources the page loads.

`--text` is gone with the markdown renderer. Topics still work and
now include API pages — `openbricks docs robotics` opens the
DriveBase reference.

The archive is committed and byte-reproducible, so a wheel build
needs no Sphinx; the docs workflow rebuilds and diffs it, failing if
it goes stale.

## 1.62.0 — a stalled run_angle reports, and says which failure it was

**A stalled task motor no longer aborts the run.** On a mission
robot, one stuck arm should not end the mission — so `run_angle`
now REPORTS a stall instead of raising, and says so three ways:
the console sees it, the run log keeps it (`log.note`, so a stall
that scrolled past is still there afterwards), and the call returns
`False`. `raise_on_stall=True` at construction restores the old
fatal behaviour.

This also aligns the two paths: the non-adopted `run_angle` has
always returned quietly when a step failed to park. Both now return
`True` when the target was reached and `False` when they gave up.

**It also gives up on stillness, not on a stopwatch.** The old rule
was a fixed budget (4x the ideal move time, so 4233 ms for the move
that prompted this). That is both too slow on a real jam and too
harsh on a move fighting a heavy load. A move now ends when the
shaft has not advanced a count for **1 second** — configurable via
`stall_idle_ms`. A loaded move that keeps inching is left alone; a
still one is caught in a second. The total budget stays as a
backstop for a move that creeps forever without arriving, and the
report says which rule ended it.

### And it says WHICH failure it was

`run_angle did not reach target within 4233 ms — wheel stalled,
blocked, or in overload protection` named three faults with three
different fixes and left you to guess. The slot has reported
travel, speed and load since 1.50.0, so it can simply say.

How far the shaft got separates them:

* **moved ~nothing** — jammed, or torque never reached it
* **moved partway** — stalled under load, or the servo's overload
  protection cut in (trips above ~80% of stall torque held ~2 s,
  clears when a new command is issued)
* **moved essentially all of it** — arriving but never *latching*,
  which is an arrival-tolerance or odometry problem and explicitly
  NOT a mechanical one

The message now carries the diagnosis, the travel ("moved 45.0 deg
of the 90.0 asked"), and the speed and load the servo reported at
the moment it gave up.

## 1.61.2 — Ctrl-C stops the robot, not just the CLI

Reported from the bench: pressing Ctrl-C during `openbricks run`
exited the CLI while the robot kept driving.

`run` already had a handler to forward the interrupt to the hub
before dropping the link — but it caught `asyncio.CancelledError`,
and a host Ctrl-C raises **`KeyboardInterrupt`**. `asyncio.run` does
not convert SIGINT to a cancellation; it raises KeyboardInterrupt at
the await point. So the forwarding never ran: the CLI printed
"aborted.", closed the BLE link, and left the program executing on
the hub.

That is the worst possible response to someone reaching for Ctrl-C,
which on a moving robot is usually a reach for the brakes.

The handler now catches both. The hub gets the same `\x03` that
`openbricks stop` sends, over the link that is already open — so the
stop is immediate rather than waiting on a reconnect.

## 1.61.1 — the SyncServoGroup refusal no longer depends on run order

Reported from the bench: *"run second time still this error, first
time ok"*. The first run was not ok — it was the silent failure.

1.58.0 refused a `SyncServoGroup` built over motors already on
native slots. But adoption can happen *after* construction: build
the group, and a `DriveBase` adopts the same motors a line later.
Construction passed, and the group was left holding wheels it could
no longer reach — writing into a contended bus exactly as before the
guard existed. On a fresh boot the motors are still on the
MicroPython bus at construction, so that is the path a first run
took; on any later run the UART is already native and the group was
refused at construction. Same script, two behaviours, depending on
whether the hub had been used since power-up.

The check now runs before every write as well as at construction, so
the refusal is deterministic. A group that becomes unusable raises
the moment it is next used, with the same message naming
`DriveBase.move_wheels`.

## 1.61.0 — the line-follower runs in CI

`examples/line_follow.py`'s control law now drives a simulated
chassis down a simulated line on every pull request. Measured on the
new `practice-line` world: **0.94 m advanced, worst deviation 6.3 mm
from a 20 mm line, intersection detected and stopped.**

The world is built from slabs rather than a texture on purpose — the
colour sensor resolves an untextured geom from its `rgba`, so it
needs no PNG and stays byte-identical across platforms. A control
test that fails should mean the control law changed, not that an
image decoded differently. It carries the two cases that matter: a
stop bar wide enough to darken both sensors (intersection), and a
branch stub that darkens only one (must be ignored, not steered
toward).

The closed-loop test lives in the host suite, not the smoke step.
Smoke answers "does the CLI start and does each world parse" in a
fraction of a second per entry; a physics control loop is seconds
long, can sit near a tolerance, and wants a trajectory in its
failure output rather than an exit code. Smoke gains one line — the
new world loads — which is the check that would catch the two
cameras silently regressing to one.

Also fixes a drift hazard found while doing it: `cli.py` and
`robot.py` each carried a world-alias table, and registering a world
in one but not the other loads from the CLI and fails from
`SimRobot`. A test now pins that the two tables match and that every
aliased file exists.

## 1.60.0 — the sim can see a line with two sensors

The simulated TCS34725 already sampled the real `mat.png` texel
under it — printed WRO lines are genuinely visible, CPU-side, no GL
context, headless on CI. But the chassis had exactly one downward
camera and every `ShimTCS34725` bound to it, so a pair of sensors
read the *same point*. Line-following is entirely the DIFFERENCE
between two sensors straddling a line, so that error was identically
zero and no control law could work in simulation.

The chassis now carries a left/right camera pair either side of the
centre line, and a shim sensor picks its camera by **mux channel** —
the same construction that selects a physical sensor
(`TCS34725(mux[1])`) selects the camera under it. A sensor built
without a mux still gets the centre camera, so every existing script
and test behaves exactly as before.

Tests pin the thing that was impossible: straddling a line both
sensors read the mat and agree; drifted off it they disagree by a
wide margin; and the sign of that error reverses with the direction
of the drift.

This is the groundwork for running a line-follow in CI. Worth being
clear about what such a test would and would not prove: it catches
control-law and integration regressions, not the bus-contention,
slot-assignment and program-boundary bugs that actually dominated
the 1.56–1.59 series — the sim emulates the bus surface rather than
reproducing it, so those are invisible there by construction.

## 1.59.1 — a newly attached slot is no longer starved of its first read

Regression in 1.59.0, and it landed on the worst possible moment:
while a script is still constructing its motors.

The weighted scheduler prefers driving slots seven times in eight.
When NOTHING is driving — exactly the state during construction —
that preference found no match and fell through to "any available",
scanned from the *hot* cursor. But a cold read advances the *cold*
cursor, so the hot one never moved and the fallback returned the
same slot every time. One slot took the large majority of reads and
the rest were starved; a slot could get literally zero.

Since a motor cannot be used until its first feedback read lands,
that failed construction outright: *"servo id 2 (native slot 2): the
bus pump never polled it — 0 reads ATTEMPTED"*.

The fallback now asks for the other kind explicitly, so the rotation
is always driven by a cursor that is actually advancing. Pinned by a
test that fails against the unfixed build with a slot on zero reads.

## 1.59.0 — parked motors no longer cost the drivebase its bandwidth

Putting four motors on one bus (1.57.0) had a price nobody had
measured: the pump polled every attached slot round-robin, so two
parked task motors took exactly as much bus as two wheels steering a
heading loop. Per-wheel odometry fell from ~220 Hz to ~110 — and
that rate, not the 1 kHz tick, is what sets the coupled controller's
real bandwidth.

Reads are now weighted by whether a slot is actually driving. A slot
is "hot" when a per-slot move is in flight, when the drivebase owns
it and is running a move, or when it has a non-zero commanded speed.
Hot slots take the bus; parked ones get one turn in eight — enough
that `angle()` and `speed()` never drift arbitrarily stale, little
enough that they cost a driving wheel almost nothing. Wheels keep
roughly 7/8 of the rate they had alone on the bus.

Driving and parked slots rotate on separate cursors, so a parked
motor's occasional turn cannot bias the rotation between the two
wheels — without that they shared unevenly, which would show up as
one wheel's odometry being fresher than the other's inside a
differential controller.

One honest consequence: a dead motor that is *parked* takes
proportionally longer to be noticed, because it is being asked less
often. Detection while driving is unchanged — that slot is hot, and
the runaway guard trips as promptly as before.

## 1.58.1 — the second run of a script adopts its wheels again

Run a script once: fine. Run it again without a power cycle:
`RuntimeError: motor bus not found in the registry`.

The native UART deliberately survives a program boundary — only the
slots are cleared. So on the FIRST run the wheels are built on a
MicroPython bus and adoption hands that UART over; on every run
after, the UART is already native, the wheels go straight onto slots
with no MicroPython bus at all, and adoption went looking for a
registry entry that was never created.

Adoption now recognises wheels that already hold slots: it takes the
wiring from the motor itself (stored at construction, since there is
no bus object to recover it from) and skips the hand-over entirely.
`attach_uart` is idempotent, so re-attaching costs nothing.

Pinned by a test that models the real ordering — a UART already
native at program start, all four motors claiming slots, and the
DriveBase driving the two its wheels actually hold.

## 1.58.0 — SyncServoGroup refuses wheels it cannot drive

The last place two drivers could still meet on one wire.

Adopting wheels into a `DriveBase` hands their UART to the native
driver, but a `SyncServoGroup` built over those same wheels writes
through the MicroPython one. Both talk; each reads the other's
replies. On the bench that surfaced as *"write to servo id 2
acknowledged by servo id 4"* from inside a line-align routine — the
motors were fine, the bus had two owners.

`SyncServoGroup` now refuses at construction if any member is driven
by the native bus, and names the replacement: `DriveBase.move_wheels(
left, right)` for wheels (same one-packet guarantee, from inside the
engine), or driving a task motor directly. Off the native bus it is
unchanged — grippers and multi-axis arms are what it is for.

`examples/line_align.py` is migrated. Its per-wheel independent stop
survives: instead of `left_motor.brake()`, the wheel's entry in a
speed pair goes to zero and both go out together, so the behaviour is
the same and every update is still one packet. Cleanup uses
`db.stop(then="brake")`.

## 1.57.3 — "never asked" and "asked, got silence" are different faults

A slot with 0 replies and 0 failed reads was reported as "not
responding: check power and TX/RX wiring". That is the opposite of
what the counters said. A servo that is unplugged shows failed reads
CLIMBING; a bus pump that never ran shows neither counter moving.

Conflating them cost a bench session hunting a wiring fault that was
really the wedged bus fixed in 1.57.2. The two now read differently:
the wiring advice and `openbricks servo-id --scan` appear only when
the bus actually asked and got silence, and a never-polled slot says
so plainly and calls itself a firmware fault rather than sending
anyone to a screwdriver.

Applies to both the task-motor slot check and the DriveBase's
construction-time wheel check.

## 1.57.2 — the bus no longer wedges at a program boundary

A latent wedge, exposed by 1.57.0 and older than it.

The hard tick pumps right up to the instant a program ends, so the
bus is normally mid-transaction when the next one starts.
`reset_runtime` cleared `tick_txn` — the flag saying *the pump owns
the in-flight transaction* — but left the bus state machine alone.
With the flag cleared, nothing ever consumed that result, so the bus
sat in a non-IDLE state permanently and `servo_pump_locked` returned
early forever: no config writes, no feedback reads, no slot ever
serviced.

`attach_uart` re-initialises the bus, which hid this for as long as
the native bus existed — a program that built its DriveBase first
always recovered by accident. 1.57.0 let a **task motor** reach the
bus first, and nothing un-wedged it.

The signature is distinctive and worth remembering: a slot reporting
**0 replies and 0 failed reads**. A dead servo shows failed reads
climbing; a wedged pump shows neither, because it never ran.

`reset_runtime` now abandons the in-flight transaction and returns
the bus to IDLE — a new program inherits no transaction, the same
way it inherits no slots. Pinned by two tests that both fail against
the unfixed build.

## 1.57.1 — a task motor is usable the moment it is constructed

1.57.0 put task motors on native slots but returned from the
constructor before the slot had any odometry. A slot goes live only
when the pump's round-robin reaches it — config writes go out first
— and the C layer refuses a position move until then, because arming
one against `counts=0` would slam the shaft toward a wrong absolute
target. So a `run_angle` issued immediately after construction died
with *"slot odometry is not live yet"*.

The constructor now waits for the first feedback read, and arms the
hard tick itself (a drivebase does that in its own constructor, but
a task motor may be the only thing on the bus).

The wait doubles as the liveness check the drivebase wheels already
get: a servo that never answers is a wiring / id / power fault,
named at construction with its bus id and the `openbricks servo-id
--scan` remedy, rather than surfacing later as a puzzling refusal.

## 1.57.0 — one bus, one owner: task motors share the DriveBase's UART

Four ST-3032s on one UART — two wheels, two task motors — used to be
impossible, and failed in the worst way: silently. Adopting the
wheels into a DriveBase hands the UART to the native IDF driver, but
a task motor constructed afterwards opened its *own*
`machine.UART` on the same pins. Two drivers on one wire: the hard
tick polls the wheels at ~1 kHz, and those replies land in the
MicroPython driver's buffer where the task motor's next packet
consumes one as its own answer.

1.56.0's write verification is what finally made it visible —
`write to servo id 4 register 0x28 was acknowledged by servo id 1`.
A servo only replies when addressed, so that acknowledgement was
proof of a second conversation on the wire. Before verification the
same interleaving silently corrupted *reads* too.

Now any `ST3215Motor` / `ST3032Motor` on a natively-owned UART takes
a **native slot** instead of opening a competing UART. The bus has
four: slots 0 and 1 stay reserved for the DriveBase's wheels (its
engine claims them by fixed index and detaches whatever is there
first, so a task motor parked in one would be evicted mid-run), and
task motors take 2 and 3. An adopted motor is not a degraded one —
`run_angle`, `hold`, `speed`, `load`, `stalled` and the rest already
route through slots.

Construction order does not matter: a task motor built *before* the
DriveBase is migrated onto a slot when adoption takes the UART, so
it never ends up talking into a closed one. A fifth motor is refused
with the reason and the remedy (a second UART) rather than silently
contending.

Ownership is asked in C (`st_bus.uart_num()`), not remembered in
Python, because the attached UART deliberately survives a program
boundary while `reset_runtime` clears the slots — a fresh program's
Python state would claim nobody owns pins the IDF driver still holds.

## 1.56.1 — don't ground a robot over an acceleration it can't store

1.56.0 verified both goal-speed and goal-acceleration and refused
the move if either disagreed. The bench then showed an ST-3032 that
**acknowledges the goal-acc write and still reports 0** — the ramp
simply isn't settable on that unit. As shipped, 1.56.0 would have
raised on every single `run_angle` there.

The two registers now get the treatment their failure modes deserve:

- **goal-speed stays strict.** 0 means full speed; a mismatch
  refuses the move, because running anyway risks the shaft.
- **goal-acc warns once and proceeds.** Refusing to move at all
  because the servo won't store a *preference* about ramp shape
  grounds a working robot to enforce a nicety. The message says the
  servo doesn't take a ramp and that motion is otherwise unaffected
  — loud, but not fatal.

Also: the speed probe's first theory is dead and the probe now says
so. "Only the first move is fast, because only it writes the EEPROM
angle limits" cannot survive a run where moves 1 *and* 2 were fast —
move 2 writes no EEPROM at all. `examples/st3032_run_angle_speed_probe.py`
now runs three sweeps that separate the variables: same-direction
(move number and position advance together), alternating-direction
(position stays put while move number climbs), and half-speed (does
"fast" mean "unlimited" or is it a scale error?).

## 1.56.0 — the first run_angle no longer ignores its speed limit

Bench report: the same `run_angle(200, -145)` moved at different
speeds on different runs. Measuring it inverted the framing — the
"slow" runs were the correct ones (197 dps against a commanded 200),
and the **first move of each run was 3.5× too fast** (697 dps, near
the servo's no-load ceiling).

Two register defects behind it, both measured, not inferred:

- **`goal_acc` read back 0** on every repetition despite the
  constructor writing 171. A write that definitely happened did not
  stick.
- The **first** move of a run is the only one that writes the EEPROM
  angle-limit registers (`_ensure_step_limits`, guarded by a
  per-*instance* flag). Those registers were already zero from the
  previous run, so every program run spent an EEPROM cycle
  rewriting a value that was already there — and left the servo busy
  immediately before the goal-speed write.

That matters far more than a normal lost write, because on a Feetech
servo **goal-speed 0 means maximum speed**. A dropped speed write
doesn't make a move slightly wrong; it makes it run flat out.

Fixes:

- `_ensure_step_limits` reads before writing and skips the EEPROM
  cycle entirely when the limits are already zero — the common case
  for any servo that has run `run_angle` before. A real write now
  settles for 20 ms before anything else is sent.
- `run_angle` re-asserts `goal_acc` alongside `goal_speed` instead of
  trusting the constructor's write to have survived, and **verifies
  both by reading them back**, retrying once and then raising. A
  servo that won't accept its motion settings is refused loudly
  rather than run at full speed.

**And the general rule, not just the two registers that bit us: a
lost write is never silent again.** `_SCServoBus.write` sent the
packet and threw the servo's status reply away, so a write that
never landed — servo busy finishing an EEPROM cycle, loose
connector, wrong id — was indistinguishable from one that
succeeded. Every write is now confirmed against that reply and
raises on a missing, mis-addressed, corrupt, or error-flagged
acknowledgement, naming the servo, the register and the value.
Broadcasts (id 0xFE) are exempt because the protocol defines no
reply for them, so the e-stop broadcast still cannot raise.

Servos configured with a status-return level that answers reads
only can opt out with `verify_writes=False` — a deliberate choice,
stated in the error message, never a silent fallback.

`examples/st3032_run_angle_speed_probe.py` also now measures actual
shaft travel, so a short elapsed time can be told apart from a
genuinely fast move.

## 1.55.0 — a dead motor raises, and says which motor

Reported from the bench: with a dead motor the drivebase did
nothing — no motion, no exception, no message. Reproducing it found
a second, worse problem behind the silence.

**The runaway.** A silent wheel's odometry freezes at its last
reading, so the coupled controller sees an ever-growing heading
error and winds that wheel's command toward the rail. Reproduced
off-hardware: 8468 steps/s commanded on the silent wheel while the
live one sat at 6. A motor that is alive but merely not *reporting*
— a broken feedback line, the common failure — would take off. The
C tick now stops driving within ~200 ms of a wheel going quiet and
latches which one.

**The diagnosis.** Every layer that could hide the failure now
checks, and every message names the motor:

```text
OSError: motor is not responding on the bus: right wheel (servo id 1,
slot 1) on UART1 tx=14 rx=6 — 0 replies, 137 failed reads (137 in a
row). Check the servo's power and TX/RX wiring, and that it really
has that bus id — `openbricks servo-id --scan` lists the ids actually
answering on the bus.
```

- **At construction** — both wheels must answer at least one
  feedback read before you get a DriveBase. `servo_attach` only
  claims a slot in C; it never asks the servo whether it exists, so
  wiring/id/power faults used to surface as mysterious non-motion
  much later.
- **During moves** — a mid-move fault raises in ~200 ms naming the
  wheel, instead of burning the 8 s settle timeout and blaming
  "stalled, blocked, or gyro frame diverged".
- **`move_wheels` / control loops** — nothing waits on a
  fire-and-forget speed command, so it checks before commanding; in
  a loop the failure lands on the next iteration.
- **`db.check_motors()`** — the same check on demand.

Halting on a fault also latches the controller's `done` flag, so the
wait loop now tests motor health *before* `done` — otherwise a
faulted move would exit reporting success, which is the exact
silent-failure shape being fixed. The settle-timeout message also
carries both wheels' traffic counters now; an asymmetry localises a
mechanical stall.

## 1.54.0 — DriveBase.move_wheels(left, right)

Direct per-wheel speed control, in wheel-deg/s, as a first-class
DriveBase method:

```python
db.move_wheels(200, 120)     # gentle right-hand arc
db.move_wheels(200, -200)    # spin in place
db.stop()
```

Non-blocking and continuous like `drive()`, and it supersedes any
move in flight. Where `drive(speed_mm_s, turn_rate_dps)` speaks
chassis kinematics, this speaks wheels — the right shape for
line-following, tank-style teleop, or any controller computing its
own per-wheel outputs.

This exists so nobody has to build a `SyncServoGroup` over the
drivebase wheels. That was never really an option anyway: adopting
motors into a DriveBase hands their UART to the native bus driver,
so the MicroPython bus a SyncServoGroup writes through is closed.
`move_wheels` gives the same one-packet guarantee from inside the
engine — both setpoints staged in one C critical section, emitted
by the planner in a single sync-write. On encoder servos both
targets are set and both servos subscribed inside one native call;
open-loop pairs are supported but can't batch (documented).

`SyncServoGroup` keeps its place for what it's actually good at —
grippers, multi-axis arms — and its docs now say so.

## 1.53.0 — straight/turn/stop command both wheels together

1.52.0 made `stop` atomic on serial-bus motors; auditing the other
two methods and the other engine found the same shape on the
**encoder-servo** path, where it had been since the native
DriveBase shipped:

- `straight()` / `turn()` subscribed the two servos as two separate
  Python `run_speed(0)` calls before arming, so between them the
  left wheel was already closed-loop holding zero — an active brake
  on a rolling chassis — while the right was untouched. The native
  drivebase now attaches both servos itself inside the same C call
  that arms the move.
- `stop(then=...)` dispatched `left.coast(); right.coast()` from
  Python. Between those two statements the second wheel's 1 kHz
  control tick kept driving at the last commanded speed, so the
  chassis veered on every stop. `DriveBase.stop([mode])` now takes
  the same 0=coast / 1=brake argument the serial engine got in
  1.52.0 and writes both bridges before returning; the no-argument
  form still halts the controller only, which is what `drive()`
  needs.

The e-stop gate that used to ride on those `run_speed(0)` calls is
now checked explicitly at the arm site (pinned by test). The sim's
`ShimDriveBase.stop(mode)` mirrors the surface.

New `tests/test_drivebase_concurrency.py` states the rule for both
engines and pins it structurally — no per-motor Python dispatch
survives on either path — plus the symmetric end state it produces.
Five of its eight tests fail against the pre-fix build.

## 1.52.0 — atomic DriveBase.stop: both wheels, one packet

`DriveBase.stop(then=...)` on serial-bus motors used to finish per
motor: `left.coast()` then `right.coast()`, each its own
single-servo torque write — so one wheel free-wheeled while the
other still drove, a bus transaction apart (holds also captured
the two wheel poses at slightly different instants). The complete
stop is now staged inside ONE C critical section
(`db_stop(mode)`): the planner batches every pending torque
command into a single sync-write (new `OB_SOP_SYNC_TORQUE`, same
Feetech 0x83 instruction the speed path already uses), so
coast releases both wheels at the same packet boundary, brake is
the existing one-packet zero-speed sync, and hold captures both
poses at the same instant before arming the C position holds.
Move starts benefit too: the first command's torque-on now engages
both wheels in one packet.

Torque syncs deliberately skip the sync/read fairness gate —
coast is e-stop-adjacent and must never queue behind feedback
reads; being one-shot, it can't starve them. A `then="hold"`
before slot odometry is live is refused loudly (RuntimeError)
instead of anchoring the holds to counts=0. The sim's emulated
bus mirrors the mode argument, so the same user code exercises
the same contract there.

## 1.51.0 — ICM-45686: heading computed inside the hard tick

The raw-IMU arc, built ahead of the part's arrival so bring-up is a
day, not a week. A seventh build-time patch
(`esp32-openbricks-spi-shim`) wraps the IDF SPI master; the
`icm45686` native module reads the 13-byte accel+gyro burst every
hard tick (~13 µs at 8 MHz) and feeds gyro-Z into the 1.50.x-era
yaw integrator — a gyro `DriveBase` with this IMU corrects heading
at 1 kHz in C with no Python pump at all (`db_gyro_source(1)`,
selected automatically by the `_hard_heading_source` marker).

`ICM45686(sck=, mosi=, miso=, cs=)` — four free GPIOs; the I2C
mux/color-sensor setup is untouched. Register logic derives from
betaflight's and Zephyr's hardware-proven drivers (WHO_AM_I 0xE9
verified at construction, loud OSError otherwise) and is pinned by
injected-transfer c-unit tests before silicon contact. Gyro bias
persists to NVS (`save_calibration()`, auto-seeded next boot — the
pbio trick), so boot-and-immediately-run starts corrected.

BENCH-VERIFY on arrival (marked in code): SPI mode (3 vs 0), burst
byte order, mounting sign — `examples/icm45686_bringup.py` walks
all three with printed evidence, then the gyro square vs the
BNO055's +0.5..+1.8°.

## 1.50.1 — load sign: bit 10 means POSITIVE (bench-pinned)

First live read of the present-load register in the project's
history, both spin directions: the servo sets bit 10 while pushing
in its POSITIVE direction — a forward-driving motor read −160 mNm,
a reverse-driving one +170. Our decode (bit 10 = negative) came
from the Feetech SDK and is inverted relative to the Pybricks
contract we promise (`load()` sign matches `speed()` sign when
driving). Flipped at both decode sites — the widened C read AND the
classic driver's `load()` (which had never been sign-validated on
hardware) — with the test expectations re-pinned to the bench
truth. `stalled()` unaffected (uses the magnitude).

## 1.50.0 — the WHOLE Motor contract works on adopted motors

`speed()`, `load()` and `stalled()` — the last three
`NotImplementedError` gates on motors adopted by a serial
`DriveBase` — now work. The pump's per-slot feedback read widens
from 2 bytes to 6: present-position (0x38), present-speed (0x3A)
and present-load (0x3C) are contiguous, so all three ride ONE
transaction for ~the old wire cost (reply +4 bytes ≈ 40 µs at
1 Mbps) — no extra transactions, no odometry-rate dilution.

Decoded per slot in `st_servo_core` (speed sign-magnitude b15, load
b10 in 0.1%-of-stall units), exposed user-frame (slot invert, same
rule as counts) via `st_bus.servo_feedback(slot)` →
`(speed_steps, load_raw, fresh)`. Staleness is loud: `speed()` /
`load()` return None and `stalled()` raises OSError while the bus
is silent — never a silent 0. `stalled()` keeps the classic
thresholds (load ≥ STALL_LOAD_PCT, |speed| ≤ STALL_SPEED_DPS). The
sim's `_SimStBus` serves live wheel speed (load reads 0 — the shim
wheel model has no torque estimate).

Bench check pending: the ST-3032 answering a 6-byte multi-register
read at 1 Mbps (standard SCS practice; the c-unit suite covers the
decode, the wire harness the transaction shape).

## 1.49.1 — every stop must arm the start gates, not just watcher stops

The 1.48.2 gates checked suppression state (`_lockout_until_ms`,
press lifecycle) that only the WATCHER's stop path armed. A
hard-path stop lands in ~2 ms — before the watcher's next 50 ms tick
— so nothing armed, the gates passed, and the stopping press's
echoes still dispatched a phantom start: the dead-next-session
signature recurred on the 1.49.0 bench with the gates in place.

Fix: `Launcher.note_external_stop()`, called from the program
teardown's `KeyboardInterrupt` handler — EVERY interrupt-unwound
run (hard-path stop, watcher stop, REPL Ctrl-C) now arms the same
lockout, consumes the stopping press's coming release, and drains
both start latches (hard `take_start` + PCNT re-sync). Pinned at
both the unit seam and the `_exec_program` integration seam.

Second gap, same bench session: a hard stop's injection was
ONE-SHOT, and a pending KeyboardInterrupt landing inside a dupterm
stream method is swallowed silently by design (Part 6 — a raising
stream gets deactivated permanently). Under a print storm the probe
caught it verbatim: `hard_stops` incremented, program ran 1.3 s
more until a second press. The watcher's retry covers only ITS
stops. Now the hard tick re-injects every ~100 ms while the stop is
in flight; the teardown's disarm (already the handler's first
statement, guarding against a retry landing mid-cleanup) ends the
flight. At-least-once delivery from the hard side.

Also validated by the same probe stream: the 1.49.0 majority-vote
debounce caught every press including taps — no misses this round.

## 1.49.0 — majority-vote debounce: chatter must not hide presses

The bench run-log cross-check showed the hard sampler missing
1-of-4 real presses outright (hard counters frozen while the
chatter-immune PCNT edge counter caught the same press) and
catching another only after the disarm. The consecutive-stability
rule was the culprit: 20 CONSECUTIVE ms at the new level, so one
sub-ms glitch anywhere in the hold restarts the count — on a
chattery line a press can never accumulate 20 clean samples.

`st_button_core` now votes: pressed when ≥15 of the last 20 one-ms
samples read pressed, released when ≤5 — the 10-sample hysteresis
gap makes mid-hold flicker unable to toggle the state, and a clean
press fires in 15 ms (faster than before). c-unit suite rewritten
around the bench chatter patterns (80%-duty chattery press fires
once; 50% alternating noise never fires; short taps rejected;
sliding-count consistency over 100k samples). `hard_button_probe`'s
fourth field is now the window count (near 20 during a healthy hold;
mid-range = heavy chatter).

## 1.48.2 — the hard start latch must pass the start gates

Closes the "first BLE session after a button stop is dead" mystery,
diagnosed from one stats tuple (presses 2→3 with hard_stops frozen):
the STOPPING press's debounce confirmation lands ~20 ms after the
disarm, so the hard sampler counts it UNARMED and latches
`start_pending` — and the idle loop consumed that latch with NO
gates, phantom-restarting the just-stopped program. A busy hub is
exactly a hub whose next raw-REPL session times out with
notify_count=0. The PCNT path has had the post-stop lockout +
press-lifecycle attribution since Parts 9–12; the hard latch
(1.44.0) bypassed all of it — the Part 12 meta-rule violated by the
newest detector.

Fix: `_start_gate_verdict()` — the shared gate both latches now
pass (post-stop lockout, same-press chatter windows). Swallowed
hard starts announce themselves ("start press ignored (...)"), per
the Part 9 rule. Three new gate tests; the hard-button verification
suite (1.48.1 bench, runs 1–3) stands: instant cuts, hard_stops
counting, zero tracebacks.

## 1.48.1 — the relay must not re-post from a Python frame

1.48.0's bench run: two of three stop presses cut instantly with
`hard_stops` counting — the hard path works. The third exposed the
relay's flaw in its own traceback (`_tick →
_resignal_stop_interrupt: KeyboardInterrupt`): a pending exception
posted FROM Python fires at the poster's very next bytecode, so the
relay re-received its own post and the interrupt escaped the
callback anyway. Self-delivery is unavoidable in any Python frame.

Fix: relay through the architecture that already solves this — the
`request_stop()` flag + the armed C-function `stop_tick` Timer
(20 ms), which runs no Python bytecodes after posting, so the
injection lands in the program. The `resignal_keyboard_interrupt`
binding is removed (an unused footgun after one unreleased-to-bench
day: its semantics ARE the self-delivering post).

## 1.48.0 — the stop interrupt must not be eaten by our own callbacks

The 1.47.2 probe run delivered the diagnosis: the hard-button path
now demonstrably works end to end (press sampled within one tick,
`hard_stops` incremented) — but its `KeyboardInterrupt` is a PENDING
exception the VM raises at the next boundary, whichever Python frame
that is. When the launcher's 50 ms watcher tick (or the BLE-toggle
poll) happened to be executing, the interrupt unwound the CALLBACK:
MicroPython printed the traceback (`launcher._tick → log.pump:
KeyboardInterrupt`) and the program kept running; the stop only
landed when the watcher's retry machinery re-injected it later. The
1.8.12 lesson in a new costume — soft callbacks unwind themselves.

Fix: the relay. `motor_process.resignal_keyboard_interrupt()` (new
binding, all builds) re-posts the interrupt; `launcher._tick` and
`BluetoothToggleButton._on_tick` catch `KeyboardInterrupt` at the
callback top level and call it, so delivery retries until it lands
in the program. Other exceptions still propagate (pinned). Restores
the hard path's bounded-latency promise.

Also confirmed by the probe runs: counters survive sessions and
reset only on power-cycle (1.47.1's idempotent config working), and
real line glitches exist that the 20-tick debounce correctly absorbs
(`raw_count` 10071→66 blip). The 1.47.1 dead-sampling boot never
reproduced across two fresh boots; kept under watch.

## 1.47.2 — hard-button sampler probe

The 1.47.1 in-stream stats settled that the hard path never sees the
press (0 presses at the stop line) while every link visible from
Python is healthy: dispatcher at 500 ticks/500 ms, config True,
`machine.Pin(4)` reads the pad, watcher stops on the same pin. New
`motor_process.hard_button_probe()` exposes the invisible links —
`(pin, ob_gpio_read(pin), raw_last, raw_count, stable_pressed)` —
and the probe example samples it at 100 ms through a press-and-hold.
Distinguishes the two survivors: shim/pad disagreement
(`ob_gpio_read` stuck at 1 during a held press) vs line chatter
defeating the 20-tick debounce (raw flips but `raw_count` keeps
resetting). Diagnostic only; no behavior change.

## 1.47.1 — hard-button evidence must survive to be read

Bench 2026-08-03: the stop button cut a blocking-I2C program cleanly
("stopped by button press"), but `hard_button_stats()` read
`(0, 0, 0, True)` afterwards — so it was impossible to tell whether
the HARD path (core 0) or the retained Python watcher delivered the
stop. Two causes, two fixes:

* `hard_button_config` re-inits zeroed the counters, and the
  launcher's setup re-runs on idle-loop restarts / BLE session
  recovery. Now idempotent: re-configuring the same pin keeps the
  debounce state and counters.
* Post-hoc reads race reboots anyway, so the CLI's composed runner
  now prints the stats tuple in the stop message itself — read
  inside the interrupted run, before anything can zero it.
  `hard_stops > 0` in the stream = the hard path fired.

## 1.47.0 — fix: per-move stops were re-baselining the gyro frame

First bench run of a gyro square on the one-class flow measured
**+7.6°** of drift (vs +0.5..+1.8 on 1.43.2) while the encoder pass
was clean (+0.1°). Root cause: `db_stop` re-captured `turn_hold`
from MEASURED heading — the stale-hold lurch fix — but the 1.45.0
one-class `DriveBase` stops after EVERY move, so the absolute gyro
frame was re-baselined at every segment boundary and each turn
banked its ~+1.9° arrival residual instead of the next move
correcting it (the exact pre-1.25.0 per-move-re-baselining failure
mode; `NativeDriveBase` never stopped between moves, which is why
1.43.x benches never saw it).

Fix (firmware `sb_db_stop` + sim `RawDriveBase.stop`, same rule):
hold capture is now **abort-only** — a mid-move stop still captures
the measured pose (the case the lurch fix was built for; the holds
otherwise carry move-START values there), but after ARRIVAL the
holds keep their end-locked ABSOLUTE targets, preserving the frame
across the per-move stops.

Regression pins: unix-MP `test_st_drivebase` drives six gyro turns
each followed by a stop on a 60%-tracking plant (a perfect plant
arrives with ~zero residual and cannot discriminate — verified: the
unfixed build passes with perfect wheels, lands 108.5/120 with laggy
ones) and asserts the absolute landing; a second test pins that an
ABORTED turn does not haunt the following straight. Sim: a full gyro
square through the shim in MuJoCo must return within 5°.

## 1.46.0 — step mode in C: run_angle/hold on adopted serial motors

The roadmap item the 1.45.0 `NotImplementedError` gates named. Adopted
ST-3215/ST-3032 motors now execute `run_angle`, `hold`, and
`done()`-polled `wait=False` moves on the hard tick:

* **`st_move_core`** — a per-slot position move, drivebase-core
  architecture on one axis: trapezoid trajectory + position-P with
  velocity feedforward, output as wheel-mode speed commands. The
  servo's own STEP-mode registers are deliberately unused (their
  present-position register reads remaining-to-target, which would
  break multi-turn odometry). `done` = profile expired AND |err| <
  34 counts (~3 wheel-deg), latched; after arrival the move keeps
  holding position against disturbance. Runs in the firmware pump,
  in the sim (`RawServoMove` in the CPython extension — same C
  file), and under the ASan/UBSan c-unit suite.
* **Arbitration: the drivebase yields when idle.** The db writes its
  slots' speed targets only from `db_straight`/`db_turn` until
  `db_stop`/`db_disable`; yielded, it keeps syncing odometry but
  stays silent. Consequences: a freshly constructed `DriveBase` no
  longer torques the wheels into a pose-zero hold; after `stop()` the
  motor layer (coast/brake/hold/run_angle/run_speed) genuinely owns
  the wheels. `servo_move`/`servo_hold` are refused while a db move
  is in flight (`RuntimeError` at the driver) and before the slot's
  first feedback read (an early move would anchor to counts=0 and
  slam the shaft).
* **Driver:** `run_angle(deg_per_s, target_angle, wait=, then=)` with
  the classic trapezoid stall budget (ideal ×4 + 1 s → RuntimeError),
  `then="coast"/"brake"/"hold"` end-state dispatch (deferred to
  `done()` for `wait=False`), and `hold()` as a hard-tick position
  lock. `speed`/`load`/`stalled` stay gated (per-slot register reads
  are the tracked follow-up). `stop(then="hold")` on a serial
  `DriveBase` now works — it routes through the motors' new `hold()`.

## 1.45.0 — ONE DriveBase class; the Python serial fallback loop is gone

Three user decisions, shipped together:

* **One drivebase class.** `NativeDriveBase` (public for one release
  window, 1.43.x) is removed. `DriveBase` takes Motor objects and
  transparently *adopts* serial-bus pairs onto the hard-tick native
  engine: `adopt_motors` releases the driver's `machine.UART`
  (explicit ownership handover — no peripheral double-claim) and
  reroutes the motors' wheel-mode API (`run_speed`/`brake`/`coast`/
  `angle`/`reset_angle`) through the engine's servo slots. Step-mode
  methods (`run_angle`, `hold`, …) raise `NotImplementedError` naming
  the step-mode-in-C roadmap.
* **No Python fallback loop.** The pure-Python heading-hold loop
  (the pre-1.42 serial drivebase) is deleted — 310 lines. A runtime
  with serial-bus motors but no native bus raises `RuntimeError` at
  construction instead of silently degrading to a starvable ~100 Hz
  loop. Open-loop motor pairs keep kinematic `drive()`/`stop()`;
  `straight()`/`turn()`/`use_gyro()` raise.
* **The sim emulates the bus, not the class.** `_SimStBus` implements
  the `st_bus` surface over MuJoCo wheels and the sim native
  extension gained `RawDriveBase` (the same C `drivebase_core`
  embedded in the CPython module), so sim, firmware, and unit tests
  all run the SAME engine and controller code path. Sim serial
  squares converge within ~1% of commanded geometry.

Examples updated to the one-class form; two obsolete
fallback-diagnostic examples removed.

## 1.34.0 — fix the actual staging bottleneck: O(n^2) BLE reads

Research (not guesswork this time — MicroPython + NimBLE sources,
Pybricks' protocol, and BLE throughput measurements) found the real
cost, and it was never the window.

MicroPython pulls stdin **one byte per call**: `mp_os_dupterm_notify`
(`extmod/modos.c`) loops `mp_os_dupterm_rx_chr()` until dry, each
doing a 1-byte `readinto` on our stream (`extmod/os_dupterm.c`).
Our `_BLEUART.read` answered every one of those with
`self._rx_buffer = self._rx_buffer[sz:]` — reallocating and copying
the entire remaining buffer **per byte**. Draining N bytes cost
O(N^2) time and N GC allocations. Measured on the MicroPython VM:
**15x slower at 8 KB, 19x at 16 KB** than the index-based version
now shipped (and that is a desktop CPU with a small heap; the ESP32
pays far more with an 8 MB PSRAM heap to collect).

Why it mattered beyond speed: on ESP32 that drain runs inside the
BLE IRQ handler, which MicroPython invokes **on the NimBLE host
task while holding the GIL**
(`MICROPY_PY_BLUETOOTH_USE_SYNC_EVENTS`). A slow drain stalls ATT
processing while the controller keeps delivering, and the 24-buffer
ACL pool starts dropping packets that write-without-response never
retransmits. So a bigger raw-paste window meant more buffered bytes,
quadratically slower drain, and *more* loss — which is precisely why
1.32.0 (window 2048) truncated programs, why 1.32.1 (1024) hung
mid-paste, and why enlarging the GATT rx buffer to 8 KB changed
nothing.

Reads are now index-based with amortised compaction: O(1) per byte,
no allocation per byte, buffer released outright when drained.

**Second fix, same research:** `_nus.py`'s `write()` claimed *"bleak
handles that internally"* about chunking large payloads. It does
not — neither bleak backend splits a write, and its API documents
the payload as bounded by `max_write_without_response_size`. We now
chunk by the negotiated size ourselves. This path fails **silently**
in both directions, which is why it went unnoticed: an ATT Write
Command has no error response by spec (NimBLE discards the return
for command opcodes), and MicroPython truncates a full
characteristic buffer while still returning success (upstream TODO
acknowledges it). At the stock window our bursts never exceeded one
MTU, so this was latent — but 1.31.0's streaming wrote up to 16 KB
in a single call straight into it.

Also documented (see `native/boards/*/mpconfigboard.h`): upstream's
window is not arbitrary. `MICROPY_REPL_STDIN_BUFFER_MAX` (256, so
window 128) is sized so two in-flight windows fit the ESP32 port's
**260-byte `stdin_ringbuf`** — and `mp_os_dupterm_notify` funnels
BLE input straight into that ring, **silently discarding whatever
doesn't fit** (it ignores `ringbuf_put`'s return). No board in
upstream MicroPython raises this value; two nRF boards *lower* it.
Raising it requires the ring, the GATT buffer, AND the drain cost to
move together — which is what `openbricks paste-probe` now measures.

## 1.33.1 — revert the window bump too; ship a probe to measure it

**The window is back to stock. Flash this.** Both raised values
broke real hardware, differently: 2048 (1.32.0) truncated pasted
programs — the hub compiled a fragment, empty stdout AND stderr —
and 1024 (1.32.1) hung mid-paste, the hub silently ceasing to
consume with no flow-control ack. Enlarging the BLE GATT rx buffer
to 8 KB did not fix 1024, so the limit is NOT that buffer alone,
and three desk-reasoned attempts is two too many.

So: stock window (MicroPython's 128), the configuration that has
always worked, plus a new **`openbricks paste-probe -n NAME`** that
measures the hub's real burst limit — pasting padded no-op programs
of increasing size through the actual raw-paste path, reporting the
largest that completes and naming each failure mode (truncated vs
hung). Any future window change must be derived from its output:
two windows can be in flight at once, so
`MICROPY_REPL_STDIN_BUFFER_MAX` is safe only at or below the
measured limit.

Kept: compression + hub-name binding (1.30.0), the enlarged BLE rx
buffer and its guards (1.32.1 — strictly safer, no downside), the
stdin-ring patch (unused at stock, ready for a measured bump), and
board comments recording why not to raise this again blind. Staging
returns to the 1.30.0 timing (~11.8 s for a 7.9 KB script — still
better than the 16.6 s it started at): correctness first, and the
speed work resumes from measurement.

## 1.33.0 — revert streamed staging; the window bump carries it

**1.31.0–1.32.1 could not stage a script over BLE. This restores
the mechanism that provably can.**

1.31.0 moved the payload out of the pasted program and into a
`sys.stdin` read performed by a small receiver *after* the raw
paste ends. On hardware that receiver executes instantly with
empty stdout **and** empty stderr — as if nothing was compiled —
so the host's first payload chunk gets answered by the
end-of-execution `0x04` instead of an ack. It failed identically
on two firmware builds, and 1.32.1's rx-buffer fix (a real bug,
kept) did not change it. The design's "verification" was invalid:
it fed the receiver from a FILE with the payload on a pipe, which
is not the raw-paste path the code actually runs on.

Staging returns to the **1.30.0 embedded form** — payload sealed
(deflate → hub-name-keyed XOR → base64) *inside* the pasted
program, one self-contained unit with no stdin dependency. That is
the version that has completed a real staging on hardware.

Speed comes from the window instead, which is what needed fixing
all along: the same 7.9 KB script is a 5.2 KB program, and at the
1024-byte window (1.32.x, with 1.32.1's buffer fix) that's **6
round trips instead of 41**.

Kept from the detour: the compression + hub-name binding (1.30.0),
the BLE rx-buffer fix and its two regression guards (1.32.1), and
the window bump (1.32.0). Dropped: the stream receiver, its ack
protocol, and `_STAGED_MARKER`. The code now carries a comment
saying not to re-attempt streaming without a way to verify it on
the real BLE raw-paste path.

## 1.32.1 — fix: 1.32.0 broke staging over BLE (undersized rx buffer)

**1.32.0 firmware is broken — reflash to this.** Raising the
raw-paste window to 2048 let the host keep 4096 bytes in flight,
but `ble_repl.py`'s GATT rx buffer was still 512 — a value its own
comment documents as "2 windows + headroom" *for the old 128
window*. NimBLE silently dropped the overflow, the hub compiled a
fragment of the staged receiver, and `openbricks run` failed with
"hub did not confirm the staged chunk" (empty stdout **and** empty
stderr — the signature of a program truncated to nothing). The
1.32.0 ring-buffer test guarded the UART/USB path; BLE has its own
buffer and had no guard.

Three layers, so this class of bug is closed by design:

* BLE GATT rx buffer 512 → **8192** (`_RX_BUFFER_BYTES`).
* Advertised window 2048 → **1024** (`MICROPY_REPL_STDIN_BUFFER_MAX`
  2048) — max in-flight 2048, a 4× margin inside the buffer, still
  8× the stock window.
* The streamed payload is now **ack-paced** in 2 KB chunks (the
  receiver prints `\x06` per chunk), so unread bytes on the hub are
  bounded by protocol rather than by buffer sizing — it cannot
  overflow regardless of link speed.

`BleRxBufferTests` pins rx-buffer ≥ 2× buffer-max in both
directions and was verified to FAIL on the exact 1.32.0
combination. Verified end to end on the MicroPython VM: three acks
then the digest, byte-identical file.

## 1.32.0 — firmware: raw-paste window 128 → 2048

The last window-paced remnants — the ~0.9 KB stream receiver, the
runner paste, every small `-c` snippet, and `mpremote`-based execs
(`openbricks flash`'s name write) — now move 16× faster per round
trip: both boards advertise a 2 KB raw-paste flow-control window
(`MICROPY_REPL_STDIN_BUFFER_MAX 4096`) instead of MicroPython's
stock 128. Safe because the ESP32 stdin ring grows to 8 KB via a
3-line build-time patch (`native/patches/`, applied by
`build_firmware.sh` — the submodule stays pinned upstream) that
makes upstream's hardcoded 260-byte array `#ifndef`-overridable;
a good upstreaming candidate since its default is upstream-
identical. `tests/test_board_config.py::RawPasteWindowTests` pins
the ring ≥ 2× buffer-max invariant that keeps UART/USB pastes from
silently dropping bytes. Firmware reflash required; the CLI adapts
automatically (it reads the advertised window).

## 1.31.0 — staging streams at full link speed (goodbye ack wall)

1.30.0 cut the wire bytes; this cuts the pacing. The 128-byte
raw-paste window (fixed in MicroPython core — the ESP32 stdin ring
is a hardcoded 260-byte array shared with USB, so raising it would
mean forking upstream) capped staging at ~0.5 KB/s regardless of
payload size. Large chunks now stage in two phases: a small
(~0.9 KB) RECEIVER program rides the windowed raw paste, then the
sealed zlib+XOR+base64 payload is written straight into the
receiver's `sys.stdin` read at full BLE speed — the hub-side BLE
REPL buffers rx in an elastic bytearray, so nothing windows the
bulk. The receiver unseals with its own NVS name (same 1.30.0
binding), writes the file, and echoes the sha256 of the plaintext
it wrote; the host verifies the digest before proceeding — a
corrupted or mis-keyed transfer aborts loudly instead of launching
a half-written program. Bench math: only ~0.9 KB of a 7.9 KB
script's transfer is still window-paced (was all 5.2 KB in 1.30.0,
all 8.3 KB in 1.29.x) — staging should drop from ~11.8 s to ~3 s.
Tiny `-c` snippets keep the single-paste plain framing. No firmware
change — the receiver travels with every transfer.

## 1.30.0 — staging is compressed and bound to the addressed hub

Bench (`--debug` capture, 2026-07-31): staging a 7.9 KB script took
~16.6 s of the run's ~20 s overhead — raw-paste flow control grants
a 128-byte window, so wire bytes ≈ time, and the old `repr()`
framing even inflated unicode-heavy source. Scripts ≥ 512 bytes now
ship zlib-compressed, XORed with a SHA-256 keystream keyed on the
**hub name + per-transfer nonce**, and base64-framed; the hub
inverts all three using its own NVS name (`openbricks.HUB_NAME`)
before writing `/program.py`. Measured on-wire size for that same
script: 63% of raw (5.2 KB vs the old 8.3 KB) — staging time drops
proportionally.

Security honesty: the hub name is broadcast in every BLE
advertisement, so the keyed layer is NOT confidentiality against
link sniffing. What it provides: a mis-targeted staging fails
loudly at decode instead of silently landing on a wrong robot, and
casual on-air obfuscation of source. Real secrecy would need a
per-hub secret as the key (possible future `flash --secret`).

Tiny payloads (interactive `-c` snippets, < 512 B) keep the plain
framing. Requirements — the `deflate`/`hashlib` extmods (on every
openbricks firmware and the unix test port) and a flashed hub
name — fail loudly via the staged program's own exception; there
is no silent downgrade. Verified end-to-end on the MicroPython VM:
correct name round-trips byte-identically, wrong name raises.

## 1.29.1 — `--debug` reports BLE startup timings

`openbricks run/upload/stop/log --debug` now prints a one-line
per-stage breakdown of the connection setup — scan, connect,
subscribe — the phases that dominate the command's startup latency
(bench report: >8 s before a script starts). The same numbers are
available programmatically as `link.timings`. Diagnostic only, no
behavior change; the post-connect timeline was already visible via
`--debug`'s per-packet timestamps.

## 1.29.0 — `openbricks docs` opens a styled page in your browser

1.28.0's terminal Markdown was readable but not pleasant (user
feedback). The default is now a proper reading experience that
stays fully offline: all bundled guides render into ONE
self-contained HTML file (real tables, code blocks, section nav,
light + dark) written to the temp directory and opened in the
system browser at the requested topic's section. `--text` keeps
the previous terminal behavior for ssh sessions and piping. Also
fixes a 1.28.0 wart: the Sphinx-only `{eval-rst}` blocks (the CLI
argparse reference, example `literalinclude`s) used to show as raw
RST offline — they now become useful pointers (`openbricks
--help`; links to the example files). New dependency: `markdown`
(pure-Python, no transitive deps). No browser available (headless)
→ clean error pointing at `--text`.

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
