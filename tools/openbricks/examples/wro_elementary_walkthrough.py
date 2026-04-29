# SPDX-License-Identifier: MIT
"""
WRO 2026 "Robot Rockstars" walkthrough — see the per-round
randomization at work.

Run::

    openbricks-sim run examples/wro_elementary_walkthrough.py \\
        --world wro-2026-elementary --seed 42 --no-shim

Re-run with a different ``--seed`` and the layout changes;
re-run with the same seed and it's identical. Pass no ``--seed``
and each invocation is non-deterministic.

What it demonstrates
--------------------

1. ``openbricks-sim run --world wro-2026-elementary --seed N``
   loads the Elementary mat + props, then applies the per-round
   randomization spec (4 of 6 notes get permuted across 4 light-
   green slots at the upper end of the mat — black, white,
   yellow, blue — while red and green stay fixed). The CLI
   prints the chosen layout to stderr in the same form a WRO
   judge announces it: ``note_black -> slot_2``.

2. The script then sees the same layout from the *robot's*
   side, reading the note bodies' actual world positions out of
   the MuJoCo model. The slot coordinates come from the
   randomization spec (which itself was extracted from the
   high-res mat artwork in 0.10.10), so a slot label and a
   sensed (x, y) cross-reference unambiguously.

3. With the layout in hand, the script prints a route plan —
   "drive in this order to visit each slot in left-to-right
   order" — to show how a real round-1 program would translate
   randomization into a path.

For an actual driving / sensor-using mission program, see
``color_drive.py`` (uses the firmware ``TCS34725`` driver
unmodified) and ``scored_mission.py`` (mission-scoring helpers).
"""

# noqa: F821 — ``robot`` is provided by openbricks-sim's ``run`` cmd.

import mujoco

from openbricks_sim import randomization


# The 4 randomizable notes per the Elementary Game Rules p7.
_RANDOMIZED_NOTES = ("note_black", "note_white", "note_yellow", "note_blue")

# Pull the slot positions straight from the spec so this script
# tracks any future re-extraction.
_SLOTS = randomization._SPECS["wro-2026-elementary"][0].slots


def _note_xy(model, data, body_name):
    """Read a body's world (x, y) in millimetres."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return float(data.xpos[bid, 0]) * 1000.0, float(data.xpos[bid, 1]) * 1000.0


def _slot_for_xy(x_mm, y_mm, tolerance_mm=10.0):
    """Match a sensed (x, y) to the nearest slot label, or 'fixed'
    if the body isn't anywhere near a randomization slot (the red
    and green notes — they don't move)."""
    for slot in _SLOTS:
        sx_mm = slot.x * 1000.0
        sy_mm = slot.y * 1000.0
        if abs(x_mm - sx_mm) <= tolerance_mm and \
           abs(y_mm - sy_mm) <= tolerance_mm:
            return slot.label
    return "fixed (not randomized)"


print()
print("=== layout reconstructed from the model ===")
note_to_slot = {}
for note in _RANDOMIZED_NOTES:
    x_mm, y_mm = _note_xy(robot.model, robot.data, note)              # noqa: F821
    label = _slot_for_xy(x_mm, y_mm)
    note_to_slot[note] = label
    print(f"  {note:14s} at ({x_mm:+7.1f}, {y_mm:+7.1f}) mm  ->  {label}")

# Sanity-check: the script's reconstructed layout matches the
# layout the CLI announced via [randomize]. (If they ever
# disagree, something's pathological — e.g. randomize() ran
# again between CLI and script.)
slot_to_note = {label: note for note, label in note_to_slot.items()}

print()
print("=== route plan (visit slots left-to-right by X) ===")
for slot in sorted(_SLOTS, key=lambda s: s.x):
    note = slot_to_note.get(slot.label, "<empty>")
    print(f"  {slot.label} ({note}): drive to "
          f"({slot.x*1000:+6.1f}, {slot.y*1000:+6.1f}) mm")

print()
print("done — re-run with a different --seed to see the layout shuffle.")
