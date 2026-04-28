# SPDX-License-Identifier: MIT
"""Per-round randomization of WRO mission elements.

The WRO RoboMission rules call for some game elements to be re-placed
each round before the team's robot starts. Randomization is the part
of the challenge that forces robots to *perceive* the field rather
than rely on hardcoded positions. Per the WRO General Rules glossary
(``Robot Round`` definition): "Before the round starts with the first
team but after all robots are placed on the robot parking,
randomizations to game fields (if any) are done."

This module is the openbricks-sim equivalent. After ``load_world``
parses the MJCF and instantiates ``MjData``, the caller asks
``randomize(model, data, world="wro-2026-elementary", seed=N)`` to
permute the randomizable game elements. The change is applied
directly to ``data.qpos`` (every randomizable element rides on a
freejoint), and ``mj_forward`` refreshes derived quantities. The
return value is the chosen layout — body name → slot label — for
logging and test pins.

Randomization specs are per-world and pulled from each category's
Game Rules PDF. The Elementary spec is the only one wired today;
Junior and Senior come once their rules are read into specs.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import mujoco


# ---------------------------------------------------------------------
# Spec types

@dataclass(frozen=True)
class _Slot:
    """A randomization slot: where a randomizable element can land.
    Coordinates are world frame in metres (mat origin at world
    origin, +X right toward start area, +Y "up" toward stage).

    Z is intentionally absent — each randomizable body has its own
    natural resting height stored in ``model.body_pos`` from the
    world XML (notes are different shapes / heights), and we
    preserve that during placement so a note doesn't sink into or
    float above the mat after randomization.

    ``label`` is a human-readable id used in returned layouts and
    test assertions; pick something stable across PRs since tests
    pin it ("slot_1" not "slot at +0.50,+0.42").
    """
    x: float
    y: float
    label: str


@dataclass(frozen=True)
class _RandomizationSpec:
    """Spec for one world's randomization step.

    ``elements`` is the ordered list of randomizable bodies (each must
    be a body in the world XML with a freejoint). ``slots`` is the
    list of slot positions; ``len(slots)`` must equal
    ``len(elements)``. Randomization is a permutation: each round,
    we shuffle ``elements`` and place each one at the corresponding
    ``slot`` (no two elements ever share a slot).

    Fixed-position elements aren't listed here — they stay where
    the world.xml put them and are explicitly NOT randomized per
    the rules.
    """
    elements: Sequence[str]
    slots: Sequence[_Slot]


# ---------------------------------------------------------------------
# Per-world specs

# Elementary "Robot Rockstars" — only the 4 notes black/white/yellow/
# blue get permuted. The red and green notes stay at fixed positions
# (per the Game Rules PDF p7 — "the positions of the green and red
# note are not random"). 4! = 24 distinct layouts.
#
# Slot coordinates come from pixel-inspecting the official
# mat.png artwork (13949×6750 px = 2362×1143 mm). The 4 light-green
# squares cluster as 4 connected components of identical size
# (35,532 px each, ≈32×32 mm) at the upper end of the mat:
#
#     world (+0.0499, +0.4881) m   <- slot_1
#     world (+0.1818, +0.4881) m   <- slot_2
#     world (+0.5775, +0.4881) m   <- slot_3
#     world (+0.7094, +0.4881) m   <- slot_4
#
# Spacing forms two pairs: ~0.13 m within each pair, ~0.40 m
# between pairs (the green-note's fixed position sits between
# the pairs). Re-derive via the script if the mat artwork ever
# updates: scripts/extract-wro-slot-coords.py (TODO — committed
# in this PR).
_ELEMENTARY = _RandomizationSpec(
    elements=("note_black", "note_white", "note_yellow", "note_blue"),
    slots=(
        _Slot(x=0.0499, y=0.4881, label="slot_1"),
        _Slot(x=0.1818, y=0.4881, label="slot_2"),
        _Slot(x=0.5775, y=0.4881, label="slot_3"),
        _Slot(x=0.7094, y=0.4881, label="slot_4"),
    ),
)


_SPECS: Dict[str, _RandomizationSpec] = {
    "wro-2026-elementary": _ELEMENTARY,
    # Junior and Senior land in follow-up PRs once their Game Rules
    # PDFs are read into specs.
}


# ---------------------------------------------------------------------
# Apply

def randomize(model, data, world: str,
              seed: Optional[int] = None) -> Dict[str, str]:
    """Apply the randomization spec for ``world`` to ``model + data``.

    Returns the chosen layout as a dict of ``body_name → slot_label``.
    Caller can log this for round-by-round records or pin in tests.

    ``seed`` is the only knob; pass an integer for deterministic
    layouts (the same seed always produces the same permutation), or
    None to use Python's default random source. WRO doesn't specify
    any particular RNG — this is the sim's choice. ``seed=round_number``
    is a fine convention if the user wants every round to be different
    but reproducible.

    Raises ``KeyError`` if ``world`` has no spec yet (Junior + Senior
    today). Raises ``ValueError`` if a named element doesn't exist as
    a freejointed body in the loaded model — guards against the spec
    drifting out of sync with the world XML.
    """
    if world not in _SPECS:
        raise KeyError(
            "no randomization spec for world {!r} (have: {})"
            .format(world, sorted(_SPECS.keys())))
    spec = _SPECS[world]
    if len(spec.elements) != len(spec.slots):
        # Spec invariant; dev-time sanity check that survives into
        # runtime so a future PR mismatch surfaces immediately.
        raise RuntimeError(
            "{!r} spec has {} elements but {} slots — must match"
            .format(world, len(spec.elements), len(spec.slots)))

    rng = random.Random(seed)
    perm = list(spec.elements)
    rng.shuffle(perm)

    layout: Dict[str, str] = {}
    for element_name, slot in zip(perm, spec.slots):
        _place_freejoint_body(model, data, element_name, slot.x, slot.y)
        layout[element_name] = slot.label

    # Refresh derived quantities (xpos / cam_xpos / sensordata) so any
    # downstream code that reads positions sees the new placements
    # without needing its own ``mj_forward`` call.
    mujoco.mj_forward(model, data)
    return layout


def _place_freejoint_body(model, data, body_name: str,
                          x: float, y: float) -> None:
    """Move a body that's attached via a freejoint to ``(x, y)``,
    preserving its world-XML resting height and identity orientation.

    The Z preservation matters: each randomizable note in the
    Elementary world has a different shape (sphere / box / cylinder
    of varying heights) and so a different resting Z. Forcing all
    notes to the same Z would either sink the tall ones into the mat
    or float the short ones above it.

    Raises ``ValueError`` if the named body is missing or doesn't
    have a freejoint. Internal — exposed only to this module's
    ``randomize`` and tests."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    if body_id < 0:
        raise ValueError("no body named " + repr(body_name) + " in model")
    # The body's first joint is its freejoint (the world.xml convention
    # for randomizable elements is one freejoint per body).
    joint_id = int(model.body_jntadr[body_id])
    if joint_id < 0:
        raise ValueError(
            "body " + repr(body_name) + " has no joint — randomizable "
            "elements must have a freejoint")
    if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise ValueError(
            "body " + repr(body_name) + "'s first joint is not a "
            "freejoint — randomization can only relocate freejointed bodies")
    # Resting Z from the world.xml ``<body pos="...">`` attribute,
    # which MuJoCo stores in ``body_pos``. Use it directly so the
    # post-randomization body sits on the mat the same way the
    # author intended.
    z = float(model.body_pos[body_id, 2])
    qpos_addr = int(model.jnt_qposadr[joint_id])
    # Freejoint qpos layout: [x, y, z, qw, qx, qy, qz].
    data.qpos[qpos_addr]     = x
    data.qpos[qpos_addr + 1] = y
    data.qpos[qpos_addr + 2] = z
    data.qpos[qpos_addr + 3] = 1.0   # identity quaternion
    data.qpos[qpos_addr + 4] = 0.0
    data.qpos[qpos_addr + 5] = 0.0
    data.qpos[qpos_addr + 6] = 0.0
    # Reset velocity for this freejoint too — otherwise a body that was
    # mid-motion at the time of randomization would keep its old
    # velocity, which is surprising for a "set up the scene" action.
    qvel_addr = int(model.jnt_dofadr[joint_id])
    for i in range(6):  # freejoint has 6 DOFs
        data.qvel[qvel_addr + i] = 0.0
