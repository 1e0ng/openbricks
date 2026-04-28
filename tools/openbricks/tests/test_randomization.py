# SPDX-License-Identifier: MIT
"""Tests for per-round randomization of WRO mission elements.

The Elementary world's randomization is the only spec wired today
(Junior + Senior land in follow-up PRs). The test surface here
pins:

  * Determinism — same seed → same layout.
  * 24 distinct layouts across all seeds (4! permutations of 4
    notes across 4 slots).
  * Each randomizable note ends up at one of the spec's slot
    coordinates; no two notes share a slot.
  * Fixed-position notes (red, green) don't move.
  * Per-note resting Z is preserved (so a sphere-shaped note
    doesn't sink into the mat after randomization).
"""

import unittest
from collections import Counter
from pathlib import Path

import mujoco

from openbricks_sim import randomization
from openbricks_sim.chassis import ChassisSpec
from openbricks_sim.world import load_world


_ELEMENTARY_PATH = (Path(__file__).resolve().parent.parent
                    / "openbricks_sim" / "worlds"
                    / "wro_2026_elementary_robot_rockstars" / "world.xml")


def _make_elementary():
    """Load the Elementary world with the default chassis spec.

    ``load_world`` returns ``MjData`` straight from ``MjModel`` —
    ``data.xpos`` etc. are still zeroed until the first
    ``mj_forward`` populates them from ``qpos``. Force that here so
    test assertions read the world.xml's ``pos`` values (and any
    pre-randomization checks see the actual starting layout)."""
    model, data, merged = load_world(
        str(_ELEMENTARY_PATH), chassis_spec=ChassisSpec())
    mujoco.mj_forward(model, data)
    return model, data, merged


def _xy_of(model, data, body_name):
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return float(data.xpos[body_id, 0]), float(data.xpos[body_id, 1])


def _z_of(model, data, body_name):
    body_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return float(data.xpos[body_id, 2])


class ElementaryRandomizationTests(unittest.TestCase):

    def test_deterministic_for_fixed_seed(self):
        # Same seed twice must place the same notes at the same
        # slot labels — pins the contract that round-N reproducibility
        # is on the seed alone.
        m1, d1, _ = _make_elementary()
        m2, d2, _ = _make_elementary()
        layout1 = randomization.randomize(
            m1, d1, world="wro-2026-elementary", seed=42)
        layout2 = randomization.randomize(
            m2, d2, world="wro-2026-elementary", seed=42)
        self.assertEqual(layout1, layout2)

    def test_different_seeds_eventually_yield_all_24_permutations(self):
        # 4 random notes × 4 slots = 4! = 24 layouts. Across enough
        # seeds we should see all 24 distinct (body→slot) maps.
        # If the implementation accidentally constrained to fewer
        # (e.g. only rotated, didn't shuffle), this test fails.
        seen = set()
        for seed in range(200):  # 200 seeds, plenty for 24 outcomes
            m, d, _ = _make_elementary()
            layout = randomization.randomize(
                m, d, world="wro-2026-elementary", seed=seed)
            seen.add(tuple(sorted(layout.items())))
        self.assertEqual(
            len(seen), 24,
            "expected 4! = 24 distinct layouts across 200 seeds; "
            "got %d — randomization may not be a true permutation"
            % len(seen))

    def test_each_note_lands_on_a_spec_slot(self):
        m, d, _ = _make_elementary()
        randomization.randomize(
            m, d, world="wro-2026-elementary", seed=7)
        # The 4 slot positions from the spec
        spec_xys = {(0.0499, 0.4881), (0.1818, 0.4881),
                    (0.5775, 0.4881), (0.7094, 0.4881)}
        for note in ("note_black", "note_white", "note_yellow", "note_blue"):
            x, y = _xy_of(m, d, note)
            # Allow tiny numerical wiggle from mj_forward
            matched = any(abs(x - sx) < 1e-3 and abs(y - sy) < 1e-3
                          for sx, sy in spec_xys)
            self.assertTrue(
                matched,
                "%s landed at (%.4f, %.4f) which isn't a spec slot"
                % (note, x, y))

    def test_no_two_notes_share_a_slot(self):
        m, d, _ = _make_elementary()
        randomization.randomize(
            m, d, world="wro-2026-elementary", seed=12345)
        positions = []
        for note in ("note_black", "note_white", "note_yellow", "note_blue"):
            positions.append(_xy_of(m, d, note))
        # Every (x, y) should appear exactly once.
        counts = Counter(
            (round(x, 3), round(y, 3)) for x, y in positions)
        for pos, cnt in counts.items():
            self.assertEqual(
                cnt, 1,
                "two notes at %s — randomization isn't a permutation" % (pos,))

    def test_fixed_position_notes_dont_move(self):
        # The red and green notes are fixed per the rules. Capture
        # their positions before randomization, randomize, verify
        # nothing moved.
        m, d, _ = _make_elementary()
        before_red = _xy_of(m, d, "note_red")
        before_green = _xy_of(m, d, "note_green")
        randomization.randomize(
            m, d, world="wro-2026-elementary", seed=99)
        after_red = _xy_of(m, d, "note_red")
        after_green = _xy_of(m, d, "note_green")
        self.assertEqual(before_red, after_red)
        self.assertEqual(before_green, after_green)

    def test_resting_z_is_preserved_per_note(self):
        # The 4 notes have different shapes (sphere / box / cylinder
        # of varying heights) and so different resting Z heights
        # in the world.xml. Randomization places each note at the
        # slot's (x, y) but must keep its existing Z — otherwise
        # the sphere note sinks into the mat or the tall cylinder
        # floats. Capture each note's pre-randomization Z, then
        # check post-randomization that Z is unchanged.
        m, d, _ = _make_elementary()
        before = {n: _z_of(m, d, n)
                  for n in ("note_black", "note_white",
                            "note_yellow", "note_blue")}
        randomization.randomize(
            m, d, world="wro-2026-elementary", seed=2026)
        for note, z_before in before.items():
            z_after = _z_of(m, d, note)
            self.assertAlmostEqual(
                z_before, z_after, delta=1e-4,
                msg="%s z changed: %.6f → %.6f" % (note, z_before, z_after))

    def test_unknown_world_raises_keyerror(self):
        m, d, _ = _make_elementary()
        with self.assertRaises(KeyError):
            randomization.randomize(
                m, d, world="wro-2026-junior", seed=1)


if __name__ == "__main__":
    unittest.main()
