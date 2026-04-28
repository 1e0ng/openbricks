# WRO 2026 RoboMission Senior — "Mosaic Masters"

A MuJoCo MJCF world for the [2026 WRO RoboMission Senior](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Senior-Game-Rules.pdf) competition. Robot acts as an apprentice mason: hands tools to colleagues, lays mosaic tiles in a frame, delivers cement of matching colour to four pour zones, and protects the barriers around the city construction site.

## Files

| | |
|---|---|
| `world.xml` | MJCF scene description |
| `mat.png` | Printed mat texture, 13949×6750 px (150 dpi rasterization of the official Game Mat Printing File PDF). Regenerable via `scripts/regen-wro-mat-textures.sh`. |

## What's in the scene

- **Static-but-jointed barriers (mission 3.4):** 4 barriers — 2 red with blue tops, 2 black with red tops. 7 pts each if undamaged & unmoved (max 28).
- **Mission 3.1:** 3 tools — rectangular trowel (→ sponsor area), cement bowl (→ parking space), masonry trowel (→ start area). 15 each completely in / 5 partial.
- **Mission 3.2:** 24 mosaic tiles (4× yellow, 4× blue, 4× green, 4× white — 8 sample tiles seeded; sim runner can clone the rest from a template). 10 each correct in the mosaic frame; 5 each incorrect; max 120.
- **Mission 3.3:** 40 cement elements (10× each of yellow/blue/green/white — 4 sample seeds in this scene). 1 each in matching-colour cement target area; max 40.
- Border walls.

## Loading the scene

```
pip install mujoco
python -m mujoco.viewer --mjcf=tools/openbricks-sim/worlds/wro_2026_senior_mosaic_masters/world.xml
```

Maximum round score: **233**.

## Caveats

Same shape as the other world READMEs — start positions ±20 mm, axis-aligned boxes/cylinders rather than LEGO-mesh fidelity, mat is a downsampled PNG. The full 24 mosaic tiles + 40 cement elements aren't all instantiated here; only one or two of each colour are seeded so the file is readable. The sim runner will clone them at scene-load time from a Python-side spec.

## Source documents

- [2026 RoboMission Senior Game Rules](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Senior-Game-Rules.pdf)
- [2026 RoboMission Senior Game Mat — Printing File](https://wro-association.org/wp-content/uploads/WRO-2026-GameMat-Senior-Printing-File.pdf)
- [2026 RoboMission Senior Building Instructions](https://wro-association.org/wp-content/uploads/WRO-2026-RM-Senior-BI-All.pdf)
- [2026 RoboMission Senior 3D Printing Files](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Senior-3D-Printing.zip)
