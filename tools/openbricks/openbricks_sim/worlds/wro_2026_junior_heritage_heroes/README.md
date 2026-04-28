# WRO 2026 RoboMission Junior — "Heritage Heroes"

A MuJoCo MJCF world for the [2026 WRO RoboMission Junior](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Junior-Game-Rules.pdf) competition. Robot guides visitors through an old town, rebuilds collapsed towers, brings excavated artefacts to a museum, and sweeps dirt off the cobblestones.

## Files

| | |
|---|---|
| `world.xml` | MJCF scene description |
| `mat.png` | Printed mat texture, 13949×6750 px (150 dpi rasterization of the official Game Mat Printing File PDF). Regenerable via `scripts/regen-wro-mat-textures.sh`. |

## What's in the scene

- **Static bonus props (mission 3.5):** parrot, two barriers — penalty 10/10/10 if moved
- **Mission 3.1:** 4 visitors in red/blue/green/yellow, delivered to matching-colour areas
- **Mission 3.2:** 2 yellow towers (composite top + base) + 2 red towers, all to be placed in their orange-bordered targets upright
- **Mission 3.3:** 4 artefacts (red/yellow/green/blue) staged at the excavation site, delivered to matching-colour exhibition spots in the museum
- **Mission 3.4:** 10 small dirt particles littering the cobblestone area; robot must sweep them out
- Border walls (40 mm) so props don't tumble off the edge during dev iteration

## Loading the scene

```
pip install mujoco
python -m mujoco.viewer --mjcf=tools/openbricks-sim/worlds/wro_2026_junior_heritage_heroes/world.xml
```

Maximum round score: **230**.

## Caveats

Same as the Elementary world — see `tools/openbricks-sim/worlds/wro_2026_elementary_robot_rockstars/README.md`. Movable-prop start positions are estimated from the rulebook overview photo (±20 mm); silhouettes are axis-aligned boxes/cylinders not LEGO-faithful meshes; mat texture is downsampled PNG.

## Source documents

- [2026 RoboMission Junior Game Rules](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Junior-Game-Rules.pdf)
- [2026 RoboMission Junior Game Mat — Printing File](https://wro-association.org/wp-content/uploads/WRO-2026-GameMat-Junior-Printing-File.pdf)
- [2026 RoboMission Junior Building Instructions](https://wro-association.org/wp-content/uploads/WRO-2026-RM-Junior-BI-All.pdf)
