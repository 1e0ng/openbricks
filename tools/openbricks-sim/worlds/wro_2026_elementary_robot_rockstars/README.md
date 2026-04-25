# WRO 2026 RoboMission Elementary — "Robot Rockstars"

A MuJoCo MJCF world for the [2026 WRO RoboMission Elementary](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Elementary-Game-Rules.pdf) competition. Robot drives onto a music-festival stage and arranges instruments, microphone, cables, and notes. The mat artwork is the official printing-ready file; physical props sit on top.

## Files

| | |
|---|---|
| `world.xml` | MJCF scene description |
| `mat.png` | Printed mat texture, ~2048 px wide (resampled from the official 9 MB printing PDF) |

## Mat — what's where

- **Mission 3.1 cable target areas** — two grey rectangles between the amplifier (top-left of stage) and each speaker
- **Mission 3.2 backstage area** — pink lounge in the lower-left, where instruments must end up
- **Mission 3.2 mic target area** — light-green rectangle on the stage
- **Mission 3.3 notes start area** — four light-green squares at the upper-right
- **Mission 3.3 note targets** — six coloured (red/blue/green/yellow/white/black) squares each ringed in grey
- **Static bonus props** — clef, amplifier, two speakers (don't damage)
- **Start area** — bottom-right, by the truck

## Loading the scene

```
pip install mujoco
python -m mujoco.viewer --mjcf=tools/openbricks-sim/worlds/wro_2026_elementary_robot_rockstars/world.xml
```

`openbricks-sim run` (when it ships) will spawn the user's robot inside this world programmatically.

## Caveats

- **Prop start positions are approximate** (±20 mm). Pixel-measure the mat for sub-mm accuracy when calibrating against a physical playfield.
- **Prop silhouettes** are axis-aligned boxes/cylinders — physics-faithful, visually crude. Replace with Blender-exported STL meshes once someone models the actual LEGO builds from the [building-instructions PDF](https://wro.swiss/wp-content/uploads/2026/01/WRO-2026-RoboMisson-Elementary-Bauanleitung-dfi.pdf).
- **The mat texture is PNG, downsampled to ~2048 px wide.** MuJoCo only accepts PNG (rejects JPG). Regenerate from the source PDF at higher resolution if you need print-grade colour matching for vision-based testing.

## Source documents

- [2026 RoboMission Elementary Game Rules](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Elementary-Game-Rules.pdf) (PDF, image-based)
- [2026 RoboMission Elementary Game Mat — Printing File](https://wro-association.org/wp-content/uploads/WRO-2026-GameMat-Elementary-Printing-File.pdf) (PDF; this is the source for `mat.png`)
- [2026 RoboMission Elementary Building Instructions](https://wro.swiss/wp-content/uploads/2026/01/WRO-2026-RoboMisson-Elementary-Bauanleitung-dfi.pdf) (PDF; LEGO step-by-step for every prop)
