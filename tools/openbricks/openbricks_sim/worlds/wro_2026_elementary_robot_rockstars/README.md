# WRO 2026 RoboMission Elementary — "Robot Rockstars"

A map (`map.json`, format `openbricks-map/1`) for the [2026 WRO RoboMission Elementary](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Elementary-Game-Rules.pdf) competition. Robot drives onto a music-festival stage and arranges instruments, microphone, cables, and notes. The mat artwork is the official printing-ready file; physical props sit on top.

## Files

| | |
|---|---|
| `map.json` | the map: physics, mat, walls, props and camera (MuJoCo's own names and units, in JSON) |
| `props/` | The props, each a Workbench build (`*.assembly.json`, format `openbricks-assembly/1`) made from the building instructions. Open one on the Workbench to change it. |
| `mat.png` | Printed mat texture, 6974×3375 px (75 dpi rasterization of the official Game Mat Printing File PDF, 256-colour palette-quantized — PyPI wheel-size budget; the TCS34725 sampling spot still spans ~9 px). Regenerable via `scripts/regen-wro-mat-textures.sh`. |

## Mat — what's where

- **Mission 3.1 cable target areas** — two grey rectangles between the amplifier (top-left of stage) and each speaker
- **Mission 3.2 backstage area** — pink lounge in the lower-left, where instruments must end up
- **Mission 3.2 mic target area** — light-green rectangle on the stage
- **Mission 3.3 notes start area** — four light-green squares at the upper-right
- **Mission 3.3 note targets** — six coloured (red/blue/green/yellow/white/black) squares each ringed in grey
- **Static bonus props** — clef, amplifier, two speakers (don't damage)
- **Start area** — bottom-right, by the truck

## Props

Every prop is a brick-for-brick Workbench build, with each brick's catalogue mass, standing where the mat prints it:

| Props | Where |
|---|---|
| microphone, keyboard, guitar, congas | on the truck, backstage start (mission 3.2) |
| six notes: black, blue, red, green, white, yellow | the note start squares along the top (mission 3.3) |
| two cables | on the pink arrows (mission 3.1) |
| clef, amplifier, two speakers | static bonus props: the amplifier on the black outline between the grey cable areas, a speaker on each tilted blue outline |

Each `openbricks sim run` or `preview` of this map shuffles the black, white, yellow and blue notes across their four start squares, like a round's randomization; `--seed N` repeats a layout. Each note keeps the heading the map gives it and lands with its footprint centred on its square. The red and green notes stay put, as the rules require.

## Loading the scene

```
openbricks sim preview --world wro-2026-elementary
```

`openbricks-sim run` (when it ships) will spawn the user's robot inside this world programmatically.

## Caveats

- **Prop start positions were placed by eye** on the printed outlines in the map editor. Pixel-measure the mat for sub-mm accuracy when calibrating against a physical playfield.
- **The mat texture is PNG, downsampled to ~2048 px wide.** MuJoCo only accepts PNG (rejects JPG). Regenerate from the source PDF at higher resolution if you need print-grade colour matching for vision-based testing.

## Source documents

- [2026 RoboMission Elementary Game Rules](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Elementary-Game-Rules.pdf) (PDF, image-based)
- [2026 RoboMission Elementary Game Mat — Printing File](https://wro-association.org/wp-content/uploads/WRO-2026-GameMat-Elementary-Printing-File.pdf) (PDF; this is the source for `mat.png`)
- [2026 RoboMission Elementary Building Instructions](https://wro.swiss/wp-content/uploads/2026/01/WRO-2026-RoboMisson-Elementary-Bauanleitung-dfi.pdf) (PDF; LEGO step-by-step for every prop)
