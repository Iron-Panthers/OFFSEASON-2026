# 2026 Game Information — FRC Reefscape (Rebuilt)

Context document for AI sessions. Derived from `Arena2026Rebuilt.java`, `FuelSim.java`, and field geometry constants in the codebase.

---

## Game Overview

- **Game name:** Reefscape (2026 rebuild season)
- **Alliance size:** 3 robots per alliance
- **Match structure:** 15 seconds autonomous + 135 seconds teleoperated
- **Game piece:** Fuel (spherical ball, ~150mm diameter)

---

## Game Piece: Fuel

| Property         | Value                                         |
| ---------------- | --------------------------------------------- |
| Shape            | Sphere                                        |
| Radius           | 0.075 m (150mm diameter)                      |
| Mass             | ~0.2033 kg (0.448 × 0.45392 lb·kg conversion) |
| Drag coefficient | 0.47 (smooth sphere)                          |
| Color            | (game-specific, typically bright)             |

Fuel is picked up from the ground by the intake and stored in the robot's hopper. It is launched by the shooter toward scoring targets.

---

## Field Dimensions

| Measurement       | Value                                                   |
| ----------------- | ------------------------------------------------------- |
| Field length      | 16.51 m                                                 |
| Field width       | 8.04 m                                                  |
| Coordinate origin | Blue alliance corner (bottom-left when blue is on left) |

---

## Field Elements

### Trench

The trench is a low tunnel structure that runs across the field on both sides (near the alliance walls). Robots can drive under it.

| Measurement                    | Value   |
| ------------------------------ | ------- |
| Trench width (opening)         | 1.265 m |
| Trench block width             | 0.305 m |
| Trench height (clearance)      | 0.565 m |
| Trench bar height above trench | 0.102 m |
| Trench bar width               | 0.152 m |

### Net / Goal

The net is the scoring target for fuel. It has a low coefficient of restitution (COR = 0.2), meaning fuel that hits it loses most of its velocity.

---

## Scoring (high-level inference from codebase)

- Fuel is scored by shooting it into/through the net
- The trench provides a path to position for shots from different angles
- Auto routines are named `xTBB`, `xBBBB`, `xTBTB` — "T" likely = trench, "B" likely = ball/baseline
- Preload auto: start with pre-loaded fuel and shoot immediately

---

## Field Geometry (from FuelSim.java)

Fuel spawn/reset positions (Translation3d — x, y, z in meters):

```
Blue side (near x=3.96–4.61):
  (3.96, 1.57, 0)        (3.96, 4.62, 0)
  (4.61, 1.57, 0.165)    (4.61, 4.62, 0.165)

Red side (near x=11.90–12.55):
  (FIELD_LENGTH - 5.18, 1.57, 0)       (FIELD_LENGTH - 5.18, 4.62, 0)
  (FIELD_LENGTH - 4.61, 1.57, 0.165)   (FIELD_LENGTH - 4.61, 4.62, 0.165)
```

Trench top corners:

```
  (3.96, 1.265, 0.565)    (3.96, 6.775, 0.565)
  (11.33, 1.265, 0.565)   (11.33, 6.775, 0.565)
```

---

## Auto Strategy Notes

The robot's autos are named with position codes:

- **T** — Trench position (robot drives into/under trench to collect fuel)
- **B** — Baseline / field-side ball collection
- Numbers (1x, 2x) — likely robot starting position or sequence variant

Example autos:

- `2x4T` — 4 game pieces collected from trench area
- `2x5T` — 5 game pieces from trench
- `2xBBBB` — 4 baseline balls
- `Preload Auto` — shoot preloaded fuel immediately from starting position

---

## Alliance Station Layout

- **Blue alliance:** Left side of field (low x values, x < 8.255m)
- **Red alliance:** Right side of field (high x values, x > 8.255m)
- Autos have Left/Right variants (mirrored across field center) for alliance side flexibility

---

## Physics Notes for Sim

The robot's FuelSim uses realistic projectile physics:

- Gravity: 9.81 m/s²
- Air resistance: `F_drag = 0.5 × 1.225 × 0.47 × π × 0.075² × v²`
- Ground friction: 20% horizontal velocity loss per second while on ground
- Collisions: elastic/inelastic depending on surface (see `robot-description.md` FuelSim section)

For shooting angle prediction, `RobotState.ShootingAnglePredictor` uses a lookup table interpolated over shot distance. Time-of-flight increases with distance, requiring lead compensation for moving targets.
