# Sim fidelity progression — q54 (primary tuning log)

Real reference, time-weighted over the 165.1 s enabled window:
peak current **318.7 A** | mean current **148.8 A** | min current **-101.7 A** |
voltage mean **10.02 V** | voltage min **7.55 V** | brownout time **0.00 s**

## Power / battery

| metric | baseline | round 1 | real |
| --- | --- | --- | --- |
| peak total current | 1172.6 A | **377.5 A** | 318.7 A |
| mean total current | 169.4 A | 103.1 A | 148.8 A |
| min total current (regen) | 0.0 A | **-233.9 A** | -101.7 A |
| battery mean | 10.37 V (model) / 12.17 V (actual) | **10.54 V** | 10.02 V |
| battery min | 4.00 V (floor, pinned 87x) | **5.25 V** | 7.55 V |
| voltage floor pinned | 87 samples | **never** | n/a |
| model reaches the motors? | **NO** (maple-sim clobbered it) | **YES** | n/a |

Baseline had two different voltage series because maple-sim overwrote ours 5x per loop.
After round 1, `SystemStats/BatteryVoltage` and `SimBattery/Voltage` are identical.

## Harness correctness

| check | baseline | round 1 |
| --- | --- | --- |
| start pose at first-enable | (13.64, 4.27, 180.0deg) -- 3.5 m / 90deg off | **(12.19, 7.45, 89.8deg)** -- exact |
| match time injected | no (constant -1) | **yes** (162 records, max 140 s) |
| shooter hub gate | open 100% of match (-1 <= 2) | gated as real |
| joystick injection fidelity | mean abs err 0.0039 (verified faithful) | unchanged |

## Remaining gaps after round 1

- Mean current 103.1 A vs real 148.8 A -- the sim now **under**-draws on average, where it
  previously over-drew. Predicted by the battery agent as the consequence of clamping without
  yet fixing steady-state load; `FlywheelSim` is frictionless, so a mechanism at setpoint
  draws ~0 A while the real robot holds 7-9 A.
- Peak 377.5 A vs 318.7 A -- still 18% over (was 268% over).
- Battery min 5.25 V vs real 7.55 V -- follows from the remaining peak overshoot.
- Regen min -233.9 A vs real -101.7 A -- overshoots; the mechanisms have no load to brake into.
- Battery parameters still at the shipped 12.8 V / 20 mOhm. The regression-fitted
  12.1 V / 13.5 mOhm should only be applied once the current model is right, or it hides the bug.

---

## MEASURED NOISE FLOOR — read before trusting any small score difference

Two runs of **identical code** (the only diff was one hood signal that did not move, 0.075 ->
0.075) produced materially different fit scores:

| category | run A (round 3) | run B (final) | delta |
| --- | --- | --- | --- |
| currents | 0.3018 | 0.3178 | +0.016 |
| mechanisms | 0.2536 | 0.2523 | -0.001 |
| aggregate | 0.2955 | 0.3015 | +0.006 |

Signal-level: **52 of 155 signals moved by >0.01**, mean absolute change **0.0212**. The largest
movers are cumulative swerve odometry (`Module2/DrivePositionRads` 0.446 -> 0.754,
`Module1/DrivePositionRads` 0.715 -> 0.487) which integrate any timing difference, and the steer
currents that follow from them.

### Consequence: several earlier keep/revert decisions were made on noise

| decision | measured delta | above noise? |
| --- | --- | --- |
| stator clamp @4.0 rejected | 0.3018 -> 0.3143 (+0.0125) | **no** |
| stator clamp @5.0 rejected | 0.3018 -> 0.3221 (+0.020) | marginal |
| viscous drag rejected | 0.3018 -> 0.3159 (+0.014) | **no** |

Those three changes should be treated as **unresolved**, not disproven. The drag revert is still
independently justified -- mean pack current moved only 113.84 -> 113.50 A against a real 148.8 A,
which is a direct physical observation and not a score difference -- but the stator clamp was
rejected on a difference indistinguishable from noise.

### What remains solidly established

The headline improvements are an order of magnitude above the noise floor and stand:

| category | baseline | now | delta | vs noise |
| --- | --- | --- | --- | --- |
| currents | 0.4510 | ~0.31 | -0.14 | 9x |
| mechanisms | 0.4008 | ~0.25 | -0.15 | 9x |
| voltage | 0.4321 | ~0.28 | -0.15 | 9x |
| other | 0.2679 | 0.23 | -0.04 | 2x |

As do the physical measurements, which are not score-based: peak current 1172.6 -> ~390 A against
a real 318.7; the battery model actually reaching the motors; regen flowing negative; the rack
stopping at its real 11.29 rotations.

### Required before any further fine tuning

Make the replay deterministic, or average over repeats. The cross-cutting analysis already
identified the mechanism: `Robot.java` advances the player by a fixed `PERIODIC_LOOP_SEC` per loop
while AdvantageKit stamps records with wall clock, so loop-count and wall-clock decouple whenever
either robot drops a loop (measured: 2.7 sim loops, 36.4 real loops, settling at a ~130 ms offset).
Stamping the sim by loop count, or scoring the mean of 3 runs, would put the noise floor below the
effects being chased.
