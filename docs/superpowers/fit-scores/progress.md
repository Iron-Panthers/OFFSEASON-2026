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
