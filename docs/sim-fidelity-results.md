# Simulation Fidelity vs Real Match Logs

What the simulation reproduces from real Worlds match logs, what it does not, and what changed.

**Method.** A real `.wpilog` is replayed through the simulation — logged joystick axes, buttons,
POVs and driver-station state injected frame-locked to the robot loop, same auto — and the sim log
is scored against the real one signal by signal.

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging "-Preplay.inputs=<real>.wpilog"
python scripts/wpilog_to_csv.py --compare <sim>.wpilog <real>.wpilog     # nRMSE by category
python scripts/drive_vision_fidelity.py <sim>.wpilog <real>.wpilog       # slip and vision
```

Score is **nRMSE**: RMSE normalised by the real signal's range. 0.20 means typical error is 20% of
the range that signal covers in the match.

**Runs are not deterministic.** PhotonVision's simulated noise is unseeded, so two runs of identical
code differ by up to ~0.006 in a category and ~0.01 on shot-dependent signals like the serializer.
Compare two runs per side before trusting a small difference.

## Fit across five matches

Only q54 was tuned against. q93 and q14 are validation, q103 was held out until the end, q64 is the
brownout stress case.

| log | aggregate | currents | mechanisms | other | voltage |
| --- | --- | --- | --- | --- | --- |
| q54 (tuned) | 0.1365 | 0.1822 | 0.2249 | 0.2048 | 0.1352 |
| q93 | 0.1587 | 0.1740 | 0.2415 | 0.1981 | 0.1573 |
| q14 | 0.0845 | 0.1525 | 0.2138 | 0.1734 | 0.1239 |
| q103 (held out) | 0.1272 | 0.1847 | 0.2364 | 0.1880 | 0.1537 |
| q64 (brownout) | 0.1153 | 0.1702 | 0.2296 | 0.2722 | 0.1551 |

q54's first replay, rescored with the same tooling, was aggregate 0.2926, currents 0.4498,
mechanisms 0.3697, voltage 0.4285.

## What matches

| quantity | real | sim |
| --- | --- | --- |
| mean pack current, q54 | 148.7 A | 160.5 A (was 85.4) |
| peak pack current, q54 | 318.7 A | 293.5 A (was 1172.6) |
| battery minimum, q54 | 7.55 V | 7.43 V |
| samples below 6.75 V brownout, q54 | 0 | 0 |
| battery fall over the match, q54 | −1.60 V | −0.92 V (was 0.00) |
| intake deploy, 1 → 10 rot | 0.500 s | 0.420 s (was 0.219) |
| driver axis reproduction | — | 0.0039 mean abs error |

Mean pack current across all five: sim 160–170 A against real 145–167 A.

**Brownout feeds back into torque.** In q64 the real flywheel spent 14.7% less time at speed than in
q54; the sim reproduces the drop at 3.5%. Right direction, a quarter of the magnitude.

**Battery** parameters are fitted per match with `V = a + b·t + c·I` and passed as
`-Preplay.battery=<volts>:<ohms>:<droopV/min>`. Internal resistance is consistent across matches
(10.5–12.0 mΩ) and is the default; open-circuit voltage and droop vary with the battery.

| log | OCV | droop V/min | R mΩ |
| --- | --- | --- | --- |
| q54 | 12.451 | 0.439 | 11.96 |
| q93 | 12.074 | 0.345 | 11.59 |
| q14 | 12.162 | 0.652 | 10.51 |
| q64 | 12.270 | 1.019 | 10.90 |

## Mechanism load

WPILib's `FlywheelSim` reports current as `(V − ω·G/Kv)/R` with `G` recovered from its own plant
matrices, so at any steady state that is **exactly zero** — for every plant, including one from
`identifyVelocitySystem`. Mechanisms holding speed drew nothing where the real ones draw 5–10 A,
which was essentially the entire pack-current gap.

The fix needs both halves: subtract a drag voltage from the plant input, and compute stator current
against the **commanded** voltage (`SimCurrentLimit.dragVolts`, `statorAmps`). Coefficients are the
median real stator current while spinning across five matches:

| mechanism | drag, A per motor per rad/s | sim stator | real median |
| --- | --- | --- | --- |
| shooter flywheel | 0.0341 | 7.92 A | 8.22 A |
| accelerator | 0.0267 | 21.66 A | 21.64 A |
| intake rollers | 0.1068 | 15.85 A | 16.56 A |
| serializer | 0.2237 | 24.56 A | 25.47 A |
| omniwheel | 0.0552 (unloaded) | 118.51 A | 41.13 A |
| intake rack | 0.61 V constant | — | — |

These are lumped match loads, 2–3× unloaded friction, because they absorb work done on game
pieces. The omniwheel is left at its unloaded value: its transients alone exceed the real current.
The rack is stationary most of the match, so it gets a constant directional load instead.

`supply = stator × dutyCycle` was checked against real data binned by duty cycle: measured/ideal is
0.95–1.08 for every mechanism.

## Vision

Four structural defects, all fixed:

- SIM passed **one** camera to `Vision`, at index 3 of a five-entry SIM array — a rear-facing camera
  the robot does not have.
- A second camera was constructed and discarded, never reaching the pose estimator.
- SIM camera transforms bore no relation to COMP's three. SIM now shares COMP's.
- `new SimCameraProperties()` is `PERFECT_90DEG`: zero pixel error, zero latency.

Noise is injected in **pixels on tag corners**, so accuracy degrades with range from the optics.

| camera target distance, q54 | sim before | sim now | real |
| --- | --- | --- | --- |
| camera 0 (rollers) | 4.66 m | 2.45 m | 2.46 m |
| camera 1 (side) | never saw a tag | 4.69 m | 3.45 m |
| camera 2 (side) | never saw a tag | 4.84 m | 4.33 m |

Vision error measured as the disagreement between two cameras on the **same loop**, which removes
the pose estimator entirely:

| distance | sim at 0.25 px | sim at 0.75 px | real |
| --- | --- | --- | --- |
| 2–3 m | 0.030 m | 0.064 m | 0.073 m |
| 3–4 m | 0.039 m | 0.048 m | 0.084 m |
| 5–6 m | — | 0.108 m | — |

This was unmeasurable before: vision logs `struct:Pose3d[]`, which the log reader did not decode.

## Wheel slip

maple-sim models skidding, but while a module grips it sets wheel speed **exactly** to the ground
projection, so simulated odometry was perfect outside skid events. Measured in autonomous, where no
one is pushing the robot:

| log | sim wheel/true | real | sim disagreement | real |
| --- | --- | --- | --- | --- |
| q93 | 1.052 | 1.077 | 0.018 | 0.037 |
| q14 | 1.053 | 1.111 | 0.017 | 0.035 |
| q103 | 1.055 | 1.096 | 0.017 | 0.038 |
| q64 | 1.062 | 1.086 | 0.018 | 0.037 |

Friction alone cannot produce the disagreement: dropping it to maple-sim's minimum 0.65 moved
wheel/true but not disagreement, because four wheels skidding together still agree. So friction
is 1.05 (published tread-on-carpet) and `ModuleIOTalonFXSim` scales **reported** odometry per
module: 1.05 bias, ±5.3% scatter. The table above is at ±3%; ±5.3% puts disagreement at 0.029.

**That trades against nRMSE.** q103 currents went 0.1743 → 0.1847. nRMSE rewards reproducing one
recorded run, and realistic noise is unpredictable by definition, so a sim with perfect odometry
always replays a log more closely while understating how hard the pose estimator works. ±5.3% was
chosen for realism; `ODOMETRY_SCALE_SCATTER` is one constant.

## What does not match

| gap | cause |
| --- | --- |
| Teleop module disagreement 0.036 vs real 0.080–0.119 | Other robots. Teleop is 2.8× the same match's auto figure; a replay has no opponents |
| Multi-tag frames 66% vs real 28%; side cameras see further | No occlusion in the sim |
| Serializer spins 92% of the match vs real 72% | Control flow: real readiness gating cycles SPIN_UP↔SHOOT 36 times in q54, sim 20. Largest share of the 8% current overshoot |
| Omniwheel stator 118 A vs 41 A; mean speed 183 vs 327 rad/s | Saturates and slews too slowly. No drag value fixes it |
| Rack position correlation 0.629 → 0.284 after its load was added | Sim obeys `SHOOTING_STOW`; the real rack often does not retract. Level, current and timing all improved |
| Rack supply current 2.52 A vs 5.67 A | Real holding load is bimodal in two matches; a constant can match mean stator or mean supply, not both |
| Regen current reaches −284 A vs −102 A | Mechanisms still slew too hard |
| Post-auto pose | Contact. Cumulative odometry is excluded from scoring |

## Deliberate approximations

**`STATOR_TO_SUPPLY_RATIO = 4.0`** is below the rollers' real 5.46–6.02×. Retested at the honest 6.5
after the load model landed: currents q54 0.1966 → 0.1951, q93 0.1836 → 0.2027, q14 0.1561 → 0.1603.
Worse on two of three, so 4.0 stays. It now compensates for mechanisms slewing too hard.

## Changes

**Replay harness** (`utility/replay/`): `LogTimeline`, `MatchInputs`, `MatchLogReader`,
`LogInputPlayer`, `PoseAnchor`. Bugs found building it: match time was never injected, so
`getTimeUntilOurHubShifts() <= 2` read `-1 <= 2` and the shooter ran enabled all match; start pose
was never seeded, putting q54 3.5 m and 90° from the real robot.

**Simulation bugs fixed:**

| where | bug |
| --- | --- |
| `RobotContainer` | `SimBattery.update()` ran before the maple-sim arena, which overwrites `RoboRioSim` voltage five times a loop |
| `GenericSuperstructureIOSim` | never applied its config, so the rack ran its position loop in rotor units (2.55× off) |
| `IntakeRackIOSim` / `ShooterHoodIOSim` | applied gearing in opposite directions |
| `GenericRollersIOSim` | `stop()` never reached the physics; accelerator spun 130 s while stopped |
| roller and superstructure sims | never registered with `MotorOutputManager` |
| roller sims | PID on mechanism velocity where Phoenix regulates rotor; `abs()` on current booked regen as draw; hardcoded ±12 V ignored sag; logged the motor group's current where the real IO logs one motor; logged `positionRads` in rotor units where the real IO logs mechanism units |
| `SerializerSim` | two unit errors in one line ran it backwards; reported a hardcoded 1.0 A |
| `GyroIOSim` | yaw rate 57.3× too small |
| `DriveConstants` | maple-sim bumper size never set, using its 0.76 m default |

The five roller sims now share one loop in `GenericRollersIOSim` and differ only in configuration.

**Constants aligned to COMP (SIM arms only):** drive geometry, bumpers, wheel radius, drive and
steer gains, max velocity and acceleration; flywheel, accelerator, omniwheel, roller and serializer
reductions and gains (SIM reductions had been set to reciprocals to mask the rotor/mechanism PID
bug); flywheel and omniwheel current limits; flywheel and accelerator plant motor counts; rack
travel limit raised to 13.0 rot, above all observed travel.

**One change affecting the real robot, approved:** `GenericSuperstructureIOTalonFX` registered the
leader's supply current with `MotorOutputManager` twice, inflating logged `TotalAmps` by ~6 A mean.
Logging only.

**Tooling:** `scripts/log_compare.py` (comparison engine, sim time rebased on
`Replay/Elapsed Seconds` to remove a 0.016 wall-clock noise floor), `scripts/drive_vision_fidelity.py`,
`Pose3d` decoding in `wpilog_to_csv.py`.

## Conclusions that were wrong

Each looked well supported on one log:

| inference | disproved by |
| --- | --- |
| Rack hard stop at 11.29 rot (never exceeded it in q54, stalled 86.7%) | q93 reaches 11.68 freely; the cap cost 2540 A·s vs a real 213 |
| 4.0× stator ceiling is physical | rollers are 5.46–6.02× in every match |
| `identifyVelocitySystem` would fix the current gap | `FlywheelSim` reports zero current at every steady state regardless |
| Rack holds against symmetric friction | holding voltage never changes sign; friction gave −0.01 A mean |

## Still unverified

- Robot mass, 54.4 kg — never weighed. It sets traction and inertia.
- Camera resolution, FOV, latency, frame rate — representative OV9281 values, not this robot's
  calibration.
- Rack SIM mass, 0.1 kg — not measured; the rack still deploys 16% fast.
- Flywheel drag excludes q54's measurement as an outlier (0.051 vs 0.013–0.016 elsewhere).
- Battery fitting is per match; nothing validates the fit on a match it did not see.
