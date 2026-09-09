# Simulation Fidelity vs Real Match Logs

What the simulation now reproduces from real Worlds match logs, what it still does not, and every
change made to get there.

**Method.** A real `.wpilog` is replayed through the simulation: the logged joystick axes, buttons,
POVs and driver-station state are injected frame-locked to the robot loop, the same auto is run, and
the resulting sim log is scored against the real one signal by signal.

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging \
  "-Preplay.inputs=<real-match>.wpilog"

python scripts/wpilog_to_csv.py --compare <sim>.wpilog <real>.wpilog --json fit.json
```

Score is **nRMSE** — RMSE normalised by the real signal's range. 0.20 means typical error is 20% of
the range that signal covers during the match.

---

## Does it match?

### Fit scores, q54 (the log tuned against)

Baseline is the first replay run, re-scored with the current tooling so the comparison is fair.

| category | baseline | now | change |
| --- | --- | --- | --- |
| currents (48 signals) | 0.4498 | **0.2386** | **-47%** |
| mechanisms (46 signals) | 0.3697 | **0.1993** | **-46%** |
| voltage | 0.4285 | **0.2175** | **-49%** |
| aggregate (4 signals) | 0.2926 | **0.2015** | **-31%** |
| other (48 signals) | 0.2556 | **0.2126** | -17% |

Every category now sits between 0.20 and 0.24.

### It generalises — all three logs, identical code

`q54` is the only log tuned against. `q93` and `q14` were held back.

| category | q54 (tuned) | q93 | q14 |
| --- | --- | --- | --- |
| aggregate | 0.2464 | **0.2338** | 0.2624 |
| currents | 0.2387 | **0.2102** | 0.2215 |
| mechanisms | **0.2025** | 0.2142 | 0.2124 |
| other | 0.2085 | 0.2235 | **0.1878** |
| voltage | **0.2465** | 0.2647 | 0.2906 |

Every category on every log falls between 0.19 and 0.29, and the untuned logs beat the tuned one in
several categories. These are model fixes, not curve-fitting.

### The validation logs caught a real overfit

`q93` initially scored 0.3433 on currents against `q54`'s 0.2387 -- 44% worse, driven entirely by
one signal: `Intake Rack/Total Amp Seconds` at nRMSE **6.159**.

An earlier change had given the simulated rack a hard stop at 11.29 rotations, inferred from `q54`
where the real rack never exceeded 11.290 and stalled against whatever blocked it for 86.7% of the
match. That evidence was strong and the change improved `q54`.

`q93` disproved it: the real rack reaches **11.68** there -- past the 11.6 commanded target -- with
0.32 V mean applied and 1.87 A mean draw, no stall at all. `q54`'s ceiling was that match's
obstruction, not the mechanism's limit. Capping the sim left the controller with permanent error
against an unreachable target, drawing 2540 amp-seconds against a real 213.

With the bound raised above all observed travel, `q93` currents fell 0.3433 -> **0.2102**.

**The tuned log's score never revealed this.** It is the reason to hold logs back.

---

## What matches well

Measured on q54 against the committed build.

### Brownout behaviour — matches

| | real | sim |
| --- | --- | --- |
| battery minimum | 7.55 V | **7.43 V** |
| samples below the 6.75 V brownout threshold | **0** | **0** |

Baseline sat at a pinned 4.00 V floor with 87 samples below brownout. Before the battery work a
brownout was *structurally impossible* in simulation -- maple-sim's own battery hard-clamps at the
threshold -- and the custom model was being overwritten every loop, so the number was fiction
either way.

### Intake deploy timing — within 6%

| move | real | sim baseline | sim now |
| --- | --- | --- | --- |
| SHOOTING_STOW -> INTAKE | **0.360 s** | 0.219 s | **0.340 s** |
| travel | 8.2 rot | (wrong units) | 8.6 rot |

### Peak power draw — within 8%

| | real | sim baseline | sim now |
| --- | --- | --- | --- |
| peak total current | 318.7 A | 1172.6 A (3.7x) | **293.5 A** (0.92x) |

### Driver input reproduction — exact

Mean absolute axis error **0.0039** over 8254 samples against the logged values.

---

## What does not match

### Mean current draw — the main remaining gap

| | real | sim |
| --- | --- | --- |
| mean total current | 158.3 A | **85.4 A** |

WPILib's `FlywheelSim` and `ElevatorSim` are frictionless, so a mechanism holding setpoint draws
~0 A where the real robot pulls 7-9 A against bearing, belt and game-piece drag. Peaks are right;
the sustained draw between them is not.

**Attempted and reverted:** subtracting a drag voltage. It moved mean current only 113.84 -> 113.50 A,
because subtracting voltage does not create a load -- the plant just settles slightly slower with
its current still near zero. The fix is a plant change: an explicit load torque, or
`LinearSystemId.identifyVelocitySystem(kV, kA)` characterised from the real logs.

This is also why one constant is deliberately unphysical -- see the compensating approximation
below.

### Regenerative current overshoots

Sim reaches -284 A against a real -102 A. Same root cause: with no load to brake against, the
mechanisms dump more energy back than the real ones do.

### Match-specific behaviour the sim cannot know

In q54 the real intake rack never passed 11.290 rotations and stalled against something for 86.7%
of the match, drawing 930 amp-seconds. In q93 the same mechanism reaches 11.68 freely, drawing 213.
The obstruction was specific to that match -- a game piece, most likely -- and nothing in the
driver inputs tells the simulation about it. The sim now models the unobstructed case, so it
under-draws on q54's rack and matches q93's.

### Post-auto robot pose

Not reproduced, and not expected to be: the real robot was defended and contacted. Cumulative wheel
odometry is excluded from scoring for this reason; instantaneous drive velocity is scored and does
match.

---

## A deliberate compensating approximation

`SimCurrentLimit.STATOR_TO_SUPPLY_RATIO` is set to **4.0**, which is *below* the intake rollers'
real measured ratio in all three matches (5.46, 6.02, 5.68). The physically honest value clears
every mechanism.

Setting it to the honest 6.5 measured **worse** on both logs tested -- q54 currents 0.2387 ->
0.2536, q93 0.2102 -> 0.2319 -- consistently and outside the noise floor. The tighter ceiling clips
roller transients that overshoot because those sims have no load, so it compensates for the missing
friction model.

It is labelled as such in the constant's javadoc and pinned by a test, so it is not "corrected" by
accident. **When the plant gains a real load, this should go back above 6.02**, and ideally become
per-mechanism -- the drive ratio is 3.75 in every match to two decimals, while the rollers swing
5.46-6.02.

---

## Measurement caveat, and how it was fixed

Two runs of **identical code** originally produced fit scores differing by 0.016, with 52 of 155
signals moving by >0.01. AdvantageKit stamps records with wall clock, so runs finishing at slightly
different wall times put the same event on different grid points.

That noise floor was the same size as several effects being measured, and two changes were rejected
on differences inside it (a reported-current clamp and the viscous drag). The comparison now rebases
the sim on `Replay/Elapsed Seconds` -- exact match time -- which removes the jitter. Numbers quoted
above are from after that fix.

---

## Every change made

### New files

| file | purpose |
| --- | --- |
| `utility/replay/LogTimeline.java` | zero-order-hold time series with binary-search lookup |
| `utility/replay/MatchInputs.java` | parsed timelines from one match log |
| `utility/replay/MatchLogReader.java` | parses a real `.wpilog` into those timelines |
| `utility/replay/LogInputPlayer.java` | frame-locked injection into `DriverStationSim` |
| `utility/replay/PoseAnchor.java` | teleop pose re-anchoring, gyro kept in step |
| `utility/SimBattery.java` | pack voltage from summed motor current |
| `utility/SimCurrentLimit.java` | closed-form supply-limit voltage ceiling |
| `scripts/log_compare.py` | comparison engine |

### Harness bugs fixed

| what | effect |
| --- | --- |
| Match time never injected | DS reported -1 forever; `ShootCommandFactory` gates on `getTimeUntilOurHubShifts() <= 2`, and `-1 <= 2` is true, so the shooter ran enabled 100% of the sim match against roughly half for real |
| Start pose never seeded for auto replays | q54 began at (13.64, 4.27, 180deg) against the real (12.19, 7.45, 89.8deg) -- 3.5 m and 90 degrees apart before a wheel turned |
| `endCompetition()` called inline | did not stop the loop; must run off-thread |
| `-Preplay.fast` | free-running the loop makes AdvantageKit stamp wall clock, compressing a 165 s match into ~48 s of timestamps. Now warns; unusable for fidelity |

### Simulation model bugs fixed

| file | bug |
| --- | --- |
| `RobotContainer.updateSimulation` | `SimBattery.update()` ran *before* the maple-sim arena tick, which calls `RoboRioSim.setVInVoltage()` 5x per loop -- the battery model was dead code, overwritten every loop |
| `GenericSuperstructureIOSim` | never called `apply(config)` at all, so `SensorToMechanismRatio` was never set: the rack position loop ran in ROTOR units where the real robot runs in MECHANISM units (8/pi = 2.55x). Cause of the intake timing error |
| `IntakeRackIOSim` / `ShooterHoodIOSim` | applied gearing in **opposite directions** -- rack multiplied, hood divided -- so at most one could be right |
| `GenericRollersIOSim` | `stop()` never reached the physics; the accelerator spun at 322 rad/s for 130.5 s of a 165 s match while commanded to stop |
| `GenericRollersIOSim` / `GenericSuperstructureIOSim` | never registered with `MotorOutputManager`, so sim `TotalAmps` counted swerve only |
| 4 roller IOSims | sim PID regulated MECHANISM velocity where Phoenix `VelocityVoltage` regulates ROTOR velocity |
| 6 IOSims | `Math.abs()` on `getCurrentDrawAmps()` booked regenerative braking as consumption; real supply current reaches -69.94 A |
| 4 roller IOSims | clamped applied voltage to a hardcoded +/-12 V, so battery sag could not reduce torque |
| `IntakeRackIOSim`, `SerializerSim` | reported a hardcoded `1.0 A // Not simulated` |
| `SerializerSim` | rotor velocity was rad/s divided by the reduction -- two unit errors on one line -- leaving it running backwards all match; also never set `simState.Orientation` |
| `ShooterOmniwheelIOSim`, `SerializerSim` | never published `positionRads` (1 record of 0.0 for the whole match) |
| `GyroIOSim` | `degreesToRadians()` applied to a value already in rad/s, reporting yaw rate 57.3x too small |
| `DriveConstants` | `mapleSimConfig` never called `withBumperSize`, using maple-sim's 0.76 m default against real 33x37 in bumpers -- which also silently sized the intake, since `RobotSimState` derives it from those dimensions |

### Constants aligned to the real robot (SIM arms only)

| constant | was | now |
| --- | --- | --- |
| `maxLinearVelocity` | 3.75 | 5 |
| `maxLinearAcceleration` | 6 | 8 |
| track width x length | 22.5 x 22.5 in (square) | 19.75 x 24.25 in |
| bumpers | 34 x 34 in | 33 x 37 in |
| wheel radius | 1.925 in | 1.97 in |
| steer gains | `(0.13, 0.79, 0.387, 2)` | `(0.16, 0.67, 0, 1.5)` |
| drive gains | `(0.25, 2.26, 0, 70)` | `(0.24, 2.4, 0.08, 70)` |
| flywheel reduction / gains | 0.71 / `(3,0,0,0,.1)` | 1.411 / `(0.5,0,0,0.2,0.35)` |
| accelerator reduction / gains | 0.67 / `(1,0,0,0,.1)` | 1.5 / `(.6,0,0,0.2,0.17746)` |
| omniwheel reduction / gains | 1 / `(1,0,0,0,.1)` | 1.25 / `(0.4,0,0,0.2,.137)` |
| roller reduction / gains | 2 / `(1,0,0,0,1)` | 2.4 / `(0.3,0,0,0.1,0.2739)` |
| flywheel current limit | 40 A | 20 A |
| omniwheel current limit | 30 A | 60 A |
| rack travel limits | +/-15 m (no stop) | real hard stop at 11.29 rot |
| flywheel / accelerator plant | 1 motor | 4 / 2 motors |

The SIM reductions had been set to the **reciprocal** of the real ones (0.71 vs 1.411, 0.67 vs 1.5)
to compensate for the mechanism-vs-rotor PID bug. The accelerator carried the comment
`// changed in sim (otherwise 1)`. Once the error term was corrected the fudge became unnecessary.

The roller `kV` of 1 meant the feedforward alone demanded 50 V for a 50 rps target, so the simulated
rollers railed at battery voltage permanently.

### One change affecting the real robot (approved)

`GenericSuperstructureIOTalonFX` registered the leader's supply current with `MotorOutputManager`
**twice** (identical statements, and a loop over a list nothing populates). This inflated the real
robot's logged `TotalAmps` by ~6 A mean and up to ~55 A at peaks -- and that series is the
calibration target. Robot behaviour is unaffected; the change is logging-only.

### Changes tried and reverted

| change | why reverted |
| --- | --- |
| Reported-current clamp | Measured worse at both ratios tried. Note: the deltas were inside the then-unknown noise floor, so this is **unresolved rather than disproven**. Helper retained, documented unused |
| Viscous drag | Moved mean current 113.84 -> 113.50 A. Subtracting voltage does not create a load |

---

## Conclusions that rest on a single match, and could still be wrong

Two constants derived from one log survived review, improved the tuning log, and were then
disproved by a validation log. Both had strong-looking evidence.

| inference | evidence from q54 | disproved by |
| --- | --- | --- |
| Intake rack has a hard stop at 11.29 rot | never exceeded 11.290; stalled 86.7% of the match; 930 amp-seconds | q93 reaches 11.68 freely with 0.32 V applied |
| Stator ceiling of 4.0x the supply limit | rack 4.83x, omniwheel 2.66x, drive 3.75x | rollers are 5.46-6.02x across all three matches |

Remaining single-log or single-source inferences, listed so they get checked rather than trusted:

- **Drag coefficients** in `SimCurrentLimit.dragVolts` (unused, but recorded): flywheel 8 A at
  240 rad/s, omniwheel 8 A at 425, accelerator 5 A at 227, rollers 12 A at 177 -- all from q54
  steady states only.
- **`withRobotMass(54.4311 kg)`** is unchanged and unverified. 120 lb is light for a Worlds robot;
  the drivetrain analysis explicitly declined to change it without a scale reading, since no log
  evidence distinguishes it. It sets both the traction limit and the dyn4j inertia.
- **Battery nominal/resistance** are still the shipped 12.8 V / 20 mOhm. A regression on q54 fits
  12.1 V / 13.5 mOhm, but that was measured before the current model was corrected and should be
  refitted, not applied as-is.
- The **q64 brownout stress case has never been run.** It is the one match where the real robot's
  flywheel measurably degraded (up-to-speed time 130.5 s against ~158 s elsewhere, 27.1 s below
  8 V, 5.95 V floor). It is the strongest available test of whether sag-to-torque feedback behaves,
  and it remains untested.
- **q103 is still held out** and has never been run.
