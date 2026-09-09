# Round 1 Findings — q54 baseline

Baseline fit (mean nRMSE, lower better): aggregate 0.2949 | currents 0.4510 | mechanisms 0.4008 | voltage 0.4321

---

## CRITICAL: maple-sim silently overrides the battery model

Verified directly, not inferred. In the baseline sim log the two voltage signals disagree:

| signal | min | max | mean |
| --- | --- | --- | --- |
| `RealOutputs/SimBattery/Voltage` (our model) | 4.000 | 12.800 | 10.366 |
| `SystemStats/BatteryVoltage` (what motors actually read) | 9.897 | 14.926 | 12.167 |

14.9 V is above our 12.8 V nominal, so something else writes that value.

**Cause:** `org.ironmaple.simulation.motorsims.SimulatedBattery.simulationSubTick()` calls
`RoboRioSim.setVInVoltage(...)` on every maple-sim sub-tick. `RobotContainer.updateSimulation()`
calls `SimBattery.update()` *first* and then `SimulatedArena.getInstance().simulationPeriodic()`,
so maple-sim clobbers our value every loop. Our battery model has been inert.

**maple-sim's battery has three properties that block the goal outright:**

1. `BATTERY_NOMINAL_VOLTAGE = 13.5` — hardcoded, not tunable per match.
2. `LinearFilter.movingAverage(50)` on total current — smooths away exactly the peaks that cause
   real brownouts.
3. It clamps voltage *at* the brownout threshold:
   `if (v < RoboRioSim.getBrownoutVoltage()) v = RoboRioSim.getBrownoutVoltage();`
   **Voltage can never go below brownout, so a brownout can never be simulated.** This alone makes
   "browning out at similar parts of the match" impossible in the current setup.
4. Only appliances registered via `addElectricalAppliances` / `addMotor` count. Our 7 mechanism
   sims are invisible to it; only the 8 swerve motors contribute.

**Fix direction:** move `SimBattery.update()` to run *after* `SimulatedArena.simulationPeriodic()`
so our value is the one that survives the loop, and additionally register mechanism currents into
`SimulatedBattery.addElectricalAppliances()` so maple-sim's own internal motor sims see a sane
voltage during their sub-steps. Then our tunable nominal/R and true sub-brownout sag apply.

Note the real robot's `MotorOutputManager/TotalAmps` reaches **-101.7 A** (regenerative braking),
and maple-sim's 14.9 V maximum is consistent with regen raising pack voltage above nominal. Our
`SimBattery.computeVoltage()` currently discards negative current; that choice needs revisiting.

---

## Intake agent findings (evidence-backed, verbatim priorities)

### Measured intake deploy/retract timing, real vs sim

| Move | REAL t90 | SIM t90 | ratio |
| --- | --- | --- | --- |
| STOW -> INTAKE (full deploy) | 0.650 s | 0.300 s | sim 2.2x too fast |
| SHOOTING_STOW -> INTAKE | 0.380 s | 0.241 s | sim 1.6x too fast |
| INTAKE -> SHOOTING_STOW (retract) | 1.287 s | 1.301 s | matches |

Retract matches because those targets carry `maxCruiseVelocity = 6`, so a Java `ProfiledPIDController`
limits both worlds. Deploy is uncapped and runs at plant limits — that is where the model is wrong.
Peak deploy velocity: real 21.7 rot/s at 6.4 V; sim 53.6 rot/s at 6.8 V. Same voltage, 2.5x velocity.

### SIM-ONLY, ranked

1. **`GenericSuperstructureIOSim` never applies `SensorToMechanismRatio`** (`:32-34`, `:87-88` only
   applies gains and motion magic; `talon.getConfigurator().apply(config)` is never called).
   COMP does apply it (`GenericSuperstructureIOTalonFX.java:71`). With reduction 8/pi = 2.546 the sim
   position loop runs in **rotor** units while COMP runs in **mechanism** units — 2.55x error in
   distance, kP stiffness and cruise velocity. Root cause of the deploy timing error. Affects every
   sim superstructure (rack and hood).
2. **`IntakeRollersIOSim` never stops** — `stop()` doesn't clear `velocitySetpointRPS`, and
   `updateInputs` ignores the talon's `NeutralOut`. During `IDLE` the sim rollers still run at
   343 rad/s and 13.5 V (real: 0.05 rad/s, ~0 V).
3. **SIM roller gains are wrong and in the wrong units** — `IntakeRollersConstants.java:25`
   `PIDGains(1, 0, 0, 0, 1, 0, 0)` vs COMP `(0.3, 0, 0, 0.1, 0.2739, 0, 0)`. kV=1 means the
   feedforward alone demands 50 V for the 50 rps target, so the sim rails at battery voltage
   permanently. `INTAKE_SLOW` is worse: the sim drives the rollers **backwards** at -11.5 V,
   which explains `Filtered Current corr = -0.355`.
4. **SIM roller reduction is 2, COMP is 2.4** (`IntakeRollersConstants.java:11` vs `:15`) — a flat
   20% error in every velocity/position/current conversion.
5. **Sim rack has no hard stop** (`IntakeRackConstants.java:86`, min/max +-15 m = +-60.8 rot). The real
   rack physically stops at 11.29 rot and stalls into it, holding 85-102 A stator / 25-30 A supply for
   86.7% of the match. Sim converges cleanly and draws ~0. This is most of the 930.7 vs 63.3
   amp-second gap.
6. **Rack `ElevatorSim` has essentially no load** — mass 0.1 kg, drum 0.1 m, gravity off gives zero
   steady-state load torque; the real rack needs ~0.58 Nm at the motor. Suggested refit:
   `drumRadiusMeters ~= 0.0157`, `massInKilograms ~= 9.6`, `simulateGravity = true`.
7. **Current limits are not enforced anywhere in sim physics** — three independent leaks:
   `GenericSuperstructureIOSim` never applies its config; `setSupplyCurrentLimit` is a no-op
   `default {}` outside the TalonFX class; and `IntakeRollersIOSim` bypasses the talon closed loop
   entirely so the 30 A limit applied in `GenericRollersIOSim`'s constructor is inert.
   Real rack supply p95 = 27.96 A against a 27 A limit (clearly saturating); sim max 10.57 A.

### COMP-AFFECTING — require Bruce's approval, do not change unilaterally

- **A. `IntakeRackTarget.INTAKE = 11.6` is past the physical hard stop at ~11.29 rot**
  (`IntakeRack.java:20`). The real robot stalls into it for a large fraction of every match:
  86.7% of the match above 10 rot, drawing 25-30 A supply against a 27 A limit, ~930 amp-seconds
  on a mechanism that should cost nearly nothing to hold. Confidence HIGH that the stall is real;
  the correct target value is a mechanical judgement.
- **B. `MOTION_MAGIC_CONFIG` cruise velocity 40 rot/s exceeds mechanism free speed**
  (`IntakeRackConstants.java:35`). 40 mech rps = 101.8 rotor rps vs ~100 rps free speed, so Motion
  Magic is permanently saturated and the profile is decorative. Observed real peak 21.7 rot/s.
- **C. Roller `kV = 0.2739` saturates the feedforward at every operating point**
  (`IntakeRollersConstants.java:26`). FF for the 50 rps target = 13.80 V, above the bus, so the
  rollers run effectively open loop and settle 35% over setpoint.

### Suggested order

Land 2 and 4 first (trivial, large effect). Then 1 (the unit fix — root cause, shifts every rack
number). Then 5 and 7 (hard stop + current limits, recovers most of the amp-second gap). Then 3 with
corrected units. Leave 6 for a final tuning pass — tuning mass against a sim with wrong units and no
current limits would bake in compensating errors.

---

## Shooter agent findings

Decomposed the worst signal (Omniwheel Total Amp Seconds, nRMSE 5.213). The `corr=0.996` is an
artifact of comparing two monotonically-increasing integrals, NOT evidence of "one wrong scalar":

| phase | SIM A.s | REAL A.s | share of error |
| --- | --- | --- | --- |
| spin-up | 4231 | 628 | 38% |
| spin-down | 4461 | 26 | **47%** |
| steady + idle | 1683 | 214 | 15% |

### SIM-ONLY, ranked

1. **Supply current limits are discarded in sim.** `GenericRollersIOSim` has no
   `setSupplyCurrentLimit` override, so `GenericRollers.periodic():60` calls the no-op default
   (`GenericRollersIO.java:25`) every loop. `FlywheelSim.getCurrentDrawAmps()` is unbounded and the
   IOSims feed `setInputVoltage()` from their own PID, so the TalonFX config never touches physics.
   Real flywheel spin-up ramps 2.2 -> 7.3 V over 1.1 s (the shape of a supply-limited motor); sim
   jumps to 11 V on loop 1 and pulls 445 A vs the real 19 A. Drives 6 of the 10 worst shooter signals.
2. **Regen reported as positive draw.** All four roller IOSims do `Math.abs(getCurrentDrawAmps())`.
   On a 90 -> -1 target flip the sim commands -13.5 V into a wheel at 553 rad/s and reports **952 A**
   supply -- above the Kraken's 483 A stall current. Real `Omniwheel/SupplyCurrentAmps` min is
   **-69.94 A**. 47% of the worst signal's error.
3. **`stop()` leaves the sim PID on a stale setpoint.** Sim accelerator spends **130.5 s of a 165 s
   match** in `ControlMode.STOP` while still spinning at 322 rad/s with 4.24 V applied. Explains
   `Accelerator/PositionRads` (+21328 rad of phantom rotation) and `VelocityRadsPerSec` corr 0.121.
4. **SIM reductions are the reciprocal of COMP** -- verified independently:
   flywheel SIM 0.71 vs COMP 1.411 (1/1.411 = 0.709); accelerator SIM 0.67 vs COMP 1.5, carrying the
   comment `// changed in sim (otherwise 1)`; omniwheel SIM 1 vs COMP 1.25; serializer SIM 5 vs
   COMP 2.833. These were fudged to compensate for 4b rather than fix it.
4b. **Sim PID regulates mechanism velocity; the real TalonFX regulates rotor velocity.** Measured
   steady ratios: omniwheel sim 0.979 vs real 0.760; accelerator sim 1.016 vs real 0.716.
   Fix the error term to `setpoint - mechVel * reduction`. Do NOT chase flywheel velocity -- it
   already scores 0.206 and could regress.
5. **`SerializerSim` is functionally non-simulated.** Never moves all match: `AppliedVolts` range
   [-3.359, 0.0] (never positive), `StatorCurrentAmps` min = max = 0.000. Real is [0, 11.11] V and
   0-42 A. Four compounding defects: velocity unit error (4x too slow), `rotations = 0` pinned,
   missing `simState.Orientation`, and `appliedVelocity` misuse.
6. **`positionRads` never written** by omniwheel and serializer IOSims (n=1, value 0 all match;
   real reaches 6954.9 and 9649.5 rad). ~2 lines.
7. **Sim logs `appliedVelocity`, the real robot never populates it** (`SerializerSim.java:65`).
   Real value is 0.000 for the whole log; the comparator scores it nRMSE 2.757 as pure noise.
   Delete the line.
8. **MOI 10x lower in SIM than COMP, and every mechanism modelled as ONE motor** (real flywheel has
   4, accelerator 2). Energy balance from the real 1.08 s spin-up gives J ~= 0.012 for the assembly,
   so 0.01 is about right *for the assembly* -- but with 1/4 the torque authority. Only tune after
   the current limit lands. Medium confidence.
9. **No friction: sim steady-state current is exactly zero** while the real robot draws 7-9 A.
   Explains flywheel `Filtered Current` mean_shift -6.238 (sim under-draws on average) alongside
   peak_shift +68.9 (over-draws in transients), corr 0.019.
10. **Independently found the maple-sim battery override** described at the top of this document.
11. Hand-rolled PID vs TalonFX closed loop is the structural root of 1, 3, 4b. Empirical argument:
    `ShooterHoodIOSim` drives physics from `getMotorVoltage()` and is by far the best-fitting shooter
    mechanism (nRMSE 0.041-0.132 across all six signals) vs 0.2-5.2 for the four hand-PID rollers.
12. **Inversion mapping is inverted between sim and real base classes.** `GenericRollersIOSim:25-26`
    maps the opposite way to every `*IOTalonFX`. Currently masked on omniwheel/accelerator because
    their IOSims bypass the Talon. **Fix the SIM side only** -- touching the TalonFX mapping would
    reverse real motor directions.

**No COMP-affecting change required for any shooter item.**
