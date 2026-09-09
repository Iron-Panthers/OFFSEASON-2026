# Log-Driven Simulation Fidelity — Design

**Date:** 2026-09-08
**Branch:** `feat/log-based-sim-optimization`

## Goal

Make the simulation reproduce real-match behavior when fed the same driver inputs. Specifically: power draw, brownout timing, and mechanism position/velocity curves should match a real Worlds match log closely enough that the sim is trustworthy for testing changes offline.

## Non-Goals

- Reproducing robot pose after autonomous. Real matches involve contact, defense, and blocking that the sim does not model. Pose is trusted during auto only.
- Reproducing scoring outcomes or game-piece trajectories.
- Changing real-robot behavior. Shared constants may change only with explicit approval.

## Ground Truth

Twelve AdvantageKit `.wpilog` files from the Johnson division at Worlds. All driver input is logged: `DriverStation/Joystick{0,1}/{AxisValues,ButtonValues,POVs}`.

Log selection, ranked by shot throughput (accelerator-on seconds, shot bursts, intake cycles):

| Role | Match | Rationale |
| --- | --- | --- |
| Primary (tune) | `johnson_q54` | Busiest match: 51.7 s shooting, 16 bursts, 46 intake cycles. Mild browning (0.2 s under 8 V) so the baseline fit is not dominated by the battery model. |
| Validation 1 | `johnson_q14` | High throughput plus real browning (8.7 s under 8 V, min 7.00 V). Exercises the battery model. |
| Validation 2 | `johnson_q93` | High throughput, negligible browning. Different battery condition — tests that per-log `R_int` fitting generalizes. |
| Held out | `johnson_q103` | Most shot bursts (17). Untouched until the final honest check. |
| Stress case | `johnson_q64` | 27.1 s under 8 V, 5.95 V floor. The only match where flywheel up-to-speed time collapsed (130.5 s vs ~158 s). Evidence that sag-to-torque-loss feedback is real. Not a tuning target. |

Excluded: `q3` (user request), `q76` / `e9` / `q114` (half the throughput of the pack — likely heavily defended), `e7` (only 57.8 s enabled, aborted).

## Current State

- `Robot.java` has headless teleop scripting driven by `System.currentTimeMillis()` on a side thread. Not frame-accurate; POV injection unsupported.
- **No battery model exists.** Zero references to `BatterySim` or `RoboRioSim` in the codebase. Sim voltage is a fixed 12 V, so brownouts cannot occur in simulation at all.
- `DriveConstants.mapleSimConfig` carries a literal `TODO: update this to be similar to comp bot drive base`; mass is 54.4311 kg, wheel COF 1.4, steer MOI 0.04 kg·m².
- `wpilog_to_csv.py` has `--summary`, `--investigate`, `--keys`, `--prefix`. No comparison mode.

## Components

### 1. `LogInputPlayer` (new, SIM-only)

`src/main/java/frc/robot/utility/LogInputPlayer.java`

Reads a real `.wpilog` at construction using `edu.wpi.first.util.datalog.DataLogReader` (confirmed present in wpiutil 2026.2.1) and builds zero-order-hold timelines:

| Log key | Injected via |
| --- | --- |
| `DriverStation/Joystick{0,1}/AxisValues` (float[]) | `GenericHIDSim.setRawAxis` |
| `DriverStation/Joystick{0,1}/ButtonValues` (int64 bitfield) | `GenericHIDSim.setRawButton` |
| `DriverStation/Joystick{0,1}/POVs` (int64[]) | `GenericHIDSim.setPOV` |
| `DriverStation/Enabled`, `/Autonomous`, `/AllianceStation` | `DriverStationSim` |
| `NetworkInputs/SmartDashboard/Auto Chooser` | auto selection |

Time base is rebased so `t=0` is the log's first `Enabled -> true`. The player is advanced from `robotPeriodic()` by exactly one `Constants.PERIODIC_LOOP_SEC` step per loop — not by wall clock — so injection is frame-exact and correct even when the loop free-runs.

Shuts the sim down via `endCompetition()` after the logged final disable plus a 1 s buffer.

Flags:

- `-Preplay.inputs=<path>` — the real log to replay (required to enter this mode)
- `-Preplay.fast` — `setUseTiming(false)` to free-run the loop. Best-effort: if maple-sim or CTRE sim misbehave untimed, fall back to realtime and report it.
- `-Preplay.teleopOnly` — skip auto, start at the logged teleop-entry pose

### 2. Pose re-anchoring

Auto runs free (pose is trusted there). During teleop, every `-Preplay.anchor=<sec>` (default `10.0`, `0` disables) the maple-sim drivetrain pose is set to the logged `RealOutputs/Robot State/Estimated Pose`, preserving chassis velocity. Always anchors once at teleop entry.

Rationale: prevents the sim robot from wedging against a wall and producing a fictitious current draw — which would corrupt the primary signal being measured — while leaving a long enough window between corrections for pose drift to accumulate visibly.

A 10 s interval is a deliberate tradeoff. A tight interval (~1 s) would keep auto-aim distance and the shooter LUT seeing near-real inputs, but it also continuously erases drift and would hide a genuinely wrong drivetrain model. At 10 s the drift between corrections is itself a measurement: the pose error accumulated just before each anchor is logged as `Replay/Anchor Error` (translation and rotation) so drivetrain fidelity can be scored directly rather than assumed. The cost is that auto-aim inputs degrade late in each window; comparisons of shooter LUT outputs are weighted toward the first seconds after an anchor.

### 3. Battery model (new)

`src/main/java/frc/robot/utility/SimBattery.java`, ticked each sim loop:

1. Sum supply current from every sim IO.
2. `BatterySim.calculateDefaultBatteryVoltage(nominalV, rInternal, currents)`.
3. `RoboRioSim.setVInVoltage(v)`.
4. **Feed `v` back into every TalonFX `SimState.setSupplyVoltage(v)`** so motors lose torque as the pack sags.

Step 4 is what makes the model physical rather than cosmetic. Without it, the logged voltage number would move but nothing would respond to it — and `q64` proves the real robot's shooter degrades under sag.

Because battery condition varied between matches, nominal voltage and internal resistance are per-run overridable: `-Preplay.battery=<nominalV>:<rOhms>`. These are fit per log and the spread is reported rather than forcing one value across all matches.

Brownout state is modelled to the roboRIO cutoff so `SystemStats/BrownedOut` and motor shutoff behave as they do on the real robot.

### 4. `--compare` mode in `scripts/wpilog_to_csv.py`

```
python scripts/wpilog_to_csv.py --compare SIM.wpilog REAL.wpilog [--json fit.json] [--top N]
```

- Aligns both logs on first-enable; reports the offset applied.
- Every numeric key present in both is resampled onto a common 20 ms grid, then scored for normalized RMSE, mean shift, p95 shift, peak shift, and shape correlation.
- Keys ranked by divergence; each reported with its three worst time windows.
- **Event-aligned diff:** state transitions (`Intake Rack/Target`, `Shooter/Target State`, `Shooter Accelerator/Target`, ...) are paired in order between the two logs, and per-event timing is compared — e.g. `STOW->INTAKE: real settles 0.34 s, sim 0.51 s (+50%)`. Robust to the two runs drifting apart in absolute time.
- `--json` writes category fit scores (mechanisms / currents / voltage / aggregate) so each tuning round can be shown to improve or regress objectively rather than by eye.

### 5. Skill updates

- `.claude/commands/simulation-agent.md` — new "Log-Driven Replay" section documenting the flags, the pose-anchoring caveat, and what is and is not trustworthy after auto.
- `.claude/commands/log-analysis.md` — new "Comparison Mode" section.

## Fidelity Targets

Optimized against, in order of trustworthiness:

1. **Mechanism position/velocity curves** — intake rack `PositionRotations`, hood position, flywheel `Current Velocity`. Same rise/settle times and steady-state values. Unaffected by robot-to-robot contact, so this is the most reliable signal.
2. **Per-mechanism current draw** — stator and supply current for all four swerve modules (drive and steer) plus flywheels, rack, rollers, hood, accelerator, omniwheel, serializer. Matched on mean, p95, peak, and shape during events.
3. **Battery voltage and brownout timing** — sag events at the same match times with similar depth.
4. **Aggregate power** — `MotorOutputManager/TotalAmps` and cumulative `TotalAmpSeconds`.
5. **Drivetrain pose drift** — `Replay/Anchor Error`, the translation and rotation error accumulated in each 10 s window before the pose is re-anchored. A drivetrain model with the right mass, MOI, and wheel friction should drift slowly and without systematic bias; large or consistently-signed drift points at a specific modelling error. Valid only in stretches where the real robot was not being hit, so outliers are inspected rather than averaged in.

## Constraints

- Sim-only constants, sim-only models, and `*IOSim` implementations may be changed freely.
- Shared constants that COMP also reads (current limits, gear ratios, PID gains) may be changed **only after presenting log evidence and getting explicit approval per change**.
- Real-robot logic is out of scope.

## Process

1. Build `LogInputPlayer`, `SimBattery`, and `--compare`. Commit each separately.
2. Replay `q54`, compare against the real log. **Checkpoint.**
3. Dispatch 5 parallel analysis agents against the comparison output to find tunable divergence sources. Compile findings. **Checkpoint.**
4. Apply fixes, resim, iterate on `q54` until the fit scores converge.
5. Validate on `q14` and `q93`; re-tune only what generalizes. **Checkpoint per milestone.**
6. Final honest check on the held-out `q103`. Report fit scores and the per-log battery spread.

## Risks

- **Overfitting to one match.** Mitigated by the held-out log and by requiring changes to improve validation logs, not just the primary.
- **`-Preplay.fast` may break maple-sim or CTRE sim.** Fall back to realtime; runs cost ~3 min each instead.
- **Pose anchoring could mask a genuine drivetrain modelling error** by hiding accumulated tracking error. Mitigated three ways: the anchor interval defaults to 10 s rather than 1 s so drift has room to develop; the drift at each correction is logged as `Replay/Anchor Error` and scored as a fidelity target in its own right; and the auto segment (free-running, pose trusted) is scored separately from teleop.
- **A 10 s anchor interval degrades auto-aim inputs late in each window.** As the sim pose drifts, distance-to-goal diverges and the shooter LUT is queried at the wrong distance, so hood angle and flywheel setpoint stop being comparable. Mitigated by weighting shooter-setpoint comparisons toward the seconds immediately after each anchor, and by treating the anchor interval as tunable if it proves too coarse.
- **Per-log battery fitting could absorb errors that belong elsewhere.** Mitigated by fitting the battery parameters once per log and holding them fixed while tuning everything else, and by reporting the spread — an implausibly wide spread means the model is absorbing something it should not.
