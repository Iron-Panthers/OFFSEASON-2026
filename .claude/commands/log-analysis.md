---
name: log-analysis
description: Use when the user wants to investigate a .wpilog file, diagnose a robot failure from a match log, or answer questions about what the robot did during a run. Trigger on words like "wpilog", "log file", "why did", "what happened", "analyze the log", or any reference to a specific .wpilog path.
---

# Log Analysis — FRC-2026

Investigate a WPILib `.wpilog` file to understand robot behavior or diagnose a failure.

**Codebase:** `C:\Users\bruce\Documents\Coding\FRC-2026`

---

## Setup

Ask the user:

1. **Where is the log file?**
   - Sim logs: `build/ai-logs/*.wpilog` — list with `Get-ChildItem build/ai-logs/ -Filter *.wpilog | Sort-Object LastWriteTime -Descending`
   - Match logs: user provides the path
2. **What question are you trying to answer?** e.g. "why did we miss this shot?", "did the auto pick up all the fuel?", "why is the flywheel not reaching speed?"

---

## Tool Reference

All analysis uses `scripts/wpilog_to_csv.py`. Three tiers of granularity:

### Tier 1 — Summary (always run first)

```bash
python scripts/wpilog_to_csv.py <FILE>.wpilog --summary
```

Outputs: total duration, every key with record count and time range. Use this to understand what subsystems logged data and whether the log is valid.

### Tier 2 — Investigation preset

```bash
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate auto
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate shooter
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate intake
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate drive
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate vision

# Narrow to a time window (seconds)
python scripts/wpilog_to_csv.py <FILE>.wpilog --investigate shooter --from 10.0 --to 16.0
```

Outputs a structured report: match state, state machine transitions, continuous channel stats, anomaly flags.

### Tier 3 — Direct key query (for follow-up on specific anomalies)

```bash
python scripts/wpilog_to_csv.py <FILE>.wpilog \
  --keys "KEY1,KEY2,KEY3" \
  --from 11.0 --to 14.0
```

Outputs per key:

- **Numeric:** stats (min/max/mean/std) + sampled values every 0.5s, plateau runs collapsed
- **String/boolean:** all transitions with timestamps

Use this after a preset report flags a suspicious time window — zoom in with `--from`/`--to` and pull exactly the keys you need.

### Tier 4 — Raw CSV (last resort, high token cost)

```bash
python scripts/wpilog_to_csv.py <FILE>.wpilog --prefix RealOutputs/Shooter --out /tmp/out.csv
```

Only use when you need every 20ms tick for a narrow prefix. Avoid if possible.

---

## Log Key Reference

All subsystem outputs are under `RealOutputs/` in AdvantageKit logs. DriverStation inputs are at the root.

| What you're looking for      | Exact key                                                    |
| ---------------------------- | ------------------------------------------------------------ |
| Robot enabled / mode         | `DriverStation/Enabled`, `DriverStation/Autonomous`          |
| Robot estimated pose         | `RealOutputs/Robot State/Estimated Pose`                     |
| Swerve position              | `RealOutputs/Swerve/Current Position`                        |
| PathPlanner target pose      | `RealOutputs/Path Planner/Target Pose`                       |
| PathPlanner current pose     | `RealOutputs/Path Planner/Current Pose`                      |
| Active path waypoints        | `RealOutputs/Path Planner/Active Path`                       |
| Distance from path setpoint  | `RealOutputs/Swerve/Distance From Setpoint`                  |
| Shooter state machine        | `RealOutputs/Shooter/Target State`                           |
| Flywheel velocity (actual)   | `RealOutputs/Shooter/Shooter Flywheels/Current Velocity`     |
| Flywheel velocity (setpoint) | `RealOutputs/Shooter/Shooter Flywheels/Target Velocity`      |
| Flywheels up to speed        | `RealOutputs/Shooter/Flywheels Up To Speed`                  |
| Hood target position         | `RealOutputs/Shooter/Shooter Hood/Target Position`           |
| Accelerator target velocity  | `RealOutputs/Shooter/Shooter Accelerator/Target Velocity`    |
| Intake rack target           | `RealOutputs/Intake/Intake Rack/Target`                      |
| Intake rack reached target   | `RealOutputs/Intake/Intake Rack/Reached Target`              |
| Intake rollers target        | `RealOutputs/Intake/Intake Rollers/Target`                   |
| Fuel count (sim)             | `RealOutputs/Field Simulation/Fuel Count`                    |
| Vision accepted poses        | `RealOutputs/Vision/Camera0/Accepted Poses`                  |
| Shooting state predictor     | `RealOutputs/RobotState/Target Shooting State/Shooter Angle` |
| Loop cycle time              | `RealOutputs/LoggedRobot/FullCycleMS`                        |

---

## Investigation Templates

### Shooter not reaching speed

```bash
python scripts/wpilog_to_csv.py <FILE> --investigate shooter
```

Then zoom into the spin-up window:

```bash
python scripts/wpilog_to_csv.py <FILE> \
  --keys "RealOutputs/Shooter/Target State,RealOutputs/Shooter/Shooter Flywheels/Current Velocity,RealOutputs/Shooter/Shooter Flywheels/Target Velocity,RealOutputs/Shooter/Flywheels Up To Speed" \
  --from <T_start> --to <T_end>
```

**What to look for:**

- Velocity plateauing far below setpoint → likely a code bug (NPE or wrong motor output), not tuning
- Velocity oscillating around setpoint → PID kP too high
- Setpoint never logged (only 1 record) → `DEFAULT_SHOOT` target velocity may be null (known NPE)
- `Flywheels Up To Speed` never goes true → shoot command will wait forever

### Auto sequence wrong

```bash
python scripts/wpilog_to_csv.py <FILE> --investigate auto
```

Then for path tracking detail:

```bash
python scripts/wpilog_to_csv.py <FILE> \
  --keys "RealOutputs/Path Planner/Target Pose,RealOutputs/Swerve/Current Position,RealOutputs/Shooter/Target State,RealOutputs/Intake/Intake Rack/Target" \
  --from <T_anomaly - 1> --to <T_anomaly + 2>
```

**What to look for:**

- Path error > 0.25m at start of a segment → robot didn't finish previous segment cleanly
- State machine stuck in `INTAKE` when shooter should be spinning up → event trigger timing
- No `Path Planner/Target Pose` records → PathPlanner auto file not found or wrong name

### Intake not picking up

```bash
python scripts/wpilog_to_csv.py <FILE> --investigate intake
```

```bash
python scripts/wpilog_to_csv.py <FILE> \
  --keys "RealOutputs/Intake/Intake Rack/Target,RealOutputs/Intake/Intake Rack/Reached Target,RealOutputs/Intake/Intake Rollers/Target,RealOutputs/Field Simulation/Fuel Count" \
  --from <T_intake_window>
```

**What to look for:**

- `Intake Rack/Target` shows `INTAKE` but `Reached Target` is false → arm not reaching position (tuning or obstruction)
- `Fuel Count` not decreasing → robot isn't over the fuel, or rollers wrong direction
- `Intake Rollers/Target` shows `IDLE` during intake window → state machine bug

### Drive not following path

```bash
python scripts/wpilog_to_csv.py <FILE> --investigate drive
```

```bash
python scripts/wpilog_to_csv.py <FILE> \
  --keys "RealOutputs/Swerve/Distance From Setpoint,RealOutputs/Swerve/Drive Mode,RealOutputs/Robot State/Estimated Pose" \
  --from <T_segment>
```

**What to look for:**

- Distance From Setpoint stays high (>0.1m) → constraint violation or pose estimation drift
- Drive mode not `PATH_FOLLOWING` during auto → auto command not scheduled
- Pose jumps → vision fusing a bad estimate

---

## Step-by-Step Workflow

1. **Summary** — duration, key presence, record counts
2. **Preset report** — get the full picture for the relevant mode
3. **Identify anomaly window** — specific timestamp where behavior diverges
4. **Key query** — `--keys ... --from T1 --to T2` to zoom in on exactly that window
5. **Diagnose** — state transitions + continuous values together tell the story
6. **Report:**
   - Answer to the question (plain English)
   - Supporting evidence (timestamps + key values)
   - Root cause (specific file, line, or state)
   - Recommended fix — or trigger `/spec-driven-dev`
   - Confidence: "clearly visible in logs" vs "inferred from Y"

**Only report what the logs confirm.** Do not speculate about causes without log evidence.
