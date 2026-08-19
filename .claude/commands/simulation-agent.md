---
name: simulation-agent
description: Use when the user wants to run, test, or debug the robot simulation. Handles running headless sim, capturing AdvantageKit .wpilog files, and analyzing logs. Trigger on words like "simulate", "run sim", "test in sim", "check the log", "wpilog", or "verify in simulation". Also used as the Verify step after spec-driven-dev.
---

# Simulation Agent — FRC-2026

Run the robot simulation autonomously, capture AdvantageKit logs, and close the debugging loop. This skill drives the "Verify" step of spec-driven development.

**Codebase:** `C:\Users\bruce\Documents\Coding\FRC-2026`

---

## Before Starting

Read `docs/robot-description.md` for subsystem context.

Ask the user:

1. **What to test?** (e.g., "teleop intake sequence", "2x4T auto", "shooter spin-up performance")
2. **Known failure?** (e.g., "the intake doesn't retract when we shoot" — gives focus for log analysis)
3. **New or replay?** Run a fresh sim, or replay an existing `.wpilog` against new code?

---

## Running a Fresh Simulation

### Start the sim headlessly

**To test a specific auto:**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging -Pauto.name=2x4TRight
```

**To run in teleop with scripted inputs:**

```bash
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging "-Pteleop.duration=15" "-Pteleop.buttons=1.0:0:6:true,6.0:0:6:false"
```

See the **Teleop Input Scripting** section below for full button reference and scenario examples.

Available auto names (filename without `.auto`): `1xTBBRight`, `2x4TRight`, `2x4TLeft`, `2x5TRight`, `2x5TLeft`, `2xBBBBRight`, `2xBBBBLeft`, `2xTBBBRight`, `2xTBBBLeft`, `2xTBTBRight`, `2xTBTBLeft`, `Preload AutoRight`, `Preload AutoLeft`

**What happens:**

1. Sim starts, robot initializes (~2s)
2. Robot auto-enables in autonomous mode
3. PathPlanner resets robot pose to the auto's starting position
4. Auto command runs to completion
5. Sim waits 1s to capture final state, then exits cleanly
6. `.wpilog` is written to `build/ai-logs/`

**Watch stdout/stderr for:**

- `HAL: [ERROR]` — HAL initialization failure
- Exception stack traces — usually a missing named command or bad path file
- `Logger: Starting log session` — confirms AdvantageKit started
- `Building auto: <name>` — confirms PathPlanner found the auto file

**Always expected — do NOT treat as errors:**

- `The robot program quit unexpectedly` — this is **always** printed when `endCompetition()` is called, which is how the headless sim exits cleanly after the auto finishes. It is NOT a crash.
- `Joystick Button X on port Y not available` — no controller is connected in headless mode, harmless.
- `Device firmware could not be retrieved` for TalonFX — expected in sim, motors use SimState not real firmware.
- `[AdvantageKit] Auto serialization is not supported for type Vector` — non-fatal logging warning.

**Real errors to act on:**

- Java stack traces with `Exception` or `Error` in the class name
- `BUILD FAILED` (vs `BUILD SUCCESSFUL`)
- Process exits in under 5 seconds (before the log file appears)

If the sim crashes on startup, check `Constants.java` — `ROBOT_TYPE` must resolve to `SIM`.

### Find the log file

```bash
# List .wpilog files, most recent first
Get-ChildItem build/ai-logs/ -Filter *.wpilog | Sort-Object LastWriteTime -Descending
```

The file will be named with a timestamp, e.g., `robot_2026-05-17_14-30-00.wpilog`.

---

## Replaying a Match Log

Replay mode runs the real robot's logged sensor inputs through the current code — deterministic, fast, no GUI needed.

1. Place the `.wpilog` file at a known path (e.g., `build/replay-input/match.wpilog`)
2. Temporarily set `Constants.REPLAY = true` in `Constants.java`
3. Run `./gradlew simulateJava --no-daemon -Pheadless -Pai.logging=true`
4. The output log will be written as `<input>_sim.wpilog`
5. Reset `Constants.REPLAY = false` after

This is ideal for investigating real match failures: the exact same code runs against real sensor data.

---

## Analyzing the Log

### Get a summary first

```bash
python scripts/wpilog_to_csv.py build/ai-logs/<LOG_FILE>.wpilog --summary
```

This outputs:

- Total duration
- All key prefixes and how many records each has
- Any timestamp gaps > 100ms (indicates loop overruns or crashes)

### Extract relevant data

```bash
# Investigation presets — structured report, low token cost
python scripts/wpilog_to_csv.py <file> --investigate shooter
python scripts/wpilog_to_csv.py <file> --investigate drive
python scripts/wpilog_to_csv.py <file> --investigate auto
python scripts/wpilog_to_csv.py <file> --investigate intake

# Narrow any investigation to a time window
python scripts/wpilog_to_csv.py <file> --investigate shooter --from 10.0 --to 16.0

# Direct key query — fetch specific keys with stats + compressed values
python scripts/wpilog_to_csv.py <file> \
  --keys "RealOutputs/Shooter/Target State,RealOutputs/Shooter/Shooter Flywheels/Current Velocity" \
  --from 11.0 --to 14.0

# Raw CSV escape hatch — only when you need every tick (high token cost)
python scripts/wpilog_to_csv.py <file> --prefix RealOutputs/Shooter --out /tmp/shooter.csv
```

**`--keys` output format:**

- Numeric keys: stats (min/max/mean/std) + values sampled every 0.5s, plateau runs collapsed
- String/boolean keys: all transitions with timestamps
- `--from` / `--to` filter all modes to a time window in seconds

**Workflow:** `--summary` to see what exists → `--investigate` for the overview → `--keys --from --to` to zoom into the anomaly window

---

## Closed-Loop Debugging Cycle

When something looks wrong in the logs:

1. **Observe:** Identify the timestamp range where the issue occurs
2. **Inspect:** Extract a narrow time window with `--prefix` and look at related keys
3. **Plan:** Form a hypothesis — is it a code bug, tuning issue, timing problem, or sensor issue?
4. **Implement:** Make the targeted fix using `/spec-driven-dev`
5. **Replay:** Run `./gradlew simulateJava --no-daemon -Pheadless -Pai.logging=true` again with the same scenario
6. **Compare:** Check if the relevant log keys improved

Repeat until the issue is resolved.

---

## Common Key Locations

All subsystem outputs are logged under `RealOutputs/` by AdvantageKit. DriverStation inputs are at the root.

| What you're looking for | Actual log key prefix                                    |
| ----------------------- | -------------------------------------------------------- |
| Robot estimated pose    | `RealOutputs/Robot State/Estimated Pose`                 |
| Swerve position         | `RealOutputs/Swerve/Current Position`                    |
| PathPlanner tracking    | `RealOutputs/Path Planner/`                              |
| Enable / mode state     | `DriverStation/Enabled`, `DriverStation/Autonomous`      |
| Flywheel velocity       | `RealOutputs/Shooter/Shooter Flywheels/Current Velocity` |
| Shooter state machine   | `RealOutputs/Shooter/Target State`                       |
| Intake rack target      | `RealOutputs/Intake/Intake Rack/Target`                  |
| Intake rollers target   | `RealOutputs/Intake/Intake Rollers/Target`               |
| Game pieces in sim      | `RealOutputs/Field Simulation/Fuel Count`                |
| Vision accepted poses   | `RealOutputs/Vision/Camera0/Accepted Poses`              |
| Loop timing             | `RealOutputs/LoggedRobot/FullCycleMS`                    |

---

## Teleop Input Scripting

Headless teleop injects joystick inputs via system properties. The robot enables in teleop mode, runs the script, then exits after the specified duration.

### Command format

```bash
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew simulateJava --no-daemon -Pheadless -Pai.logging \
  "-Pteleop.duration=15" \
  "-Pteleop.buttons=1.0:0:6:true,6.0:0:6:false" \
  "-Pteleop.axes=0.0:0:1:-0.3"
```

- **`teleop.duration`** — total run time in seconds (default 15)
- **`teleop.buttons`** — comma-separated button events: `time:port:button:true|false`
- **`teleop.axes`** — comma-separated axis events: `time:port:axis:value`

### Controller layout

**Driver A — port 0**

| Button       | Number | Action                             |
| ------------ | ------ | ---------------------------------- |
| A            | 1      | Auto-aim shoot (hold)              |
| B            | 2      | Intake                             |
| X            | 3      | Align-to-pose shoot (hold)         |
| Y            | 4      | Stow everything                    |
| Left bumper  | 5      | Align + shoot (hold)               |
| Right bumper | 6      | Default shoot — no auto-aim (hold) |
| Back         | 7      | —                                  |
| Start        | 8      | Zero gyro                          |
| POV Up       | —      | Defense mode                       |
| POV Down     | —      | Trench shoot (hold)                |
| POV Left     | —      | Shooting stow                      |
| POV Right    | —      | Pass to pose                       |

| Axis          | Number | Action                     |
| ------------- | ------ | -------------------------- |
| Left stick X  | 0      | Strafe                     |
| Left stick Y  | 1      | Drive (negative = forward) |
| Left trigger  | 2      | Rotate CCW                 |
| Right trigger | 3      | Rotate CW                  |
| Right stick X | 4      | —                          |
| Right stick Y | 5      | —                          |

**Driver B — port 1**

| Button        | Number | Action                                 |
| ------------- | ------ | -------------------------------------- |
| A             | 1      | Shooting stow                          |
| B             | 2      | Force shoot (hold, bypasses alignment) |
| X             | 3      | Emergency stop all                     |
| Y             | 4      | Intake slow                            |
| Left bumper   | 5      | Reverse intake + shooter               |
| Right bumper  | 6      | Intake down                            |
| POV Down      | —      | Zero shooter                           |
| POV Left      | —      | Zero intake                            |
| Right trigger | —      | Set "being defended" flag              |
| Left trigger  | —      | Clear "being defended" flag            |

> POV (d-pad) injection is not yet supported in scripted teleop. Use button bindings instead.

### Common scenarios

**Test default shoot (simplest — no alignment needed):**

```bash
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew simulateJava --no-daemon -Pheadless -Pai.logging \
  "-Pteleop.duration=12" \
  "-Pteleop.buttons=1.0:0:6:true,8.0:0:6:false"
```

RB held from T=1s to T=8s → ShooterState transitions to DEFAULT_SHOOT, flywheel spins up.

**Test intake then auto-aim shoot:**

```bash
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew simulateJava --no-daemon -Pheadless -Pai.logging \
  "-Pteleop.duration=20" \
  "-Pteleop.buttons=1.0:0:2:true,1.1:0:2:false,5.0:0:1:true,12.0:0:1:false"
```

B pressed at T=1s (intake), A held from T=5-12s (auto-aim shoot).

**Test reverse intake (eject):**

```bash
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew simulateJava --no-daemon -Pheadless -Pai.logging \
  "-Pteleop.duration=10" \
  "-Pteleop.buttons=1.0:1:5:true,5.0:1:5:false"
```

Driver B LB held T=1-5s → IntakeState REVERSE, ShooterState REVERSE.

**Drive forward while shooting:**

```bash
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew simulateJava --no-daemon -Pheadless -Pai.logging \
  "-Pteleop.duration=15" \
  "-Pteleop.axes=0.0:0:1:-0.4" \
  "-Pteleop.buttons=2.0:0:6:true,10.0:0:6:false"
```

Left stick forward (axis 1 = -0.4) immediately, default shoot from T=2-10s.

### After the run — analyze

```bash
python scripts/wpilog_to_csv.py build/ai-logs/<LOG>.wpilog --investigate shooter
python scripts/wpilog_to_csv.py build/ai-logs/<LOG>.wpilog --investigate intake
```

---

## Reporting Results

After analysis, report:

1. **Duration:** How long did the sim run?
2. **Key observations:** What happened in each relevant subsystem?
3. **Timeline:** Reconstruct what the robot did from T=0 to end
4. **Root cause (if investigating a failure):** What specific log evidence points to the bug?
5. **Recommended fix:** Specific file, line, and change — or trigger `/spec-driven-dev` for the fix

**Critical: only report what the logs confirm.** Do not infer or speculate about errors unless you have read the actual log data. If you saw something in stdout during the sim run, verify it against the log before reporting it as a confirmed issue. Stdout warnings during a successful `BUILD SUCCESSFUL` run are usually noise.
