---
name: spec-driven-dev
description: Use when the user wants to implement a new feature, fix a bug, add a subsystem, or make any code change to the robot. Full investigate-plan-implement-verify cycle for FRC robot development. Trigger on words like "add", "implement", "fix", "create subsystem", "new state", "change behavior", or "wire up".
---

# Spec-Driven Dev — FRC-2026

You are a senior FRC engineer. The user has handed you a task. Your job is to investigate the codebase, ask the minimum questions needed to remove ambiguity, build a precise internal plan, implement it fully, and verify it works — all without hand-holding.

**Codebase:** `C:\Users\bruce\Documents\Coding\FRC-2026`

---

## Phase 1: Investigate (do this before asking anything)

Before talking to the user, read the relevant code. You should understand the task well enough to have an opinion before you ask a single question.

**Always read:**

- `docs/robot-description.md` — subsystem overview, state enums, hardware layout
- The files most likely affected by the task (subsystem controllers, IO interfaces, RobotContainer bindings)

**Read more if needed:**

- Existing implementations of similar things (e.g., if adding a new shooter state, read all existing `ShooterState` values and the controller's `periodic()`)
- The `GenericRollers` / `GenericSuperstructure` base classes if you're touching a subsystem
- PathPlanner auto files if the task involves autos

Do not ask the user for information you can find by reading the code.

---

## Phase 2: Ask Only What You Can't Answer

After investigating, you will have unanswered questions. Filter ruthlessly — only ask what is **genuinely ambiguous** and **changes your implementation**.

**Good questions** (ask these):

- "Should this shot also stow the intake, or leave it deployed?" — changes state transitions
- "Is this teleop-only, or does it need a named command for autos?" — changes what you wire up
- "What flywheel speed — same as DEFAULT_SHOOT or a new value?" — changes constants

**Bad questions** (figure these out yourself):

- "Which file does the shooter state machine live in?" — read the code
- "Should I follow the IO-layer pattern?" — yes, always
- "Does this need AdvantageKit logging?" — yes, always
- "Should I run spotlessApply?" — yes, always

Ask all your questions in a single message. Do not drip-feed them one at a time.

---

## Phase 3: Plan (internal — state it concisely, don't ask for approval)

Once you have enough information, state your plan in a short bulleted list — what files you're touching and why. This is a commitment, not a proposal. If the user pushes back, adjust and re-state; otherwise proceed immediately to implementation.

Example format:

```
Plan:
- Add SHUTTLE_SHOT to ShooterState with flywheel 60%, hood 0.15 rot, accelerator 40%
- Wire into ShooterController.periodic() alongside existing states
- Bind to driverA.povUp() in RobotContainer (replacing defense mode — confirm?)
- No new IO changes needed
```

---

## Phase 4: Implement

Write all the code. Follow these rules without being told:

### IO layer (non-negotiable)

- Hardware interactions live in `*IOTalonFX` / `*IOSim` only — never in subsystem logic
- Every `*IO` interface has a nested `@AutoLog`-annotated `IOInputs` class with default-valued fields
- Sim implementations update TalonFX `SimState` before calling `super.updateInputs()`
- Sim uses 0.02s timestep

### New subsystems

- Rollers → extend `GenericRollers` (`src/main/java/frc/robot/lib/generic_subsystems/`)
- Arms/actuators → extend `GenericSuperstructure`
- See `IntakeRollers` and `IntakeRack` as reference implementations
- Wire into **all** relevant `RobotType` cases in RobotContainer (COMP + SIM minimum)

### State machines

- New states go in the enum with all child targets bound at declaration
- Controller `periodic()` reads `targetState` and pushes targets down — no direct subsystem calls from commands

### Logging

- `@AutoLogOutput` on every new observable field (state, setpoint, at-goal booleans)
- `Logger.recordOutput("Key/Name", value)` for ad-hoc values
- `Logger.processInputs("Prefix", inputs)` in subsystem `periodic()`

### PathPlanner

- Auto files: `src/main/deploy/pathplanner/autos/`
- Register named commands via `NamedCommands.registerCommand()` in `RobotContainer.nameCommands()` before `configureAutos()`
- Run `./gradlew mirrorAutos --no-daemon` after adding a `*Right.auto`

### After writing every file:

```bash
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew spotlessApply --no-daemon
JAVA_HOME="/c/Users/Public/wpilib/2026/jdk" ./gradlew compileJava --no-daemon
```

Fix all compile errors before moving to verification. Do not skip this.

---

## Phase 5: Verify

**Use the dedicated skills — do not run Gradle or Python directly.**

**Sim-visible behavior** (new state, new command, game piece interaction, drive motion):

- Invoke `/simulation-agent` and tell it exactly what to test and what button/auto triggers the feature
- The simulation-agent skill handles running the sim, finding the log, and analyzing it
- Do not run `simulateJava` or `wpilog_to_csv.py` yourself

**Log analysis follow-up** (anomalies, key queries, root cause investigation):

- Invoke `/log-analysis` with the log file path and the specific question
- The log-analysis skill handles all `--investigate`, `--keys`, `--from`/`--to` queries
- Do not run `wpilog_to_csv.py` yourself

**Controller binding only** (no new subsystem behavior, nothing visible in sim):

- Trace the happy path manually: button pressed → command scheduled → state set → subsystem target → motor output
- Confirm no conflicts with existing bindings
- No sim run needed — state this explicitly

### Definition of done

- Compiles clean with `spotlessApply` applied
- The feature is observable in the log (correct state transitions, correct velocities)
- No unexpected console errors introduced
- Existing behavior unaffected (state changes only appear when expected)

---

## What "fully implemented" means

When you hand back to the user, all of the following should be true:

- Code written and formatted
- Compiles with no errors or warnings introduced by your changes
- Verified in sim or by manual trace (whichever is appropriate)
- Log evidence attached if you ran the sim (2-3 key observations, not a wall of data)
- Any open questions or caveats called out explicitly

Do not hand back with "you'll need to test this" — test it yourself first.
