# FRC-2026 Robot Codebase — Claude Context

Team 6328 (Mechanical Advantage) robot code for the 2026 FRC season.

## Build Commands

```bash
./gradlew compileJava --no-daemon          # compile only (fast check)
./gradlew build --no-daemon                # compile + spotless check + tests
./gradlew spotlessApply --no-daemon        # fix all formatting (Google Java Format)
./gradlew simulateJava --no-daemon         # run sim with GUI (normal dev use)
./gradlew simulateJava --no-daemon -Pheadless -Pai.logging=true  # run sim headlessly, writes .wpilog to build/ai-logs/
./gradlew test --no-daemon                 # run JUnit tests
./gradlew mirrorAutos --no-daemon          # auto-generate Left-mirrored autos from Right variants
```

**Always pass `--no-daemon`** for every Gradle invocation in agent/automated contexts. Without it, the Gradle wrapper process hangs after "BUILD SUCCESSFUL" on Windows because it stays connected to the background daemon instead of exiting cleanly.

Spotless runs automatically on `build` and `deploy`. **Always run `spotlessApply` before committing.** The build will fail with a diff if formatting is wrong.

## Robot Modes & Types

```java
// Constants.java
public static final boolean REPLAY = false;          // set true to replay a log file
public static RobotType ROBOT_TYPE =
    (RobotBase.isReal() || REPLAY) ? RobotType.COMP : RobotType.SIM;  // auto-switches
```

`RobotType` enum: `COMP`, `ALPHA`, `VISION`, `SIM`
`Mode` enum: `REAL` (COMP/ALPHA/VISION), `SIM`, `REPLAY`

- To run in sim: `ROBOT_TYPE` auto-resolves to `SIM` when not on real hardware and `REPLAY=false`
- To replay a log: set `REPLAY = true`, place the `.wpilog` file, and point `LogFileUtil.findReplayLog()` to it
- `BuildConstants.java` is **auto-generated** — never modify it

## IO-Layer Pattern (Critical — follow this for every hardware interaction)

Every subsystem abstracts hardware behind a `*IO` interface. There are always three implementations:

| Implementation                | When used                                     |
| ----------------------------- | --------------------------------------------- |
| `*IOTalonFX` / `*IOReal`      | `COMP`, `ALPHA`, `VISION` robot types         |
| `*IOSim`                      | `SIM` robot type (WPILib physics sim classes) |
| Auto-generated replay wrapper | `REPLAY` mode (AdvantageKit generates this)   |

**Rules:**

- Never call hardware (CAN, sensors, actuators) from subsystem logic classes — only from IO implementations
- Every `*IO` interface has a nested `@AutoLog`-annotated `IOInputs` class. All fields must have default values.
- The subsystem calls `Logger.processInputs("SubsystemName", inputs)` in `periodic()`
- Sim implementations must update TalonFX `SimState` **before** calling `super.updateInputs()`
- Sim implementations use a **0.02s timestep** (matches `Constants.PERIODIC_LOOP_SEC`)

## Generic Base Classes (use these for new subsystems)

Located at `src/main/java/frc/robot/lib/generic_subsystems/`:

- **`GenericRollers`** + `GenericRollersIO/IOSim/IOTalonFX` — any spinning wheel mechanism (rollers, flywheels, omniwheels)
- **`GenericSuperstructure`** + `GenericSuperstructureIO/IOSim/IOTalonFX` — any single-jointed arm or linear actuator

New subsystems **must** extend these rather than implementing from scratch. See `IntakeRollers` (rollers) and `IntakeRack` (superstructure) as reference implementations.

## AdvantageKit Logging

```java
@AutoLogOutput                           // auto-log a field every loop
Logger.recordOutput("Key/Name", value);  // log ad-hoc values
Logger.processInputs("Prefix", inputs);  // log IO inputs struct (in periodic())
```

Every observable state (target, current state, setpoint, at-goal booleans) should be `@AutoLogOutput`. Log key names follow `SubsystemName/FieldName` conventions. Replay mode is deterministic — the exact same code runs against the logged inputs.

## State Machine Pattern

Controllers (`IntakeController`, `ShooterController`) own child subsystems and drive them via state enums:

```java
public enum IntakeState {
    STOW(IntakeRackTarget.STOW, IntakeRollersTarget.IDLE),
    INTAKE(IntakeRackTarget.INTAKE, IntakeRollersTarget.INTAKE),
    REVERSE(IntakeRackTarget.INTAKE, IntakeRollersTarget.EJECT),
    // ...
}
```

Each enum value binds a target for every child subsystem. The controller's `periodic()` reads `targetState` and pushes targets down. Do not call child subsystem methods directly from commands — set the controller's state instead.

## Subsystem Hierarchy

```
Drive (swerve)
  └── Module ×4  (ModuleIOTalonFXReal / ModuleIOTalonFXSim)
  └── GyroIO     (GyroIOPigeon2 / GyroIOSim)

IntakeController
  ├── IntakeRack    (GenericSuperstructure, single-jointed arm)
  └── IntakeRollers (GenericRollers, spinning wheels)

ShooterController
  ├── ShooterFlywheel    (GenericRollers, 4 motors)
  ├── ShooterHood        (GenericSuperstructure, angle adjustment)
  ├── ShooterAccelerator (GenericRollers)
  ├── ShooterOmniwheel   (GenericRollers)
  └── Serializer         (GenericRollers, game piece feeding)

Vision  (VisionIOPhotonvision / VisionIOPhotonvisionSim)
RGB     (RGBIO only — LEDs)
CANWatchdog
```

Full subsystem detail: see `docs/robot-description.md`

## RobotContainer Pattern

Subsystems are instantiated in `RobotContainer` inside a `switch (Constants.getRobotType())` block:

```java
switch (Constants.getRobotType()) {
    case COMP -> { /* real TalonFX IOs */ }
    case SIM  -> { /* sim IOs, IronMaple physics */ }
    // ...
}
```

When adding a new subsystem: add it to **every** relevant case (or comment why it's omitted). Controller bindings use `CommandXboxController`: port 0 = driverA, port 1 = driverB. Use `.onTrue()`, `.whileTrue()`, `.onFalse()`.

## PathPlanner Autos

- Auto files: `src/main/deploy/pathplanner/autos/*.auto`
- Path files: `src/main/deploy/pathplanner/paths/*.path`
- Mirror utility: `./gradlew mirrorAutos` auto-generates `*Left.auto` from `*Right.auto` variants
- Named commands are registered in `RobotContainer` via `NamedCommands.registerCommand("name", command)`
- **All named commands referenced in `.auto` files must be registered** — CodeRabbit checks this

## Key Singletons

- `RobotState.getInstance()` — pose estimation (`SwerveDrivePoseEstimator`), shooting LUT (`InterpolatingTreeMap`), vision fusion
- `RobotSimState.getInstance()` — **SIM mode only** — manages game piece physics, registered intakes/shooters, IronMaple arena
- `ElasticSetpoints.getInstance()` — live parameter tuning via SmartDashboard/Elastic

Do not call `RobotSimState.getInstance()` outside of SIM mode.

## Simulation Infrastructure (AI use)

The headless sim writes logs when invoked with `-Pheadless -Pai.logging=true`. Logs go to `build/ai-logs/`. Convert with `scripts/wpilog_to_csv.py`.

See `docs/` for:

- `docs/robot-description.md` — full subsystem and hardware details
- `docs/game-info.md` — 2026 game context (field, game pieces, scoring)
- `docs/ai-prompts.md` — prompt templates for common tasks

See `.claude/commands/` for skills:

- `/simulation-agent` — run sim, capture logs, analyze, close the debugging loop
- `/spec-driven-dev` — walk through Goal → Plan → Review → Implement → Verify
- `/log-analysis` — investigate a `.wpilog` file for a specific failure
