# AI Prompt Templates — FRC-2026

Copy-paste prompt library for common robot code tasks. Each prompt is self-contained and references the correct files and patterns for this codebase.

---

## Add a New Subsystem

```
Add a new subsystem called [NAME] to the FRC-2026 robot codebase at C:\Users\bruce\Documents\Coding\FRC-2026.

Read CLAUDE.md and docs/robot-description.md for architecture context before starting.

Hardware: [describe what the physical mechanism does — e.g., "a spinning wheel that indexes game pieces from the hopper to the serializer"]
Motor type: [TalonFX / other]
Motion type: [roller (continuous rotation) or arm/actuator (position-controlled)]

Create these files following the GenericRollers or GenericSuperstructure patterns in src/main/java/frc/robot/lib/generic_subsystems/:
1. src/main/java/frc/robot/subsystems/[name]/[Name]IO.java         — interface with @AutoLog IOInputs
2. src/main/java/frc/robot/subsystems/[name]/[Name]IOTalonFX.java  — real hardware implementation
3. src/main/java/frc/robot/subsystems/[name]/[Name]IOSim.java      — WPILib sim implementation
4. src/main/java/frc/robot/subsystems/[name]/[Name].java           — subsystem logic
5. src/main/java/frc/robot/subsystems/[name]/[Name]Constants.java  — motor IDs, PID gains, physical constants

Wire it into RobotContainer.java in both the COMP case (TalonFX IO) and the SIM case (Sim IO).
Add @AutoLogOutput to all observable state fields.
Run ./gradlew spotlessApply --no-daemon && ./gradlew compileJava --no-daemon when done.
```

---

## Tune a PID Controller

```
I want to analyze and tune the [SUBSYSTEM] PID gains.

Log file is at: [PATH TO .wpilog]
Constants file: src/main/java/frc/robot/subsystems/[path]/[Name]Constants.java

Step 1: Run this to extract the relevant data:
  python scripts/wpilog_to_csv.py [LOG_PATH] --prefix [SubsystemName] --out /tmp/[name].csv

Step 2: Analyze the CSV for:
- Rise time: how fast does velocity/position reach the setpoint after a step command?
- Overshoot: does it exceed the setpoint before settling?
- Oscillation: does it cycle above/below the setpoint?
- Steady-state error: does it fully reach the setpoint, or stop slightly short?
- Current draw: does appliedVolts spike unreasonably high?

Step 3: Based on the analysis, suggest new values for kP, kI, kD, kS, kV, kA (and kG if it's a gravity-affected arm).
Explain your reasoning for each change.
```

---

## Add a PathPlanner Auto

```
Add a new autonomous routine called "[AUTO_NAME]" to the FRC-2026 robot.

Read docs/robot-description.md and docs/game-info.md for robot/field context.
Read an existing .auto file in src/main/deploy/pathplanner/autos/ for the JSON structure.
Read an existing .path file in src/main/deploy/pathplanner/paths/ for path structure.

The auto should do: [describe sequence — e.g., "start at center, shoot preloaded fuel, drive to right trench, pick up 2 fuel, return to shoot zone, shoot both"]

Create:
1. src/main/deploy/pathplanner/autos/[AUTO_NAME]Right.auto    — main auto file
2. Any new .path files needed in src/main/deploy/pathplanner/paths/

For any new named commands used in the .auto file that don't already exist:
- Register them in RobotContainer.java with NamedCommands.registerCommand("[name]", command)
- The existing named commands are "Smart zero" and "Auto shoot full hopper (no intake)"

After creating the files, run ./gradlew mirrorAutos --no-daemon to auto-generate the Left variant.
```

---

## Debug a Specific Failure with Logs

```
Something went wrong during [MATCH/TEST DESCRIPTION]. Investigate the log file to find the root cause.

Log file: [PATH TO .wpilog]
What happened: [describe the symptom — e.g., "robot didn't pick up the second ball", "shooter was slow to spin up", "auto path deviated at T=8s"]
Time window (approximate): [e.g., "around T=20s to T=35s in the match"]

Step 1: Run a summary:
  python scripts/wpilog_to_csv.py [LOG_PATH] --summary

Step 2: Run the relevant investigation preset:
  python scripts/wpilog_to_csv.py [LOG_PATH] --investigate [shooter|drive|auto|intake|vision]

Step 3: Extract the specific time window around the failure and identify:
- Exact timestamp of failure
- What state each relevant subsystem was in
- What changed just before the failure
- Root cause (code bug, PID tuning, timing, sensor issue, or mechanical)

Step 4: Suggest a specific fix (file, line, change).
```

---

## Add a Controller Button Binding

```
Add a new button binding to the [driverA / driverB] controller in RobotContainer.java.

Read the existing bindings in RobotContainer.configureButtonBindings() for the pattern.

Button: [button name — e.g., Y button, right bumper, left trigger > 0.5]
Controller: [driverA (port 0) / driverB (port 1)]
Behavior: [onTrue / whileTrue / onFalse]
Command: [describe what it should do — e.g., "set ShooterController state to PASS"]

Follow the existing .onTrue()/.whileTrue()/.onFalse() pattern.
If it changes a controller state, use an InstantCommand wrapping a setState call.
Run ./gradlew spotlessApply --no-daemon && ./gradlew compileJava --no-daemon when done.
```

---

## Add a New ShooterState or IntakeState

```
Add a new state to [ShooterController / IntakeController] called [STATE_NAME].

Read src/main/java/frc/robot/subsystems/[shooter|intake]/[Shooter|Intake]Controller.java for the existing state pattern.
Read docs/robot-description.md for the full list of available targets for each child subsystem.

The new state should:
- [describe what the robot should do — e.g., "hold the hood at 30 degrees and spin the flywheel at 50% speed for shuttle shots"]
- Use these child subsystem targets: [list each target enum value, e.g., ShooterHoodTarget.SHUTTLE, ShooterFlywheelTarget.SHUTTLE]

If new target enum values are needed for child subsystems:
- Add them to the target enum in the relevant subsystem file
- Add the corresponding setpoint in the subsystem's periodic() or configuration

Add @AutoLogOutput to any new state field.
Run ./gradlew spotlessApply --no-daemon && ./gradlew compileJava --no-daemon when done.
```

---

## Analyze Energy / Performance From Logs

```
Analyze energy usage and motor performance from this log file.

Log file: [PATH TO .wpilog]
Focus: [e.g., "shooter flywheel current draw", "swerve drive motor efficiency", "total battery voltage drop"]

Step 1:
  python scripts/wpilog_to_csv.py [LOG_PATH] --prefix [SubsystemName] --out /tmp/energy.csv

Step 2: Analyze:
- Peak current per motor
- Average current during active operation
- Applied voltage vs battery voltage (if available)
- Whether current limits are being hit frequently
- Estimated heat load (I²×t for each motor)

Step 3: Suggest optimizations:
- PID gains that reduce motor effort for same accuracy
- Current limit adjustments
- Gear ratio changes (if pattern suggests mechanical inefficiency)
- Operating mode changes (e.g., FOC vs non-FOC tradeoffs)
```

---

## Fix a Compile Error After GenericSubsystem Change

```
The generic subsystem base class at [FILE PATH] was changed, and now several subsystems that extend it have compile errors.

Run ./gradlew compileJava --no-daemon to see the current errors.

For each error:
1. Identify which subsystem file is affected
2. Understand what the base class change requires
3. Update the affected subsystem to satisfy the new interface
4. Do NOT change the base class to work around the errors — fix the subclasses

After fixing all errors, run ./gradlew spotlessApply --no-daemon && ./gradlew compileJava --no-daemon to confirm clean build.
```
