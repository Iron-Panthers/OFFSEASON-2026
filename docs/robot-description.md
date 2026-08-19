# Robot Description — FRC-2026

Detailed hardware and software description. Reference from AI sessions when writing subsystem code or analyzing logs.

---

## Physical Hardware

### Drivetrain

- **Type:** Swerve drive (4 independent modules)
- **Motors:** Kraken X60 FOC (drive + steer, per module)
- **Controller:** TalonFX (Phoenix 6)
- **Gyro:** Pigeon 2 IMU (CTRE)
- **Sim model:** MapleSim `SwerveDriveSimulation` via IronMaple

### Intake System

- **IntakeRack:** Single-jointed arm (TalonFX), extends/retracts the intake out of frame perimeter
  - IO: `GenericSuperstructureIO` / `IntakeRackIOTalonFX` / `IntakeRackIOSim`
  - Targets: `STOW`, `SHOOTING_STOW`, `INTAKE`, `MIDDLE`
- **IntakeRollers:** Spinning rollers (TalonFX) to grab game pieces
  - IO: `GenericRollersIO` / `IntakeRollersIOTalonFX` / `IntakeRollersIOSim`
  - Targets: `IDLE`, `INTAKE`, `INTAKE_SLOW`, `EJECT`

### Shooter System

- **ShooterFlywheel:** 4 motors (Kraken X60 FOC via TalonFX), velocity-controlled
  - IO: `ShooterFlywheelIO` / `ShooterFlywheelIOTalonFX` / `ShooterFlywheelIOSim`
- **ShooterHood:** Single-jointed arm for launch angle (TalonFX)
  - IO: `GenericSuperstructureIO` / `ShooterHoodIOTalonFX` / `ShooterHoodIOSim`
  - Hood targets: `STOW`, `SHOOT_TEMP`, `DEFAULT_SHOOT`, `PASS`
- **ShooterAccelerator:** Rollers to feed game piece into flywheel (TalonFX)
- **ShooterOmniwheel:** Lateral correction wheel (TalonFX)
- **Serializer:** Controls game piece movement within the hopper (TalonFX)

### Vision

- **System:** PhotonVision (3 cameras on COMP robot)
  - `CamA` (index 1), `CamB` (index 2), `CamC` (index 0)
- **Purpose:** AprilTag-based pose estimation, fused into `SwerveDrivePoseEstimator`
- **Sim:** `VisionIOPhotonvisionSim` — simulates camera transforms in MapleSim

### Other

- **RGB:** Addressable LEDs or CANdle (currently commented out in RobotContainer)
- **CANWatchdog:** CAN bus health monitoring (currently commented out)

---

## Robot Configurations

Configured in `RobotContainer` via `switch (Constants.getRobotType())`:

| Type     | Hardware                                               | Notes              |
| -------- | ------------------------------------------------------ | ------------------ |
| `COMP`   | All real TalonFX IOs, Pigeon2, 3 PhotonVision cameras  | Competition robot  |
| `ALPHA`  | Similar to COMP with different CAN IDs / motor configs | Practice robot     |
| `VISION` | Drivetrain + vision only                               | Vision development |
| `SIM`    | All sim IOs, MapleSim physics, PhotonvisionSim         | No real hardware   |

---

## Subsystem Hierarchy & Controller State Machines

### IntakeController (`IntakeController.java`)

Controls `IntakeRack` + `IntakeRollers` together via `IntakeState` enum.

| State           | Rack Target   | Rollers Target | Purpose                                  |
| --------------- | ------------- | -------------- | ---------------------------------------- |
| `STOW`          | STOW          | IDLE           | Default stowed position                  |
| `SHOOTING_STOW` | SHOOTING_STOW | IDLE           | Tucked in while shooting                 |
| `SHOOT`         | INTAKE        | INTAKE_SLOW    | Intake deployed during shooting sequence |
| `MID`           | MIDDLE        | INTAKE_SLOW    | Mid-position for specific maneuvers      |
| `IDLE`          | INTAKE        | IDLE           | Deployed but not spinning                |
| `INTAKE`        | INTAKE        | INTAKE         | Actively intaking game pieces            |
| `INTAKE_SLOW`   | INTAKE        | INTAKE_SLOW    | Slow intake (sensitive pickup)           |
| `REVERSE`       | INTAKE        | EJECT          | Eject game pieces                        |
| `ZEROING`       | STOW          | IDLE           | Homing / encoder zeroing routine         |

### ShooterController (`ShooterController.java`)

Controls Flywheel + Hood + Accelerator + Omniwheel + Serializer via `ShooterState` enum.

| State              | Purpose                                                                |
| ------------------ | ---------------------------------------------------------------------- |
| `IDLE`             | Nothing spinning, hood stowed                                          |
| `REVERSE`          | Reverse serializer to clear jams                                       |
| `FLYWHEEL_SPIN_UP` | Pre-spin flywheels before committing to shoot                          |
| `HOLD`             | Hold game piece in hopper without shooting                             |
| `INTAKE`           | Accept game piece from intake (slow flywheel reverse, slow serializer) |
| `SHOOT`            | Full shooting sequence (hood at SHOOT_TEMP, all motors)                |
| `DEFAULT_SHOOT`    | Default shooting position                                              |
| `TRENCH_SHOOT`     | Shooting from trench position                                          |
| `TOTAL_SPIN_UP`    | Full spin-up including serializer                                      |
| `COMPACT_SPIN_UP`  | Spin up with hood stowed                                               |
| `ZEROING`          | Homing routine                                                         |
| `PASS`             | Pass game piece to alliance partner (lower angle, slower speed)        |
| `PASS_SPIN_UP`     | Pre-spin for passing                                                   |

---

## Key Singletons

### RobotState (`RobotState.java`)

- `SwerveDrivePoseEstimator` — fuses odometry + vision measurements
- `ShootingAnglePredictor` (inner class) — `InterpolatingTreeMap`-based LUT
  - 7–8 distance points from ~1.3m to ~5.2m
  - Returns: hood angle (rotations), flywheel speed (RPS), time-of-flight (sec)
  - Live-tunable via NetworkTables: `Tuning/Shooter/[distance]m/[param]`
- `addVisionMeasurement()` — fuses PhotonVision AprilTag estimates with configurable std devs

### RobotSimState (`RobotSimState.java`) — SIM MODE ONLY

- Singleton managing IronMaple `SimulatedArena` and `FuelSim` game pieces
- Tracks fuel piece positions, registering intakes/shooters for game piece interaction
- Do **not** call outside SIM mode

### ElasticSetpoints (`ElasticSetpoints.java`)

- HashMap-based live parameter system via SmartDashboard/Elastic dashboard
- Used for tuning setpoints without redeploying

---

## Simulation Physics (`FuelSim.java`)

Custom physics engine for game pieces:

- **Fuel radius:** 0.075m (150mm diameter sphere)
- **Fuel mass:** 0.448 × 0.45392 = 0.2033 kg
- **Drag coefficient:** 0.47 (smooth sphere)
- **Air density:** 1.225 kg/m³
- **Field COR:** √(22/51.5) ≈ 0.653 (bounce coefficient vs field)
- **Fuel-fuel COR:** 0.5
- **Net COR:** 0.2
- **Robot COR:** 0.1
- **Ground friction:** 0.2 (horizontal velocity loss per second on ground)

---

## Auto Routines (PathPlanner)

16 autos in `src/main/deploy/pathplanner/autos/`:

| Auto         | Variants     |
| ------------ | ------------ |
| 1xTBB        | Right only   |
| 2x4T         | Left + Right |
| 2x5T         | Left + Right |
| 2xBBBB       | Left + Right |
| 2xTBBB       | Left + Right |
| 2xTBTB       | Left + Right |
| Preload Auto | Left + Right |

Left variants are auto-generated by `./gradlew mirrorAutos`.

Named commands registered in `RobotContainer`:

- `"Smart zero"` — zeroing routine
- `"Auto shoot full hopper (no intake)"` — shoot without intaking

---

## Controller Layout

`driverA` = `CommandXboxController(0)`, `driverB` = `CommandXboxController(1)`

Bindings are declared in `RobotContainer.configureButtonBindings()`. Pattern:

```java
driverA.rightTrigger().whileTrue(new IntakeCommand(intakeController, shooterController));
driverA.a().onTrue(Commands.runOnce(() -> intakeController.setTargetState(IntakeState.STOW)));
```

---

## AdvantageKit Log Key Prefixes (for log analysis)

| Prefix                | Subsystem                       |
| --------------------- | ------------------------------- |
| `Drive/`              | Swerve drivetrain               |
| `Swerve/`             | Pose estimation, odometry       |
| `IntakeController/`   | State machine                   |
| `IntakeRack/`         | Arm position, velocity          |
| `IntakeRollers/`      | Roller velocity, current        |
| `ShooterController/`  | State machine                   |
| `ShooterFlywheel/`    | Velocity, voltage, current      |
| `ShooterHood/`        | Angle, position                 |
| `ShooterAccelerator/` | Velocity, voltage               |
| `Serializer/`         | Velocity, voltage               |
| `Vision/`             | Pose estimates, camera poses    |
| `RobotState/`         | Estimated pose, shooting params |
| `RobotSimState/`      | Fuel positions (SIM only)       |
| `DriverStation/`      | Enabled, mode, FMS data         |
| `PathPlanner/`        | Active path, target pose        |
