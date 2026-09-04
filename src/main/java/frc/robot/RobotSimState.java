package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.intake.intake_rack.IntakeRackConstants;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.utility.FuelSim;
import frc.robot.utility.TrenchTerrainSim;
import java.util.ArrayList;
import java.util.List;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotSimState {

  public static final int START_FUEL_CAPACITY = 8;

  private int fuelCount = START_FUEL_CAPACITY;
  private boolean intakeActive = false;
  List<SwerveDriveSimulation> obstacles = new ArrayList<SwerveDriveSimulation>();

  private RobotSimState() {
    // init the arena (drive sim only, no game piece placement)
    Arena2026Rebuilt arena = new Arena2026Rebuilt(false);
    // arena.setEfficiencyMode(true);
    arena.clearGamePieces();
    arena.setShouldRunClock(true);

    // Add the drive simulation
    driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.mapleSimConfig, RobotState.getInstance().getEstimatedPose());
    arena.addDriveTrainSimulation(driveSimulation);

    SimulatedArena.overrideInstance(arena);

    // init fuel sim
    fuelSim = new FuelSim("Field Simulation");
    fuelSim.registerRobot(
        DriveConstants.mapleSimConfig.bumperWidthY, // from left to right in meters
        DriveConstants.mapleSimConfig.bumperLengthX, // from front to back in meters
        Units.Inches.of(7), // from floor to top of bumpers in meters
        driveSimulation::getSimulatedDriveTrainPose, // Supplier<Pose2d> of robot pose
        driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative);

    // Register intake on the back of the robot
    double halfLength = DriveConstants.mapleSimConfig.bumperLengthX.in(Meters) / 2.0;
    double halfWidth = DriveConstants.mapleSimConfig.bumperWidthY.in(Meters) / 2.0;
    double intakeReach = 0.1; // meters beyond bumper
    fuelSim.registerIntake(
        -halfLength - intakeReach,
        -halfLength,
        -halfWidth,
        halfWidth,
        () -> intakeActive && fuelCount < 60,
        () -> fuelCount++);

    fuelSim.spawnStartingFuel();
    fuelSim.setLoggingFrequency(20);
    fuelSim.start();

    // opponent simulation

    // Blue: To go left, (6.17, 5.881)
    // Blue: To go right, (6.17, 2.6). Probably
    // Red: Right, (10.385, 5.759)
    // Red: Left, (10.385, 2.082)
    // When on red, obstacle should nbe spawning on -y
    addObstacleToSim(new Pose2d(new Translation2d(10.8, 1.5), new Rotation2d()));
  }

  // Get sim state from RobotContainer

  public List<Pose2d> getObstaclePositions() {
    return obstacles.stream().map((obstacle) -> obstacle.getSimulatedDriveTrainPose()).toList();
  }

  public void addObstacleToSim(Pose2d pose) {
    SwerveDriveSimulation obstacle = new SwerveDriveSimulation(DriveConstants.obstacleConfig, pose);
    obstacles.add(obstacle);
    SimulatedArena.getInstance().addDriveTrainSimulation(obstacle);
  }

  // Singleton instance
  private static RobotSimState instance = null;

  public static RobotSimState getInstance() {
    if (Constants.getRobotType() != RobotType.SIM) {
      // idiot proofing
      System.out.println(
          "WARNING: YOU ARE TRYING TO ACCESS ROBOT SIM STATE FROM AN ACTUAL ROBOT -- THIS IS A CODE"
              + " ERROR");
    }
    if (instance == null) instance = new RobotSimState();
    return instance;
  }

  // Drive simulation
  private SwerveDriveSimulation driveSimulation;

  // Terrain contact state — updated each loop after simulationPeriodic()
  private TrenchTerrainSim.TerrainContactState terrainState =
      new TrenchTerrainSim.TerrainContactState(
          DriveConstants.DRIVE_CONFIG.wheelRadius(), 0.0, 0.0, 0.0, new double[4]);

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  private FuelSim fuelSim;

  public FuelSim getFuelSim() {
    return fuelSim;
  }

  // Get attributes of physical drivebase
  public Pose2d getRobotPose2d() {
    return driveSimulation.getSimulatedDriveTrainPose();
  }

  public Pose3d getRobotPose3d() {
    Pose2d robotPose2d = driveSimulation.getSimulatedDriveTrainPose();
    return new Pose3d(
        new Translation3d(robotPose2d.getX(), robotPose2d.getY(), terrainState.robotBodyZ()),
        new Rotation3d(
            terrainState.pitchRad(),
            terrainState.rollRad(),
            robotPose2d.getRotation().getRadians()));
  }

  /**
   * Updates the terrain contact state from the robot's current simulated pose. Call this once per
   * loop <em>after</em> {@code SimulatedArena.getInstance().simulationPeriodic()}.
   */
  public void updateTerrainState() {
    terrainState =
        TrenchTerrainSim.compute(
            driveSimulation.getSimulatedDriveTrainPose(),
            DriveConstants.MODULE_TRANSLATIONS,
            DriveConstants.DRIVE_CONFIG.wheelRadius(),
            DriveConstants.DRIVE_CONFIG.trackWidth(), // front-to-rear (wheelbase)
            DriveConstants.DRIVE_CONFIG.trackLength()); // left-to-right (track width)
  }

  /**
   * Injects a gravity resistance force into the Dyn4j physics engine to simulate the robot climbing
   * or descending the trench bump. Call this once per loop <em>before</em> {@code
   * SimulatedArena.getInstance().simulationPeriodic()}.
   *
   * <p>The force is scaled by the number of physics sub-ticks because Dyn4j clears the {@code
   * Body.force} accumulator after each sub-tick integration step. A single {@code applyForce()}
   * call placed before {@code simulationPeriodic()} would otherwise only affect the first sub-tick.
   */
  public void applyTerrainGravityForce() {
    double gradientX = terrainState.avgGradientX();
    if (gradientX == 0.0) return; // flat ground — no bump force needed

    double mass = driveSimulation.getMass().getMass(); // kg, from Dyn4j Body
    double subTickDtSec =
        SimulatedArena.getSimulationDt().in(edu.wpi.first.units.Units.Seconds); // typically 0.004 s
    int nSubTicks = (int) Math.round(Constants.PERIODIC_LOOP_SEC / subTickDtSec); // typically 5

    // F = −m · g · dZ/dX (opposing forward motion on an uphill ramp)
    // Multiply by nSubTicks so the impulse integrates to the correct total over the full 20 ms loop
    double fx = -mass * 9.81 * gradientX * nSubTicks;
    driveSimulation.applyForce(new Vector2(fx, 0.0));
  }

  @AutoLogOutput(key = "Robot Sim State/Terrain/PitchDegrees")
  public double getRobotPitchDegrees() {
    return Math.toDegrees(terrainState.pitchRad());
  }

  @AutoLogOutput(key = "Robot Sim State/Terrain/RollDegrees")
  public double getRobotRollDegrees() {
    return Math.toDegrees(terrainState.rollRad());
  }

  @AutoLogOutput(key = "Robot Sim State/Terrain/AltitudeMeters")
  public double getRobotAltitudeMeters() {
    return terrainState.robotBodyZ();
  }

  public ChassisSpeeds getChassisSpeedsFieldRelative() {
    return driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
  }

  // Shooting utilities
  public void shootFuel(
      Angle launchAngle,
      Transform3d shooterTransform3d,
      LinearVelocity launchVelocity,
      Distance shooterWidth) {
    if (fuelCount <= 0) return; // no fuel to shoot
    fuelCount--;

    // Build a transform that includes the shooter's position and combines the hood
    // pitch with the
    // shooter's yaw
    Transform3d shooterOffset =
        new Transform3d(
            new Translation3d(0, (Math.random() * 2 - 1) * (shooterWidth.in(Units.Meters) * .5), 0),
            new Rotation3d());

    Transform3d launchTransform =
        new Transform3d(
                shooterTransform3d.getTranslation(),
                new Rotation3d(0, 0, shooterTransform3d.getRotation().getZ()))
            .plus(shooterOffset);

    fuelSim.launchFuel(launchVelocity, launchAngle, launchTransform);
  }

  // Intake simulation (backed by FuelSim)

  public void setIntakeState(boolean extended) {
    intakeActive = extended;
  }

  public int getFuelCount() {
    return fuelCount;
  }

  public Pose3d[] getIntakeGamePieces() {
    Pose3d[] gamePiecePoses = new Pose3d[fuelCount];
    Pose3d robotPose = getRobotPose3d();

    // Fuel ball dimensions (matches FuelSim constants)
    double fuelRadius = 0.075; // meters
    double fuelDiameter = fuelRadius * 2.0;
    double bumperHeight = Units.Inches.of(7).in(Units.Meters);

    // Hopper x-bounds in robot frame (positive x = front of robot):
    //   Rear  — balls cannot go further back than the intake rack pivot.
    //   Front — balls stop ~2 ball-widths from the front bumper face.
    double xRear = IntakeRackConstants.BASE_TO_INTAKE_RACK_TRANSFORM.getTranslation().getX();
    double xFront = DriveConstants.DRIVE_CONFIG.bumperWidthX() / 2.0 - 2.0 * fuelDiameter;

    // Hopper y-bounds: centre the grid, 80% of half-width each side.
    double hopperHalfWidth = DriveConstants.DRIVE_CONFIG.bumperWidthY() / 2.0 * 0.8;

    // Number of ball positions along each axis
    int ballsPerCol = Math.max(1, (int) ((xFront - xRear) / fuelDiameter)); // x-axis
    int ballsPerRow = Math.max(1, (int) (hopperHalfWidth * 2.0 / fuelDiameter)); // y-axis

    // Grid origin in robot-local frame.
    // x: first ball center is one radius in from the rear (intake rack) boundary.
    // y: grid centred on the robot centreline.
    // z: just above bumper top.
    double gridStartX = xRear + fuelRadius;
    double gridStartY = -(ballsPerRow - 1) * fuelDiameter / 2.0;
    double gridStartZ = bumperHeight + fuelRadius;

    for (int i = 0; i < fuelCount; i++) {
      // Pack layer by layer (z), filling each layer row-by-row (y) then column-by-column (x).
      int ballsPerLayer = ballsPerRow * ballsPerCol;
      int layer = i / ballsPerLayer;
      int withinLayer = i % ballsPerLayer;
      int col = withinLayer / ballsPerRow; // x index
      int row = withinLayer % ballsPerRow; // y index

      double x = gridStartX + col * fuelDiameter;
      double y = gridStartY + row * fuelDiameter;
      double z = gridStartZ + layer * fuelDiameter;

      // Transform from robot-local frame to world frame.
      // Robot only rotates about world-z (yaw), so z stays vertical and x/y follow heading.
      gamePiecePoses[i] = robotPose.transformBy(new Transform3d(x, y, z, new Rotation3d()));
    }
    return gamePiecePoses;
  }

  // Automatic shooter state tracking
  private boolean isShooterRunning = false;
  private double lastShootTime = 0.0;
  private double shootIntervalSeconds = 0.0;

  /**
   * Tells the RobotSimState that the shooter is currently running and should shoot fuel
   * automatically.
   *
   * @param shotsPerSecond The rate at which to shoot fuel (e.g., 2.0 for 2 shots per second)
   * @param shooterAngle The angle at which to shoot
   * @param shooterTransform3d The 3D transform of the shooter relative to the robot
   * @param launchVelocity The velocity at which to launch the fuel
   */
  public void setShooterRunning(
      boolean running,
      double shotsPerSecond,
      Angle shooterAngle,
      Transform3d shooterTransform3d,
      LinearVelocity launchVelocity,
      Distance shooterWidth) {
    if (running && !isShooterRunning) {
      // Starting the shooter
      isShooterRunning = true;
      shootIntervalSeconds = 1.0 / shotsPerSecond;
      lastShootTime = Timer.getFPGATimestamp();
    } else if (!running) {
      // Stopping the shooter
      isShooterRunning = false;
    }

    // Store the shooting parameters for use in periodic
    this.currentShooterAngle = shooterAngle;
    this.currentShooterTransform = shooterTransform3d;
    this.currentLaunchVelocity = launchVelocity;
    this.currentShooterWidth = shooterWidth;
  }

  // Store current shooting parameters
  private Angle currentShooterAngle = Units.Radians.of(0);
  private Transform3d currentShooterTransform = new Transform3d();
  private LinearVelocity currentLaunchVelocity = MetersPerSecond.of(0);
  private Distance currentShooterWidth = Meters.of(0);

  /**
   * Should be called periodically (e.g., in Robot.java's simulationPeriodic). Handles automatic
   * shooting when the shooter is running.
   */
  public void periodicShooter() {
    if (!isShooterRunning) {
      return;
    }

    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastShootTime >= shootIntervalSeconds) {
      // Time to shoot another ball
      shootFuel(
          currentShooterAngle, currentShooterTransform, currentLaunchVelocity, currentShooterWidth);
      lastShootTime = currentTime;
      Logger.recordOutput("Robot Sim State/Auto Shooter Active", true);
    }
  }

  @AutoLogOutput(key = "Robot Sim State/Shooter Running")
  public boolean isShooterRunning() {
    return isShooterRunning;
  }
}
