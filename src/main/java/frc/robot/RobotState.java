// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.ShootingAnglePredictor.HoodParams;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/* based on wpimath/../PoseEstimator.java */
public class RobotState {
  public static final double fieldSizeX = Units.feetToMeters(57.573);
  public static final double fieldSizeY = Units.feetToMeters(26.417);

  public record OdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionMeasurement(Pose2d visionPose, double timestamp) {}

  private static final double poseBufferSizeSeconds = 2; // shorter?
  private static final Pose2d initialPose =
      isAllianceRed()
          ? FlippingUtil.flipFieldPose(DriveConstants.INITIAL_POSE)
          : DriveConstants.INITIAL_POSE;

  private final Matrix<N3, N1> matrixQ = new Matrix<>(Nat.N3(), Nat.N1());

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.KINEMATICS,
          new Rotation2d(),
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          },
          initialPose,
          DriveConstants.STATE_STD_DEVS,
          VisionConstants.VISION_STATE_STD_DEVS);

  private Pose2d estimatedPose = initialPose; // vision adjusted

  private Pose2d lastApproachPose = new Pose2d();

  private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      matrixQ.set(
          i, 0, DriveConstants.STATE_STD_DEVS.get(i, 0) * DriveConstants.STATE_STD_DEVS.get(i, 0));
    }
  }

  /* update pose estimation based on odometry measurements*/
  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    poseEstimator.updateWithTime(
        measurement.timestamp(), measurement.gyroAngle(), measurement.wheelPositions());

    // integrate to find difference in pose over time, add to pose estimate
    estimatedPose = poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(VisionMeasurement measurement, Matrix<N3, N1> visionStdDevs) {
    poseEstimator.setVisionMeasurementStdDevs(visionStdDevs);
    poseEstimator.addVisionMeasurement(measurement.visionPose(), measurement.timestamp());
    estimatedPose = poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    poseEstimator.resetPose(pose);
  }

  @AutoLogOutput(key = "Robot State/Estimated Pose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  @AutoLogOutput(key = "Robot State/Velocity")
  /* meters per second */
  public Translation2d getVelocity() {
    return new Translation2d(
            ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, estimatedPose.getRotation())
                .vxMetersPerSecond,
            ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, estimatedPose.getRotation())
                .vyMetersPerSecond)
        .rotateBy(Rotation2d.kPi);
  }

  /* In inches because we are imperial... */
  @AutoLogOutput(key = "Robot State/Error")
  public double alignError() {
    return lastApproachPose.getTranslation().getDistance(estimatedPose.getTranslation())
        * 100
        / 2.54;
  }

  private Pose2d translateByVector(Pose2d pose, double mag, Rotation2d theta) {
    double scalarX = theta.getCos() * mag;
    double scalarY = theta.getSin() * mag;

    Transform2d transform = new Transform2d(scalarX, scalarY, Rotation2d.kZero);
    return pose.transformBy(transform);
  }

  // translate + rotate
  private Pose2d offsetByVector(Pose2d pose, double mag, Rotation2d theta) {
    return translateByVector(pose, mag, theta).transformBy(new Transform2d(0, 0, theta));
  }

  /**
   * Gets the scuffed path planner built command for following a path to a certain pose
   *
   * @param approachPose2d
   * @param underTrench
   * @return
   */
  public Command getPathPlannerApproachPoseCommand(Pose2d approachPose2d, boolean underTrench) {
    Logger.recordOutput("Robot State/Estimated Pose", estimatedPose);
    Logger.recordOutput("Robot State/Approach Pose", approachPose2d);

    Command finalPathfindingCommand = null;

    if (underTrench) {
      Pathfinding.setDynamicObstacles(
          DriveConstants.OBSTACLES_FOR_TRENCH_PATHFINDING, estimatedPose.getTranslation());
      finalPathfindingCommand =
          AutoBuilder.pathfindToPose(approachPose2d, DriveConstants.ALIGN_PATH_CONSTRAINTS, 0.0);
    } else {
      Pathfinding.setDynamicObstacles(
          DriveConstants.OBSTACLES_FOR_BUMP_PATHFINDING, estimatedPose.getTranslation());
      finalPathfindingCommand =
          AutoBuilder.pathfindToPose(approachPose2d, DriveConstants.ALIGN_PATH_CONSTRAINTS, 0.0);
    }

    return finalPathfindingCommand;
  }

  public void addRobotSpeeds(ChassisSpeeds chassisSpeeds) {
    this.robotSpeeds = chassisSpeeds;
  }

  public Pose2d getAlignPose() {
    return lastApproachPose;
  }

  // methods that use the shootingAnglePredictor -- as an abstraction

  private ShootingAnglePredictor shootingAnglePredictor;

  public void initializeShootingAnglePredictor(
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<LinearVelocity> shooterVelocitySupplier,
      Supplier<Transform3d> shooterPositionSupplier,
      Angle shooterYaw) {
    shootingAnglePredictor =
        new ShootingAnglePredictor(
            chassisSpeedsSupplier, shooterVelocitySupplier, shooterPositionSupplier, shooterYaw);
  }

  public TargetShootingState calculateTargetShootingState() {
    TargetShootingState targetShootingState = shootingAnglePredictor.calculateTargetShootingState();
    Logger.recordOutput(
        "RobotState/Target Shooting State/Drivebase Yaw", targetShootingState.drivebaseYaw());
    Logger.recordOutput(
        "RobotState/Target Shooting State/Shooter Angle", targetShootingState.shooterAngle());
    Logger.recordOutput(
        "RobotState/Target Shooting State/Shooter Speed", targetShootingState.shooterSpeed());
    return targetShootingState;
  }

  /** Gets interpolated stationary hood params from specified distance (meters) */
  public HoodParams getStationaryHoodParams(double distance) {
    return shootingAnglePredictor.getHoodParamsFromDistance(distance);
  }

  // shooting predictor
  public class ShootingAnglePredictor {

    // different variable suppliers -- used later for calculations
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Supplier<LinearVelocity> shooterVelocitySupplier;
    private Supplier<Transform3d> shooterPositionSupplier;

    public LoggedNetworkNumber tempShooterAngle =
        new LoggedNetworkNumber("Tuning/TempShooterAngle", 70);

    // NT entries for LUT tuning — created once per key, reused every frame
    private final HashMap<String, LoggedNetworkNumber> ntLutEntries = new HashMap<>();

    private LoggedNetworkNumber getLutNTEntry(String key, double defaultValue) {
      return ntLutEntries.computeIfAbsent(key, k -> new LoggedNetworkNumber(k, defaultValue));
    }

    // Moving average filters for smooth velocity measurements
    private final LinearFilter vxFilter = LinearFilter.movingAverage(5);
    private final LinearFilter vyFilter = LinearFilter.movingAverage(5);

    private final InterpolatingTreeMap<Double, HoodParams> shooterTable =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), HoodParams::interpolate);

    public ShootingAnglePredictor(
        Supplier<ChassisSpeeds> chassisSpeedsSupplier,
        Supplier<LinearVelocity> shooterVelocitySupplier,
        Supplier<Transform3d> shooterPositionSupplier,
        Angle shooterYaw) {
      this.chassisSpeedsSupplier = chassisSpeedsSupplier;
      this.shooterPositionSupplier =
          () ->
              (new Transform3d(
                      new Translation3d(0, 0, 0), new Rotation3d(0, 0, shooterYaw.in(Radian))))
                  .plus(shooterPositionSupplier.get());
      this.shooterVelocitySupplier = shooterVelocitySupplier;

      initializeShooterTable();
    }

    public void initializeShooterTable() {
      this.shooterTable.clear();
      switch (Constants.getRobotType()) {
        case SIM -> {
          addEntry(1.3, new HoodParams(87, 9, 1.621));
          addEntry(2.0, new HoodParams(83.5, 9, 1.621));
          addEntry(2.5, new HoodParams(81, 9, 1.601));
          addEntry(3.0, new HoodParams(78.5, 9, 1.602));
          addEntry(3.5, new HoodParams(76.5, 9, 1.581));
          addEntry(4.0, new HoodParams(74.5, 9, 1.561));
          addEntry(4.5, new HoodParams(73, 9, 1.561));
        }
        default -> {
          addEntry(1.3, new HoodParams(83, 8.5, 1.09));
          addEntry(2.0, new HoodParams(77, 8.4, .97));
          addEntry(2.5, new HoodParams(73, 9, 1.14));
          addEntry(3.0, new HoodParams(73, 9.4, 1.15)); // tuned to here
          addEntry(3.5, new HoodParams(69, 9.5, 1.22));
          addEntry(4.0, new HoodParams(67, 10.2, 1.3));
          addEntry(4.5, new HoodParams(67, 10.5, 1.34));
          addEntry(5.2, new HoodParams(64, 10.9, 1.39));
        }
      }
    }

    private void addEntry(double distance, HoodParams defaults) {
      String prefix = String.format("Tuning/Shooter/%.1fm/", distance);
      double angle = getLutNTEntry(prefix + "shooterAngle", defaults.shooterAngle).get();
      double speed = getLutNTEntry(prefix + "shooterSpeed", defaults.shooterSpeed).get();
      double tof = getLutNTEntry(prefix + "timeOfFlight", defaults.timeOfFlight).get();
      HoodParams params = new HoodParams(angle, speed, tof);
      shooterTable.put(distance, params);
    }

    /**
     * @param distance in meters
     * @return the Hood params for shooting stationary from that distance
     */
    public HoodParams getHoodParamsFromDistance(double distance) {
      return shooterTable.get(distance);
    }

    public TargetShootingState calculateTargetShootingState() {

      initializeShooterTable();

      // Get target hub position
      final Translation3d hubPosition3d =
          isAllianceRed() ? DriveConstants.RED_HUB_ORIGIN : DriveConstants.BLUE_HUB_ORIGIN;

      // Get chassis speeds and apply moving average filter for smoothness
      ChassisSpeeds rawSpeeds = chassisSpeedsSupplier.get();
      double filteredVx = vxFilter.calculate(rawSpeeds.vxMetersPerSecond);
      double filteredVy = vyFilter.calculate(rawSpeeds.vyMetersPerSecond);

      Translation2d robotVelocity = new Translation2d(filteredVx, filteredVy);

      // Log the raw and filtered velocities for tuning
      Logger.recordOutput("Shooting Predictor/Raw Vx", rawSpeeds.vxMetersPerSecond);
      Logger.recordOutput("Shooting Predictor/Raw Vy", rawSpeeds.vyMetersPerSecond);
      Logger.recordOutput("Shooting Predictor/Filtered Vx", filteredVx);
      Logger.recordOutput("Shooting Predictor/Filtered Vy", filteredVy);

      // Get the initial important things
      Pose3d robotPose3d = new Pose3d(getEstimatedPose());

      double latencyCompensation =
          0.15; // Tune later // TODO: make this an actual constant (if you change it later this is
      // the one for sim)

      // 1. Project future position
      Translation2d futurePos =
          robotPose3d
              .getTranslation()
              .toTranslation2d()
              .plus(robotVelocity.times(latencyCompensation));

      // 2. Get target vector
      Translation2d toGoal = hubPosition3d.toTranslation2d().minus(futurePos);
      double distance = toGoal.getNorm();
      Translation2d targetDirection = toGoal.div(distance);

      // 3. Look up baseline velocity from table
      HoodParams baseline = shooterTable.get(distance);
      double baselineVelocity = distance / baseline.timeOfFlight;

      // 4. Build target velocity vector
      Translation2d targetVelocity = targetDirection.times(baselineVelocity);

      // 5. THE MAGIC: subtract robot velocity
      Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

      // 6. Extract turret angle from horizontal velocity compensation
      Rotation2d turretAngle =
          shotVelocity
              .getAngle()
              .plus(isAllianceRed() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));

      // modify the above line for a shooter offset
      double shooterOffsetY =
          0; // meters, tune this later based on where the shooter is // TODO: make this an
      // actual constant
      Rotation2d shooterAngleOffset = Rotation2d.fromRadians(Math.atan2(shooterOffsetY, distance));
      // turretAngle = turretAngle.plus(shooterAngleOffset);

      double shotHorizontalSpeed = shotVelocity.getNorm();

      // 7. Decompose the LUT's tuned trajectory into horizontal & vertical velocity
      //    v_v comes from the tuned hood angle — this preserves the tuned vertical trajectory
      double baselineVerticalVelocity =
          baselineVelocity * Math.tan(Math.toRadians(baseline.shooterAngle));

      // 8. Recompute hood angle: keep the tuned v_v, use the compensated horizontal speed
      double adjustedHoodAngle =
          Math.toDegrees(Math.atan2(baselineVerticalVelocity, shotHorizontalSpeed));

      // 9. Scale shooter speed by ratio of new vs static total exit velocity
      double staticExitSpeed = baselineVelocity / Math.cos(Math.toRadians(baseline.shooterAngle));
      double newExitSpeed =
          Math.sqrt(
              shotHorizontalSpeed * shotHorizontalSpeed
                  + baselineVerticalVelocity * baselineVerticalVelocity);
      double adjustedShooterSpeed = baseline.shooterSpeed * (newExitSpeed / staticExitSpeed);

      Logger.recordOutput("Shooting Predictor/Distance", distance);
      Logger.recordOutput("Shooting Predictor/Baseline Vh", baselineVelocity);
      Logger.recordOutput("Shooting Predictor/Baseline Vv", baselineVerticalVelocity);
      Logger.recordOutput("Shooting Predictor/Shot Horizontal Speed", shotHorizontalSpeed);
      Logger.recordOutput("Shooting Predictor/Turret Angle", turretAngle);
      Logger.recordOutput("Shooting Predictor/Adjusted Hood Angle", adjustedHoodAngle);
      Logger.recordOutput("Shooting Predictor/Adjusted Shooter Speed", adjustedShooterSpeed);
      Logger.recordOutput("Shooting Predictor/Shooter Offset Y", shooterOffsetY);
      Logger.recordOutput("Shooting Predictor/Shooter Angle Offset", shooterAngleOffset);

      return new TargetShootingState(
          turretAngle, Degrees.of(adjustedHoodAngle), MetersPerSecond.of(adjustedShooterSpeed));
    }

    // Simple data class for the LUT
    // shooterAngle in degrees, shooterSpeed in m/s (surface speed), timeOfFlight in seconds
    public record HoodParams(double shooterAngle, double shooterSpeed, double timeOfFlight)
        implements Interpolatable<HoodParams> {
      @Override
      public HoodParams interpolate(HoodParams endValue, double t) {
        return new HoodParams(
            MathUtil.interpolate(this.shooterAngle, endValue.shooterAngle, t),
            MathUtil.interpolate(this.shooterSpeed, endValue.shooterSpeed, t),
            MathUtil.interpolate(this.timeOfFlight, endValue.timeOfFlight, t));
      }
    }
  }

  public record TargetShootingState(
      Rotation2d drivebaseYaw, Angle shooterAngle, LinearVelocity shooterSpeed) {}

  public Pose2d getShootingPose() {
    Pose2d shootingPoseOne =
        getShootingPose(2.154).plus(new Transform2d(new Translation2d(), Rotation2d.kZero));
    // Pose2d shootingPoseTwo = getShootingPose(4.0); //edit forf climb
    // Pose2d flippedEstimatedPose = isAllianceRed()
    //                 ? FlippingUtil.flipFieldPose(estimatedPose)
    //                 : estimatedPose;
    Logger.recordOutput("Robot State/Shooting Pose One", shootingPoseOne);
    // Logger.recordOutput("RobotState/ShootingPoseTwo", shootingPoseTwo);
    // if (shootingPoseOne.getTranslation().getDistance(flippedEstimatedPose.getTranslation()) <
    //     shootingPoseTwo.getTranslation().getDistance(flippedEstimatedPose.getTranslation())){
    //   return shootingPoseOne;
    // } else {
    //   return shootingPoseTwo;
    // }
    return shootingPoseOne;
  }

  public Pose2d getShootingPose(double distanceTargetToHub) {
    Pose2d flippedEstimatedPose =
        isAllianceRed() ? FlippingUtil.flipFieldPose(estimatedPose) : estimatedPose;
    Translation2d hubCoords = new Pose2d(4.62, 4.03, new Rotation2d()).getTranslation();
    Translation2d translHubCoords = hubCoords.minus(flippedEstimatedPose.getTranslation());
    double distanceToHub = translHubCoords.getNorm();
    double angle = Math.atan2(translHubCoords.getY(), translHubCoords.getX());

    if (distanceTargetToHub >= 2.5
        && (angle > -35.64 / 180 * Math.PI && Math.abs(angle) < 28.25 / 180 * Math.PI)) {
      distanceTargetToHub = 2;
    }

    double y =
        -distanceTargetToHub * Math.sin(angle)
            + translHubCoords.getY()
            + flippedEstimatedPose.getY();
    double x =
        -distanceTargetToHub * Math.cos(angle)
            + translHubCoords.getX()
            + flippedEstimatedPose.getX();

    if (y > 7.307) {
      return new Pose2d(
          2.326,
          7.307,
          new Rotation2d(Math.atan2(hubCoords.getY() - 7.307, hubCoords.getX() - 2.326)));
    }
    if (y < 0.753) {
      return new Pose2d(
          2.326,
          0.753,
          new Rotation2d(Math.atan2(hubCoords.getY() - 0.753, hubCoords.getX() - 2.326)));
    }
    if (x > 3.322) {
      if (angle > 0) {
        return new Pose2d(
            3.322,
            2.502,
            new Rotation2d(Math.atan2(hubCoords.getY() - 2.502, hubCoords.getX() - 3.322)));
      }
      if (angle < 0) {
        return new Pose2d(
            3.322,
            5.522,
            new Rotation2d(Math.atan2(hubCoords.getY() - 5.522, hubCoords.getX() - 3.322)));
      }
    }
    return new Pose2d(x, y, new Rotation2d(angle));
  }

  @AutoLogOutput(key = "Robot State/isAllianceRed")
  public static boolean isAllianceRed() {
    // where true is red and false is blue
    var alliance = DriverStation.getAlliance();
    if (RobotBase.isReal()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return true;
  }

  @AutoLogOutput(key = "Robot State/isUnderTrench")
  public boolean isUnderTrench() {
    Pose2d robotPose = getEstimatedPose();
    Pose2d flippedTrenchPose = FlippingUtil.flipFieldPose(DriveConstants.TRENCH_POSE);
    boolean underTrench =
        ((Math.abs(robotPose.getX() - DriveConstants.TRENCH_POSE.getX())
                    <= DriveConstants.TRENCH_LENGTH
                && Math.abs(robotPose.getY() - DriveConstants.TRENCH_POSE.getY())
                    <= DriveConstants.TRENCH_WIDTH)
            || (Math.abs(robotPose.getX() - flippedTrenchPose.getX())
                    <= DriveConstants.TRENCH_LENGTH
                && Math.abs(robotPose.getY() - flippedTrenchPose.getY())
                    <= DriveConstants.TRENCH_WIDTH)
            || (Math.abs(robotPose.getX() - DriveConstants.TRENCH_POSE.getX())
                    <= DriveConstants.TRENCH_WIDTH
                && Math.abs(robotPose.getY() - flippedTrenchPose.getY())
                    <= DriveConstants.TRENCH_LENGTH)
            || (Math.abs(robotPose.getX() - flippedTrenchPose.getX()) <= DriveConstants.TRENCH_WIDTH
                && Math.abs(robotPose.getY() - DriveConstants.TRENCH_POSE.getY())
                    <= DriveConstants.TRENCH_LENGTH));
    return underTrench;
  }
}
