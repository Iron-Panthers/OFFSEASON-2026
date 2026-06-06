package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.getRobotType;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  // measures in meters (per sec) and radians (per sec)
  public static final DrivebaseConfig DRIVE_CONFIG =
      switch (getRobotType()) {
        case COMP -> new DrivebaseConfig(
            Units.inchesToMeters(1.97),
            Units.inchesToMeters(19.75),
            Units.inchesToMeters(24.25),
            Units.inchesToMeters(33),
            Units.inchesToMeters(37),
            5,
            10,
            8);
        case VISION -> new DrivebaseConfig(
            Units.inchesToMeters(1.99),
            Units.inchesToMeters(19.75),
            Units.inchesToMeters(23.75),
            Units.inchesToMeters(34),
            Units.inchesToMeters(34),
            4,
            10,
            10);
        case ALPHA -> new DrivebaseConfig(
            Units.inchesToMeters(1.925),
            Units.inchesToMeters(19.75),
            Units.inchesToMeters(23.75),
            Units.inchesToMeters(34),
            Units.inchesToMeters(34),
            4.5,
            10,
            6);
        case SIM -> new DrivebaseConfig(
            Units.inchesToMeters(1.925),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(22.5),
            Units.inchesToMeters(34),
            Units.inchesToMeters(34),
            3.75, // 3.75,
            10,
            // TODO: make it actually max acceleration in m/s^2
            6); // (multiply by max velocity to get m/s^2)
      };

  // max velocity of the robot for shooting while moving
  public static final double MAX_SCOPED_VELOCITY =
      switch (getRobotType()) {
        case SIM -> 1.5;
        default -> 1.5;
      };

  public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.001, 0.001, 0.001);

  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(DRIVE_CONFIG.trackWidth() / 2.0, DRIVE_CONFIG.trackLength() / 2.0),
        new Translation2d(DRIVE_CONFIG.trackWidth() / 2.0, -DRIVE_CONFIG.trackLength() / 2.0),
        new Translation2d(-DRIVE_CONFIG.trackWidth() / 2.0, DRIVE_CONFIG.trackLength() / 2.0),
        new Translation2d(-DRIVE_CONFIG.trackWidth() / 2.0, -DRIVE_CONFIG.trackLength() / 2.0)
      }; // meters relative to center, NWU convention; fl, fr, bl, br

  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(MODULE_TRANSLATIONS);

  public static final int GYRO_ID = 0;

  public static final boolean IS_GYRO_UPSIDEDOWN =
      switch (getRobotType()) {
        case COMP -> false;
        default -> false;
      };

  public static final Rotation2d GYRO_ROTATION_OFFSET =
      switch (getRobotType()) {
        case COMP -> Rotation2d.kCW_90deg;
        default -> Rotation2d.kZero;
      };

  // fl, fr, bl, br; negate offsets
  public static final ModuleConfig[] MODULE_CONFIGS =
      switch (getRobotType()) {
          // TODO: Check that InvertedValue.(Counter)Clockwise_Positive is for true or false
        case COMP -> new ModuleConfig[] {
          new ModuleConfig(
              CAN.at(11, "FL Drive"),
              CAN.at(62, "FL Steer"),
              3,
              new Rotation2d(2.216602),
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new ModuleConfig(
              CAN.at(35, "FR Drive"),
              CAN.at(6, "FR Steer"),
              12,
              new Rotation2d(2.101554),
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.CounterClockwise_Positive),
          new ModuleConfig(
              CAN.at(3, "BL Drive"),
              CAN.at(4, "BL Steer"),
              6,
              new Rotation2d(2.847068),
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new ModuleConfig(
              CAN.at(2, "BR Drive"),
              CAN.at(1, "BR Steer"),
              25,
              new Rotation2d(1.377515),
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.CounterClockwise_Positive)
        };
        case VISION -> new ModuleConfig[] {
          new ModuleConfig(
              CAN.at(3, "FL Drive"),
              CAN.at(4, "FL Steer"),
              6,
              new Rotation2d(-1.876059),
              InvertedValue.CounterClockwise_Positive, // steer
              InvertedValue.Clockwise_Positive), // drive
          new ModuleConfig(
              CAN.at(11, "FR Drive"),
              CAN.at(10, "FR Steer"),
              12,
              new Rotation2d(0.619728),
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.CounterClockwise_Positive),
          new ModuleConfig(
              CAN.at(2, "BL Drive"),
              CAN.at(1, "BL Steer"),
              3,
              new Rotation2d(-2.323981),
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new ModuleConfig(
              CAN.at(38, "BR Drive"),
              CAN.at(6, "BR Steer"),
              9,
              new Rotation2d(1.078388),
              InvertedValue.CounterClockwise_Positive,
              InvertedValue.CounterClockwise_Positive)
        };
        case ALPHA -> new ModuleConfig[] {
          new ModuleConfig(
              CAN.at(3, "FL Drive"),
              CAN.at(4, "FL Steer"),
              6,
              new Rotation2d(2.058602),
              InvertedValue.Clockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new ModuleConfig(
              CAN.at(11, "FR Drive"),
              CAN.at(10, "FR Steer"),
              3,
              new Rotation2d(-2.161379),
              InvertedValue.Clockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new ModuleConfig(
              CAN.at(2, "BL Drive"),
              CAN.at(1, "BL Steer"),
              3,
              new Rotation2d(0.48934),
              InvertedValue.Clockwise_Positive,
              InvertedValue.CounterClockwise_Positive),
          new ModuleConfig(
              CAN.at(5, "BR Drive"),
              CAN.at(7, "BRSteer"),
              2,
              new Rotation2d(-0.271515),
              InvertedValue.Clockwise_Positive,
              InvertedValue.Clockwise_Positive)
        };
        case SIM -> new ModuleConfig[] {
          new ModuleConfig(
              CAN.at(19, "FL Drive"),
              CAN.at(18, "FL Steer"),
              2,
              new Rotation2d(-1.148),
              InvertedValue.Clockwise_Positive,
              InvertedValue.CounterClockwise_Positive),
          new ModuleConfig(
              CAN.at(17, "FR Drive"),
              CAN.at(16, "FR Steer"),
              1,
              new Rotation2d(-0.405),
              InvertedValue.Clockwise_Positive,
              InvertedValue.Clockwise_Positive),
          new ModuleConfig(
              CAN.at(21, "BL Drive"),
              CAN.at(20, "BL Steer"),
              3,
              new Rotation2d(1.0139),
              InvertedValue.Clockwise_Positive,
              InvertedValue.CounterClockwise_Positive),
          new ModuleConfig(
              CAN.at(23, "BR Drive"),
              CAN.at(22, "BRSteer"),
              4,
              new Rotation2d(-2.8148),
              InvertedValue.Clockwise_Positive,
              InvertedValue.Clockwise_Positive)
        };
      };

  public static final ModuleConstants MODULE_CONSTANTS =
      switch (getRobotType()) {
        case COMP -> new ModuleConstants(
            new Gains(0.24, 2.4, 0.08, 70, 0, 0),
            new MotionProfileGains(4, 64, 640),
            new Gains(0.16, 0.67, 0, 1.5, 0, 0),
            (30.0 / 15) * (25.0 / 32) * (54.0 / 14), // Mk5n L2.5 16 tooth
            287.0 / 11,
            3.125);
        case VISION -> new ModuleConstants(
            new Gains(0.24, 2.4, 0.08, 70, 0, 0),
            new MotionProfileGains(4, 64, 640),
            new Gains(0.16, 0.67, 0, 1.5, 0, 0),
            (30.0 / 15) * (25.0 / 32) * (54.0 / 14), // Mk5n L2.5 16 tooth
            287.0 / 11,
            3.125);
        case ALPHA -> new ModuleConstants(
            new Gains(0.25, 2.26, 0, 50, 0, 0),
            new MotionProfileGains(4, 64, 640),
            new Gains(0.16, 0.67, 0, 1.5, 0, 0),
            (45.0 / 15) * (17.0 / 27) * (50.0 / 16), // MK4i L2.5 16 tooth
            150.0 / 7,
            3.125);
        case SIM -> new ModuleConstants(
            new Gains(0.25, 2.26, 0, 70, 0, 0),
            new MotionProfileGains(4, 64, 640),
            new Gains(0.13, 0.79, 0.387, 2, 0, 0),
            (30.0 / 15) * (25.0 / 32) * (54.0 / 14), // MK5n R2 ratio
            287.0 / 11,
            3.125);
      };

  public static final double STEER_CURRENT_LIMIT_AMPS = 10;
  public static final double DRIVE_CURRENT_LIMIT_AMPS = 40;

  /**
   * These are the configs for the maple sim drivebase This should be updated to be similar to the
   * comp bot drivebase
   */
  public static final DriveTrainSimulationConfig
      mapleSimConfig = // TODO: update this to be similar to comp bot drive base
      DriveTrainSimulationConfig.Default()
              .withRobotMass(Kilograms.of(54.4311))
              .withCustomModuleTranslations(MODULE_TRANSLATIONS)
              .withGyro(COTS.ofPigeon2())
              .withSwerveModule(
                  new SwerveModuleSimulationConfig(
                      DCMotor.getKrakenX60(1),
                      DCMotor.getKrakenX60(1),
                      MODULE_CONSTANTS.driveReduction,
                      MODULE_CONSTANTS.steerReduction,
                      Volts.of(0.13),
                      Volts.of(0.25),
                      Meters.of(DRIVE_CONFIG.wheelRadius()),
                      KilogramSquareMeters.of(0.04),
                      1.4));

  public static final TrajectoryFollowerConstants TRAJECTORY_CONFIG =
      switch (getRobotType()) {
        case COMP -> new TrajectoryFollowerConstants(
            new PIDConstants(8, 0), new PIDConstants(4, 0));
        case VISION -> new TrajectoryFollowerConstants(
            new PIDConstants(8, 0), new PIDConstants(4, 0));
        case ALPHA -> new TrajectoryFollowerConstants(
            new PIDConstants(8, 0), new PIDConstants(4, 0));
        case SIM -> new TrajectoryFollowerConstants(new PIDConstants(8, 0), new PIDConstants(4, 0));
        default -> new TrajectoryFollowerConstants(new PIDConstants(0, 0), new PIDConstants(0, 0));
      };

  // Tolerance in Radians
  public static final HeadingControllerConstants HEADING_CONTROLLER_CONSTANTS =
      switch (getRobotType()) {
        case COMP -> new HeadingControllerConstants(6, 0, 5, 200, 0.03);
        case SIM -> new HeadingControllerConstants(6, 0, 5, 200, 0.01);
        case VISION -> new HeadingControllerConstants(3, 0, 5, 15, 0.007);
        case ALPHA -> new HeadingControllerConstants(6, 0, 5, 200, 0.002);
        default -> new HeadingControllerConstants(0, 0, 0, 0, 0);
      };

  public static final PIDAutoAlignControllerConstants PID_AUTOALIGN_CONSTANTS =
      switch (getRobotType()) {
        case COMP -> new PIDAutoAlignControllerConstants(
            8, 0, 0, 3, 3, 0.03); /*FIXME: tune these constants*/
        case VISION -> new PIDAutoAlignControllerConstants(
            8, 0, 0, 3, 3, 0.01); /*FIXME: tune these constants*/
        case ALPHA -> new PIDAutoAlignControllerConstants(
            7, 0, 0, 1, 1, 0.01); /* FIXME: tune these constants */
        case SIM -> new PIDAutoAlignControllerConstants(7, 0.0, 0.0, 3, 4, 0.01);
        default -> new PIDAutoAlignControllerConstants(0, 0, 0, 0, 0, 0.01);
      };
  public static final double ROTATION_FINISH_PERCENT = 0.9;

  public static final double PATHPLANNER_PID_OFFSET = 1.5;

  public static final double AUTOALIGN_POSITION_DEADBAND = 0.01;

  public static final double AUTOALIGN_VELOCITY_DEADBAND = 0.01;

  public static final Pose2d INITIAL_POSE = new Pose2d(2.9, 3.8, new Rotation2d(1, 0));

  public static final PPHolonomicDriveController HOLONOMIC_DRIVE_CONTROLLER =
      new PPHolonomicDriveController(
          TRAJECTORY_CONFIG.linearPID(),
          TRAJECTORY_CONFIG.rotationPID(),
          Constants.PERIODIC_LOOP_SEC);

  public static final PathConstraints PP_PATH_CONSTRAINTS =
      new PathConstraints(
          3, 3, Units.degreesToRadians(540), Units.degreesToRadians(5000), 12, false);

  public static final PathConstraints ALIGN_PATH_CONSTRAINTS =
      new PathConstraints(
          3, 4, Units.degreesToRadians(540), Units.degreesToRadians(720), 12, false);
  // unused
  public static final PathConstraints APPROACH_PATH_CONSTRAINTS =
      new PathConstraints(
          1.5, 1.5, Units.degreesToRadians(540), Units.degreesToRadians(720), 12, false);

  // pathfinding constants
  public static final List<Pair<Translation2d, Translation2d>> OBSTACLES_FOR_TRENCH_PATHFINDING =
      List.of(
          Pair.of(new Translation2d(4.039, 6.590), new Translation2d(5.216, 4.572)),
          Pair.of(new Translation2d(4.039, 3.472), new Translation2d(5.216, 1.570)),
          Pair.of(
              FlippingUtil.flipFieldPosition(new Translation2d(4.039, 6.590)),
              FlippingUtil.flipFieldPosition(new Translation2d(5.216, 4.572))),
          Pair.of(
              FlippingUtil.flipFieldPosition(new Translation2d(4.039, 3.472)),
              FlippingUtil.flipFieldPosition(new Translation2d(5.216, 1.570))));

  public static final List<Pair<Translation2d, Translation2d>> OBSTACLES_FOR_BUMP_PATHFINDING =
      List.of(
          Pair.of(new Translation2d(4.039, 8.117), new Translation2d(5.216, 6.746)),
          Pair.of(new Translation2d(4.039, 1.337), new Translation2d(5.216, 0)),
          Pair.of(
              FlippingUtil.flipFieldPosition(new Translation2d(4.039, 8.117)),
              FlippingUtil.flipFieldPosition(new Translation2d(5.216, 6.746))),
          Pair.of(
              FlippingUtil.flipFieldPosition(new Translation2d(4.039, 1.337)),
              FlippingUtil.flipFieldPosition(new Translation2d(5.216, 0))));

  public static final Translation2d CENTER_OF_FIELD = new Translation2d(8.27, 4.035);
  public static final Translation3d BLUE_HUB_ORIGIN = new Translation3d(4.5974, 4.034536, 1.5748);
  public static final Translation3d RED_HUB_ORIGIN = new Translation3d(11.938, 4.034536, 1.5748);
  public static final double HUB_WIDTH = Units.inchesToMeters(24);

  public record DrivebaseConfig(
      double wheelRadius,
      double trackWidth,
      double trackLength,
      double bumperWidthX,
      double bumperWidthY,
      double maxLinearVelocity,
      double maxAngularVelocity,
      double maxLinearAcceleration) {}

  public record ModuleConfig(
      int driveID,
      int steerID,
      int encoderID,
      Rotation2d absoluteEncoderOffset,
      InvertedValue steerInverted,
      InvertedValue driveInverted) {}

  public record ModuleConstants(
      Gains steerGains,
      MotionProfileGains steerMotionGains,
      Gains driveGains,
      double driveReduction,
      double steerReduction,
      double couplingGearReduction) {}

  public record TrajectoryFollowerConstants(PIDConstants linearPID, PIDConstants rotationPID) {}

  public record Gains(double kS, double kV, double kA, double kP, double kI, double kD) {}

  public record MotionProfileGains(double cruiseVelocity, double acceleration, double jerk) {}

  /* tolerance in degrees */
  public record HeadingControllerConstants(
      double kP, double kD, double maxVelocity, double maxAcceleration, double tolerance) {}

  public record PIDAutoAlignControllerConstants(
      double kP,
      double kI,
      double kD,
      double maxVelocity,
      double maxAcceleration,
      double tolerance) {}

  public record ApproachPose(Pose2d pose) {
    public static ApproachPose[] fromPose2ds(Pose2d... poses) {
      List<ApproachPose> approachPoses = new ArrayList<ApproachPose>();
      for (Pose2d pose : poses) {
        approachPoses.add(new ApproachPose(pose));
      }
      return approachPoses.toArray(new ApproachPose[approachPoses.size()]);
    }

    public Pose2d getAlliancePose() {
      return RobotState.isAllianceRed() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public Pose2d getPose() {
      return pose;
    }

    public PathPlannerPath generatePath() {
      // approach @ 12 inch off, advance to 6 in.
      List<Waypoint> waypoints =
          PathPlannerPath.waypointsFromPoses(
              getAlliancePose(), getAlliancePose().exp(new Twist2d(0.1524, 0, 0)));

      return new PathPlannerPath(
          waypoints,
          PP_PATH_CONSTRAINTS,
          new IdealStartingState(2, getAlliancePose().getRotation()),
          new GoalEndState(0, getAlliancePose().getRotation()));
    }
  }

  // TODO: maybe change these?
  public static final double TRENCH_WIDTH = 0.65;
  public static final double TRENCH_LENGTH = 0.6;
  public static final Pose2d TRENCH_POSE = new Pose2d(4.6, 0.65, new Rotation2d());

  public static final Pose2d BLUE_LEFT_CLIMB_DEFENSE_POSE =
      new Pose2d(0.835, 4.718, new Rotation2d(Math.toRadians(-11)));
  public static final Pose2d BLUE_RIGHT_CLIMB_DEFENSE_POSE =
      new Pose2d(0.944, 2.768, new Rotation2d(Math.toRadians(19)));
  public static final Pose2d BLUE_UNDER_TRENCH_DEFENSE_POSE =
      new Pose2d(3.75, 0.9, new Rotation2d());
}
