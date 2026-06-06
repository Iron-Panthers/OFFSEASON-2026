package frc.robot.subsystems.shooter.shooter_hood;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class ShooterHoodConstants {
  public static final ShooterHoodConfig SHOOTER_HOOD_CONFIG =
      // TODO update the id's and info
      switch (Constants.getRobotType()) {
        case COMP -> new ShooterHoodConfig(
            // reduction between sensor and mechanism
            CAN.at(30, "Shooter Hood"), 36 / 12.0 * 158 / 10);
        case SIM -> new ShooterHoodConfig(
            // Reduction between motor and mechanism
            CAN.at(8, "Shooter Hood"), 12 * .375);
        default -> new ShooterHoodConfig(0, 1);
      };

  // TODO update all the PID information
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(1000, 0, 0, 0, 5.1282, 0, 0.8);
        case SIM -> new PIDGains(60, 0, 0, 0, 2.265488, 0.1, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  // TODO update Motion Magic
  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(6, 1);
        case SIM -> new MotionMagicConfig(7.5, 10);
        default -> new MotionMagicConfig(0, 0);
      };

  public record ShooterHoodConfig(int motorID, double reduction) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double accelerations, double cruiseVelocity) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final InvertedValue MOTOR_DIRECTION = InvertedValue.CounterClockwise_Positive;

  public static final double POSITION_TARGET_EPSILON = 0.005;

  /** Distance from center of hood rotation to hood end */
  public static final double SHOOTER_HOOD_LENGTH = 10; // in inches

  // TODO Update Limits
  public static final double UPPER_VOLT_LIMIT = 6;
  public static final double LOWER_VOLT_LIMIT = -6;
  public static final double SUPPLY_CURRENT_LIMIT = 20;

  // TODO Change Limits
  public static final double ZEROING_VOLTS = -3;
  public static final double ZEROING_OFFSET = 0; // offset in degrees
  public static final double ZEROING_VOLTAGE_THRESHOLD = 5;

  public static final double SENSOR_DISCONTINUITY_POINT = 0.82; // TODO: do we need this

  // PHYSICAL CONSTANTS
  public static final Transform3d BASE_TO_SHOOTER_HOOD_TRANSFORM =
      switch (Constants.getRobotType()) {
        default -> new Pose3d()
            .plus(
                new Transform3d(
                    new Translation3d(
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(-9.5),
                        Units.inchesToMeters(20.234)),
                    // Units.inchesToMeters(0),
                    // Units.inchesToMeters(0),
                    // Units.inchesToMeters(0)),
                    new Rotation3d(0, 0, Math.toRadians(0))))
            .rotateBy(new Rotation3d(0, 0, Math.toRadians(90)))
            .minus(new Pose3d());
      };

  public static record ShooterHoodPhysicalConstants(
      double momentOfInertia,
      double lengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulatedGravity) {}

  // TODO Add in phhysical constants
  public static final ShooterHoodPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterHoodPhysicalConstants(
            0.001,
            Units.inchesToMeters(SHOOTER_HOOD_LENGTH),
            Units.degreesToRadians(0),
            Units.degreesToRadians(360),
            false);
        case COMP -> new ShooterHoodPhysicalConstants(0.1, 0, 0, 0, false);
        default -> new ShooterHoodPhysicalConstants(0.1, 0, 0, 0, false);
      };
}
