package frc.robot.subsystems.intake.intake_rack;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

// making changes
public class IntakeRackConstants {
  public static final IntakeRackConfig INTAKE_RACK_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new IntakeRackConfig(
            // Reduction between sensor and mechansim
            CAN.at(19, "Intake Rack"), 8 / Math.PI, InvertedValue.Clockwise_Positive);
        case SIM -> new IntakeRackConfig(
            // Reduction between motor and mechansim
            CAN.at(9, "Intake Rack"), 8 / Math.PI, InvertedValue.Clockwise_Positive);
        default -> new IntakeRackConfig(0, 1, InvertedValue.CounterClockwise_Positive);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case COMP -> new PIDGains(7, 0, 0, 0.55, 0.24, 0, 0);
        case SIM -> new PIDGains(7, 0, 0, 0.55, 0.24, 0, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(400, 40);
        case SIM -> new MotionMagicConfig(400, 40);
        default -> new MotionMagicConfig(0, 0);
      };

  public record IntakeRackConfig(int motorID, double reduction, InvertedValue motorDirection) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public record MotionMagicConfig(double acceleration, double cruiseVelocity) {}

  public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;

  public static final double POSITION_TARGET_EPSILON = 3;

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 12;
  public static final double LOWER_VOLT_LIMIT = -12;
  public static final double SUPPLY_CURRENT_LIMIT = 27;

  // ZEROING CONSTANTS
  public static final double ZEROING_VOLTS = -3;
  public static final double ZEROING_OFFSET = 0; // offset in rotations

  public static final Transform3d BASE_TO_INTAKE_RACK_TRANSFORM =
      switch (Constants.getRobotType()) {
        default -> new Pose3d()
            .plus(
                new Transform3d(
                    new Translation3d(
                        // Units.inchesToMeters(0),
                        // Units.inchesToMeters(0),
                        // Units.inchesToMeters(0)),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(9.990),
                        Units.inchesToMeters(7.709)),
                    new Rotation3d(0, 0, 0)))
            .rotateBy(new Rotation3d(0, 0, Math.toRadians(90)))
            .minus(new Pose3d());
      };

  public static record IntakeRackPhysicalConstants(
      double massInKilograms,
      double drumRadiusMeters,
      double minExtensionMeters,
      double maxExtensionMeters,
      boolean simulateGravity) {}

  public static final IntakeRackPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new IntakeRackPhysicalConstants(0.1, 0.1, -15, 15, false);
        case COMP -> new IntakeRackPhysicalConstants(0.1, 0, 0, 0, false);
        default -> new IntakeRackPhysicalConstants(0.1, 0, 0, 0, false);
      };
}
