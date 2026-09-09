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

  /**
   * Upper travel bound for the simulated rack, in mechanism rotations.
   *
   * <p>NOT a mechanical hard stop. An earlier value of 11.29 was inferred from q54, where the real
   * rack never exceeded 11.290 and stalled against whatever stopped it for most of the match. The
   * q93 validation log disproved that: the real rack reaches **11.68** there, past the 11.6
   * commanded target, with 0.32 V mean applied and 1.87 A mean draw -- no stall at all. So q54's
   * ceiling was that match's obstruction, not the mechanism's limit.
   *
   * <p>Capping the sim at 11.29 left the controller with permanent error against an unreachable
   * target, pushing forever: 2540 amp-seconds against a real 213 on q93, a 12x over-draw, which
   * alone accounted for that log's currents score being 44% worse than q54's.
   *
   * <p>Set above every observed value so it bounds the ElevatorSim without ever binding.
   */
  public static final double RACK_HARD_STOP_ROTATIONS = 13.0;

  // In MECHANISM rotations now that the sim applies SensorToMechanismRatio.
  private static final double RACK_HARD_STOP_METERS =
      RACK_HARD_STOP_ROTATIONS * 2.0 * Math.PI * 0.1;

  public static final IntakeRackPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
          // Travel limits bound the ElevatorSim without binding in normal operation. The old
          // +/-15 m (= +/-60.8 rotations) was effectively no bound at all. Previously the real
          // robot
          // never exceeds
          // 11.290 rotations all match: it parks at 11.18-11.20 against a mechanical stop and
          // holds 85-102 A stator / 25-30 A supply for 86.7% of the match, because the commanded
          // INTAKE target of 11.6 sits past that stop. The old +/-15 m (= +/-60.8 rotations) let
          // the sim converge cleanly and draw nothing, which is most of the 930.7 vs 63.3
          // amp-second gap. 11.29 rot at a 0.1 m drum = 11.29 * 2*pi * 0.1 / reduction metres.
          //
          // The real robot's INTAKE target is deliberately left alone -- the goal is for the sim
          // to reproduce the Worlds logs, not to change the robot that produced them.
        case SIM -> new IntakeRackPhysicalConstants(0.1, 0.1, 0, RACK_HARD_STOP_METERS, false);
        case COMP -> new IntakeRackPhysicalConstants(0.1, 0, 0, 0, false);
        default -> new IntakeRackPhysicalConstants(0.1, 0, 0, 0, false);
      };
}
