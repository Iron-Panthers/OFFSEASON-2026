package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class IntakeRollersConstants {
  // MOTOR AND SENSOR CONFIGURATION
  public static final IntakeRollerConfig INTAKE_ROLLER_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new IntakeRollerConfig(
            CAN.at(64, "Intake Roller"), CAN.at(65, "Intake Roller 2"), 2, false, true);
        case COMP -> new IntakeRollerConfig(
            CAN.at(8, "Intake Roller Left"),
            CAN.at(9, "Intake Roller Right"),
            36.0 / 15.0,
            false,
            true);
        default -> new IntakeRollerConfig(
            CAN.at(0, "Intake Roller"), CAN.at(0, "Intake Roller 2"), 2, false, true);
      };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(1, 0, 0, 0, 1, 0, 0);
        case COMP -> new PIDGains(0.3, 0, 0, 0.1, 0.2739, 0, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final boolean OPPOSE_MOTOR = true;

  // CURRENT LIMITS
  public static final double UPPER_VOLT_LIMIT = 12;
  public static final double LOWER_VOLT_LIMIT = -12;
  public static final int CURRENT_LIMIT_AMPS = 30;
  public static final double STATOR_CURRENT_LIMIT = 50;

  public static final IntakeRollerPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new IntakeRollerPhysicalConstants(0.01);
        default -> new IntakeRollerPhysicalConstants(0.1);
      };

  // RECORDS
  public record IntakeRollerConfig(
      int motorID, int motorID2, double reduction, boolean inverted, boolean brake) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record IntakeRollerPhysicalConstants(double momentOfInertia) {}
}
