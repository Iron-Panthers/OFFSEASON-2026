package frc.robot.subsystems.shooter.shooter_omniwheel;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class ShooterOmniwheelConstants {
  public static final ShooterOmniwheelConfig SHOOTER_OMNIWHEEL_CONFIG =
      switch (Constants.getRobotType()) {
          // SIM now uses the real robot's value. The old SIM number was fudged to compensate
          // for the sim PID regulating MECHANISM velocity where the real TalonFX regulates
          // ROTOR velocity; that is fixed in the IOSim, so the honest value belongs here.
        case SIM -> new ShooterOmniwheelConfig(
            CAN.at(39, "Shooter Omniwheel"), 20.0 / 16, false, true);
        default -> new ShooterOmniwheelConfig(
            CAN.at(12, "Shooter Omniwheel"), 20.0 / 16, false, true);
      };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(0.4, 0, 0, 0.2, .137, 0, 0);
        default -> new PIDGains(0.4, 0, 0, 0.2, .137, 0, 0);
      };

  public static final boolean OPPOSE_MOTOR = true;

  // public static final double STATOR_CURRENT_LIMIT = 60;

  public static final int CURRENT_LIMIT_AMPS =
      switch (Constants.getRobotType()) {
        case COMP -> 60;
          // Was 30; the real robot runs 60.
        case SIM -> 60;
        default -> 30;
      };

  public static final ShooterOmniwheelPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterOmniwheelPhysicalConstants(0.01);
        case COMP -> new ShooterOmniwheelPhysicalConstants(0.1);
        default -> new ShooterOmniwheelPhysicalConstants(0.1);
      };

  // RECORDS
  public record ShooterOmniwheelConfig(
      int motorID, double reduction, boolean inverted, boolean brake) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record ShooterOmniwheelPhysicalConstants(double momentOfInertia) {}
}
