package frc.robot.subsystems.shooter.shooter_flywheel;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class ShooterFlywheelConstants {
  public static final ShooterFlywheelConfig SHOOTER_FLYWHEEL_CONFIG =
      switch (Constants.getRobotType()) {
          // TODO update motor id for 3 and 4
        case SIM -> new ShooterFlywheelConfig(
            CAN.at(36, "Shooter Flywheel 1"),
            CAN.at(37, "Shooter Flywheel 2"),
            CAN.at(0, "Shooter Flywheel 3"),
            CAN.at(0, "Shooter Flywheel 4"),
            0.71,
            false,
            false,
            true,
            false,
            false);
        default -> new ShooterFlywheelConfig(
            CAN.at(18, "Shooter Flywheel Left Bottom"),
            CAN.at(10, "Shooter Flywheel Left Top"),
            CAN.at(16, "Shooter Flywheel Right Bottom"),
            CAN.at(15, "Shooter Flywheel Right Top"),
            1.411,
            true,
            false,
            false,
            true,
            true);
      };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(3, 0, 0, 0, .1, 0, 0);
        default -> new PIDGains(0.5, 0, 0, 0.2, 0.35, 0, 0);
      };

  public static final double VELOCITY_ADJUSTMENT = 0.98;
  public static final int CURRENT_LIMIT_AMPS =
      switch (Constants.getRobotType()) {
        case SIM -> 40;
        default -> 20;
      };

  public static final ShooterFlywheelPhysicalConstants PHYSICAL_CONSTANTS = // TODO: update values
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterFlywheelPhysicalConstants(0.01, 0.23938936);
        default -> new ShooterFlywheelPhysicalConstants(0.1, 0.23938936);
      };

  // RECORDS
  public record ShooterFlywheelConfig(
      int motorID1,
      int motorID2,
      int motorID3,
      int motorID4,
      double reduction,
      boolean inverted,
      boolean brake,
      boolean opposeMotor1,
      boolean opposeMotor2,
      boolean opposeMotor3) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record ShooterFlywheelPhysicalConstants(
      double momentOfInertia, double circumferenceMeters) {}
}
