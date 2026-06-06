package frc.robot.subsystems.shooter.shooter_accelerator;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class ShooterAcceleratorConstants {
  public static final ShooterAcceleratorConfig SHOOTER_ACCELERATOR_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterAcceleratorConfig(
            CAN.at(38, "Shooter Accelerator 1"),
            CAN.at(39, "Shooter Accelerator 2"),
            0.67, // changed in sim (otherwise 1)
            true,
            false,
            true);
        default -> new ShooterAcceleratorConfig(
            CAN.at(25, "Shooter Accelerator Left"),
            CAN.at(13, "Shooter Accelerator Right"),
            1.5,
            true,
            true,
            true);
      };

  // CONTROL LOOP GAINS AND MOTION MAGIC CONFIG
  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
        case SIM -> new PIDGains(1, 0, 0, 0, .1, 0, 0);
        default -> new PIDGains(.6, 0, 0, 0.2, 0.17746, 0, 0);
      };

  public static final int CURRENT_LIMIT_AMPS =
      switch (Constants.getRobotType()) {
        case COMP -> 30;
        case SIM -> 30;
        default -> 30;
      };

  public static final ShooterAcceleratorPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new ShooterAcceleratorPhysicalConstants(0.01);
        case COMP -> new ShooterAcceleratorPhysicalConstants(0.1);
        default -> new ShooterAcceleratorPhysicalConstants(0.1);
      };

  // RECORDS
  public record ShooterAcceleratorConfig(
      int motorID1,
      int motorID2,
      double reduction,
      boolean inverted,
      boolean brake,
      boolean oppose_motor) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record ShooterAcceleratorPhysicalConstants(double momentOfInertia) {}
}
