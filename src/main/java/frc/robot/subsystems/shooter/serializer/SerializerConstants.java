package frc.robot.subsystems.shooter.serializer;

import frc.robot.Constants;
import frc.robot.subsystems.can_watchdog.CANWatchdogConstants.CAN;

public class SerializerConstants {
  public static final SerializerConfig SERIALIZER_CONFIG =
      switch (Constants.getRobotType()) {
        case SIM -> new SerializerConfig(
            CAN.at(32, "Serializer"),
            CAN.at(0, "Serializer 2"),
            // Was 5, the last SIM reduction still left at a made-up value after the other four
            // rollers were aligned to their real ones. It ran the simulated serializer at
            // 80.2 rad/s against the real 58.7.
            2.833333,
            true,
            false,
            true);
        case COMP -> new SerializerConfig(
            CAN.at(41, "Serializer Left"),
            CAN.at(33, "Serializer Right"),
            2.833333,
            false,
            false,
            true);
        default -> new SerializerConfig(
            CAN.at(40, "Serializer"), CAN.at(0, "Serializer 2"), 5, true, false, true);
      };

  public static final PIDGains GAINS =
      switch (Constants.getRobotType()) {
          // SIM shares COMP's gains. SerializerSim drives the physics from the Talon's own
          // simulated control loop, so different gains here mean a different closed loop.
        case SIM -> new PIDGains(0.5, 0, 0, 0.2, 0.344827586, 0, 0);
        case COMP -> new PIDGains(0.5, 0, 0, 0.2, 0.344827586, 0, 0);
        default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final int CURRENT_LIMIT_AMPS = 20;

  public static final double STATOR_CURRENT_LIMIT = 30;

  public static final SerializerPhysicalConstants PHYSICAL_CONSTANTS =
      switch (Constants.getRobotType()) {
        case SIM -> new SerializerPhysicalConstants(0.000105);
        default -> new SerializerPhysicalConstants(0.000105);
      };

  // RECORDS
  public record SerializerConfig(
      int motorID,
      int motorID2,
      double reduction,
      boolean inverted,
      boolean brake,
      boolean opposeMotor) {}

  public record PIDGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  public static record SerializerPhysicalConstants(double momentOfIntertia) {}
}
