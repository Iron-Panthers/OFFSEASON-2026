package frc.robot.lib.generic_subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollersIO {
  @AutoLog
  class GenericRollersIOInputs {
    public boolean connected = true;
    public double positionRads = 0;
    public double velocityRadsPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
    public double appliedVelocity;
    public double statorCurrentAmps = 0;
  }

  default void updateInputs(GenericRollersIOInputs inputs) {}

  default void runVelocity(double velocity) {}

  default void stop() {}

  default void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {}

  default void setSupplyCurrentLimit(double amps) {}

  default void setStatorCurrentLimit(double amps) {}
}
