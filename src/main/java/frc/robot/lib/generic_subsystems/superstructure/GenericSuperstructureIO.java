package frc.robot.lib.generic_subsystems.superstructure;

import com.ctre.phoenix6.signals.GravityTypeValue;
import org.littletonrobotics.junction.AutoLog;

public interface GenericSuperstructureIO {
  @AutoLog
  class GenericSuperstructureIOInputs {
    public boolean isConnected = true;
    public double positionRotations = 0;
    public double velocityRotPerSec = 0;
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0;
    public double statorCurrent = 0;
    public boolean isZeroing = false;
  }

  default void updateInputs(GenericSuperstructureIOInputs inputs) {}

  default void runPosition(double position) {}

  default void runCharacterization() {}

  default void setSlot0(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double jerk,
      GravityTypeValue gravityTypeValue) {}

  default void stop() {}

  default void setOffset() {}

  default void setSupplyCurrentLimit(double amps) {}

  default void setMaxCruiseVelocity(double cruiseVelocity) {}
}
