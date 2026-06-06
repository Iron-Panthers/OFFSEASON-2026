package frc.robot.subsystems.shooter.shooter_omniwheel;

import static frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.*;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;

public class ShooterOmniwheelIOTalonFX extends GenericRollersIOTalonFX
    implements ShooterOmniwheelIO {
  public ShooterOmniwheelIOTalonFX() {
    super(
        new GenericRollersConfiguration()
            .withID(SHOOTER_OMNIWHEEL_CONFIG.motorID())
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withMotorDirection(
                SHOOTER_OMNIWHEEL_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            // .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
            .withNeutralMode(SHOOTER_OMNIWHEEL_CONFIG.brake())
            .withReduction(SHOOTER_OMNIWHEEL_CONFIG.reduction()));
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
