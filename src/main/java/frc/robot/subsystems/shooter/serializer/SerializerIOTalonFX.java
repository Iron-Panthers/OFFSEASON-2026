package frc.robot.subsystems.shooter.serializer;

import static frc.robot.subsystems.shooter.serializer.SerializerConstants.*;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class SerializerIOTalonFX extends GenericRollersIOTalonFX {

  public SerializerIOTalonFX() {
    //   super(SERIALIZER_CONFIG.motorID(), CURRENT_LIMIT_AMPS, SERIALIZER_CONFIG.inverted(),
    // SERIALIZER_CONFIG.brake(), SERIALIZER_CONFIG.reduction());
    super(
        new GenericRollersConfiguration()
            .withID(SERIALIZER_CONFIG.motorID())
            .withMotorDirection(
                SERIALIZER_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withReduction(SERIALIZER_CONFIG.reduction())
            .withNeutralMode(SERIALIZER_CONFIG.brake())
            .withAdditionalFollowerMotor(
                SERIALIZER_CONFIG.motorID2(), SERIALIZER_CONFIG.opposeMotor())
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT));
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
