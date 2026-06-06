package frc.robot.subsystems.intake.intake_rollers;

import static frc.robot.subsystems.intake.intake_rollers.IntakeRollersConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersConfiguration;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;

public class IntakeRollersIOTalonFX extends GenericRollersIOTalonFX implements IntakeRollersIO {
  protected TalonFX talon1;
  protected TalonFX talon2;

  public IntakeRollersIOTalonFX() {
    super(
        new GenericRollersConfiguration()
            .withID(INTAKE_ROLLER_CONFIG.motorID())
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withStatorCurrentLimit(STATOR_CURRENT_LIMIT)
            .withMotorDirection(
                INTAKE_ROLLER_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withNeutralMode(INTAKE_ROLLER_CONFIG.brake())
            .withReduction(INTAKE_ROLLER_CONFIG.reduction())
            .withAdditionalFollowerMotor(
                INTAKE_ROLLER_CONFIG.motorID2(),
                OPPOSE_MOTOR ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
