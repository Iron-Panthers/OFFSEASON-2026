package frc.robot.subsystems.shooter.shooter_flywheel;

import static frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterFlywheelIOTalonFX extends GenericRollersIOTalonFX implements ShooterFlywheelIO {
  protected TalonFX talon1;
  protected TalonFX talon2;
  protected TalonFX talon3;
  protected TalonFX talon4;

  public ShooterFlywheelIOTalonFX() {
    super(
        new GenericRollersConfiguration()
            .withID(SHOOTER_FLYWHEEL_CONFIG.motorID1())
            .withSupplyCurrentLimit(CURRENT_LIMIT_AMPS)
            .withMotorDirection(
                SHOOTER_FLYWHEEL_CONFIG.inverted()
                    ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive)
            .withNeutralMode(SHOOTER_FLYWHEEL_CONFIG.brake())
            .withReduction(SHOOTER_FLYWHEEL_CONFIG.reduction())
            .withAdditionalFollowerMotor(
                SHOOTER_FLYWHEEL_CONFIG.motorID2(), SHOOTER_FLYWHEEL_CONFIG.opposeMotor1())
            .withAdditionalFollowerMotor(
                SHOOTER_FLYWHEEL_CONFIG.motorID3(), SHOOTER_FLYWHEEL_CONFIG.opposeMotor2())
            .withAdditionalFollowerMotor(
                SHOOTER_FLYWHEEL_CONFIG.motorID4(), SHOOTER_FLYWHEEL_CONFIG.opposeMotor3()));
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
