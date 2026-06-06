package frc.robot.lib.generic_subsystems.rollers;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.lib.generic_subsystems.mechanism.GenericMechanismConfiguration;

public class GenericRollersConfiguration extends GenericMechanismConfiguration {

  public NeutralModeValue neutralMode = NeutralModeValue.Brake;

  /**
   * Sets whether the motor is in brake or coast mode when no power is applied
   *
   * @param brake
   * @return
   */
  public GenericRollersConfiguration withNeutralMode(boolean brake) {
    this.neutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    return this;
  }

  // Override parent class methods to return GenericRollersConfiguration for method chaining
  @Override
  public GenericRollersConfiguration withID(int id) {
    super.withID(id);
    return this;
  }

  @Override
  public GenericRollersConfiguration withMotorDirection(InvertedValue motorDirection) {
    super.withMotorDirection(motorDirection);
    return this;
  }

  @Override
  public GenericRollersConfiguration withSupplyCurrentLimit(double supplyCurrentLimit) {
    super.withSupplyCurrentLimit(supplyCurrentLimit);
    return this;
  }

  @Override
  public GenericRollersConfiguration withStatorCurrentLimit(double statorCurrentLimit) {
    super.withStatorCurrentLimit(statorCurrentLimit);
    return this;
  }

  @Override
  public GenericRollersConfiguration withCANCoderID(int canCoderID) {
    super.withCANCoderID(canCoderID);
    return this;
  }

  @Override
  public GenericRollersConfiguration withCANCoderOffset(double canCoderOffset) {
    super.withCANCoderOffset(canCoderOffset);
    return this;
  }

  @Override
  public GenericRollersConfiguration withCANCoderDirection(SensorDirectionValue canCoderDirection) {
    super.withCANCoderDirection(canCoderDirection);
    return this;
  }

  @Override
  public GenericRollersConfiguration withReduction(double reduction) {
    super.withReduction(reduction);
    return this;
  }

  @Override
  public GenericRollersConfiguration withAdditionalFollowerMotor(
      int id, MotorAlignmentValue motorAlignmentValue) {
    super.withAdditionalFollowerMotor(id, motorAlignmentValue);
    return this;
  }

  public GenericRollersConfiguration withAdditionalFollowerMotor(int id, boolean opposeMotor) {
    withAdditionalFollowerMotor(
        id, opposeMotor ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned);
    return this;
  }

  public GenericRollersConfiguration withUpperVoltageLimit(double upperVoltLimit) {
    super.withUpperVoltageLimit(upperVoltLimit);
    return this;
  }

  public GenericRollersConfiguration withLowerVoltageLimit(double lowerVoltLimit) {
    super.withLowerVoltageLimit(lowerVoltLimit);
    return this;
  }
}
