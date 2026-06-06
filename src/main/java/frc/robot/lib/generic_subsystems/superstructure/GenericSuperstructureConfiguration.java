package frc.robot.lib.generic_subsystems.superstructure;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.lib.generic_subsystems.mechanism.GenericMechanismConfiguration;

public class GenericSuperstructureConfiguration extends GenericMechanismConfiguration {
  /**
   * Positive position soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> 3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   */
  public double upperExtensionLimit = 0;

  public boolean upperExtensionLimitEnabled = false;

  /**
   * Positive position soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> 3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withUpperExtensionLimit(double upperExtensionLimit) {
    this.upperExtensionLimit = upperExtensionLimit;
    this.upperExtensionLimitEnabled = true;
    return this;
  }

  /**
   * Negative extension soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> -3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   */
  public double lowerExtensionLimit = 0;

  public boolean lowerExtensionLimitEnabled = false;

  /**
   * Negative extension soft limit (please implement if applicable)
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -3.4e+38
   *   <li><b>Maximum Value:</b> 3.4e+38
   *   <li><b>Default Value:</b> -3.4e+38
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withLowerExtensionLimit(double lowerExtensionLimit) {
    this.lowerExtensionLimit = lowerExtensionLimit;
    this.lowerExtensionLimitEnabled = true;
    return this;
  }

  /**
   * Voltage applied to the motor during zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   */
  public double zeroingVolts = 0;

  /**
   * Voltage applied to the motor during zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withZeroingVolts(double zeroingVolts) {
    this.zeroingVolts = zeroingVolts;
    return this;
  }

  /**
   * Offset applied to the extension after zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   */
  public double zeroingOffset = 0;

  /**
   * Offset applied to the extension after zeroing.
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withZeroingOffset(double zeroingOffset) {
    this.zeroingOffset = zeroingOffset;
    return this;
  }

  /** Sensor discontinuity */
  public double sensorDiscontinuityPoint = 0.5;

  /**
   * Sensor discontinuity
   *
   * @return itself
   */
  public GenericSuperstructureConfiguration withSensorDiscontinuityPoint(
      double sensorDiscontinuityPoint) {
    this.sensorDiscontinuityPoint = sensorDiscontinuityPoint;
    return this;
  }

  // Override parent class methods to return GenericSuperstructureConfiguration for method chaining
  @Override
  public GenericSuperstructureConfiguration withID(int id) {
    super.withID(id);
    return this;
  }

  @Override
  public GenericSuperstructureConfiguration withMotorDirection(InvertedValue motorDirection) {
    super.withMotorDirection(motorDirection);
    return this;
  }

  @Override
  public GenericSuperstructureConfiguration withSupplyCurrentLimit(double supplyCurrentLimit) {
    super.withSupplyCurrentLimit(supplyCurrentLimit);
    return this;
  }

  @Override
  public GenericSuperstructureConfiguration withCANCoderID(int canCoderID) {
    super.withCANCoderID(canCoderID);
    return this;
  }

  @Override
  public GenericSuperstructureConfiguration withCANCoderOffset(double canCoderOffset) {
    super.withCANCoderOffset(canCoderOffset);
    return this;
  }

  @Override
  public GenericSuperstructureConfiguration withCANCoderDirection(
      SensorDirectionValue canCoderDirection) {
    super.withCANCoderDirection(canCoderDirection);
    return this;
  }

  @Override
  public GenericSuperstructureConfiguration withReduction(double reduction) {
    super.withReduction(reduction);
    return this;
  }

  @Override
  public GenericSuperstructureConfiguration withUpperVoltageLimit(double upperVoltLimit) {
    super.withUpperVoltageLimit(upperVoltLimit);
    return this;
  }

  @Override
  public GenericSuperstructureConfiguration withLowerVoltageLimit(double lowerVoltLimit) {
    super.withLowerVoltageLimit(lowerVoltLimit);
    return this;
  }
}
