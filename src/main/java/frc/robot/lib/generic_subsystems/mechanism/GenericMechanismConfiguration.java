package frc.robot.lib.generic_subsystems.mechanism;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import java.util.ArrayList;

public abstract class GenericMechanismConfiguration {

  /**
   * Non-zero ID of mechanism motor
   *
   * <ul>
   *   <li><b>Default Value:</b> -1
   * </ul>
   */
  public int id = -1;

  /**
   * Non-zero ID of mechanism motor
   *
   * <ul>
   *   <li><b>Default Value:</b> 0
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withID(int id) {
    this.id = id;
    return this;
  }

  /**
   * Direction of the motor when it is moving in the positive direction, as seen from the front
   *
   * <ul>
   *   <li><b>Default Value:</b> Clockwise_Positive
   * </ul>
   */
  public InvertedValue motorDirection = InvertedValue.Clockwise_Positive;

  /**
   * Direction of the motor when it is moving in the positive direction, as seen from the front
   *
   * <ul>
   *   <li><b>Default Value:</b> Clockwise_Positive
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withMotorDirection(InvertedValue motorDirection) {
    this.motorDirection = motorDirection;
    return this;
  }

  /**
   * The absolute maximum amount of supply current allowed.
   *
   * <p>Supply current limits commonly range from 20-80 A depending on the breaker used.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 800.0
   *   <li><b>Default Value:</b> 70
   *   <li><b>Units:</b> A
   * </ul>
   */
  public double supplyCurrentLimit = 70;

  /**
   * The absolute maximum amount of supply current allowed.
   *
   * <p>Supply current limits commonly range from 20-80 A depending on the breaker used.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 800.0
   *   <li><b>Default Value:</b> 70
   *   <li><b>Units:</b> A
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withSupplyCurrentLimit(double supplyCurrentLimit) {
    this.supplyCurrentLimit = supplyCurrentLimit;
    return this;
  }

  /**
   * The absolute maximum amount of stator current allowed.
   *
   * <p>Supply current limits commonly range from 20-80 A depending on the breaker used.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 800.0
   *   <li><b>Default Value:</b> 120
   *   <li><b>Units:</b> A
   * </ul>
   */
  public double statorCurrentLimit = 140;

  /**
   * The absolute maximum amount of supply current allowed.
   *
   * <p>Supply current limits commonly range from 20-80 A depending on the breaker used.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> 0.0
   *   <li><b>Maximum Value:</b> 800.0
   *   <li><b>Default Value:</b> 120
   *   <li><b>Units:</b> A
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withStatorCurrentLimit(double statorCurrentLimit) {
    this.statorCurrentLimit = statorCurrentLimit;
    return this;
  }

  /**
   * The non-zero ID for a CANCoder. The Talon will update its position and velocity whenever
   * CANcoder publishes its information on CAN bus, and the Talon internal rotor will not be used.
   */
  public int canCoderID = -1;

  /**
   * The non-zero ID for a CANCoder. The Talon will update its position and velocity whenever
   * CANcoder publishes its information on CAN bus, and the Talon internal rotor will not be used.
   *
   * @return itself
   */
  public GenericMechanismConfiguration withCANCoderID(int canCoderID) {
    this.canCoderID = canCoderID;
    return this;
  }

  /**
   * This offset is added to the reported CANCoder position if CANCoder ID is not 0.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -1
   *   <li><b>Maximum Value:</b> 1
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   */
  public double canCoderOffset = 0;

  /**
   * This offset is added to the reported CANCoder position if CANCoder ID is not 0.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -1
   *   <li><b>Maximum Value:</b> 1
   *   <li><b>Default Value:</b> 0
   *   <li><b>Units:</b> rotations
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withCANCoderOffset(double canCoderOffset) {
    this.canCoderOffset = canCoderOffset;
    return this;
  }

  /**
   * Direction of the CANCoder to determine positive rotation, as seen facing the LED side of the
   * CANcoder.
   *
   * <ul>
   *   <li><b>Default Value:</b> CounterClockwise_Positive
   * </ul>
   */
  public SensorDirectionValue canCoderDirection = SensorDirectionValue.CounterClockwise_Positive;

  /**
   * Direction of the CANCoder to determine positive rotation, as seen facing the LED side of the
   * CANcoder.
   *
   * <ul>
   *   <li><b>Default Value:</b> CounterClockwise_Positive
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withCANCoderDirection(
      SensorDirectionValue canCoderDirection) {
    this.canCoderDirection = canCoderDirection;
    return this;
  }

  /**
   * The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a
   * reduction.
   *
   * <p>This is equivalent to the mechanism's gear ratio if the sensor is located on the input of a
   * gearbox. If sensor is on the output of a gearbox, then this is typically set to 1.
   *
   * <p>Don't use this config to perform unit conversion.
   *
   * <p>*
   *
   * <ul>
   *   <li><b>Default Value:</b> 70
   * </ul>
   */
  public double reduction = 1.0;

  /**
   * The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a
   * reduction.
   *
   * <p>This is equivalent to the mechanism's gear ratio if the sensor is located on the input of a
   * gearbox. If sensor is on the output of a gearbox, then this is typically set to 1.
   *
   * <p>Don't use this config to perform unit conversion.
   *
   * <p>*
   *
   * <ul>
   *   <li><b>Default Value:</b> 70
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withReduction(double reduction) {
    this.reduction = reduction;
    return this;
  }

  public record FollowerMotorConfig(int id, MotorAlignmentValue motorAlignmentValue) {}

  public ArrayList<FollowerMotorConfig> followerMotors = new ArrayList<>();

  /**
   * Adds a follower motor to the mechanism configuration.
   *
   * @param id CAN ID of the follower motor
   * @param inverted Inversion state of the follower motor
   * @return itself
   */
  public GenericMechanismConfiguration withAdditionalFollowerMotor(
      int id, MotorAlignmentValue motorAlignmentValue) {
    this.followerMotors.add(new FollowerMotorConfig(id, motorAlignmentValue));
    return this;
  }

  /**
   * The upper voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> 16
   *   <li><b>Units:</b> V
   * </ul>
   */
  public double upperVoltLimit = 16;

  /**
   * The upper voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> 16
   *   <li><b>Units:</b> V
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withUpperVoltageLimit(double upperVoltLimit) {
    this.upperVoltLimit = upperVoltLimit;
    return this;
  }

  /**
   * The lower voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> -16
   *   <li><b>Units:</b> V
   * </ul>
   */
  public double lowerVoltLimit = 16;

  /**
   * The lower voltage limit for the motor.
   *
   * <ul>
   *   <li><b>Minimum Value:</b> -16
   *   <li><b>Maximum Value:</b> 16
   *   <li><b>Default Value:</b> -16
   *   <li><b>Units:</b> V
   * </ul>
   *
   * @return itself
   */
  public GenericMechanismConfiguration withLowerVoltageLimit(double lowerVoltLimit) {
    this.lowerVoltLimit = lowerVoltLimit;
    return this;
  }
}
