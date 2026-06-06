package frc.robot.subsystems.shooter.shooter_hood;

import static frc.robot.subsystems.shooter.shooter_hood.ShooterHoodConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;

public class ShooterHoodIOTalonFX extends GenericSuperstructureIOTalonFX implements ShooterHoodIO {

  public ShooterHoodIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(SHOOTER_HOOD_CONFIG.motorID())
            .withMotorDirection(MOTOR_DIRECTION)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(SHOOTER_HOOD_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET)
            .withSensorDiscontinuityPoint(SENSOR_DISCONTINUITY_POINT));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.accelerations(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }
}
