package frc.robot.subsystems.intake.intake_rack;

import static frc.robot.subsystems.intake.intake_rack.IntakeRackConstants.*;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureConfiguration;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOTalonFX;

public class IntakeRackIOTalonFX extends GenericSuperstructureIOTalonFX implements IntakeRackIO {

  public IntakeRackIOTalonFX() {
    super(
        new GenericSuperstructureConfiguration()
            .withID(INTAKE_RACK_CONFIG.motorID())
            .withMotorDirection(INTAKE_RACK_CONFIG.motorDirection())
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withReduction(INTAKE_RACK_CONFIG.reduction())
            .withUpperVoltageLimit(UPPER_VOLT_LIMIT)
            .withLowerVoltageLimit(LOWER_VOLT_LIMIT)
            .withZeroingVolts(ZEROING_VOLTS)
            .withZeroingOffset(ZEROING_OFFSET));

    setSlot0(
        GAINS.kP(),
        GAINS.kI(),
        GAINS.kD(),
        GAINS.kS(),
        GAINS.kV(),
        GAINS.kA(),
        GAINS.kG(),
        MOTION_MAGIC_CONFIG.acceleration(),
        MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        GRAVITY_TYPE);
  }
}
