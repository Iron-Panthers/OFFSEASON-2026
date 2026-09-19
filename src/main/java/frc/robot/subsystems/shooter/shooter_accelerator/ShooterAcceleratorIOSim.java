package frc.robot.subsystems.shooter.shooter_accelerator;

import static frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class ShooterAcceleratorIOSim extends GenericRollersIOSim implements ShooterAcceleratorIO {
  public ShooterAcceleratorIOSim() {
    super(
        SHOOTER_ACCELERATOR_CONFIG.motorID1(),
        CURRENT_LIMIT_AMPS,
        SHOOTER_ACCELERATOR_CONFIG.inverted(),
        SHOOTER_ACCELERATOR_CONFIG.brake(),
        SHOOTER_ACCELERATOR_CONFIG.reduction(),
        new RollerSim(2, 2, PHYSICAL_CONSTANTS.momentOfInertia(), 0.0267));
    setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
