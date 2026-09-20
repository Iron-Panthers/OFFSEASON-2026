package frc.robot.subsystems.shooter.shooter_omniwheel;

import static frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheelConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class ShooterOmniwheelIOSim extends GenericRollersIOSim implements ShooterOmniwheelIO {
  // Drag left at its unloaded value: this plant's transients already exceed the real current.
  public ShooterOmniwheelIOSim() {
    super(
        SHOOTER_OMNIWHEEL_CONFIG.motorID(),
        SUPPLY_CURRENT_LIMIT_AMPS,
        SHOOTER_OMNIWHEEL_CONFIG.inverted(),
        SHOOTER_OMNIWHEEL_CONFIG.brake(),
        SHOOTER_OMNIWHEEL_CONFIG.reduction(),
        new RollerSim(1, 1, PHYSICAL_CONSTANTS.momentOfInertia(), 0.0552));
    setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
