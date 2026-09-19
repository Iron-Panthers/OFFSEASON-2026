package frc.robot.subsystems.shooter.shooter_flywheel;

import static frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class ShooterFlywheelIOSim extends GenericRollersIOSim implements ShooterFlywheelIO {
  public ShooterFlywheelIOSim() {
    super(
        SHOOTER_FLYWHEEL_CONFIG.motorID1(),
        CURRENT_LIMIT_AMPS,
        SHOOTER_FLYWHEEL_CONFIG.inverted(),
        SHOOTER_FLYWHEEL_CONFIG.brake(),
        SHOOTER_FLYWHEEL_CONFIG.reduction(),
        new RollerSim(4, 4, PHYSICAL_CONSTANTS.momentOfInertia(), 0.0341));
    setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }
}
