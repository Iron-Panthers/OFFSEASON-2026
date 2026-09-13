package frc.robot.subsystems.shooter.serializer;

import static frc.robot.subsystems.shooter.serializer.SerializerConstants.*;

import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class SerializerSim extends GenericRollersIOSim {
  public SerializerSim() {
    super(
        SERIALIZER_CONFIG.motorID(),
        CURRENT_LIMIT_AMPS,
        SERIALIZER_CONFIG.inverted(),
        SERIALIZER_CONFIG.brake(),
        SERIALIZER_CONFIG.reduction(),
        new RollerSim(1, 2, PHYSICAL_CONSTANTS.momentOfIntertia(), 0.2237));
    setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
  }

  /** The serializer runs Phoenix's closed loop directly, matching SerializerIOTalonFX. */
  @Override
  protected boolean followsTalonClosedLoop() {
    return true;
  }
}
