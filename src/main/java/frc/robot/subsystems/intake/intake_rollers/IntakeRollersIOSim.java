package frc.robot.subsystems.intake.intake_rollers;

import static frc.robot.subsystems.intake.intake_rollers.IntakeRollersConstants.*;

import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class IntakeRollersIOSim extends GenericRollersIOSim implements IntakeRollersIO {
  public IntakeRollersIOSim() {
    super(
        INTAKE_ROLLER_CONFIG.motorID(),
        CURRENT_LIMIT_AMPS,
        INTAKE_ROLLER_CONFIG.inverted(),
        INTAKE_ROLLER_CONFIG.brake(),
        INTAKE_ROLLER_CONFIG.reduction(),
        new RollerSim(1, 2, PHYSICAL_CONSTANTS.momentOfInertia(), 0.1068));
    setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    talon.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }
}
