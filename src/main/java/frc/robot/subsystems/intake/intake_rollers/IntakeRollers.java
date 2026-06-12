package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;

public class IntakeRollers extends GenericRollers<IntakeRollers.IntakeRollersTarget> {

  public enum IntakeRollersTarget implements GenericRollers.VelocityTarget {
    INTAKE(50, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
    INTAKE_SLOW(5, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
    INTAKE_REALLY_SLOW(1, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
    IDLE(0.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
    INTAKE_DOWN(-1, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
    EJECT(-20.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS),
    HOLD(1.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS);

    private double velocity;
    private double supplyCurrentLimit;

    private IntakeRollersTarget(double velocity, double supplyCurrentLimit) {
      this.velocity = velocity;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public double getVelocity() {
      return velocity;
    }

    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  public IntakeRollers(IntakeRollersIO intakeRollersIO) {
    super("Intake/Intake Rollers", intakeRollersIO);
    setVelocityTarget(IntakeRollersTarget.IDLE);
  }
}
