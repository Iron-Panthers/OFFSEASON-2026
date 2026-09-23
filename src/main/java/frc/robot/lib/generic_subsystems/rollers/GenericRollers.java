package frc.robot.lib.generic_subsystems.rollers;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants;
import org.littletonrobotics.junction.Logger;

public abstract class GenericRollers<G extends GenericRollers.VelocityTarget> {
  public interface VelocityTarget {
    double getVelocity();

    double getSupplyCurrentLimit();
  }

  public enum ControlMode {
    VELOCITY,
    STOP
  }

  private ControlMode controlMode = ControlMode.STOP;

  private LinearFilter filter;
  private double filteredCurrent;
  private double totalAmps = 0;

  private final String name;
  private final GenericRollersIO rollerIO;
  protected GenericRollersIOInputsAutoLogged inputs = new GenericRollersIOInputsAutoLogged();

  protected G velocityTarget;
  protected double manualVelocityRPS = 0;
  protected double manualSupplyCurrentAmps = 0;
  protected boolean useManualVelocity = false;

  private double supplyCurrentLimitOverrideAmps = 0;
  private boolean useSupplyCurrentLimitOverride = false;

  public GenericRollers(String name, GenericRollersIO rollerIO) {
    this.name = name;
    this.rollerIO = rollerIO;
    this.filter = LinearFilter.movingAverage(100);
  }

  public void periodic() {
    rollerIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    Logger.recordOutput(
        name + "/Manual Target",
        manualVelocityRPS * ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters());
    Logger.recordOutput(name + "/Target", velocityTarget.toString());
    Logger.recordOutput(name + "/Target Velocity", velocityTarget.getVelocity());
    Logger.recordOutput(name + "/Max Current Amps", velocityTarget.getSupplyCurrentLimit());
    Logger.recordOutput(name + "/Supply Current Limit Applied", getSupplyCurrentLimitAmps());
    Logger.recordOutput(name + "/Supply Current Limit Overridden", useSupplyCurrentLimitOverride);

    filteredCurrent = this.filter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput(name + "/Filtered Current", filteredCurrent);

    totalAmps += (getSupplyCurrentAmps() / 50);
    Logger.recordOutput(name + "/Total Amp Seconds", totalAmps);

    Logger.recordOutput(name + "/Control Mode", controlMode.toString());
    switch (controlMode) {
      case VELOCITY -> {
        rollerIO.setSupplyCurrentLimit(getSupplyCurrentLimitAmps());
        rollerIO.runVelocity(useManualVelocity ? manualVelocityRPS : velocityTarget.getVelocity());
      }
      case STOP -> {
        rollerIO.stop();
      }
    }
  }

  public G getVelocityTarget() {
    return velocityTarget;
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public void setVelocityTarget(G velocityTarget) {
    setControlMode(velocityTarget.getVelocity() == 0 ? ControlMode.STOP : ControlMode.VELOCITY);
    this.velocityTarget = velocityTarget;
    this.useManualVelocity = false;
  }

  public void setVelocityTargetManual(double velocityRPS, double supplyCurrentAmps) {
    // Run setamps and put that as parameter and where you call the method(intakerollers), get the
    // number of amps from the enum
    setControlMode(ControlMode.VELOCITY);
    this.manualVelocityRPS = velocityRPS;
    this.manualSupplyCurrentAmps = supplyCurrentAmps;
    this.useManualVelocity = true;
  }

  /**
   * The supply current limit actually pushed to the motor this loop. An override, when set, wins
   * over both the manual limit and the current target's limit.
   *
   * @return supply current limit in amps
   */
  public double getSupplyCurrentLimitAmps() {
    if (useSupplyCurrentLimitOverride) {
      return supplyCurrentLimitOverrideAmps;
    }
    return useManualVelocity ? manualSupplyCurrentAmps : velocityTarget.getSupplyCurrentLimit();
  }

  /**
   * Overrides the supply current limit for every target until {@link
   * #clearSupplyCurrentLimitOverride()} is called. Survives {@link #setVelocityTarget}, so callers
   * that re-set their target every loop keep the override.
   *
   * @param amps supply current limit in amps
   */
  protected void setSupplyCurrentLimitOverride(double amps) {
    this.supplyCurrentLimitOverrideAmps = amps;
    this.useSupplyCurrentLimitOverride = true;
  }

  /** Drops the override and goes back to the limit carried by the target enum. */
  protected void clearSupplyCurrentLimitOverride() {
    this.useSupplyCurrentLimitOverride = false;
  }

  protected boolean isSupplyCurrentLimitOverridden() {
    return useSupplyCurrentLimitOverride;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    this.controlMode = controlMode;
  }
}
