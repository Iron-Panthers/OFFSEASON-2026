package frc.robot.lib.generic_subsystems.superstructure;

import edu.wpi.first.math.filter.LinearFilter;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public abstract class GenericSuperstructure<G extends GenericSuperstructure.PositionTarget> {
  public interface PositionTarget {
    double getPosition(); // TODO: make this a consistant unit

    /**
     * Retrieves the tolerance value
     *
     * @return The tolerance value as a double.
     */
    double getEpsilon();

    double getSupplyCurrentLimit();
  }

  public enum ControlMode {
    POSITION,
    POSITION_MANUAL,
    ZEROING,
    STOP;
  }

  private ControlMode controlMode = ControlMode.STOP;

  protected final String name;

  private double totalAmps = 0;

  protected final GenericSuperstructureIO superstructureIO;

  protected Optional<Double> positionTargetManual = Optional.empty();
  protected double manualSupplyCurrentAmps = 0;

  private final LinearFilter linearFilter = LinearFilter.movingAverage(15);
  private double filteredCurrent;

  protected GenericSuperstructureIOInputsAutoLogged inputs =
      new GenericSuperstructureIOInputsAutoLogged();
  protected G positionTarget;

  public GenericSuperstructure(String name, GenericSuperstructureIO superstructureIO) {
    this.name = name;
    this.superstructureIO = superstructureIO;
  }

  public void periodic() {
    double filteredAmps = linearFilter.calculate(getSupplyCurrentAmps());

    totalAmps += (getSupplyCurrentAmps() / 50);

    // Process inputs
    superstructureIO.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    // Process control mode
    switch (controlMode) {
      case POSITION -> {
        superstructureIO.setSupplyCurrentLimit(positionTarget.getSupplyCurrentLimit());
        superstructureIO.runPosition(positionTarget.getPosition());
      }
      case POSITION_MANUAL -> {
        if (positionTargetManual.isPresent()) {
          superstructureIO.setSupplyCurrentLimit(manualSupplyCurrentAmps);
          superstructureIO.runPosition(positionTargetManual.get());
        }
      }
      case ZEROING -> {
        superstructureIO.runCharacterization();
      }
      case STOP -> {
        superstructureIO.stop();
      }
    }

    Logger.recordOutput(name + "/Target", positionTarget.toString());
    Logger.recordOutput(name + "/Control Mode", controlMode.toString());
    Logger.recordOutput(name + "/Reached Target", reachedTarget());
    Logger.recordOutput(name + "/Target Position", positionTarget.getPosition());
    Logger.recordOutput(name + "/Target Position Manual", positionTargetManual.orElse(0.0));
    filteredCurrent = this.linearFilter.calculate(inputs.supplyCurrentAmps);
    Logger.recordOutput(name + "/Filtered Current", filteredCurrent);
    Logger.recordOutput(name + "/Total Amp Seconds", totalAmps);
  }

  public G getPositionTarget() {
    return positionTarget;
  }

  public void setPositionTarget(G positionTarget) {
    setControlMode(ControlMode.POSITION);
    this.positionTarget = positionTarget;
  }

  public void setPositionTargetManual(double position, double supplyCurrentAmps) {
    setControlMode(ControlMode.POSITION_MANUAL);
    positionTargetManual = Optional.of(position);
    this.manualSupplyCurrentAmps = supplyCurrentAmps;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public void setControlMode(ControlMode controlMode) {
    if (controlMode == ControlMode.POSITION_MANUAL) {
      positionTargetManual = Optional.of(inputs.positionRotations);
    } else {
      positionTargetManual = Optional.empty();
    }
    this.controlMode = controlMode;
  }

  /** This is the zeroing function for the subsystem. */
  public void setOffset() {
    superstructureIO.setOffset();
  }

  public void endZeroing() {
    superstructureIO.setOffset();
    setControlMode(ControlMode.STOP);
  }

  public double getSupplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

  public double getPosition() {
    return inputs.positionRotations;
  }

  /**
   * This function returns whether or not the subsystem has reached its position target
   *
   * @return whether the subsystem has reached its position target
   */
  public boolean reachedTarget() {
    double targetPosition =
        switch (controlMode) {
          case POSITION -> positionTarget.getPosition();
          case POSITION_MANUAL -> positionTargetManual.orElse(0d);
          case STOP -> inputs.positionRotations;
          default -> 0;
        };
    return Math.abs(inputs.positionRotations - targetPosition) <= positionTarget.getEpsilon();
  }
}
