package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;

public class IntakeRollers extends GenericRollers<IntakeRollers.IntakeRollersTarget> {

  // for logging the VelocityRadsPerSec and SupplyCurrentAmps every 5 seconds in test mode
  private final java.util.ArrayList<Double> speedIntervalReadings = new java.util.ArrayList<>();
  private final java.util.ArrayList<Double> supplyCurrentIntervalReadings = new java.util.ArrayList<>();

  public enum IntakeRollersTarget implements GenericRollers.VelocityTarget {
    INTAKE(60, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    INTAKE_SLOW(20, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    INTAKE_REALLY_SLOW(1, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    IDLE(0.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    INTAKE_DOWN(-1, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    EJECT(-20.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS), // TODO: CHANGE maxCurrentAmps
    HOLD(1.0, IntakeRollersConstants.CURRENT_LIMIT_AMPS); // TODO: CHANGE maxCurrentAmps

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

  public void resetTestState() {
    speedIntervalReadings.clear();
    supplyCurrentIntervalReadings.clear();
  }

  // Returns "avgRad/s,avgCurrent" at each 5s transition, null otherwise
  public String runTestLogger(double testStartTime, double currentTime, boolean isNextTransition) {
    if (testStartTime == -1) {
      testStartTime = currentTime;
    }

    double currentSpeed = inputs.velocityRadsPerSec;
    double currentSupplyCurrent = inputs.supplyCurrentAmps;
    speedIntervalReadings.add(currentSpeed);
    supplyCurrentIntervalReadings.add(currentSupplyCurrent);

    if (isNextTransition) {
      double sumSpeed = 0;
      double sumSupplyCurrent = 0;
      for (double val : speedIntervalReadings) {
        sumSpeed += val;
      }
      for (double val : supplyCurrentIntervalReadings) {
        sumSupplyCurrent += val;
      }

      double avgSpeed = speedIntervalReadings.isEmpty() ? 0 : sumSpeed / speedIntervalReadings.size();
      double avgSupplyCurrent = supplyCurrentIntervalReadings.isEmpty() ? 0 : sumSupplyCurrent / supplyCurrentIntervalReadings.size();

      speedIntervalReadings.clear();
      supplyCurrentIntervalReadings.clear();

      return String.format("%.2f", avgSpeed) + "," + String.format("%.2f", avgSupplyCurrent);
    }
    return null;
  }
}
