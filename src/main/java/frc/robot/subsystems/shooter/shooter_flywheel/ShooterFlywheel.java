package frc.robot.subsystems.shooter.shooter_flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers;

public class ShooterFlywheel extends GenericRollers<ShooterFlywheel.ShooterFlywheelTarget> {

  // for logging the VelocityRadsPerSec and SupplyCurrentAmps every 5 seconds in test mode
  private final java.util.ArrayList<String> loggedVelocitySupplyCurrent = new java.util.ArrayList<>();
  private double lastPrintTime = 0;
  private double testStartTime = -1; // -1 means test mode hasn't started yet
  private final java.util.ArrayList<Double> speedIntervalReadings = new java.util.ArrayList<>();
  private final java.util.ArrayList<Double> supplyCurrentIntervalReadings = new java.util.ArrayList<>();


  public enum ShooterFlywheelTarget implements GenericRollers.VelocityTarget {
    IDLE(0, ShooterFlywheelConstants.CURRENT_LIMIT_AMPS),
    INTAKE(8.5, ShooterFlywheelConstants.CURRENT_LIMIT_AMPS),
    SHOOT(RobotBase.isReal() ? 8.6 : 8.6, ShooterFlywheelConstants.CURRENT_LIMIT_AMPS),
    SPEEDY_SHOOT(9, ShooterFlywheelConstants.CURRENT_LIMIT_AMPS),
    PASS(9, ShooterFlywheelConstants.CURRENT_LIMIT_AMPS); // TODO: make this uniform

    private double velocity;
    private double supplyCurrentLimit;

    /** Input velocity in meters per second */
    private ShooterFlywheelTarget(double velocity, double supplyCurrentLimit) {
      this.velocity = velocity / ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters();
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    /** Velocity in rotations per second */
    public double getVelocity() {
      return velocity;
    }

    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  public ShooterFlywheel(ShooterFlywheelIO io) {
    super("Shooter/Shooter Flywheels", io);
  }

  // Making a periodic to log flywheel VelocityRadsPerSec
  @Override
  public void periodic() {
    super.periodic();

    if (edu.wpi.first.wpilibj.RobotState.isTest()) {
      runTestLogger();
    } else {
      testStartTime = -1; 
      lastPrintTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
  }

  public java.util.ArrayList<String> getLoggedData() {
    return loggedVelocitySupplyCurrent;
  }

  public double getTestModeStartTime(){
    return testStartTime;
  }
  public void clearLoggedData() {
    loggedVelocitySupplyCurrent.clear();
    speedIntervalReadings.clear();
    supplyCurrentIntervalReadings.clear();
  }

  @AutoLogOutput(key = "Shooter/Shooter Flywheels/Current Velocity")
  public LinearVelocity getCurrentVelocity() {
    return MetersPerSecond.of(
        Units.radiansToRotations(inputs.velocityRadsPerSec)
            * ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters());
  }

  /** Set flywheel to an arbitrary surface speed (m/s) from the LUT, bypassing the enum targets. */
  public void setVelocityManual(LinearVelocity velocity, double supplyCurrentAmps) {
    setVelocityTargetManual(
        ShooterFlywheelConstants.VELOCITY_ADJUSTMENT
            * velocity.in(MetersPerSecond)
            / ShooterFlywheelConstants.PHYSICAL_CONSTANTS.circumferenceMeters(),
        supplyCurrentAmps);
  }

  public boolean reachedVelocityTarget() {
    if (super.useManualVelocity) {
      return Math.abs(super.inputs.velocityRadsPerSec - Units.rotationsToRadians(manualVelocityRPS))
          < 40;
    } else {
      if (velocityTarget == null) return false;
      return Math.abs(
              super.inputs.velocityRadsPerSec - Units.rotationsToRadians(velocityTarget.velocity))
          < 40;
    }
  }

  public void runTestLogger(){
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

      if (testStartTime == -1) {
        testStartTime = currentTime;
      }

      double relativeTime = currentTime - testStartTime;

      // If it's been more than 25 seconds
      if (relativeTime >= 25.0) {
        return; // Exits the method
      }
      double currentSpeed = inputs.velocityRadsPerSec;
      double currentSupplyCurrent = inputs.supplyCurrentAmps;
      speedIntervalReadings.add(currentSpeed);
      supplyCurrentIntervalReadings.add(currentSupplyCurrent);

      if (currentTime - lastPrintTime >= 5.0) {
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

        loggedVelocitySupplyCurrent.add(String.format("%.1f", relativeTime) + "," + String.format("%.2f", avgSpeed) + "," + String.format("%.2f", avgSupplyCurrent));
        speedIntervalReadings.clear();
        supplyCurrentIntervalReadings.clear();
        lastPrintTime = currentTime;
      }
  }
}
