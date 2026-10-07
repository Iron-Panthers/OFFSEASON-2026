package frc.robot.subsystems.shooter.shooter_accelerator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAccelerator
    extends GenericRollers<ShooterAccelerator.ShooterAcceleratorTarget> {

  // for logging the VelocityRadsPerSec and SupplyCurrentAmps every 5 seconds in test mode
  private final java.util.ArrayList<Double> speedIntervalReadings = new java.util.ArrayList<>();
  private final java.util.ArrayList<Double> supplyCurrentIntervalReadings = new java.util.ArrayList<>();

  public enum ShooterAcceleratorTarget implements GenericRollers.VelocityTarget {
    IDLE(0, ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS),
    SHOOT(50.44, ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS),
    WARMUP_ACCELERATOR(60, ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS),
    PASS(70, ShooterAcceleratorConstants.CURRENT_LIMIT_AMPS);

    private double velocity;
    private double supplyCurrentLimit;

    private ShooterAcceleratorTarget(double velocity, double supplyCurrentLimit) {
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

  public ShooterAccelerator(ShooterAcceleratorIO io) {
    super("Shooter/Shooter Accelerator", io);
  }

  public AngularVelocity getCurrentVelocity() {
    return Units.RadiansPerSecond.of(inputs.velocityRadsPerSec);
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
