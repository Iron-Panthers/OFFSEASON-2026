package frc.robot.subsystems.shooter.serializer;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;
import org.littletonrobotics.junction.AutoLogOutput;

public class Serializer extends GenericRollers<Serializer.SerializerTarget> {
  // for logging the VelocityRadsPerSec and SupplyCurrentAmps every 5 seconds in test mode
  private final java.util.ArrayList<String> loggedVelocitySupplyCurrent = new java.util.ArrayList<>();
  private final java.util.ArrayList<Double> speedIntervalReadings = new java.util.ArrayList<>();
  private final java.util.ArrayList<Double> supplyCurrentIntervalReadings = new java.util.ArrayList<>();

  public enum SerializerTarget implements GenericRollers.VelocityTarget {
    IDLE(0, SerializerConstants.CURRENT_LIMIT_AMPS),
    SLOW(-20, SerializerConstants.CURRENT_LIMIT_AMPS),
    REVERSE(40, SerializerConstants.CURRENT_LIMIT_AMPS),
    SPIN_UP(-10, SerializerConstants.CURRENT_LIMIT_AMPS),
    SHOOT(-40, SerializerConstants.CURRENT_LIMIT_AMPS),
    HOLD(-1, SerializerConstants.CURRENT_LIMIT_AMPS);

    private double velocity;
    private double supplyCurrentLimit;

    private SerializerTarget(double velocity, double supplyCurrentLimit) {
      this.velocity = velocity;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    @Override
    public double getVelocity() {
      return velocity;
    }

    @Override
    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  }

  public Serializer(GenericRollersIO IntakeRollersIO) {
    super("Serializer", IntakeRollersIO);
  }

  public double getVelocityRadsPerSec() {
    return inputs.velocityRadsPerSec;
  }

  /**
   * Returns true when the serializer is applying amps but not going anywhere
   *
   * @return
   */
  @AutoLogOutput(key = "Serializer/Serializer Stalling")
  public boolean serializerStalling() {
    return getFilteredCurrent() > 15d && getVelocityRadsPerSec() < 3d;
  }

  public java.util.ArrayList<String> getLoggedData() {
    return loggedVelocitySupplyCurrent;
  }
  public void clearLoggedData() {
    loggedVelocitySupplyCurrent.clear();
    speedIntervalReadings.clear();
    supplyCurrentIntervalReadings.clear();
  }

  public void runTestLogger(double testStartTime, double currentTime, boolean isNextTransition){
    

      if (testStartTime == -1) {
        testStartTime = currentTime;
      }

      double relativeTime = currentTime - testStartTime;

      
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

        loggedVelocitySupplyCurrent.add(String.format("%.1f", relativeTime) + "," + String.format("%.2f", avgSpeed) + "," + String.format("%.2f", avgSupplyCurrent));
        speedIntervalReadings.clear();
        supplyCurrentIntervalReadings.clear();
       
      }
  }
}
