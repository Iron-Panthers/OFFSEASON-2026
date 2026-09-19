package frc.robot.subsystems.shooter.serializer;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;
import org.littletonrobotics.junction.AutoLogOutput;

public class Serializer extends GenericRollers<Serializer.SerializerTarget> {
  public enum SerializerTarget implements GenericRollers.VelocityTarget {
    IDLE(0, SerializerConstants.CURRENT_LIMIT_AMPS),
    SLOW(-20, SerializerConstants.CURRENT_LIMIT_AMPS),
    REVERSE(40, SerializerConstants.CURRENT_LIMIT_AMPS),
    SPIN_UP(-10, SerializerConstants.CURRENT_LIMIT_AMPS),
    SHOOT(-40, SerializerConstants.CURRENT_LIMIT_AMPS),
    MAX_SPEED(-80, SerializerConstants.CURRENT_LIMIT_AMPS),
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
   * Overrides the supply current limit the serializer runs at, ignoring the limit carried by {@link
   * SerializerTarget}. Stays in effect through target changes until {@link #setDefaultMaxAmps()} is
   * called.
   *
   * @param amps supply current limit in amps
   */
  public void setMaxAmps(double amps) {
    setSupplyCurrentLimitOverride(amps);
  }

  /** Returns the serializer to the supply current limit carried by its current target. */
  public void setDefaultMaxAmps() {
    clearSupplyCurrentLimitOverride();
  }

  /**
   * The supply current limit the serializer is actually running at this loop.
   *
   * @return supply current limit in amps
   */
  @AutoLogOutput(key = "Serializer/Max Amps")
  public double getMaxAmps() {
    return getSupplyCurrentLimitAmps();
  }

  @AutoLogOutput(key = "Serializer/Max Amps Overridden")
  public boolean isMaxAmpsOverridden() {
    return isSupplyCurrentLimitOverridden();
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
}
