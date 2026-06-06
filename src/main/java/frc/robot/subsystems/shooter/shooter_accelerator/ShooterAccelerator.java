package frc.robot.subsystems.shooter.shooter_accelerator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterAccelerator
    extends GenericRollers<ShooterAccelerator.ShooterAcceleratorTarget> {
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
}
