package frc.robot.subsystems.shooter.shooter_omniwheel;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.generic_subsystems.rollers.*;

public class ShooterOmniwheel extends GenericRollers<ShooterOmniwheel.ShooterOmniwheelTarget> {
  public enum ShooterOmniwheelTarget implements GenericRollers.VelocityTarget {
    IDLE(-1, ShooterOmniwheelConstants.CURRENT_LIMIT_AMPS),
    SLOW_REVERSE(-1, ShooterOmniwheelConstants.CURRENT_LIMIT_AMPS),
    SHOOT(90, ShooterOmniwheelConstants.CURRENT_LIMIT_AMPS);

    private double velocity;
    private double supplyCurrentLimit;

    private ShooterOmniwheelTarget(double velocity, double supplyCurrentLimit) {
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

  public ShooterOmniwheel(ShooterOmniwheelIO io) {
    super("Shooter/Shooter Omniwheel", io);
  }

  public AngularVelocity getCurrentVelocity() {
    return Units.RadiansPerSecond.of(inputs.velocityRadsPerSec);
  }
}
