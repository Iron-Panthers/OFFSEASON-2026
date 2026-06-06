package frc.robot.subsystems.shooter.shooter_flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.lib.generic_subsystems.rollers.*;
import org.littletonrobotics.junction.AutoLogOutput;

public class ShooterFlywheel extends GenericRollers<ShooterFlywheel.ShooterFlywheelTarget> {
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
      return Math.abs(
              super.inputs.velocityRadsPerSec - Units.rotationsToRadians(velocityTarget.velocity))
          < 40;
    }
  }
}
