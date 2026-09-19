package frc.robot.subsystems.shooter.shooter_hood;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.utility.LoggableMechanism3d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterHood extends GenericSuperstructure<ShooterHood.ShooterHoodTarget>
    implements LoggableMechanism3d {
  public enum ShooterHoodTarget implements GenericSuperstructure.PositionTarget {
    STOW(0, ShooterHoodConstants.SUPPLY_CURRENT_LIMIT), // need to update
    SHOOT_TEMP(12, ShooterHoodConstants.SUPPLY_CURRENT_LIMIT), // need to update
    PASS(32, ShooterHoodConstants.SUPPLY_CURRENT_LIMIT),
    DEFAULT_SHOOT(14, ShooterHoodConstants.SUPPLY_CURRENT_LIMIT); // might need to update?

    private double position; // in rotations
    private double supplyCurrentLimit;
    private static final double EPSILON = ShooterHoodConstants.POSITION_TARGET_EPSILON;

    /**
     * @param positionDeg in degrees
     */
    private ShooterHoodTarget(double positionDeg, double supplyCurrentLimit) {
      this.position = positionDeg / 360d;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    public double getPosition() {
      return position;
    }

    @Override
    public double getEpsilon() {
      return EPSILON;
    }

    public double getSupplyCurrentLimit() {
      return supplyCurrentLimit;
    }
  } // close enum

  public ShooterHood(ShooterHoodIO io) {
    super("Shooter/Shooter Hood", io);
    setPositionTarget(ShooterHoodTarget.STOW);
    setControlMode(ControlMode.STOP);
  }

  public LoggableMechanism3d loggableMechanism3dParent = null;

  @Override
  public void periodic() {
    super.periodic();
    if (super.getPositionTarget() == ShooterHoodTarget.STOW && reachedTarget()) {
      super.setControlMode(ControlMode.STOP);
    }
    Logger.recordOutput(
        "Shooter/Shooter Hood/Position Target Rotations", // TODO: add naming convention to notion
        // doc
        getPositionTarget().getPosition());
  }

  @Override
  public Pose3d getParentPosition() {
    if (loggableMechanism3dParent != null) {
      return loggableMechanism3dParent.getDisplayPose3d();
    }
    return new Pose3d();
  }

  @Override
  public void setParent(LoggableMechanism3d parent) {
    if (parent == null) {
      throw new IllegalArgumentException("Parent cannot be null");
    }
    if (parent == this) {
      throw new IllegalArgumentException("Parent cannot be itself");
    }
    this.loggableMechanism3dParent = parent;
  }

  // TODO make sure logic is correct for getting Display Pose3D
  @AutoLogOutput(key = "Shooter/Shooter Hood/Display Pose3d")
  @Override
  public Pose3d getDisplayPose3d() {
    return getParentPosition()
        .plus(ShooterHoodConstants.BASE_TO_SHOOTER_HOOD_TRANSFORM)
        .plus(
            new Transform3d(
                Translation3d.kZero, new Rotation3d(Math.toRadians(getPosition() * 360), 0, 0)));
  }

  public boolean reachedPositionTargetManual() {

    return Math.abs(
            super.inputs.positionRotations
                - (positionTargetManual.isPresent()
                    ? positionTargetManual.get()
                    : positionTarget.position))
        < ShooterHoodConstants.POSITION_TARGET_EPSILON;
  }
} // close class
