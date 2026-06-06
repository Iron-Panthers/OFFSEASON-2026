package frc.robot.subsystems.swerve.controllers.translation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Supplier;

public abstract class BaseTranslationController {
  protected final Supplier<Rotation2d> yawSupplier;

  /* teleop control with specified yaw supplier, typically "arbitrary" yaw */
  public BaseTranslationController(Supplier<Rotation2d> yawSupplier) {
    this.yawSupplier = yawSupplier;
  }

  public abstract ChassisSpeeds update();
}
