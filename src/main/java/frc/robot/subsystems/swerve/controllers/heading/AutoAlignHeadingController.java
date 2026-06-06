package frc.robot.subsystems.swerve.controllers.heading;

import static frc.robot.subsystems.swerve.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.swerve.DriveConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoAlignHeadingController extends BaseHeadingController {

  public AutoAlignHeadingController(
      Supplier<Rotation2d> headingSupplier,
      Rotation2d targetHeading,
      double timeLeft,
      double rotationFinishPercent) {
    super(headingSupplier, targetHeading, HEADING_CONTROLLER_CONSTANTS);
    setTargetHeading(targetHeading, timeLeft, rotationFinishPercent);
  }

  @Override
  public void setTargetHeading(Rotation2d targetHeading) {
    setTargetHeading(targetHeading, 0, DriveConstants.ROTATION_FINISH_PERCENT);
  }

  public void setTargetHeading(Rotation2d targetHeading, double t, double rotationFinishPercent) {
    super.setTargetHeading(targetHeading);
    double a = HEADING_CONTROLLER_CONSTANTS.maxAcceleration();
    double v = HEADING_CONTROLLER_CONSTANTS.maxVelocity();
    t = rotationFinishPercent * t;
    double d = Math.abs(super.getHeadingSupplier().get().minus(targetHeading).getRadians());
    if (a != 0 && v != 0) {
      if ((t * t) - (4 / a) * d > 0) {
        v = (-t + Math.sqrt((t * t) - (4 / a) * d)) / (-2 / a);
      }
    } else {
      System.out.println("AutoAlignHeadingController: max velocity or acceleration is set to 0");
    }
    Logger.recordOutput("Swerve/PID Autoalign/Velocity Wanted", v);
    v = Math.min(v, HEADING_CONTROLLER_CONSTANTS.maxVelocity());
    super.getController().setConstraints(new Constraints(v, a));
  }
}
