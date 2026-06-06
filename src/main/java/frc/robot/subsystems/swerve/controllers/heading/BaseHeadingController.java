package frc.robot.subsystems.swerve.controllers.heading;

import static frc.robot.subsystems.swerve.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DriveConstants.HeadingControllerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public abstract class BaseHeadingController {
  // the PID controller
  protected ProfiledPIDController controller;
  protected Supplier<Rotation2d> headingSupplier;
  protected Rotation2d targetHeading;
  protected boolean hasReachedTarget = false;

  public BaseHeadingController(
      Supplier<Rotation2d> headingSupplier,
      Rotation2d targetHeading,
      HeadingControllerConstants headingControllerConstants) {
    this.headingSupplier = headingSupplier;
    this.targetHeading = targetHeading;

    // setting the following controller
    controller =
        new ProfiledPIDController(
            headingControllerConstants.kP(),
            0,
            headingControllerConstants.kD(),
            new Constraints(
                headingControllerConstants.maxVelocity(),
                headingControllerConstants.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);

    controller.setTolerance(Units.degreesToRadians(headingControllerConstants.tolerance()));
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.reset(headingSupplier.get().getRadians());
  }

  /**
   * Called every 20 milliseconds to calculate the output Omega Radians Per Second
   *
   * @return omega radians per second of the heading controller
   */
  public double update() {
    double pidOutput =
        controller.calculate(headingSupplier.get().getRadians(), targetHeading.getRadians());
    double output = pidOutput + controller.getSetpoint().velocity;
    Logger.recordOutput("Swerve/Heading Controller/PID Output", pidOutput);
    Logger.recordOutput(
        "Swerve/Heading Controller/Setpoint Velocity", controller.getSetpoint().velocity);
    Logger.recordOutput("Swerve/Heading Controller/Output", output);
    Logger.recordOutput(
        "Swerve/Heading Controller/Setpoint Position", controller.getSetpoint().position);
    Logger.recordOutput(
        "Swerve/Heading Controller/Current Position", headingSupplier.get().getRadians());
    Logger.recordOutput("Swerve/Heading Controller/At Target", atTarget());
    if (atTarget()) {
      return 0;
    }
    return output; // To prevent jittering
  }

  public boolean atTarget() {
    return hasReachedTarget =
        epsilonEquals(
            headingSupplier.get().getRadians(),
            controller.getGoal().position,
            HEADING_CONTROLLER_CONSTANTS.tolerance() * (hasReachedTarget ? 4 : 1));
  }

  protected boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  /**
   * Setting the target heading
   *
   * @param targetHeading
   */
  public void setTargetHeading(Rotation2d targetHeading) {
    this.targetHeading = targetHeading;
  }

  // -- Getter methods --

  /**
   * @return Target heading of the controller
   */
  public Rotation2d getTargetHeading() {
    return targetHeading;
  }

  /**
   * @return Profiled PID controller
   */
  protected ProfiledPIDController getController() {
    return controller;
  }

  protected Supplier<Rotation2d> getHeadingSupplier() {
    return headingSupplier;
  }

  // TUrn compare heading into get Error (setpoint vs current ^^^)
  // Pass headingController into robotContainer into Drive to pass lambda
  // lambda should call getError
  // pass lambda into commandFactory for shootCommandFactory
  public Rotation2d getError() {
    Rotation2d error =
        Rotation2d.fromDegrees(
            (Math.abs(targetHeading.getDegrees() - headingSupplier.get().getDegrees()) + 360)
                % 360);
    Logger.recordOutput("Swerve/Heading Controller/Heading Error", error);
    Logger.recordOutput(
        "Swerve/Heading Controller/Absolute Error",
        error.getDegrees() > 180 ? error.getDegrees() - 360 : error.getDegrees());
    return error;
  }
}
