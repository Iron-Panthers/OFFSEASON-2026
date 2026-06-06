package frc.robot.subsystems.swerve.controllers.translation;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.swerve.DriveConstants.AUTOALIGN_POSITION_DEADBAND;
import static frc.robot.subsystems.swerve.DriveConstants.AUTOALIGN_VELOCITY_DEADBAND;
import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AxisAssist extends BaseTranslationController {

  // supplies the position values
  private ProfiledPIDController magController;
  private Supplier<Double> positionSupplier;
  private Supplier<Rotation2d> headingSupplier;

  // target position
  private double targetPosition;
  private double startPosition;
  private Distance pidAxisVel;
  private final Supplier<Distance> pidAxisVelocity;
  protected boolean hasReachedTarget = false;

  private double acceleration;
  private boolean controlY;
  private double controllerX;
  private double controllerY;

  public AxisAssist(
      Supplier<Pose2d> positionSupplier,
      Supplier<Rotation2d> yawSupplier,
      double targetPosition,
      boolean controlY) {
    super(yawSupplier);
    this.positionSupplier =
        () -> controlY ? positionSupplier.get().getX() : positionSupplier.get().getY();
    headingSupplier = () -> positionSupplier.get().getRotation();
    this.targetPosition = targetPosition;
    this.controlY = controlY;
    this.pidAxisVelocity =
        () ->
            controlY
                ? RobotState.getInstance().getVelocity().getMeasureX()
                : RobotState.getInstance().getVelocity().getMeasureY();
    pidAxisVel = pidAxisVelocity.get();

    // setting up the ProfiledPIDController
    magController =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            PID_AUTOALIGN_CONSTANTS.kI(),
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
    setTargetPosition(targetPosition);
    magController.disableContinuousInput();
    magController.setTolerance(0, 0);
  }

  /* accept driver input from joysticks */
  public void acceptJoystickInput(double controllerX, double controllerY, double acceleration) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.acceleration = acceleration;
  }

  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    double currToTargDx = positionSupplier.get() - targetPosition;

    double startToTargDx = startPosition - targetPosition;

    double startToCurrDx = startPosition - positionSupplier.get();

    // the naming is very important
    double magTranslCurrPos = startToCurrDx;
    double magTranslTargPos = startToTargDx;
    // can change to simpler varaibles above, and the problem being we use magnitude, so we combine
    // x and y, but we have to pslit them at a larger level
    double pidOutput = magController.calculate(magTranslCurrPos, magTranslTargPos);
    double magVel = pidOutput + magController.getSetpoint().velocity;
    magVel = (Math.abs(magVel) < AUTOALIGN_VELOCITY_DEADBAND ? 0 : magVel);

    pidAxisVel = Units.Meters.of(magVel);
    if (Math.abs(positionSupplier.get() - targetPosition) < AUTOALIGN_POSITION_DEADBAND) {
      pidAxisVel = Units.Meters.of(0);
    }

    Logger.recordOutput("Swerve/Axis Assist/SetpointPos", magController.getSetpoint().position);
    Logger.recordOutput("Swerve/Axis Assist/CurrPos", magTranslCurrPos);
    Logger.recordOutput("Swerve/Axis Assist/TargPos", magTranslTargPos);
    Logger.recordOutput("Swerve/Axis Assist/MagVel", magVel);
    Logger.recordOutput("Swerve/Axis Assist/Target", targetPosition);
    Logger.recordOutput("Swerve/Axis Assist/TrapVel", magController.getSetpoint().velocity);
    Logger.recordOutput("Swerve/Axis Assist/PIDVel", pidOutput);
  }

  public Distance calculateLinearVelocity(double y) {
    double magnitude = MathUtil.applyDeadband(Math.abs(y), 0.1);
    magnitude = Math.pow(magnitude, 1.5) * 3;
    if (RobotState.isAllianceRed()) {
      if (y < 0) {
        magnitude = magnitude * -1;
      }
    } else {
      if (y > 0) {
        magnitude = magnitude * -1;
      }
    }
    return Units.Meters.of(magnitude);
  }

  // update the values
  public ChassisSpeeds update() {
    calculateLinearMovement();
    Distance controlAxisVel = calculateLinearVelocity(controlY ? controllerY : controllerX);
    Logger.recordOutput("Swerve/Axis Assist/ControlAxisVel", controlAxisVel);
    Logger.recordOutput("Swerve/Axis Assist/PidAxisVel", pidAxisVel);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        controlY ? pidAxisVel.in(Units.Meters) : controlAxisVel.in(Units.Meters),
        controlY ? controlAxisVel.in(Units.Meters) : pidAxisVel.in(Units.Meters),
        0,
        headingSupplier.get().plus(Rotation2d.k180deg));
  }

  // log your data in advantage kit
  public double getTargetPosition() {
    return targetPosition;
  }

  public Distance getXVel() {
    return Units.Meters.of(0).minus(controlY ? pidAxisVel : Meters.of(0));
  }

  public Distance getYVel() {
    return Units.Meters.of(0).minus(!controlY ? pidAxisVel : Meters.of(0));
  }

  // public Distance getYVel() {
  //   return Units.Meters.of(0).minus(yVel);
  // }

  public void setTargetPosition(double targetPosition) {
    startPosition = positionSupplier.get();
    this.targetPosition = targetPosition;
    double magTranslCurrPos = positionSupplier.get() - startPosition;
    double magTanslTargPos = targetPosition - startPosition;
    magController.setGoal(magTanslTargPos);
    magController.reset(magTranslCurrPos, calculateForwardVelocity().in(Units.Meters));
  }

  public Distance calculateForwardVelocity() {
    return pidAxisVel;
  }

  public boolean atTarget() {
    return hasReachedTarget =
        Math.abs(positionSupplier.get() - targetPosition)
            < PID_AUTOALIGN_CONSTANTS.tolerance() * (hasReachedTarget ? 4 : 1);
  }

  private double getMaxLinearVelocity() {
    return DRIVE_CONFIG.maxLinearVelocity();
  }
}
