package frc.robot.subsystems.swerve.controllers.translation;

import static frc.robot.subsystems.swerve.DriveConstants.AUTOALIGN_POSITION_DEADBAND;
import static frc.robot.subsystems.swerve.DriveConstants.AUTOALIGN_VELOCITY_DEADBAND;
import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PIDAutoAlignController extends BaseTranslationController {

  // supplies the position values
  private ProfiledPIDController magController;
  private Supplier<Pose2d> positionSupplier;

  // target position
  private Pose2d targetPosition;
  private Pose2d startPosition;
  private double xVel;
  private double yVel;
  private final Supplier<Translation2d> velocity;
  protected boolean hasReachedTarget = false;

  public PIDAutoAlignController(
      Supplier<Pose2d> positionSupplier, Supplier<Rotation2d> yawSupplier, Pose2d targetPosition) {
    super(yawSupplier);
    this.positionSupplier = positionSupplier;
    this.targetPosition = targetPosition;
    this.velocity = () -> RobotState.getInstance().getVelocity();

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

  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    double currToTargDy = positionSupplier.get().getY() - targetPosition.getY();
    double currToTargDx = positionSupplier.get().getX() - targetPosition.getX();
    Rotation2d currToTargAngle = new Rotation2d(Math.atan2(currToTargDy, currToTargDx));

    double startToTargDy = startPosition.getY() - targetPosition.getY();
    double startToTargDx = startPosition.getX() - targetPosition.getX();
    Rotation2d startToTargAngle = new Rotation2d(Math.atan2(startToTargDy, startToTargDx));

    double startToCurrDy = startPosition.getY() - positionSupplier.get().getY();
    double startToCurrDx = startPosition.getX() - positionSupplier.get().getX();
    // probably could have calculated this with triangle stuff... welp
    Rotation2d startToCurrAngle = new Rotation2d(Math.atan2(startToCurrDy, startToCurrDx));

    // the naming is very important
    double magTranslCurrPos =
        Math.hypot(startToCurrDx, startToCurrDy)
            * (Math.abs(startToTargAngle.minus(startToCurrAngle).getRadians()) > Math.PI / 2
                ? -1
                : 1);
    double magTranslTargPos = Math.hypot(startToTargDx, startToTargDy);
    // can change to simpler varaibles above, and the problem being we use magnitude, so we combine
    // x and y, but we have to pslit them at a larger level
    double pidOutput = magController.calculate(magTranslCurrPos, magTranslTargPos);
    double magVel = pidOutput + magController.getSetpoint().velocity;
    magVel = (Math.abs(magVel) < AUTOALIGN_VELOCITY_DEADBAND ? 0 : magVel);
    yVel =
        magVel
            * currToTargAngle.getSin()
            * (Math.abs(currToTargAngle.minus(startToTargAngle).getRadians()) > Math.PI / 2
                ? 1
                : -1);
    xVel =
        magVel
            * currToTargAngle.getCos()
            * (Math.abs(currToTargAngle.minus(startToTargAngle).getRadians()) > Math.PI / 2
                ? 1
                : -1);
    if (positionSupplier.get().getTranslation().getDistance(targetPosition.getTranslation())
        < AUTOALIGN_POSITION_DEADBAND) {
      xVel = 0;
      yVel = 0;
    }

    Logger.recordOutput("Swerve/PID Autoalign/Angle", currToTargAngle);
    Logger.recordOutput("Swerve/PID Autoalign/Origin Angle", startToTargAngle);
    Logger.recordOutput("Swerve/PID Autoalign/SetpointPos", magController.getSetpoint().position);
    Logger.recordOutput("Swerve/PID Autoalign/CurrPos", magTranslCurrPos);
    Logger.recordOutput("Swerve/PID Autoalign/TargPos", magTranslTargPos);
    Logger.recordOutput("Swerve/PID Autoalign/MagVel", magVel);
    Logger.recordOutput("Swerve/PID Autoalign/Target", targetPosition);
    Logger.recordOutput("Swerve/PID Autoalign/TrapVel", magController.getSetpoint().velocity);
    Logger.recordOutput("Swerve/PID Autoalign/PIDVel", pidOutput);
  }

  public double calculateTimeLeft() {
    double d = startPosition.getTranslation().getDistance(targetPosition.getTranslation());
    TrapezoidProfile trapezoidProfile =
        new TrapezoidProfile(
            new Constraints(
                magController.getConstraints().maxVelocity,
                magController.getConstraints().maxAcceleration));
    trapezoidProfile.calculate(0, new State(d, -calculateForwardVelocity()), new State(0, 0));
    double totalTime = trapezoidProfile.totalTime();
    double timeLeft =
        totalTime
            * (positionSupplier.get().getTranslation().getDistance(targetPosition.getTranslation())
                / d);
    Logger.recordOutput("Swerve/PID Autoalign/Time Left", totalTime);
    return timeLeft;
  }

  // update the values
  public ChassisSpeeds update() {
    calculateLinearMovement();
    Logger.recordOutput("Swerve/PID Autoalign/XVel", xVel);
    Logger.recordOutput("Swerve/PID Autoalign/YVel", yVel);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        -xVel, -yVel, 0, positionSupplier.get().getRotation().plus(Rotation2d.k180deg));
  }

  // log your data in advantage kit
  public Pose2d getTargetPosition() {
    return targetPosition;
  }

  public double getXVel() {
    return -xVel;
  }

  public double getYVel() {
    return -yVel;
  }

  public void setTargetPosition(Pose2d targetPosition) {
    startPosition = positionSupplier.get();
    this.targetPosition = targetPosition;
    double magTranslCurrPos =
        Math.hypot(
            positionSupplier.get().getX() - startPosition.getX(),
            positionSupplier.get().getY() - startPosition.getY());
    double magTanslTargPos =
        Math.hypot(
            targetPosition.getX() - startPosition.getX(),
            targetPosition.getY() - startPosition.getY());
    magController.setGoal(magTanslTargPos);
    magController.reset(magTranslCurrPos, calculateForwardVelocity());
  }

  public double calculateForwardVelocity() {
    Translation2d vel = velocity.get();
    double x = vel.getX();
    double y = vel.getY();
    Pose2d relativeTargetPosition =
        new Pose2d(
            positionSupplier.get().getX() - targetPosition.getX(),
            positionSupplier.get().getY() - targetPosition.getY(),
            new Rotation2d());
    Rotation2d targetAngle =
        new Rotation2d(Math.atan2(relativeTargetPosition.getY(), relativeTargetPosition.getX()));
    Rotation2d currentVelAngle = new Rotation2d(Math.atan2(y, x));
    Rotation2d angleDiff = targetAngle.minus(currentVelAngle);
    double forwardVelocity = Math.cos(angleDiff.getRadians()) * vel.getNorm();
    return forwardVelocity;
  }

  public boolean atTarget() {
    return hasReachedTarget =
        positionSupplier.get().getTranslation().getDistance(targetPosition.getTranslation())
            < PID_AUTOALIGN_CONSTANTS.tolerance() * (hasReachedTarget ? 4 : 1);
  }

  public boolean almostAtTarget() {
    return hasReachedTarget =
        positionSupplier.get().getTranslation().getDistance(targetPosition.getTranslation())
            < 0.2 * (hasReachedTarget ? 3 : 1);
  }
}
