package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.DriveConstants.HEADING_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.swerve.DriveConstants.KINEMATICS;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.controllers.heading.AutoAlignHeadingController;
import frc.robot.subsystems.swerve.controllers.heading.TeleopHeadingController;
import frc.robot.subsystems.swerve.controllers.translation.AxisAssist;
import frc.robot.subsystems.swerve.controllers.translation.PIDAutoAlignController;
import frc.robot.subsystems.swerve.controllers.translation.TeleopTranslationController;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public enum DriveModes {
    TELEOP,
    TRAJECTORY,
    AUTO_ALIGN,
    AXIS_ASSIST,
    DEFENSE;
  }

  private DriveModes driveMode = DriveModes.TELEOP;

  private boolean isScoped = false;
  private boolean isBeingDefended = false;
  private boolean isHDefense = false;
  private boolean isFromTeleop = false;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private Module[] modules = new Module[4];

  private Rotation2d fieldRelativeYaw = new Rotation2d();

  @AutoLogOutput(key = "Swerve/Gyro Yaw Offset")
  private Rotation2d gyroYawOffset = new Rotation2d();

  @AutoLogOutput(key = "Swerve/Current Position")
  private Pose2d currentPosition = new Pose2d();

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
  private ChassisSpeeds trajectorySpeeds = new ChassisSpeeds();

  private double speedMagnitude =
      Math.hypot(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond);

  private Pose2d targetPosition = new Pose2d();

  // controllers
  private final TeleopTranslationController teleopController;
  private TeleopHeadingController headingController = null;
  private PIDAutoAlignController pidAutoAlignController = null;
  private AutoAlignHeadingController autoAlignHeadingController = null;
  private AxisAssist axisAssistController;

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;

    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);

    teleopController = new TeleopTranslationController(() -> fieldRelativeYaw);
  }

  @Override
  public void periodic() {
    currentPosition = RobotState.getInstance().getEstimatedPose();
    // update inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    fieldRelativeYaw =
        Rotation2d.fromDegrees(
            normalizeDegrees(gyroInputs.yawPosition.minus(gyroYawOffset).getDegrees()));

    for (Module module : modules) {
      module.updateInputs();
    }

    // pass odometry data to robotstate
    SwerveModulePosition[] wheelPositions =
        Arrays.stream(modules)
            .map(module -> module.getModulePosition())
            .toArray(SwerveModulePosition[]::new);

    RobotState.getInstance()
        .addOdometryMeasurement(
            new RobotState.OdometryMeasurement(
                wheelPositions, gyroInputs.yawPosition, Timer.getTimestamp()));

    switch (driveMode) {
      case TELEOP -> {
        targetSpeeds = teleopController.update();
        if (headingController != null) {
          // 0.d0001 to make the wheels stop in a diamond shape instead of straight so they do not
          // vibrate
          double rotationVelocity = headingController.update();
          targetSpeeds.omegaRadiansPerSecond =
              Math.abs(rotationVelocity) > 0.0001 ? rotationVelocity : 0.0001;
        }

        if (speedMagnitude < 0.01
            && Math.abs(targetSpeeds.omegaRadiansPerSecond) < 0.1
            && isScoped
            && isBeingDefended) {
          if (Math.abs(headingController.getTargetHeading().plus(Rotation2d.kCW_90deg).getDegrees())
                  < 17
              || Math.abs(
                      headingController.getTargetHeading().plus(Rotation2d.kCCW_90deg).getDegrees())
                  < 17) {
            setDefenseMode(true);
          } else {
            setDefenseMode(false);
          }
        }
        isFromTeleop = true;
      }
      case TRAJECTORY -> {
        Logger.recordOutput(
            "Swerve/Distance From Setpoint",
            RobotState.getInstance()
                .getEstimatedPose()
                .getTranslation()
                .getDistance(targetPosition.getTranslation()));
        targetSpeeds = trajectorySpeeds;
        if (headingController != null && isScoped) {
          // 0.d0001 to make the wheels stop in a diamond shape instead of straight so they do not
          // vibrate
          double rotationVelocity = headingController.update();
          targetSpeeds.omegaRadiansPerSecond =
              Math.abs(rotationVelocity) > 0.0001 ? rotationVelocity : 0.0001;
        }
      }
      case AUTO_ALIGN -> {
        if (pidAutoAlignController != null) {
          targetSpeeds = pidAutoAlignController.update();
          targetSpeeds.omegaRadiansPerSecond = autoAlignHeadingController.update();

          if (speedMagnitude < 0.01
              && Math.abs(targetSpeeds.omegaRadiansPerSecond) < 0.1
              && isScoped
              && isBeingDefended) {
            if (Math.abs(
                        autoAlignHeadingController
                            .getTargetHeading()
                            .plus(Rotation2d.kCW_90deg)
                            .getDegrees())
                    < 17
                || Math.abs(
                        autoAlignHeadingController
                            .getTargetHeading()
                            .plus(Rotation2d.kCCW_90deg)
                            .getDegrees())
                    < 17) {
              setDefenseMode(true);
            } else {
              setDefenseMode(false);
            }
          }
          isFromTeleop = false;
        } else if (headingController != null) {
          targetSpeeds.omegaRadiansPerSecond = headingController.update();
        }
      }
      case AXIS_ASSIST -> {
        if (axisAssistController != null) {
          targetSpeeds = axisAssistController.update();

          if (headingController != null) {
            targetSpeeds.omegaRadiansPerSecond = headingController.update();
          }
        }
      }
      case DEFENSE -> {
        targetSpeeds = teleopController.update();
        if (headingController != null) {
          // 0.d0001 to make the wheels stop in a diamond shape instead of straight so they do not
          // vibrate
          double rotationVelocity = headingController.update();
          targetSpeeds.omegaRadiansPerSecond =
              Math.abs(rotationVelocity) > 0.0001 ? rotationVelocity : 0.0001;
        }
        double speedMagnitude =
            Math.hypot(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond);

        Logger.recordOutput("Swerve/Speed Magnitude", speedMagnitude);
        Logger.recordOutput("Swerve/Angular Velocity", targetSpeeds.omegaRadiansPerSecond);

        if (isFromTeleop) {
          if (speedMagnitude > 0.015 || Math.abs(targetSpeeds.omegaRadiansPerSecond) > 0.4) {
            driveMode = DriveModes.TELEOP;
          }
        } else {
          if (speedMagnitude > 0.015 || Math.abs(targetSpeeds.omegaRadiansPerSecond) > 0.4) {
            driveMode = DriveModes.AUTO_ALIGN;
          }
        }

        if (isHDefense) {
          modules[0].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(0))));
          modules[1].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(0))));
          modules[2].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(0))));
          modules[3].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(0))));
        } else {
          modules[0].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(-135))));
          modules[1].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(135))));
          modules[2].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(-225))));
          modules[3].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(225))));
        }
      }
    }

    RobotState.getInstance().addRobotSpeeds(getRobotSpeeds());
    // run modules
    /* use kinematics to get desired module states */
    if (driveMode != DriveModes.DEFENSE) {
      ChassisSpeeds discretizedSpeeds =
          ChassisSpeeds.discretize(targetSpeeds, Constants.PERIODIC_LOOP_SEC);

      SwerveModuleState[] moduleTargetStates = KINEMATICS.toSwerveModuleStates(discretizedSpeeds);

      SwerveDriveKinematics.desaturateWheelSpeeds(
          moduleTargetStates, DriveConstants.DRIVE_CONFIG.maxLinearVelocity());

      for (int i = 0; i < modules.length; i++) {
        modules[i].runToSetpoint(moduleTargetStates[i]);
      }

      Logger.recordOutput("Swerve/Module Target States", moduleTargetStates);
    }
    Logger.recordOutput("Swerve/Is Scoped", isScoped);
    Logger.recordOutput("Swerve/Target Speeds", targetSpeeds);
    Logger.recordOutput("Swerve/Drive Mode", driveMode);
    Logger.recordOutput(
        "Swerve/Magnitude",
        Math.hypot(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond));
    Logger.recordOutput("Swerve/Field Relative Yaw", fieldRelativeYaw);
    Logger.recordOutput("Swerve/Trajectory Speeds", trajectorySpeeds);
    if (headingController != null) {
      Logger.recordOutput(
          "Swerve/Heading Target", headingController.getTargetHeading().getRadians());
    }
    Logger.recordOutput("Swerve/Estimated X", RobotState.getInstance().getEstimatedPose().getX());
    Logger.recordOutput("Swerve/Estimated Y", RobotState.getInstance().getEstimatedPose().getY());
    if (pidAutoAlignController != null) {
      Logger.recordOutput("Swerve/PID/Velocity X", pidAutoAlignController.getXVel());
      Logger.recordOutput("Swerve/PID/Velocity Y", pidAutoAlignController.getYVel());
    }
    if (axisAssistController != null) {
      Logger.recordOutput("Swerve/PID/Velocity X", axisAssistController.getXVel());
      Logger.recordOutput("Swerve/PID/Velocity Y", axisAssistController.getYVel());
    }
  }

  public void setDefenseMode() {
    driveMode = DriveModes.DEFENSE;
  }

  public void setTeleopMode() {
    driveMode = DriveModes.TELEOP;
  }

  public void driveTeleopController(double xAxis, double yAxis, double omega, double acceleration) {
    if (DriverStation.isTeleopEnabled()) {
      if (driveMode != DriveModes.TELEOP
          && driveMode != DriveModes.AXIS_ASSIST
          && driveMode != DriveModes.DEFENSE) {
        driveMode = DriveModes.TELEOP;
        teleopController.setPastLinearVelocity(new Translation2d());
      }
      teleopController.acceptJoystickInput(xAxis, yAxis, omega, acceleration);
      if (axisAssistController != null && driveMode == DriveModes.AXIS_ASSIST) {
        axisAssistController.acceptJoystickInput(xAxis, yAxis, acceleration);
      }
    }
  }

  public void setTrajectorySpeeds(ChassisSpeeds speeds) {
    driveMode = DriveModes.TRAJECTORY;
    this.trajectorySpeeds = speeds;
  }

  private void zeroGyro() {
    gyroYawOffset = gyroInputs.yawPosition;
    // Will be reinitialized in setTargetHeading
    headingController = null;
  }

  public Command zeroGyroCommand() {
    return this.runOnce(() -> zeroGyro());
  }

  public void smartZeroGyro() {
    gyroYawOffset =
        gyroInputs.yawPosition.minus(
            RobotState.isAllianceRed()
                ? FlippingUtil.flipFieldRotation(
                    RobotState.getInstance().getEstimatedPose().getRotation())
                : RobotState.getInstance().getEstimatedPose().getRotation());
  }

  @AutoLogOutput(key = "Swerve/Module States")
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules)
        .map(module -> module.getModuleState())
        .toArray(SwerveModuleState[]::new);
  }

  @AutoLogOutput(key = "Swerve/Robot Speeds")
  public ChassisSpeeds getRobotSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void setTargetHeading(Rotation2d targetHeading) {
    if (headingController == null) {
      headingController =
          new TeleopHeadingController(
              () -> fieldRelativeYaw, targetHeading, HEADING_CONTROLLER_CONSTANTS);
    } else {
      headingController.setTargetHeading(targetHeading);
    }
  }

  public void setMovementScoped(boolean scoped) {
    this.isScoped = scoped;
    teleopController.setScoped(scoped);
    if (headingController == null) {
      headingController =
          new TeleopHeadingController(
              () -> fieldRelativeYaw,
              RobotState.getInstance().calculateTargetShootingState().drivebaseYaw(),
              HEADING_CONTROLLER_CONSTANTS);
    }
    headingController.setScoped(scoped);
  }

  public void clearHeadingControl() {
    headingController = null;
  }

  public void setTargetPosition(Pose2d targetPosition) {
    this.targetPosition = targetPosition;
  }

  public double setAxisPosition(Distance targetPosition, Rotation2d targetAngle, boolean controlY) {
    // nguerrna be smart
    clearHeadingControl();
    driveMode = DriveModes.AXIS_ASSIST;
    if (axisAssistController == null) {
      axisAssistController =
          new AxisAssist(
              () -> RobotState.getInstance().getEstimatedPose(),
              () -> fieldRelativeYaw,
              targetPosition.in(Units.Meters),
              controlY);
    }
    if (headingController == null) {
      headingController =
          new TeleopHeadingController(
              () -> fieldRelativeYaw, targetAngle, HEADING_CONTROLLER_CONSTANTS);
    } else {
      headingController.setTargetHeading(targetAngle);
    }
    return targetPosition.in(Units.Meters);
  }

  public Pose2d setPIDAutoAlignTargetPosition(Pose2d targetPosition) {
    setTargetPosition(targetPosition);

    clearHeadingControl();
    driveMode = DriveModes.AUTO_ALIGN;
    if (pidAutoAlignController == null) {
      pidAutoAlignController =
          new PIDAutoAlignController(
              () -> RobotState.getInstance().getEstimatedPose(),
              () -> fieldRelativeYaw,
              targetPosition);
    } else {
      pidAutoAlignController.setTargetPosition(targetPosition);
    }

    if (autoAlignHeadingController == null) {
      autoAlignHeadingController =
          new AutoAlignHeadingController(
              () -> RobotState.getInstance().getEstimatedPose().getRotation(),
              targetPosition.getRotation(),
              pidAutoAlignController.calculateTimeLeft(),
              DriveConstants.ROTATION_FINISH_PERCENT);
    } else {
      autoAlignHeadingController.setTargetHeading(
          targetPosition.getRotation(),
          pidAutoAlignController.calculateTimeLeft(),
          DriveConstants.ROTATION_FINISH_PERCENT);
    }
    return targetPosition;
  }

  public void clearTargetPositionController() {
    pidAutoAlignController = null;
    autoAlignHeadingController = null;
    axisAssistController = null;
    targetSpeeds = new ChassisSpeeds();
  }

  public Command setTargetPositionCommand(Pose2d targetPosition) {
    return new FunctionalCommand(
        () -> setPIDAutoAlignTargetPosition(targetPosition),
        () -> {},
        (t) -> clearTargetPositionController(),
        () -> false,
        this);
  }

  public boolean isTeleop() {
    return driveMode == DriveModes.TELEOP;
  }

  public double normalizeDegrees(double degrees) {
    return (degrees % 360 + 360) % 360;
  }

  public boolean isPIDAutoAlign() {
    return driveMode == DriveModes.AUTO_ALIGN;
  }

  public boolean isHeadingCorrect() {
    return headingController == null || headingController.atTarget();
  }

  public void setIsBeingDefended(boolean isBeingDefended) {
    this.isBeingDefended = isBeingDefended;
  }

  public Rotation2d getShootingError() {
    if (headingController == null) {
      return Rotation2d.fromDegrees(0);
    }
    return headingController.getError();
  }

  /**
   * Sets the defense mode and sets the defense type to that supplied in the params
   *
   * @param isHDefense wether or not to use H defense versus X defense
   */
  public void setDefenseMode(boolean isHDefense) {
    setDefenseMode();
    this.isHDefense = isHDefense;
  }

  public boolean getIsHDefense() {
    return isHDefense;
  }

  public void setDriveSupplyCurrentLimits(double amps) {
    for (Module module : modules) {
      module.setDriveSupplyCurrentLimit(amps);
    }
  }

  public boolean reachedAutoAlignTarget() {
    if (driveMode != DriveModes.AUTO_ALIGN) {
      return false;
    }
    return autoAlignHeadingController.atTarget() && pidAutoAlignController.atTarget();
  }

  public boolean almostReachedAutoAlignTarget() {
    if (driveMode != DriveModes.AUTO_ALIGN || driveMode != DriveModes.DEFENSE) {
      return false;
    }
    return pidAutoAlignController.almostAtTarget();
  }

  public void setNeutralMode(NeutralModeValue value) {
    for (Module module : modules) {
      module.setNeutralMode(value);
    }
  }

  public boolean getIsScoped() {
    return isScoped;
  }
}
