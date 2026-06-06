// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToPoseCommand extends Command {
  private Command poseAlignCommand;
  private Drive drive;
  private Supplier<Pose2d> approachPose;
  private Pose2d currentApproachPose;
  private boolean underTrench;
  private boolean endOnAccurate = false;

  public AlignToPoseCommand(Drive drive, Supplier<Pose2d> approachPose, boolean underTrench) {
    // all of this jank is basically so that we can get a command that generates the pose on the fly
    // and still figure out when it ends
    this.drive = drive;
    this.approachPose =
        RobotState.isAllianceRed()
            ? () -> FlippingUtil.flipFieldPose(approachPose.get())
            : approachPose;
    this.underTrench = underTrench;

    addRequirements(drive);
  }

  public AlignToPoseCommand(
      Drive drive, Supplier<Pose2d> approachPose, boolean underTrench, boolean endOnAccurate) {
    this(drive, approachPose, underTrench);
    this.endOnAccurate = endOnAccurate;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentApproachPose = approachPose.get();
    drive.setTargetPosition(currentApproachPose); // :)
    try {
      poseAlignCommand =
          new VelocityClamp(drive)
              .andThen(
                  RobotState.getInstance()
                      .getPathPlannerApproachPoseCommand(currentApproachPose, underTrench));
      poseAlignCommand.initialize();
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("Already at target.");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!poseAlignCommand.isFinished()
        && RobotState.getInstance()
                .getEstimatedPose()
                .getTranslation()
                .getDistance(currentApproachPose.getTranslation())
            >= DriveConstants.PATHPLANNER_PID_OFFSET) {
      poseAlignCommand.execute();
    } else {
      if (!drive.isPIDAutoAlign()) {
        drive.setPIDAutoAlignTargetPosition(currentApproachPose);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    poseAlignCommand.end(interrupted);
    drive.clearTargetPositionController();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (endOnAccurate
        && currentApproachPose
                .getTranslation()
                .getDistance(RobotState.getInstance().getEstimatedPose().getTranslation())
            < 0.04
        && Math.abs(
                currentApproachPose
                    .getRotation()
                    .minus(RobotState.getInstance().getEstimatedPose().getRotation())
                    .getDegrees())
            < 2.5) {
      return true;
    }
    return false;
  }
}
