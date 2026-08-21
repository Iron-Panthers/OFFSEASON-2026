// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.DriveConstants.DrivebaseConfig;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WaitUnitlRobotStuckCommand extends SequentialCommandGroup {
  /** Creates a new WaitUnitlRobotStuck. */
  public WaitUnitlRobotStuckCommand(Drive swerve, Supplier<Pose2d> shootingPoseSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new WaitUntilCommand(
                () ->
                    RobotState.getInstance()
                                .getPathPlannerTargetPose()
                                .getTranslation()
                                .getDistance(
                                    RobotState.getInstance().getEstimatedPose().getTranslation())
                            > 1.6
                        && !swerve.isPIDAutoAlign()));


                      double speed =
                          Math.sqrt(
                              Math.pow((swerve.getTargetSpeed().vxMetersPerSecond), 2)
                                  + Math.pow((swerve.getTargetSpeed().vyMetersPerSecond), 2));

                      double width = DriveConstants.DRIVE_CONFIG.bumperWidthX();
                      double length = DriveConstants.DRIVE_CONFIG.bumperWidthY();

                      Translation2d halfWidth = new Translation2d(width, length);
                      double distanceAway = halfWidth.getNorm();
                      // double edgeDistance = Math.sqrt((Math.pow(width,2) + Math.pow(length,2)) /
                      // 4) * 2;
                      Translation2d otherRobotTranslation2d =
                          RobotState.getInstance()
                              .getEstimatedPose()
                              .getTranslation()
                              .plus(
                                  new Translation2d(
                                      (swerve.getTargetSpeed().vyMetersPerSecond)
                                          * distanceAway
                                          / (speed),
                                      (swerve.getTargetSpeed().vxMetersPerSecond)
                                          * distanceAway
                                          / (speed)));
                      if(Math.abs(halfWidth.getY() - otherRobotTranslation2d.getY()) > halfWidth.getY() - 1.9){
                       addCommands(new AlignToPoseCommand(swerve, shootingPoseSupplier, false));
                      }else{
                      Logger.recordOutput(
                          "PathPlanner/Other Robot Position",
                          new Pose2d(otherRobotTranslation2d, Rotation2d.kZero));
                      Translation2d lowerBound = otherRobotTranslation2d.minus(halfWidth);
                      Translation2d upperBound = otherRobotTranslation2d.plus(halfWidth);
                      RobotState.getInstance()
                          .addDynamicObstacle(
                              new Pair<Translation2d, Translation2d>(lowerBound, upperBound));
                      }
  }
}
