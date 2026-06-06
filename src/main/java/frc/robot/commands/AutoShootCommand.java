// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.swerve.Drive;

public class AutoShootCommand extends SequentialCommandGroup {
  public AutoShootCommand(
      Drive swerve,
      ShooterController shooterController,
      IntakeController intakeController,
      ElasticUpdater matchTimerUpdater,
      boolean intakeActive) {
    addCommands(
        (new InstantCommand(() -> shooterController.setTargetState(ShooterState.TOTAL_SPIN_UP))
                .andThen(new WaitCommand(0.2)))
            .andThen(
                new InstantCommand(() -> shooterController.setTargetState(ShooterState.SHOOT))
                    .alongWith(
                        new WaitCommand(1.3)
                            .andThen(
                                intakeController.setTargetStateCommand(IntakeState.SHOOTING_STOW)))
                    .withDeadline(new WaitCommand(3))),
        (intakeActive
            ? new IntakeCommand(intakeController, shooterController)
            : new InstantCommand()),
        shooterController.setTargetStateCommand(ShooterState.FLYWHEEL_SPIN_UP));
  }
}
