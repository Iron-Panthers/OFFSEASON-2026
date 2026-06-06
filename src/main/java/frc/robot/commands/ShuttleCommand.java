package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.swerve.Drive;

/**
 * Aligns the heading to 0° and sets the shooter to shuttle mode. Intended to be used with
 * whileTrue.
 */
public class ShuttleCommand extends SequentialCommandGroup {
  public ShuttleCommand(Drive swerve, ShooterController shooterController) {
    addCommands(
        new InstantCommand(() -> swerve.setTargetHeading(new Rotation2d(0))),
        shooterController.setTargetStateCommand(ShooterState.PASS));
  }
}
