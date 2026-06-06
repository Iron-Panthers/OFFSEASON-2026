package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.swerve.Drive;

/**
 * Aligns the robot to the shooting pose and spins up the shooter once close enough. Intended to be
 * used with whileTrue.
 */
public class AlignToShootCommand extends Command {
  private Drive swerve;
  private ShooterController shooterController;

  public AlignToShootCommand(Drive swerve, ShooterController shooterController) {
    this.swerve = swerve;
    this.shooterController = shooterController;
  }

  public void initialize() {
    swerve.setMovementScoped(true);
    shooterController.setAutoAim(true);
  }

  public void execute() {
    swerve.setMovementScoped(true);
    shooterController.setAutoAim(true);
  }

  public void end(boolean interrupted) {
    swerve.setMovementScoped(false);
    shooterController.setAutoAim(false);
  }
}
