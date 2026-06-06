package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

/** Stows the robot: moves intake to middle stow, idles the shooter, and sets serializer to slow. */
public class StowCommand extends ParallelCommandGroup {
  public StowCommand(IntakeController intakeController, ShooterController shooterController) {
    addCommands(
        intakeController.setTargetStateCommand(IntakeState.SHOOTING_STOW),
        shooterController.setTargetStateCommand(ShooterState.IDLE));
  }
}
