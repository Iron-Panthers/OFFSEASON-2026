package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

/**
 * Deploys the intake to pick up game pieces. Stows climb, sequences intake deploy then intake,
 * idles the shooter, and sets serializer to idle.
 */
public class IntakeCommand extends SequentialCommandGroup {
  public IntakeCommand(IntakeController intakeController, ShooterController shooterController) {
    addCommands(
        intakeController
            .setTargetStateCommand(IntakeState.INTAKE)
            .alongWith(shooterController.setTargetStateCommand(ShooterState.INTAKE)));
  }
}
