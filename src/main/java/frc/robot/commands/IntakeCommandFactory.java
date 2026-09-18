package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.shooter.serializer.Serializer;
import frc.robot.subsystems.shooter.serializer.Serializer.SerializerTarget;

/**
 * Deploys the intake to pick up game pieces. Stows climb, sequences intake deploy then intake,
 * idles the shooter, and sets serializer to idle.
 */
public class IntakeCommandFactory {
  private IntakeController intakeController;
  private ShooterController shooterController;
  private Serializer serializer;

  public IntakeCommandFactory(
      IntakeController intakeController,
      ShooterController shooterController,
      Serializer serializer) {

    this.intakeController = intakeController;
    this.shooterController = shooterController;
    this.serializer = serializer;
  }

  public Command whileHeld() {
    return intakeController
        .setTargetStateCommand(IntakeState.INTAKE)
        .alongWith(shooterController.setTargetStateCommand(ShooterState.INTAKE))
        .alongWith(new InstantCommand(() -> serializer.setVelocityTarget(SerializerTarget.SLOW)));
  }

  public Command onRelease() {
    return new WaitCommand(0.5)
        .andThen(intakeController.setTargetStateCommand(IntakeState.IDLE))
        .andThen(shooterController.setTargetStateCommand(ShooterState.FLYWHEEL_SPIN_UP))
        .andThen(new InstantCommand(() -> serializer.setVelocityTarget(SerializerTarget.IDLE)));
  }
}
