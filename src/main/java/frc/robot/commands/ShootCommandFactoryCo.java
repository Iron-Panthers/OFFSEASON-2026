package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import java.util.function.Supplier;

/**
 * Handles the shooting sequence while held. Toggles between spin-up and shoot states, runs
 * serializer intake, and cycles the intake up/down while shooting.
 *
 * <p>Use {@link #whileHeld()} for the whileTrue binding and {@link #onRelease()} for the onFalse
 * binding.
 */
public class ShootCommandFactoryCo {
  private final ShooterController shooterController;
  private final IntakeController intakeController;
  private final ElasticUpdater matchTimerUpdater;
  private final Supplier<Rotation2d> getHeadingError;

  private boolean justShoot = false;
  double time = Timer.getFPGATimestamp();

  public ShootCommandFactoryCo(
      ShooterController shooterController,
      IntakeController intakeController,
      ElasticUpdater matchTimerUpdater,
      Supplier<Rotation2d> getHeadingError) {
    this.shooterController = shooterController;
    this.intakeController = intakeController;
    this.matchTimerUpdater = matchTimerUpdater;
    this.getHeadingError = getHeadingError;
    SmartDashboard.putNumber("Intake Rack In Time", 1.5);
  }

  /** Command to bind to whileTrue – repeats while the button is held. */
  public Command whileHeld() {
    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> intakeController.setTargetState(IntakeState.SHOOT)),
            setJustShootCommand(false)),
        Commands.waitUntil(() -> shooterController.getTargetState() == ShooterState.SHOOT),
        Commands.parallel(
            Commands.sequence(
                    Commands.runOnce(() -> intakeController.setTargetState(IntakeState.MID)),
                    new WaitCommand(0.1),
                    Commands.runOnce(() -> intakeController.setTargetState(IntakeState.SHOOT)),
                    new WaitCommand(0.2))
                .repeatedly()
                .until(() -> intakeController.getTargetState() == IntakeState.SHOOTING_STOW),
            Commands.run(
                    () -> {
                      shooterController.setTargetState(
                          (shooterController.getTargetState() == ShooterState.TOTAL_SPIN_UP
                                      || shooterController.getTargetState() == ShooterState.SHOOT)
                                  && shooterController.flywheelsUpToSpeed()
                                  && (matchTimerUpdater.isOurHubActive()
                                      || matchTimerUpdater.getTimeUntilOurHubShifts() <= 2
                                      || matchTimerUpdater.getTimeUntilOurHubShifts()
                                          >= 24) // time correct
                                  && ((getHeadingError.get().getDegrees() < 4
                                          || getHeadingError.get().getDegrees() > 356)
                                      || justShoot) // angle correct
                              ? ShooterState.SHOOT
                              : ShooterState.TOTAL_SPIN_UP);
                    })
                .repeatedly(),
            Commands.sequence(
                    Commands.waitUntil(
                        () -> shooterController.getTargetState() == ShooterState.SHOOT),
                    Commands.runOnce(() -> time = Timer.getFPGATimestamp()),
                    Commands.waitUntil(
                        () ->
                            ((SmartDashboard.getNumber("Intake Rack In Time", 1.5) + time)
                                < Timer.getFPGATimestamp())),
                    intakeController.setTargetStateCommand(IntakeState.SHOOTING_STOW))
                .repeatedly()));
  }

  /** Command to bind to onFalse – runs when the button is released. */
  public Command onRelease() {
    return new InstantCommand(
        () -> {
          if (shooterController.getTargetState() == ShooterState.SHOOT) {
            shooterController.setTargetState(ShooterState.COMPACT_SPIN_UP);
          }
        });
  }

  /** Command to bind to whileTrue – repeats while the button is held. */
  public Command whileHeldPassing() {
    return new InstantCommand(
            () -> {
              shooterController.setTargetState(
                  (shooterController.getTargetState() == ShooterState.PASS_SPIN_UP
                              || shooterController.getTargetState() == ShooterState.PASS)
                          && shooterController.flywheelsUpToSpeed() // time correct
                      ? ShooterState.PASS
                      : ShooterState.PASS_SPIN_UP);
            })
        .repeatedly()
        .alongWith(
            new WaitCommand(1.5)
                .andThen(intakeController.setTargetStateCommand(IntakeState.SHOOTING_STOW)));
  }

  public Command setJustShootCommand(boolean justShoot) {
    return new InstantCommand(() -> this.justShoot = justShoot);
  }
}
