package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import java.util.function.Supplier;

public class SmartIntakeCommand {

  Supplier<Double> pressure;
  private final IntakeController intakeController;
  private final ShooterController shooterController;
  private final ElasticUpdater matchTimerUpdater;
  private final Supplier<Rotation2d> getHeadingError;
  double time = Timer.getFPGATimestamp();
  private boolean justShoot = false;

  public SmartIntakeCommand(
      Supplier<Double> pressure,
      IntakeController intakeController,
      ShooterController shooterController,
      ElasticUpdater matchTimerUpdater,
      Supplier<Rotation2d> getHeadingError) {
    this.pressure = pressure;
    this.intakeController = intakeController;
    this.shooterController = shooterController;
    this.matchTimerUpdater = matchTimerUpdater;
    this.getHeadingError = getHeadingError;
  }

  // TODO: Change the intake states to reflect the pressure
  public Command whileHeld() {

    return (pressure.get() > 60.0)
        ? new InstantCommand(() -> intakeController.setTargetState(IntakeState.STOW))
        : (pressure.get() > 40.0)
            ? new InstantCommand(() -> intakeController.setTargetState(IntakeState.SHOOTING_STOW))
        : (pressure.get() > 20.0)
            ? new InstantCommand(() -> intakeController.setTargetState(IntakeState.INTAKE))
        : new InstantCommand(() -> intakeController.setTargetState(IntakeState.MID))
                .alongWith(
                    new InstantCommand(
                            () -> {
                              shooterController.setTargetState(
                                  (shooterController.getTargetState() == ShooterState.TOTAL_SPIN_UP
                                              || shooterController.getTargetState()
                                                  == ShooterState.SHOOT)
                                          && shooterController.flywheelsUpToSpeed()
                                          && (matchTimerUpdater.isOurHubActive()
                                              || matchTimerUpdater.getTimeUntilOurHubShifts() <= 2
                                              || matchTimerUpdater.getTimeUntilOurHubShifts()
                                                  >= 24) // time correct
                                          && (getHeadingError.get().getDegrees() < 6
                                              || getHeadingError.get().getDegrees() > 354
                                              || justShoot) // angle correct
                                      ? ShooterState.SHOOT
                                      : ShooterState.TOTAL_SPIN_UP);
                            })
                        .repeatedly()
                        .alongWith(
                            (new WaitUntilCommand(
                                        () ->
                                            shooterController.getTargetState()
                                                == ShooterState.SHOOT)
                                    .andThen(
                                        new InstantCommand(() -> time = Timer.getFPGATimestamp()))
                                    .andThen(
                                        new WaitUntilCommand(
                                            () ->
                                                ((SmartDashboard.getNumber("Intake Rack In Time", 2)
                                                        + time)
                                                    < Timer.getFPGATimestamp())))
                                    .andThen(
                                        intakeController.setTargetStateCommand(IntakeState.STOW))
                                    .withDeadline(
                                        new WaitUntilCommand(
                                            () ->
                                                (shooterController.getTargetState()
                                                    == ShooterState.TOTAL_SPIN_UP))))
                                .repeatedly()));
  }
}
