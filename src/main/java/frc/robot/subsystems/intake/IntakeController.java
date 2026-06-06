package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers.ControlMode;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.subsystems.intake.intake_rack.IntakeRack;
import frc.robot.subsystems.intake.intake_rack.IntakeRack.IntakeRackTarget;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers;
import frc.robot.subsystems.intake.intake_rollers.IntakeRollers.IntakeRollersTarget;
import org.littletonrobotics.junction.Logger;

public class IntakeController extends SubsystemBase {
  public enum IntakeState {
    STOW(IntakeRackTarget.STOW, IntakeRollersTarget.IDLE),
    SHOOTING_STOW(IntakeRackTarget.SHOOTING_STOW, IntakeRollersTarget.IDLE),
    SHOOT(IntakeRackTarget.INTAKE, IntakeRollersTarget.INTAKE_SLOW),
    MID(IntakeRackTarget.MIDDLE, IntakeRollersTarget.INTAKE_SLOW),
    IDLE(IntakeRackTarget.INTAKE, IntakeRollersTarget.IDLE),
    INTAKE(IntakeRackTarget.INTAKE, IntakeRollersTarget.INTAKE),
    INTAKE_SLOW(IntakeRackTarget.INTAKE, IntakeRollersTarget.INTAKE_SLOW),
    REVERSE(IntakeRackTarget.INTAKE, IntakeRollersTarget.EJECT),
    ZEROING(IntakeRackTarget.STOW, IntakeRollersTarget.IDLE);

    private IntakeRackTarget intakeRackTarget;
    private IntakeRollersTarget intakeRollersTarget;

    private IntakeState(
        IntakeRackTarget intakeRackTarget, IntakeRollersTarget intakeRollersTarget) {
      this.intakeRackTarget = intakeRackTarget;
      this.intakeRollersTarget = intakeRollersTarget;
    }

    public IntakeRackTarget getIntakeRackTarget() {
      return intakeRackTarget;
    }

    public IntakeRollersTarget getIntakeRollersTarget() {
      return intakeRollersTarget;
    }
  }

  private IntakeState targetState = IntakeState.STOW;
  private boolean stopped = false;

  private boolean intakeRackActive = true;

  private final IntakeRack intakeRack;
  private final IntakeRollers intakeRollers;

  public IntakeController(IntakeRack intakeRack, IntakeRollers intakeRollers) {
    this.intakeRack = intakeRack;
    this.intakeRollers = intakeRollers;
  }

  @Override
  public void periodic() {
    if (stopped) {
      intakeRollers.setControlMode(ControlMode.STOP);
      intakeRack.setControlMode(GenericSuperstructure.ControlMode.STOP);
      // if else set control mode to zero
    } else if (intakeRack.getControlMode() == GenericSuperstructure.ControlMode.ZEROING) {
      intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
    } else if (intakeRack.getPosition() < 1.5 && targetState == IntakeState.INTAKE) {
      intakeRollers.setVelocityTarget(IntakeRollersTarget.IDLE);
      intakeRack.setPositionTarget(targetState.getIntakeRackTarget());
    } else if ((targetState == IntakeState.STOW || targetState == IntakeState.SHOOTING_STOW)
        && !intakeRack.reachedTarget()) {
      intakeRollers.setVelocityTarget(IntakeRollersTarget.INTAKE);
      intakeRack.setPositionTarget(targetState.getIntakeRackTarget());
    } else {
      // set target states to those in the current controller state
      intakeRack.setPositionTarget(targetState.getIntakeRackTarget());
      intakeRollers.setVelocityTarget(targetState.getIntakeRollersTarget());
    }
    if (!intakeRackActive) {
      intakeRack.setPositionTarget(IntakeRackTarget.INTAKE);
    }
    intakeRack.periodic();
    intakeRollers.periodic();

    Logger.recordOutput("Intake/Active", intakeRackActive);
  }

  // GETTTERS AND SETTERS
  public void setTargetState(IntakeState targetState) {
    setStopped(false);
    this.targetState = targetState;
  }

  public IntakeState getTargetState() {
    return targetState;
  }

  public Command setTargetStateCommand(IntakeState targetState) {
    return new InstantCommand(() -> setTargetState(targetState), this)
        .andThen(
            new WaitCommand(0.2).andThen(new WaitUntilCommand(() -> intakeRack.reachedTarget())));
  }

  public void setStopped(boolean stopped) {
    this.stopped = stopped;
  }

  public Command setStoppedCommand(boolean stopped) {
    return new InstantCommand(() -> setStopped(stopped));
  }

  public Command zeroCommand() {
    return new InstantCommand(
            () -> intakeRack.setControlMode(GenericSuperstructure.ControlMode.ZEROING))
        .alongWith(setTargetStateCommand(IntakeState.ZEROING).alongWith(setStoppedCommand(false)));
  }

  public Command stopZeroingCommand() {
    return new InstantCommand(() -> intakeRack.endZeroing());
  }

  public void stopZeroing() {
    intakeRack.endZeroing();
  }

  public void setIntakeRackActive(boolean isActive) {
    intakeRackActive = isActive;
  }

  public boolean getIntakeRackActive() {
    return intakeRackActive;
  }
}
