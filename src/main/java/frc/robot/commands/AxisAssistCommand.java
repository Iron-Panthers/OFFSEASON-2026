package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.Supplier;

public class AxisAssistCommand extends Command {
  Drive swerve;
  Supplier<Distance> targetXPosition;
  Supplier<Rotation2d> targetHeading;
  Supplier<Boolean> isControlledOnY;

  public AxisAssistCommand(
      Drive swerve,
      Supplier<Distance> targetXPosition,
      Supplier<Rotation2d> targetHeading,
      Supplier<Boolean> isControlledOnY) {
    this.swerve = swerve;
    this.targetXPosition = targetXPosition;
    this.targetHeading = targetHeading;
    this.isControlledOnY = isControlledOnY;
  }

  @Override
  public void initialize() {
    swerve.setAxisPosition(targetXPosition.get(), targetHeading.get(), isControlledOnY.get());
  }

  @Override
  public void end(boolean interrupted) {
    swerve.clearTargetPositionController();
    swerve.setTeleopMode();
  }
}
