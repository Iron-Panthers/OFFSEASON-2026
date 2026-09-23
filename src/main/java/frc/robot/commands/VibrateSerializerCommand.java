package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.serializer.Serializer;
import frc.robot.subsystems.shooter.serializer.Serializer.SerializerTarget;

public class VibrateSerializerCommand extends Command {
  Serializer serializer;

  public VibrateSerializerCommand(Serializer serializer) {
    this.serializer = serializer;
  }

  @Override
  public void initialize() {
    new InstantCommand(() -> serializer.setVelocityTarget(SerializerTarget.REVERSE))
        .andThen(new WaitCommand(0.2))
        .andThen(() -> serializer.setVelocityTarget(SerializerTarget.SHOOT))
        .andThen(new WaitCommand(0.2))
        .initialize();
  }
}
