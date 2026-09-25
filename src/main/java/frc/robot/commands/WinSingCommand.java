package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class WinSingCommand extends Command {
    Orchestra orchestra = new Orchestra();

    public WinSingCommand() {
        for (int i = 1; i < 100; i++) {
            TalonFX motor = new TalonFX(i);
            orchestra.addInstrument(motor);
        }
    }

    @Override
    public void initialize() {
        orchestra.loadMusic("square.chrp");
        orchestra.loadMusic("triangle.chrp");
        orchestra.loadMusic("noise.chrp");
        orchestra.play();
    }
}
