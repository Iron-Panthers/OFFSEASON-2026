package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class HappyBirthdayCommand extends Command {
  // Times this method was ran:
  // 3/24/2026: Nora's Birthday YEEEPEEEE
  TalonFX motor = new TalonFX(1); // your CAN ID

  Orchestra orchestra = new Orchestra();

  public HappyBirthdayCommand() {
    orchestra.addInstrument(motor);
  }

  @Override
  public void initialize() {
    orchestra.loadMusic("happyBirthday.chrp");
    orchestra.play();
  }
}
