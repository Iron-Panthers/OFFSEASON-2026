package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;

public class HappyBirthdayCommand extends Command {
  // Times this method was ran:
  // 3/24/2026: Nora's Birthday YEEEPEEEE
  // 9/3/2026: Nolan's birthday
  Orchestra orchestra = new Orchestra();

  public HappyBirthdayCommand() {
    for(int i = 1; i < 100; i++){
      TalonFX motor = new TalonFX(i);
      orchestra.addInstrument(motor);
    }
  }

  @Override
  public void initialize() {
    orchestra.loadMusic("happyBirthday.chrp");
    orchestra.play();
  }
}
