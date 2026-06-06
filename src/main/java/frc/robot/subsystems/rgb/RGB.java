package frc.robot.subsystems.rgb;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rgb.RGBConstants.RGBMessage;
import frc.robot.subsystems.rgb.RGBConstants.RGBMessage.MessagePriority;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class RGB extends SubsystemBase {
  public static enum RGBMessages {
    CRITICAL_NETWORK_FAILURE(
        new RGBMessage(
            LEDPattern.solid(Color.kOrange).blink(Seconds.of(1 / 8)),
            MessagePriority.A_CRITICAL_NETWORK_FAILURE,
            true)),
    MISSING_CAN_DEVICE(
        new RGBMessage(LEDPattern.solid(Color.kRed), MessagePriority.B_MISSING_CAN_DEVICE, true)),
    READY_TO_INTAKE(
        new RGBMessage(
            LEDPattern.solid(Color.kBlue).blink(Seconds.of(1 / 8)),
            MessagePriority.D_READY_TO_INTAKE,
            true)),
    L2(
        new RGBMessage(
            LEDPattern.solid(Color.kOrange).blink(Seconds.of(1 / 8)), MessagePriority.E_L2, true)),
    L3(
        new RGBMessage(
            LEDPattern.solid(Color.kYellow).blink(Seconds.of(1 / 8)), MessagePriority.F_L3, true)),
    L4(
        new RGBMessage(
            LEDPattern.solid(Color.kBlue).blink(Seconds.of(1 / 8)), MessagePriority.G_L4, true)),
    L1(
        new RGBMessage(
            LEDPattern.solid(Color.kRed).blink(Seconds.of(1 / 8)), MessagePriority.H_L1, true)),
    CORAL_DETECTED(
        new RGBMessage(
            LEDPattern.solid(Color.kGreen).breathe(Seconds.of(2)),
            MessagePriority.I_CORAL_DETECTED,
            true)),
    DEFAULT(
        new RGBMessage(
            LEDPattern.rainbow(255, 51)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), Meters.of(0.01)),
            MessagePriority.J_DEFAULT,
            false));

    RGBMessage rgbMessage;

    private RGBMessages(RGBMessage rgbMessage) {
      this.rgbMessage = rgbMessage;
    }

    public void setIsExpired(boolean isExpired) {
      rgbMessage.setIsExpired(isExpired);
    }
  }

  private final RGBIO rgbIO;
  private RGBIOInputsAutoLogged inputs = new RGBIOInputsAutoLogged();
  private Optional<RGBMessage> currentMessage = Optional.empty();

  public RGB(RGBIO rgbIO) {
    this.rgbIO = rgbIO;
  }

  @Override
  public void periodic() {
    currentMessage = Optional.empty();
    int total = 0;
    for (RGBMessages message : RGBMessages.values()) {
      if (!message.rgbMessage.getIsExpired()
          && (currentMessage.isPresent()
              ? currentMessage.get().getPriority().compareTo(message.rgbMessage.getPriority()) > 0
              : true)) {
        currentMessage = Optional.of(message.rgbMessage);
        total++;
      }
    }
    if (currentMessage.isPresent()) {
      if (RGBMessages.CORAL_DETECTED.rgbMessage.getIsExpired()
          // if one of the level messages
          && (currentMessage.get().equals(RGBMessages.L1.rgbMessage)
              || currentMessage.get().equals(RGBMessages.L2.rgbMessage)
              || currentMessage.get().equals(RGBMessages.L3.rgbMessage)
              || currentMessage.get().equals(RGBMessages.L4.rgbMessage))) {
        currentMessage = Optional.of(RGBMessages.DEFAULT.rgbMessage);
      }
      rgbIO.displayMessage(currentMessage.get());
    } else {
      rgbIO.displayMessage(RGBMessages.DEFAULT.rgbMessage);
    }
    rgbIO.updateInputs(inputs);
    Logger.processInputs("RGB", inputs);
    Logger.recordOutput("RGB/Total Messages Not Expired", total);
    Logger.recordOutput(
        "RGB/Message",
        currentMessage.isPresent() ? currentMessage.get().getPriority().name() : "None");
  }

  public Command startMessageCommand(RGBMessages message) {
    return new InstantCommand(() -> message.setIsExpired(false));
  }

  public Command endMessageCommand(RGBMessages message) {
    return new InstantCommand(() -> message.setIsExpired(true));
  }

  public Command clearLevelCommands() {
    return new InstantCommand(
        () -> {
          RGBMessages.L1.setIsExpired(true);
          RGBMessages.L2.setIsExpired(true);
          RGBMessages.L3.setIsExpired(true);
          RGBMessages.L4.setIsExpired(true);
        });
  }
}
