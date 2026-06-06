package frc.robot.subsystems.elastic_updater;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class ElasticUpdater extends SubsystemBase {

  private double matchTime;
  private double timeUntilOurHubShifts;
  private boolean ourHubActive;

  @Override
  public void periodic() {

    Logger.recordOutput("Match Time", matchTime = DriverStation.getMatchTime());

    if (DriverStation.isAutonomous()) {
      updateMatchData((matchTime >= 0 && matchTime <= 30) ? "Auto" : "Invalid Timeframe");

    } else if (DriverStation.isTeleop()) {

      if (matchTime > 140) {
        updateMatchData("Invalid Timeframe");
      } else if (matchTime > 130) {
        updateMatchData("Transition Shift");
      } else if (matchTime > 105) {
        updateMatchData("Shift 1");
      } else if (matchTime > 80) {
        updateMatchData("Shift 2");
      } else if (matchTime > 55) {
        updateMatchData("Shift 3");
      } else if (matchTime > 30) {
        updateMatchData("Shift 4");
      } else if (matchTime > 0) {
        updateMatchData("End Game");
      } else {
        updateMatchData("Invalid Timeframe");
      }
    } else {
      updateMatchData("Invalid Timeframe");
    }
  }

  private void updateMatchData(String timeframe) {
    Logger.recordOutput("Match Time frame", timeframe);

    boolean firstAllianceIsRed = true;
    // DriverStation.getGameSpecificMessage returns the team that is active on shifts 2 and 4.
    // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
    // If it doesn't get set to anything (i.e. it's not a match currently) then I set it so the
    // default is red goes first.
    if (!DriverStation.getGameSpecificMessage().isEmpty()
        && !(DriverStation.getGameSpecificMessage() == null)) {
      // in case someone forgets to capitalize the r...
      firstAllianceIsRed = !(DriverStation.getGameSpecificMessage().toUpperCase().charAt(0) == 'R');
    }

    boolean redHubActive = !(timeframe == "Invalid Timeframe");
    boolean blueHubActive = !(timeframe == "Invalid Timeframe");
    if (timeframe.startsWith("Shift")) {
      redHubActive = firstAllianceIsRed ^ (timeframe == "Shift 2" || timeframe == "Shift 4");
      blueHubActive = !redHubActive;
    }
    Logger.recordOutput("Red Hub Active", redHubActive);
    Logger.recordOutput("Blue Hub Active", blueHubActive);

    ourHubActive = RobotState.isAllianceRed() ? redHubActive : blueHubActive;
    Logger.recordOutput("Our Hub Active", ourHubActive);

    if (timeframe == "Auto") {
      timeUntilOurHubShifts = matchTime;
    } else if (timeframe == "Transition Shift") {
      timeUntilOurHubShifts =
          matchTime - (firstAllianceIsRed ^ RobotState.isAllianceRed() ? 130 : 105);
    } else if (timeframe == "Shift 1") {
      timeUntilOurHubShifts = matchTime - 105;
    } else if (timeframe == "Shift 2") {
      timeUntilOurHubShifts = matchTime - 80;
    } else if (timeframe == "Shift 3") {
      timeUntilOurHubShifts = matchTime - 55;
    } else if (timeframe == "Shift 4") {
      timeUntilOurHubShifts =
          matchTime - (firstAllianceIsRed ^ RobotState.isAllianceRed() ? 0 : 30);
    } else if (timeframe == "End Game") {
      timeUntilOurHubShifts = matchTime;
    } else {
      timeUntilOurHubShifts = -1;
    }

    Logger.recordOutput("Time Until Our Hub Shifts", timeUntilOurHubShifts);
  }

  public double getTimeUntilOurHubShifts() {
    return timeUntilOurHubShifts;
  }

  public boolean isOurHubActive() {
    return ourHubActive;
  }

  public double getTime() {
    return matchTime;
  }
}
