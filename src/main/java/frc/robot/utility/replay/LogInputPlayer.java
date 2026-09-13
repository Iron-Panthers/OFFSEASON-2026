package frc.robot.utility.replay;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import org.littletonrobotics.junction.Logger;

/**
 * Injects a real match's logged driver inputs into simulation, one loop period per step. SIM only.
 */
public final class LogInputPlayer {

  /** Axis and button counts to advertise for each simulated controller. */
  private static final int AXIS_COUNT = 6;

  private static final int BUTTON_COUNT = 10;

  private final MatchInputs inputs;
  private final GenericHIDSim[] controllers = new GenericHIDSim[MatchInputs.PORT_COUNT];

  private double elapsedSeconds = 0.0;
  private boolean finished = false;

  public LogInputPlayer(MatchInputs inputs) {
    this.inputs = inputs;

    for (int port = 0; port < MatchInputs.PORT_COUNT; port++) {
      GenericHIDSim controller = new GenericHIDSim(port);
      controller.setAxisCount(AXIS_COUNT);
      controller.setButtonCount(BUTTON_COUNT);
      controller.setPOVCount(1);
      controller.setPOV(-1);
      controller.notifyNewData();
      controllers[port] = controller;
    }

    // Alliance station is fixed for the match, so apply it once up front.
    Long station = inputs.allianceStation().valueAt(inputs.matchStartSeconds());
    if (station != null) {
      AllianceStationID[] ids = AllianceStationID.values();
      int ordinal = (int) (long) station;
      if (ordinal >= 0 && ordinal < ids.length) {
        DriverStationSim.setAllianceStationId(ids[ordinal]);
      }
    }
    if (inputs.gameMessage() != null) {
      DriverStationSim.setGameSpecificMessage(inputs.gameMessage());
    }
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
  }

  /** The logged robot pose at the moment the match was enabled, as {x, y, theta}, or null. */
  public double[] startPose() {
    return inputs.estimatedPose().valueAt(inputs.matchStartSeconds());
  }

  /** The auto selected in the source log, or null if it recorded none. */
  public String autoName() {
    return inputs.autoName();
  }

  /** Seconds of match time replayed so far. */
  public double elapsedSeconds() {
    return elapsedSeconds;
  }

  /**
   * Skip autonomous and start the replay where teleop began.
   *
   * @return the {x, y, theta} pose at teleop entry, or null if the log has no pose data there
   */
  public double[] seekToTeleop() {
    double teleopStart = inputs.autonomous().firstTimestampWhere(value -> !value);
    if (Double.isNaN(teleopStart) || teleopStart < inputs.matchStartSeconds()) {
      // The log never left autonomous; nothing to seek to.
      return null;
    }
    elapsedSeconds = teleopStart - inputs.matchStartSeconds();
    return inputs.estimatedPose().valueAt(teleopStart);
  }

  /** True once the replay has passed the end of the logged match. */
  public boolean isFinished() {
    return finished;
  }

  /** True while the source log was in autonomous at the current replay position. */
  public boolean isAutonomous() {
    return Boolean.TRUE.equals(inputs.autonomous().valueAt(logTime(), false));
  }

  /** The logged robot pose at the current replay position, as {x, y, theta}, or null. */
  public double[] loggedPose() {
    return inputs.estimatedPose().valueAt(logTime());
  }

  /** Current position in the source log's own time base. */
  private double logTime() {
    return inputs.matchStartSeconds() + elapsedSeconds;
  }

  /**
   * Advance the replay by {@code dtSeconds} and push the resulting driver station and joystick
   * state into simulation.
   */
  public void step(double dtSeconds) {
    if (finished) {
      return;
    }
    elapsedSeconds += dtSeconds;
    double now = logTime();

    if (elapsedSeconds > inputs.durationSeconds()) {
      finished = true;
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      return;
    }

    for (int port = 0; port < MatchInputs.PORT_COUNT; port++) {
      GenericHIDSim controller = controllers[port];

      float[] axisValues = inputs.joystickAxes()[port].valueAt(now);
      if (axisValues != null) {
        for (int axis = 0; axis < Math.min(axisValues.length, AXIS_COUNT); axis++) {
          controller.setRawAxis(axis, axisValues[axis]);
        }
      }

      Long bitfield = inputs.joystickButtons()[port].valueAt(now);
      if (bitfield != null) {
        for (int button = 1; button <= BUTTON_COUNT; button++) {
          // WPILib numbers buttons from 1; the log packs button N into bit N-1.
          controller.setRawButton(button, (bitfield & (1L << (button - 1))) != 0);
        }
      }

      long[] povValues = inputs.joystickPovs()[port].valueAt(now);
      controller.setPOV(0, povValues != null && povValues.length > 0 ? (int) povValues[0] : -1);

      controller.notifyNewData();
    }

    // ShootCommandFactory gates on match time; without it the DS reports -1 and the gate passes.
    Double matchTimeNow = inputs.matchTime().valueAt(now);
    if (matchTimeNow != null) {
      DriverStationSim.setMatchTime(matchTimeNow);
    }

    DriverStationSim.setEnabled(Boolean.TRUE.equals(inputs.enabled().valueAt(now, false)));
    DriverStationSim.setAutonomous(Boolean.TRUE.equals(inputs.autonomous().valueAt(now, false)));
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();

    Logger.recordOutput("Replay/Elapsed Seconds", elapsedSeconds);
    Logger.recordOutput("Replay/Log Time Seconds", now);
  }
}
