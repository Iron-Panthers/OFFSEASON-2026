package frc.robot.utility.replay;

/**
 * Every driver-station and joystick timeline parsed out of a real match log, plus the match
 * boundaries needed to rebase time.
 *
 * <p>All timestamps are raw FPGA seconds as they appear in the source log. Consumers rebase against
 * {@link #matchStartSeconds()}.
 *
 * @param joystickAxes per-port axis values; index 0 is port 0 (driverA), index 1 is port 1
 *     (driverB)
 * @param joystickButtons per-port button bitfields — WPILib button N is bit {@code N-1}
 * @param joystickPovs per-port POV values in degrees, or -1 when centered
 * @param enabled DriverStation enable state
 * @param autonomous true during autonomous
 * @param allianceStation raw AllianceStation enum ordinal from the log
 * @param estimatedPose robot pose as {x metres, y metres, theta radians}
 * @param autoName the auto selected on the dashboard, or null if the log did not record one
 * @param matchStartSeconds timestamp of the first enable — the replay time origin
 * @param matchEndSeconds timestamp of the final disable
 */
public record MatchInputs(
    LogTimeline<float[]>[] joystickAxes,
    LogTimeline<Long>[] joystickButtons,
    LogTimeline<long[]>[] joystickPovs,
    LogTimeline<Boolean> enabled,
    LogTimeline<Boolean> autonomous,
    LogTimeline<Long> allianceStation,
    LogTimeline<double[]> estimatedPose,
    String autoName,
    double matchStartSeconds,
    double matchEndSeconds) {

  /** Number of joystick ports this replay carries. Ports 0 and 1 are driverA and driverB. */
  public static final int PORT_COUNT = 2;

  /** Total match length in seconds. */
  public double durationSeconds() {
    return matchEndSeconds - matchStartSeconds;
  }
}
