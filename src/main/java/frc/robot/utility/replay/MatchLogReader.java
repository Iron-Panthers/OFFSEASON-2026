package frc.robot.utility.replay;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;

/**
 * Parses a real match {@code .wpilog} into the driver-station and joystick timelines needed to
 * replay it in simulation.
 *
 * <p>Pure parsing: no HAL, no WPILib runtime state. Safe to unit test.
 */
public final class MatchLogReader {

  private static final String KEY_ENABLED = "DriverStation/Enabled";
  private static final String KEY_AUTONOMOUS = "DriverStation/Autonomous";
  private static final String KEY_STATION = "DriverStation/AllianceStation";
  private static final String KEY_POSE = "RealOutputs/Robot State/Estimated Pose";
  private static final String KEY_CHOOSER = "NetworkInputs/SmartDashboard/Auto Chooser";
  private static final String KEY_MATCH_TIME = "DriverStation/MatchTime";
  private static final String KEY_GAME_MESSAGE = "DriverStation/GameSpecificMessage";

  /** A struct:Pose2d payload is three little-endian doubles: x, y, then theta in radians. */
  private static final int POSE2D_BYTES = 24;

  private MatchLogReader() {}

  /**
   * Read {@code path} and extract everything needed to replay the match.
   *
   * @throws IOException if the file cannot be read
   * @throws IllegalStateException if the log never shows the robot enabled
   */
  @SuppressWarnings("unchecked")
  public static MatchInputs read(String path) throws IOException {
    DataLogReader reader = new DataLogReader(path);
    if (!reader.isValid()) {
      throw new IOException("Not a valid wpilog file: " + path);
    }

    Map<Integer, String> entryNames = new HashMap<>();

    LogTimeline.Builder<float[]>[] axes = new LogTimeline.Builder[MatchInputs.PORT_COUNT];
    LogTimeline.Builder<Long>[] buttons = new LogTimeline.Builder[MatchInputs.PORT_COUNT];
    LogTimeline.Builder<long[]>[] povs = new LogTimeline.Builder[MatchInputs.PORT_COUNT];
    for (int port = 0; port < MatchInputs.PORT_COUNT; port++) {
      axes[port] = LogTimeline.builder();
      buttons[port] = LogTimeline.builder();
      povs[port] = LogTimeline.builder();
    }

    LogTimeline.Builder<Boolean> enabled = LogTimeline.builder();
    LogTimeline.Builder<Boolean> autonomous = LogTimeline.builder();
    LogTimeline.Builder<Long> station = LogTimeline.builder();
    LogTimeline.Builder<double[]> pose = LogTimeline.builder();
    LogTimeline.Builder<Double> matchTime = LogTimeline.builder();
    String autoName = null;
    String gameMessage = null;

    for (DataLogRecord record : reader) {
      if (record.isControl()) {
        if (record.isStart()) {
          DataLogRecord.StartRecordData start = record.getStartData();
          // Some producers write keys with a leading slash; normalise it away.
          entryNames.put(
              start.entry, start.name.startsWith("/") ? start.name.substring(1) : start.name);
        }
        continue;
      }

      String name = entryNames.get(record.getEntry());
      if (name == null) {
        continue;
      }
      double seconds = record.getTimestamp() / 1_000_000.0;

      switch (name) {
        case KEY_ENABLED -> enabled.add(seconds, record.getBoolean());
        case KEY_AUTONOMOUS -> autonomous.add(seconds, record.getBoolean());
        case KEY_STATION -> station.add(seconds, record.getInteger());
        case KEY_CHOOSER -> autoName = record.getString();
        case KEY_MATCH_TIME -> matchTime.add(seconds, record.getDouble());
        case KEY_GAME_MESSAGE -> gameMessage = record.getString();
        case KEY_POSE -> {
          double[] parsed = parsePose2d(record.getRaw());
          if (parsed != null) {
            pose.add(seconds, parsed);
          }
        }
        default -> {
          int port = joystickPort(name);
          if (port >= 0 && port < MatchInputs.PORT_COUNT) {
            if (name.endsWith("/AxisValues")) {
              axes[port].add(seconds, record.getFloatArray());
            } else if (name.endsWith("/ButtonValues")) {
              buttons[port].add(seconds, record.getInteger());
            } else if (name.endsWith("/POVs")) {
              povs[port].add(seconds, record.getIntegerArray());
            }
          }
        }
      }
    }

    LogTimeline<Boolean> enabledTimeline = enabled.build();
    double start = enabledTimeline.firstTimestampWhere(value -> value);
    if (Double.isNaN(start)) {
      throw new IllegalStateException(
          "Log never shows DriverStation/Enabled = true, so there is no match to replay: " + path);
    }
    // Match end is the last disable *after* the first enable, matching how
    // scripts/log_compare.py bounds its comparison window. Falling back to the
    // last record keeps a log that was never disabled from ending at t=start.
    double end = enabledTimeline.lastTimestampWhere(value -> !value);
    if (Double.isNaN(end) || end <= start) {
      end = enabledTimeline.lastTimestamp();
    }

    LogTimeline<float[]>[] axesBuilt = new LogTimeline[MatchInputs.PORT_COUNT];
    LogTimeline<Long>[] buttonsBuilt = new LogTimeline[MatchInputs.PORT_COUNT];
    LogTimeline<long[]>[] povsBuilt = new LogTimeline[MatchInputs.PORT_COUNT];
    for (int port = 0; port < MatchInputs.PORT_COUNT; port++) {
      axesBuilt[port] = axes[port].build();
      buttonsBuilt[port] = buttons[port].build();
      povsBuilt[port] = povs[port].build();
    }

    return new MatchInputs(
        axesBuilt,
        buttonsBuilt,
        povsBuilt,
        enabledTimeline,
        autonomous.build(),
        station.build(),
        pose.build(),
        matchTime.build(),
        autoName,
        gameMessage,
        start,
        end);
  }

  /**
   * Extract the port number from a key like {@code DriverStation/Joystick0/AxisValues}, or -1 if
   * the key is not a joystick key.
   */
  private static int joystickPort(String key) {
    final String prefix = "DriverStation/Joystick";
    if (!key.startsWith(prefix) || key.length() <= prefix.length()) {
      return -1;
    }
    char digit = key.charAt(prefix.length());
    return Character.isDigit(digit) ? digit - '0' : -1;
  }

  /** Decode a WPILib struct:Pose2d payload into {x, y, theta}, or null if it is malformed. */
  private static double[] parsePose2d(byte[] raw) {
    if (raw.length < POSE2D_BYTES) {
      return null;
    }
    ByteBuffer buffer = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN);
    return new double[] {buffer.getDouble(), buffer.getDouble(), buffer.getDouble()};
  }
}
