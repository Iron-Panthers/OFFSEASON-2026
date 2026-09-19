package frc.robot.utility.replay;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.util.datalog.DataLogWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

/**
 * Extends {@link frc.robot.sim.SimTestBase} because {@link DataLogWriter} writes through
 * DataLogJNI, which needs the wpiutil native library loaded. {@link
 * edu.wpi.first.util.datalog.DataLogReader} is pure Java, so reading works either way — only
 * writing the synthetic fixtures needs the HAL initialised.
 */
class MatchLogReaderTest extends frc.robot.sim.SimTestBase {

  private static final java.util.concurrent.atomic.AtomicInteger FIXTURE_SEQ =
      new java.util.concurrent.atomic.AtomicInteger();

  private static final String WORLDS_Q54 =
      "C:\\Users\\bruce\\Downloads\\LOGS-2026-main\\LOGS-2026-main\\Worlds\\"
          + "akit_26-04-30_14-50-56_johnson_q54.wpilog";

  /**
   * Fixtures live under build/ rather than in a JUnit {@code @TempDir}: on Windows the native
   * DataLog file handle is not released when the writer closes, so JUnit's temp-directory cleanup
   * fails and reports that IOException as a test failure.
   */
  private static Path fixture(String name) throws IOException {
    Path dir = Path.of("build", "test-fixtures");
    Files.createDirectories(dir);
    Path file = dir.resolve(name);
    Files.deleteIfExists(file);
    return file;
  }

  /**
   * Writes a minimal but realistic match log: idle, enable, auto, teleop, disable.
   *
   * <p>Each call gets a unique filename. Sharing one would be flaky on Windows, where the previous
   * test's native file handle may still hold a lock when the next test tries to overwrite it.
   */
  private Path writeSyntheticLog() throws IOException {
    Path file = fixture("synthetic-" + FIXTURE_SEQ.incrementAndGet() + ".wpilog");
    try (DataLogWriter log = new DataLogWriter(file.toString())) {
      int enabled = log.start("DriverStation/Enabled", "boolean", "");
      int auto = log.start("DriverStation/Autonomous", "boolean", "");
      int station = log.start("DriverStation/AllianceStation", "int64", "");
      int axes = log.start("DriverStation/Joystick0/AxisValues", "float[]", "");
      int buttons = log.start("DriverStation/Joystick0/ButtonValues", "int64", "");
      int povs = log.start("DriverStation/Joystick0/POVs", "int64[]", "");
      int chooser = log.start("NetworkInputs/SmartDashboard/Auto Chooser", "string", "");

      // Timestamps are microseconds in the wpilog format.
      log.appendBoolean(enabled, false, 1_000_000L);
      log.appendInteger(station, 1L, 1_000_000L);
      log.appendString(chooser, "2x4TRight", 1_000_000L);

      log.appendBoolean(enabled, true, 10_000_000L); // match starts at t=10s
      log.appendBoolean(auto, true, 10_000_000L);
      log.appendFloatArray(axes, new float[] {0.0f, -0.5f}, 10_000_000L);
      log.appendInteger(buttons, 0L, 10_000_000L);
      log.appendIntegerArray(povs, new long[] {-1L}, 10_000_000L);

      log.appendBoolean(auto, false, 25_000_000L); // teleop at t=25s
      log.appendFloatArray(axes, new float[] {0.25f, -1.0f}, 26_000_000L);
      log.appendInteger(buttons, 0b100000L, 26_000_000L); // button 6 held
      log.appendIntegerArray(povs, new long[] {180L}, 26_000_000L);

      log.appendBoolean(enabled, false, 40_000_000L); // match ends at t=40s

      // Trailing post-match records. Real robot logs always have these (q3
      // disables at 226s but the file runs to 282s), and without them the final
      // record of the file is the one under test -- which Java's DataLogReader
      // drops, while the Python reader still sees it.
      log.appendFloatArray(axes, new float[] {0.0f, 0.0f}, 45_000_000L);
      log.appendInteger(buttons, 0L, 45_000_000L);

      // DataLogWriter buffers; without an explicit flush the tail of the log is
      // still unwritten when we reopen the file to read it back.
      log.flush();
    }
    return file;
  }

  @Test
  void readsMatchBoundaries() throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog().toString());
    assertEquals(10.0, inputs.matchStartSeconds(), 1e-6);
    assertEquals(40.0, inputs.matchEndSeconds(), 1e-6);
    assertEquals(30.0, inputs.durationSeconds(), 1e-6);
  }

  @Test
  void readsAxesWithZeroOrderHold() throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog().toString());
    assertArrayEquals(new float[] {0.0f, -0.5f}, inputs.joystickAxes()[0].valueAt(20.0), 1e-6f);
    assertArrayEquals(new float[] {0.25f, -1.0f}, inputs.joystickAxes()[0].valueAt(30.0), 1e-6f);
  }

  @Test
  void readsButtonBitfield() throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog().toString());
    assertEquals(0L, inputs.joystickButtons()[0].valueAt(20.0));
    assertEquals(0b100000L, inputs.joystickButtons()[0].valueAt(30.0));
  }

  @Test
  void readsPovs() throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog().toString());
    assertEquals(-1L, inputs.joystickPovs()[0].valueAt(20.0)[0]);
    assertEquals(180L, inputs.joystickPovs()[0].valueAt(30.0)[0]);
  }

  @Test
  void readsAutonomousTransition() throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog().toString());
    assertTrue(inputs.autonomous().valueAt(15.0));
    assertFalse(inputs.autonomous().valueAt(30.0));
  }

  @Test
  void readsAutoChooserName() throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog().toString());
    assertEquals("2x4TRight", inputs.autoName());
  }

  @Test
  void readsAllianceStation() throws IOException {
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog().toString());
    assertEquals(1L, inputs.allianceStation().valueAt(15.0));
  }

  @Test
  void missingFileThrows() {
    assertThrows(IOException.class, () -> MatchLogReader.read("does-not-exist.wpilog"));
  }

  @Test
  void logWithNoEnableThrows() throws IOException {
    Path file = fixture("never-enabled-" + FIXTURE_SEQ.incrementAndGet() + ".wpilog");
    try (DataLogWriter log = new DataLogWriter(file.toString())) {
      int enabled = log.start("DriverStation/Enabled", "boolean", "");
      log.appendBoolean(enabled, false, 1_000_000L);
      // Trailing record so the disable above is not the final record in the file.
      log.appendBoolean(enabled, false, 5_000_000L);
      log.flush();
    }
    // A log the robot was never enabled in has no match to replay; failing loudly
    // beats silently replaying an empty match.
    assertThrows(IllegalStateException.class, () -> MatchLogReader.read(file.toString()));
  }

  @Test
  void matchEndIgnoresPreMatchDisableRecords() throws IOException {
    // The pre-match disable at t=1s must not be mistaken for the match end.
    MatchInputs inputs = MatchLogReader.read(writeSyntheticLog().toString());
    assertEquals(40.0, inputs.matchEndSeconds(), 1e-6);
  }

  @Test
  void matchEndFallsBackWhenNeverDisabled() throws IOException {
    Path file = fixture("never-disabled-" + FIXTURE_SEQ.incrementAndGet() + ".wpilog");
    try (DataLogWriter log = new DataLogWriter(file.toString())) {
      int enabled = log.start("DriverStation/Enabled", "boolean", "");
      int filler = log.start("DriverStation/Autonomous", "boolean", "");
      log.appendBoolean(enabled, true, 10_000_000L);
      log.appendBoolean(filler, false, 30_000_000L);
      log.appendBoolean(filler, false, 31_000_000L);
      log.flush();
    }
    MatchInputs inputs = MatchLogReader.read(file.toString());
    assertEquals(10.0, inputs.matchStartSeconds(), 1e-6);
    // No disable ever arrives, so the end falls back rather than collapsing to
    // the start and yielding a zero-length match.
    assertEquals(10.0, inputs.matchEndSeconds(), 1e-6);
  }

  @Test
  void parsesRealWorldsLog() throws IOException {
    // The synthetic log proves well-formed input parses. This proves a 62 MB
    // file written by a real robot with 481 keys does too. Skips cleanly on a
    // machine without the log folder.
    Assumptions.assumeTrue(Files.exists(Path.of(WORLDS_Q54)));
    MatchInputs inputs = MatchLogReader.read(WORLDS_Q54);

    // q54 is a full match: roughly 165 s of enabled time.
    assertTrue(
        inputs.durationSeconds() > 150.0 && inputs.durationSeconds() < 180.0,
        "expected a full match, got " + inputs.durationSeconds() + "s");
    assertTrue(inputs.joystickAxes()[0].size() > 1000, "expected dense axis data");
    assertFalse(inputs.estimatedPose().isEmpty(), "expected pose data for anchoring");
  }
}
