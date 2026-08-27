package frc.robot.sim;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;

/**
 * Base class for simulation-based JUnit tests. Initializes the WPILib HAL before any tests run.
 *
 * <p>Extend this class for any test that needs to interact with WPILib simulation APIs
 * (DriverStationSim, GenericHIDSim, motor sim classes, etc.).
 *
 * <p>Example:
 *
 * <pre>{@code
 * class MySubsystemTest extends SimTestBase {
 *   @Test
 *   void testIntakeDeployment() {
 *     SimInputHarness.setEnabled(true, false); // teleop
 *     SimInputHarness.stepLoops(50);           // run 1 second (50 loops at 20ms)
 *     // assert something about sim state
 *   }
 * }
 * }</pre>
 */
public abstract class SimTestBase {

  @BeforeAll
  static void initWPILib() {
    // Initialize the HAL with 500ms timeout, team number 0.
    // This must happen before any WPILib sim class is used.
    HAL.initialize(500, 0);
  }
}
