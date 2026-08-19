package frc.robot.sim;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;

/**
 * Programmatic input injection for simulation tests.
 *
 * <p>Wraps WPILib's DriverStationSim and GenericHIDSim to set driver station state and joystick
 * inputs without opening the sim GUI. Use this in JUnit tests that extend {@link SimTestBase} to
 * script robot scenarios.
 *
 * <p>Controller ports: port 0 = driverA, port 1 = driverB (matches RobotContainer).
 */
public final class SimInputHarness {

  private SimInputHarness() {}

  // ─── Driver Station ───────────────────────────────────────────────────────

  /**
   * Set the robot's enabled state and match mode.
   *
   * @param enabled whether the robot is enabled
   * @param autonomous true for autonomous mode, false for teleop
   */
  public static void setEnabled(boolean enabled, boolean autonomous) {
    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();
  }

  /** Enable the robot in teleop mode. */
  public static void enableTeleop() {
    setEnabled(true, false);
  }

  /** Enable the robot in autonomous mode. */
  public static void enableAuto() {
    setEnabled(true, true);
  }

  /** Disable the robot. */
  public static void disable() {
    setEnabled(false, false);
  }

  // ─── Joystick Axes ────────────────────────────────────────────────────────

  /**
   * Set a joystick axis value.
   *
   * @param port controller port (0 = driverA, 1 = driverB)
   * @param axis axis index (see XboxController.Axis enum for indices)
   * @param value axis value in [-1.0, 1.0]
   */
  public static void setAxis(int port, int axis, double value) {
    GenericHIDSim hid = new GenericHIDSim(port);
    hid.setRawAxis(axis, value);
    hid.notifyNewData();
  }

  /** Zero all axes on a controller. */
  public static void zeroAxes(int port) {
    GenericHIDSim hid = new GenericHIDSim(port);
    for (int i = 0; i < 6; i++) {
      hid.setRawAxis(i, 0.0);
    }
    hid.notifyNewData();
  }

  // ─── Joystick Buttons ─────────────────────────────────────────────────────

  /**
   * Press (hold) a button.
   *
   * @param port controller port (0 = driverA, 1 = driverB)
   * @param button button number (1-indexed, matches WPILib button numbering)
   */
  public static void pressButton(int port, int button) {
    GenericHIDSim hid = new GenericHIDSim(port);
    hid.setRawButton(button, true);
    hid.notifyNewData();
  }

  /**
   * Release a button.
   *
   * @param port controller port
   * @param button button number (1-indexed)
   */
  public static void releaseButton(int port, int button) {
    GenericHIDSim hid = new GenericHIDSim(port);
    hid.setRawButton(button, false);
    hid.notifyNewData();
  }

  /**
   * Press and release a button (single click).
   *
   * @param port controller port
   * @param button button number (1-indexed)
   */
  public static void clickButton(int port, int button) {
    pressButton(port, button);
    releaseButton(port, button);
  }

  // ─── Xbox Button Indices ──────────────────────────────────────────────────

  // These match edu.wpi.first.wpilibj.XboxController.Button ordinals + 1
  public static final int XBOX_A = 1;
  public static final int XBOX_B = 2;
  public static final int XBOX_X = 3;
  public static final int XBOX_Y = 4;
  public static final int XBOX_LEFT_BUMPER = 5;
  public static final int XBOX_RIGHT_BUMPER = 6;
  public static final int XBOX_BACK = 7;
  public static final int XBOX_START = 8;
  public static final int XBOX_LEFT_STICK = 9;
  public static final int XBOX_RIGHT_STICK = 10;

  // Xbox axis indices
  public static final int XBOX_LEFT_X = 0;
  public static final int XBOX_LEFT_Y = 1;
  public static final int XBOX_LEFT_TRIGGER = 2;
  public static final int XBOX_RIGHT_TRIGGER = 3;
  public static final int XBOX_RIGHT_X = 4;
  public static final int XBOX_RIGHT_Y = 5;
}
