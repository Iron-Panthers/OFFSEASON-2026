package frc.robot.utility.ab;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import frc.robot.Constants;
import frc.robot.RobotSimState;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.utility.FuelSim;
import frc.robot.utility.replay.PoseAnchor;
import org.littletonrobotics.junction.Logger;

/**
 * Repeatable scoring benchmarks for A/B testing a code change.
 *
 * <p>Selected with {@code -Pbench=<name>}. Two benchmarks need robot-side setup and live here:
 *
 * <ul>
 *   <li><b>drain</b> — preload the robot with N fuel and hold the shoot button until it is empty.
 *       Measures sustained throughput: balls per second, accuracy under continuous feed, and how
 *       far the flywheel sags between shots. The benchmark for serializer, accelerator and flywheel
 *       changes.
 *   <li><b>shotgrid</b> — teleport to a series of distances from the hub, fire exactly one ball at
 *       each, and record whether it scored. Produces a hit-rate-versus-distance curve, which is the
 *       most direct measurement available of a shooter LUT or hood angle change, and gets far more
 *       information per second of wall clock than watching a full auto.
 * </ul>
 *
 * <p>The other two benchmarks the A/B runner offers need no code here, because the existing
 * headless modes already produce what they measure. {@code step} is a scripted teleop run whose
 * mechanism velocity traces are analysed for rise time and overshoot, and {@code path} is an
 * ordinary auto run scored on PathPlanner tracking error instead of on game outcome. Both are
 * recipes in the runner.
 *
 * <p>This class drives the simulated joysticks directly, so it takes the place of the scripted
 * teleop input thread rather than running alongside it — two writers on the same {@link
 * GenericHIDSim} would fight.
 */
public final class AbBench {

  private static final String BENCH_PROPERTY = "ab.bench";

  /** Driver A. Button 1 is auto-aim shoot, which both benchmarks hold. */
  private static final int DRIVER_PORT = 0;

  private static final int AUTO_AIM_SHOOT_BUTTON = 1;

  private static final String BENCH = System.getProperty(BENCH_PROPERTY, "").trim().toLowerCase();

  /** Fuel to preload for {@code drain}. */
  private static final int DRAIN_COUNT = Integer.getInteger("ab.bench.count", 40);

  /** Hard stop for {@code drain}, in case the robot cannot empty itself at all. */
  private static final double DRAIN_TIMEOUT_SEC =
      Double.parseDouble(System.getProperty("ab.bench.timeout", "45.0"));

  /**
   * Seconds spent aiming at each {@code shotgrid} pose before a ball is handed over.
   *
   * <p>This phase is not optional. After the first shot the flywheel is already at speed and the
   * shooter is still in SHOOT, so a ball provided at the same instant as the teleport is fired
   * immediately — before auto-aim has rotated the robot or the hood has retargeted for the new
   * distance. Every shot after the first then misses for a reason that has nothing to do with the
   * shooter, and the benchmark measures its own setup instead of the robot.
   */
  private static final double AIM_SEC =
      Double.parseDouble(System.getProperty("ab.bench.aim", "2.0"));

  /** Seconds after the ball is handed over, covering the shot and its flight to the hub. */
  private static final double FLIGHT_SEC =
      Double.parseDouble(System.getProperty("ab.bench.dwell", "2.5"));

  /** Distances from the hub centre, in metres, that {@code shotgrid} fires from. */
  private static final double[] SHOT_DISTANCES =
      parseDistances(System.getProperty("ab.bench.distances", "2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5"));

  private static GenericHIDSim driverHid;
  private static boolean started = false;
  private static boolean finished = false;
  private static double startSeconds = 0.0;

  // drain state
  private static double ranDrySeconds = -1.0;

  // shotgrid state
  private static int shotIndex = -1;
  private static double shotStartSeconds = 0.0;
  private static boolean ballReleased = false;
  private static int scoreBeforeShot = 0;
  private static int shotsBeforeShot = 0;
  private static int hits = 0;
  private static int fired = 0;

  private AbBench() {}

  private static double[] parseDistances(String raw) {
    String[] parts = raw.split(",");
    double[] out = new double[parts.length];
    for (int i = 0; i < parts.length; i++) {
      out[i] = Double.parseDouble(parts[i].trim());
    }
    return out;
  }

  /** True when a benchmark was requested and we are in simulation. */
  public static boolean isActive() {
    return !BENCH.isEmpty() && Constants.getRobotMode() == Constants.Mode.SIM;
  }

  /** True for the benchmarks that drive the robot themselves. */
  public static boolean drivesRobot() {
    return isActive() && (BENCH.equals("drain") || BENCH.equals("shotgrid"));
  }

  /** The selected benchmark name, or an empty string. */
  public static String name() {
    return BENCH;
  }

  /** True once the benchmark has run to completion and the log can be closed. */
  public static boolean isFinished() {
    return finished;
  }

  /** Alliance-relative hub score, which is what the benchmark is actually counting. */
  private static int currentScore() {
    return RobotState.getInstance().isAllianceRed()
        ? FuelSim.Hub.RED_HUB.getScore()
        : FuelSim.Hub.BLUE_HUB.getScore();
  }

  private static void holdShoot(boolean pressed) {
    driverHid.setRawButton(AUTO_AIM_SHOOT_BUTTON, pressed);
    driverHid.notifyNewData();
    DriverStationSim.notifyNewData();
  }

  /** Seconds since the logger started, which is the clock every log key is stamped against. */
  private static double nowSeconds() {
    return Logger.getTimestamp() / 1.0e6;
  }

  /**
   * Places the robot {@code distance} metres from the hub on the alliance side, facing it.
   *
   * <p>Both the physics body and the pose estimator are moved. Moving only the physics body would
   * leave the estimator believing the robot is elsewhere, and auto-aim would then aim at the hub
   * from a position the robot is not in — measuring the estimator's error rather than the shooter.
   */
  private static void teleportTo(double distance) {
    boolean red = RobotState.getInstance().isAllianceRed();
    var hub = red ? DriveConstants.RED_HUB_ORIGIN : DriveConstants.BLUE_HUB_ORIGIN;

    // Stand off on the side of the hub the alliance approaches from, facing inward.
    double x = red ? hub.getX() + distance : hub.getX() - distance;
    Rotation2d heading = red ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
    Pose2d target = new Pose2d(x, hub.getY(), heading);

    PoseAnchor.seedPose(RobotSimState.getInstance().getDriveSimulation(), target);
    RobotState.getInstance().resetPose(target);
  }

  /** Called from teleopInit once the robot is enabled. */
  public static void onTeleopInit() {
    if (!drivesRobot() || started) {
      return;
    }
    started = true;
    startSeconds = nowSeconds();

    driverHid = new GenericHIDSim(DRIVER_PORT);
    driverHid.setAxisCount(6);
    driverHid.setButtonCount(10);
    driverHid.setPOVCount(1);
    driverHid.setPOV(-1);
    driverHid.notifyNewData();
    DriverStationSim.notifyNewData();

    Logger.recordOutput("AB/Bench", BENCH);

    if (BENCH.equals("drain")) {
      RobotSimState.getInstance().setFuelCount(DRAIN_COUNT);
      Logger.recordOutput("AB/Drain/Preloaded", DRAIN_COUNT);
      holdShoot(true);
    }
  }

  /** Called every loop from robotPeriodic. Drives whichever benchmark is selected. */
  public static void periodic() {
    if (!drivesRobot() || !started || finished) {
      return;
    }
    double now = nowSeconds();

    switch (BENCH) {
      case "drain" -> drainPeriodic(now - startSeconds);
      case "shotgrid" -> shotGridPeriodic(now);
      default -> {
        // Unreachable: drivesRobot() gates this to the two benchmarks above.
      }
    }
  }

  /**
   * Holds the shoot button until the robot is empty.
   *
   * <p>Keeps running for a moment after the last ball leaves, so the final shot's flight and any
   * score it earns land inside the log rather than being cut off by shutdown.
   */
  private static void drainPeriodic(double elapsed) {
    int held = RobotSimState.getInstance().getFuelCount();
    Logger.recordOutput("AB/Drain/Elapsed", elapsed);
    Logger.recordOutput("AB/Drain/Held", held);

    if (held <= 0) {
      if (ranDrySeconds < 0.0) {
        ranDrySeconds = elapsed;
        Logger.recordOutput("AB/Drain/EmptyAt", elapsed);
      } else if (elapsed - ranDrySeconds > 1.5) {
        holdShoot(false);
        finished = true;
      }
      return;
    }

    if (elapsed > DRAIN_TIMEOUT_SEC) {
      System.err.println(
          "[AB] drain timed out with " + held + " fuel still held after " + elapsed + "s");
      Logger.recordOutput("AB/Drain/TimedOut", true);
      holdShoot(false);
      finished = true;
    }
  }

  /**
   * Teleports to each distance in turn and fires exactly one ball from each.
   *
   * <p>Each pose runs in two phases. During AIM the robot is held at the new position with the
   * shoot button down but <b>no fuel</b>, which lets auto-aim rotate and the hood retarget without
   * anything being launched. Only then is one ball handed over, and FLIGHT waits for it to reach
   * the hub. Without the split, every shot after the first fires on the teleport frame.
   */
  private static void shotGridPeriodic(double now) {
    if (shotIndex < 0) {
      beginShot(0, now);
      return;
    }

    double elapsed = now - shotStartSeconds;

    if (!ballReleased) {
      if (elapsed >= AIM_SEC) {
        RobotSimState.getInstance().setFuelCount(1);
        ballReleased = true;
        Logger.recordOutput("AB/ShotGrid/ReleasedAt", now);
      }
      return;
    }

    if (elapsed < AIM_SEC + FLIGHT_SEC) {
      return;
    }

    // Score the shot that just finished, then move on.
    boolean scored = currentScore() > scoreBeforeShot;
    // A pose where the robot never released the ball is a different failure from one where it
    // shot and missed -- out of range, or never satisfying its own ready-to-shoot condition.
    // Collapsing the two would hide a change that made the robot refuse shots it used to take.
    boolean tookShot = RobotSimState.getInstance().getShotsFired() > shotsBeforeShot;
    if (scored) {
      hits++;
    }
    if (tookShot) {
      fired++;
    }
    Logger.recordOutput("AB/ShotGrid/Shot" + shotIndex + "/Distance", SHOT_DISTANCES[shotIndex]);
    Logger.recordOutput("AB/ShotGrid/Shot" + shotIndex + "/Scored", scored);
    Logger.recordOutput("AB/ShotGrid/Shot" + shotIndex + "/Fired", tookShot);
    Logger.recordOutput("AB/ShotGrid/Hits", hits);
    Logger.recordOutput("AB/ShotGrid/Fired", fired);

    int next = shotIndex + 1;
    if (next >= SHOT_DISTANCES.length) {
      holdShoot(false);
      Logger.recordOutput("AB/ShotGrid/Attempts", SHOT_DISTANCES.length);
      Logger.recordOutput("AB/ShotGrid/HitRate", (double) hits / SHOT_DISTANCES.length);
      // Of the shots actually taken, how many went in. Separates aim from willingness to fire.
      Logger.recordOutput("AB/ShotGrid/Precision", fired == 0 ? 0.0 : (double) hits / fired);
      finished = true;
      return;
    }
    beginShot(next, now);
  }

  /** Moves to pose {@code index} and starts its aim phase with no fuel aboard. */
  private static void beginShot(int index, double now) {
    shotIndex = index;
    shotStartSeconds = now;
    ballReleased = false;

    teleportTo(SHOT_DISTANCES[index]);
    // Empty the robot so the already-spinning shooter has nothing to fire while it re-aims.
    RobotSimState.getInstance().setFuelCount(0);
    scoreBeforeShot = currentScore();
    shotsBeforeShot = RobotSimState.getInstance().getShotsFired();
    holdShoot(true);

    Logger.recordOutput("AB/ShotGrid/Index", index);
    Logger.recordOutput("AB/ShotGrid/CurrentDistance", SHOT_DISTANCES[index]);
  }
}
