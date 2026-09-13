package frc.robot.utility.replay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * During teleop, periodically snaps the simulated robot to the logged pose so contact the sim does
 * not model cannot wedge it into a wall. Drift before each snap is logged as {@code Replay/Anchor
 * Error}.
 */
public final class PoseAnchor {

  private final SwerveDriveSimulation driveSimulation;
  private final double intervalSeconds;

  private double lastAnchorSeconds = Double.NEGATIVE_INFINITY;
  private boolean anchoredAtTeleopStart = false;

  /**
   * @param driveSimulation the maple-sim drivetrain to reposition
   * @param intervalSeconds seconds between corrections; zero or negative disables anchoring
   */
  public PoseAnchor(SwerveDriveSimulation driveSimulation, double intervalSeconds) {
    this.driveSimulation = driveSimulation;
    this.intervalSeconds = intervalSeconds;
  }

  /**
   * Teleport the drivetrain to {@code target}, moving the gyro sim too since maple-sim does not.
   */
  public static void seedPose(SwerveDriveSimulation driveSimulation, Pose2d target) {
    driveSimulation.setSimulationWorldPose(target);
    driveSimulation.getGyroSimulation().setRotation(target.getRotation());
  }

  /** True when anchoring is switched off entirely. */
  public boolean isDisabled() {
    return intervalSeconds <= 0.0;
  }

  /**
   * Consider anchoring at the current replay position.
   *
   * <p>Does nothing during autonomous — pose is trusted there and left free-running so the auto
   * segment stays a valid comparison.
   */
  public void update(LogInputPlayer player) {
    if (isDisabled() || player.isAutonomous()) {
      return;
    }

    double[] logged = player.loggedPose();
    if (logged == null) {
      return;
    }

    double now = player.elapsedSeconds();
    boolean teleopEntry = !anchoredAtTeleopStart;
    if (!teleopEntry && now - lastAnchorSeconds < intervalSeconds) {
      return;
    }

    Pose2d target = new Pose2d(logged[0], logged[1], new Rotation2d(logged[2]));
    Pose2d actual = driveSimulation.getSimulatedDriveTrainPose();

    // Publish the drift BEFORE correcting it — that gap is the measurement.
    if (!teleopEntry) {
      Logger.recordOutput(
          "Replay/Anchor Error/Translation Meters",
          actual.getTranslation().getDistance(target.getTranslation()));
      Logger.recordOutput(
          "Replay/Anchor Error/Rotation Degrees",
          Math.abs(actual.getRotation().minus(target.getRotation()).getDegrees()));
      Logger.recordOutput("Replay/Anchor Error/Window Seconds", now - lastAnchorSeconds);
    }

    // setSimulationWorldPose zeroes velocity; restore it so only position is corrected.
    ChassisSpeeds speeds = driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    seedPose(driveSimulation, target);
    driveSimulation.setRobotSpeeds(speeds);

    lastAnchorSeconds = now;
    anchoredAtTeleopStart = true;
  }
}
