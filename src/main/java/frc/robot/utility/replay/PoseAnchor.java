package frc.robot.utility.replay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * Periodically snaps the simulated robot back to the pose the real robot held, and logs how far it
 * had drifted first.
 *
 * <p>Real matches involve contact the simulation does not model, so after autonomous the simulated
 * robot diverges. Left uncorrected it eventually wedges against a wall and draws current that never
 * happened, corrupting the power measurements this whole exercise exists to compare.
 *
 * <p>The interval defaults to 10 seconds rather than something tight. A short interval would keep
 * auto-aim inputs accurate but would also continuously erase drift — hiding a drivetrain model that
 * is genuinely wrong. At 10 seconds the drift accumulated before each correction is published as
 * {@code Replay/Anchor Error}, making drivetrain fidelity directly measurable.
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
   * Teleport the simulated drivetrain to {@code target}, keeping the gyro in step.
   *
   * <p>{@code setSimulationWorldPose} moves only the dyn4j body -- it does not touch {@code
   * GyroSimulation}. Without the matching {@code setRotation} the robot's believed heading stays
   * put while its actual heading jumps, so every field-relative driver command afterwards lands in
   * a rotated frame.
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

    // maple-sim's setSimulationWorldPose zeroes linear velocity:
    //     super.transform.set(...); super.linearVelocity.set(0, 0);
    // Teleporting without restoring velocity would stop the robot dead 14 times a
    // match and force the drive motors to re-accelerate from standstill each time,
    // injecting current spikes that never happened. Capture the speeds first and
    // put them back afterwards so only position is corrected.
    //
    // Field-relative is the correct frame here: setRobotSpeeds converts through
    // toDyn4jLinearVelocity into dyn4j world coordinates.
    ChassisSpeeds speeds = driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    seedPose(driveSimulation, target);
    driveSimulation.setRobotSpeeds(speeds);

    lastAnchorSeconds = now;
    anchoredAtTeleopStart = true;
  }
}
