package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotSimState;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

/**
 * Scores each camera frame against where the simulation says the fuel actually is. SIM only.
 *
 * <p>Only informative when detections come from the rendered camera through {@link
 * ObjectDetectionIOCoprocessor}. Against {@link ObjectDetectionIOSim}, which reads the ground truth
 * directly, every number is perfect by construction.
 *
 * <p>Two sides, because they fail differently. Error and match fraction are about the detections
 * the robot has: whether each one is a real ball, and how far off it was placed, which is where a
 * wrong lens model or a wrong capture pose shows up. Recall is about the balls it missed, which is
 * the detector's problem rather than the geometry's. Recall counts every resting ball inside the
 * frustum, including ones hidden behind other balls, so it reads low in a dense pile by design.
 *
 * <p>Each frame is scored against the field as it was when the frame was captured, not when it
 * arrived. The pipeline takes most of a second, and in that time the intake has usually pushed the
 * nearest balls somewhere else, which would charge the geometry for the balls moving.
 */
public final class ObjectDetectionSimAccuracy {
  /** A detection this close to a real ball counts as having found it. About three ball radii. */
  private static final double MATCH_RADIUS_M = 0.25;

  /** Fuel with its centre above this is in flight or in a robot, not resting on the floor. */
  private static final double RESTING_HEIGHT_M = ObjectDetectionConstants.BALL_RADIUS_M + 0.05;

  /** Field history kept for scoring late frames. Longer than any frame the IO would accept. */
  private static final double HISTORY_SEC = ObjectDetectionConstants.MAX_FRAME_AGE_SEC + 0.5;

  /** The fuel and the true camera pose at one instant. */
  private record FieldState(Translation3d[] fuel, Pose3d camera) {}

  /** Keyed by the same stamp the renderer puts on the frame it draws from this state. */
  private final TreeMap<Double, FieldState> history = new TreeMap<>();

  private double lastScoredFrame = Double.NEGATIVE_INFINITY;

  /**
   * Records this instant and scores the latest frame, once. Call every loop from simulation, after
   * the physics step, which is when the renderer takes its snapshot.
   */
  public void update(ObjectDetection objectDetection) {
    double stamp = Timer.getTimestamp() + Constants.PERIODIC_LOOP_SEC;
    history.put(
        stamp,
        new FieldState(
            RobotSimState.getInstance().getFuelSim().getFuelPositions(),
            RobotSimState.getInstance()
                .getRobotFramePose3d()
                .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA)));
    history.headMap(stamp - HISTORY_SEC).clear();

    double frame = objectDetection.getLatestFrameTimestamp();
    if (frame <= lastScoredFrame) {
      return;
    }
    lastScoredFrame = frame;
    FieldState atCapture = nearest(frame);
    if (atCapture == null) {
      return;
    }

    Translation3d[] fuel = atCapture.fuel();
    List<Translation2d> resting = new ArrayList<>();
    for (Translation3d ball : fuel) {
      if (ball.getZ() <= RESTING_HEIGHT_M) {
        resting.add(ball.toTranslation2d());
      }
    }
    Translation2d[] detected = objectDetection.getVisibleBalls();

    double[] errors = new double[detected.length];
    int matched = 0;
    double matchedErrorSum = 0.0;
    for (int i = 0; i < detected.length; i++) {
      errors[i] = nearest(detected[i], resting);
      if (errors[i] <= MATCH_RADIUS_M) {
        matched++;
        matchedErrorSum += errors[i];
      }
    }

    // The true frustum, so recall measures what the detector missed and not pose estimate error.
    Pose3d camera = atCapture.camera();
    int inFrustum = 0;
    int found = 0;
    for (Translation3d ball : fuel) {
      if (ball.getZ() > RESTING_HEIGHT_M || !ObjectDetectionIOSim.isVisible(camera, ball)) {
        continue;
      }
      inFrustum++;
      if (nearest(ball.toTranslation2d(), Arrays.asList(detected)) <= MATCH_RADIUS_M) {
        found++;
      }
    }

    String prefix = "ObjectDetection/Sim Accuracy/";
    Logger.recordOutput(prefix + "Detected Count", detected.length);
    Logger.recordOutput(prefix + "In Frustum Count", inFrustum);
    Logger.recordOutput(
        prefix + "Matched Fraction",
        detected.length == 0 ? 1.0 : (double) matched / detected.length);
    Logger.recordOutput(prefix + "Recall", inFrustum == 0 ? 1.0 : (double) found / inFrustum);
    Logger.recordOutput(
        prefix + "Matched Mean Error (m)", matched == 0 ? 0.0 : matchedErrorSum / matched);
    Logger.recordOutput(prefix + "Median Error (m)", median(errors));
  }

  /** The recorded instant closest to a frame's capture stamp. */
  private FieldState nearest(double stamp) {
    Map.Entry<Double, FieldState> below = history.floorEntry(stamp);
    Map.Entry<Double, FieldState> above = history.ceilingEntry(stamp);
    if (below == null) {
      return above == null ? null : above.getValue();
    }
    if (above == null) {
      return below.getValue();
    }
    return stamp - below.getKey() <= above.getKey() - stamp ? below.getValue() : above.getValue();
  }

  private static double nearest(Translation2d point, List<Translation2d> candidates) {
    double best = Double.POSITIVE_INFINITY;
    for (Translation2d candidate : candidates) {
      best = Math.min(best, point.getDistance(candidate));
    }
    return best;
  }

  private static double median(double[] values) {
    if (values.length == 0) {
      return 0.0;
    }
    double[] sorted = values.clone();
    Arrays.sort(sorted);
    int middle = sorted.length / 2;
    return sorted.length % 2 == 1 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) / 2.0;
  }
}
