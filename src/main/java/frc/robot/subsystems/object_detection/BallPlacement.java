package frc.robot.subsystems.object_detection;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;

/**
 * Turns the bearings a detector reports into balls on the field, the same way for every detector.
 *
 * <p>A detector hands over one direction per ball, in the camera frame, and the time its frame was
 * captured. This looks up where the robot was at that instant, follows each direction down to the
 * plane a resting ball's centre sits on, and drops anything that lands off the field. {@link
 * ObjectDetectionIOCoprocessor} in simulation and {@link ObjectDetectionIOLimelight} on the robot
 * both go through it, so a ball seen along a given bearing lands in the same place whichever of
 * them saw it.
 */
final class BallPlacement {
  private BallPlacement() {}

  /** One frame, already placed on the field. An IO holds its latest until a newer one arrives. */
  record Frame(
      double timestamp,
      double latencySeconds,
      Pose3d cameraPose,
      Translation2d[] balls,
      double[] pixelU,
      double[] pixelV) {
    static final Frame NONE =
        new Frame(
            Double.NEGATIVE_INFINITY,
            0.0,
            Pose3d.kZero,
            new Translation2d[0],
            new double[0],
            new double[0]);

    /** Copies this frame into the inputs. Connection and drop counts are the IO's own. */
    void writeTo(ObjectDetectionIO.ObjectDetectionIOInputs inputs) {
      inputs.frameTimestamp = Double.isFinite(timestamp) ? timestamp : 0.0;
      inputs.cameraPose = cameraPose;
      inputs.latencySeconds = latencySeconds;
      inputs.ballPositions = balls;
      inputs.pixelU = pixelU;
      inputs.pixelV = pixelV;
    }
  }

  /**
   * Places one frame, if it is recent enough to place.
   *
   * @param captureTime robot time the frame was captured, in the pose estimator's clock
   * @param now robot time the frame arrived
   * @param cameraRays one direction per detection, in the WPILib camera frame; length is ignored
   * @param pixelU box centres as the detector reported them, kept for the log
   * @param pixelV as {@code pixelU}
   * @param robotPoseAt the pose estimator's history, {@code RobotState.getEstimatedPoseAt}
   * @return the placed frame, or empty when it is older than {@link
   *     ObjectDetectionConstants#MAX_FRAME_AGE_SEC} or from before the pose history begins
   */
  static Optional<Frame> place(
      double captureTime,
      double now,
      Translation3d[] cameraRays,
      double[] pixelU,
      double[] pixelV,
      DoubleFunction<Optional<Pose2d>> robotPoseAt) {
    if (now - captureTime > ObjectDetectionConstants.MAX_FRAME_AGE_SEC) {
      return Optional.empty();
    }
    Optional<Pose2d> robotAtCapture = robotPoseAt.apply(captureTime);
    if (robotAtCapture.isEmpty()) {
      return Optional.empty();
    }
    Pose3d camera =
        new Pose3d(robotAtCapture.get()).transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

    List<Translation2d> balls = new ArrayList<>(cameraRays.length);
    for (Translation3d ray : cameraRays) {
      projectToBallPlane(camera, ray)
          .filter(BallPlacement::isOnField)
          .ifPresent(ball -> balls.add(ball.toTranslation2d()));
    }
    return Optional.of(
        new Frame(
            captureTime,
            now - captureTime,
            camera,
            balls.toArray(Translation2d[]::new),
            pixelU,
            pixelV));
  }

  /**
   * Follows a camera ray down to the plane a resting ball's centre sits on.
   *
   * <p>The ball's radius is known, so so is the height of its centre, and a bearing alone fixes
   * where along the ray the ball must be. That reads only the box's centre, which a box merged
   * across two neighbours or clipped by the frame edge barely moves, rather than its size, which
   * both of those wreck.
   *
   * @param cameraPose where the camera was when it saw the ball
   * @param cameraRay the direction through the box centre, in the camera frame
   * @return the ball centre, or empty when the ray never reaches the plane or meets it outside the
   *     range a ball is reported over
   */
  static Optional<Translation3d> projectToBallPlane(Pose3d cameraPose, Translation3d cameraRay) {
    Translation3d ray = cameraRay.div(cameraRay.getNorm()).rotateBy(cameraPose.getRotation());
    double drop = ObjectDetectionConstants.BALL_RADIUS_M - cameraPose.getZ();
    if (ray.getZ() >= -1e-9 || drop >= 0.0) {
      return Optional.empty(); // at or above the horizon, or a camera no higher than the ball
    }
    double range = drop / ray.getZ();
    if (range < ObjectDetectionConstants.MIN_RANGE_M
        || range > ObjectDetectionConstants.MAX_RANGE_M) {
      return Optional.empty();
    }
    return Optional.of(cameraPose.getTranslation().plus(ray.times(range)));
  }

  /** Fuel cannot be outside the field, so a projection landing there is not a ball on the floor. */
  private static boolean isOnField(Translation3d ball) {
    double tolerance = ObjectDetectionConstants.FIELD_BOUNDS_TOLERANCE_M;
    return ball.getX() >= -tolerance
        && ball.getX() <= FlippingUtil.fieldSizeX + tolerance
        && ball.getY() >= -tolerance
        && ball.getY() <= FlippingUtil.fieldSizeY + tolerance;
  }
}
