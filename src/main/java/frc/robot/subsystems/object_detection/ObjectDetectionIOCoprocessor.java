package frc.robot.subsystems.object_detection;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.utility.CameraIntrinsics;
import frc.robot.utility.coprocessor.CoprocessorModule;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Balls found by the coprocessor's detector, read off NetworkTables and placed on the field.
 *
 * <p>The coprocessor reports where each ball's box sits in the image and when the image was taken,
 * and nothing that needs robot state. Everything that turns a pixel into a field position happens
 * here, from the constants: the lens in {@link ObjectDetectionConstants#CAMERA_INTRINSICS}, warp
 * included; the mounting in {@link ObjectDetectionConstants#ROBOT_TO_CAMERA}; and the robot's pose
 * at the instant the frame was captured, not the instant it arrived. Each box centre becomes a ray,
 * and the ray is followed down to the plane a resting ball's centre sits on.
 *
 * <p>In simulation the image is rendered, and the render carries the robot time its scene was
 * captured at, so the pose lookup is exact. Against a real camera no such stamp exists and the
 * capture time is estimated from the coprocessor's reported latency instead.
 */
public class ObjectDetectionIOCoprocessor implements ObjectDetectionIO {
  private final IntegerSubscriber sequenceSub;
  private final IntegerSubscriber sequenceBeginSub;
  private final DoubleArraySubscriber uSub;
  private final DoubleArraySubscriber vSub;
  private final IntegerSubscriber frameWidthSub;
  private final IntegerSubscriber frameHeightSub;
  private final DoubleSubscriber frameTimestampSub;
  private final DoubleSubscriber latencyMsSub;
  private final BooleanSubscriber connectedSub;

  private long lastSequence = 0;
  private Frame latest = Frame.NONE;

  /** Robot time a frame from a connected camera last arrived, whether or not it was used. */
  private double lastArrival = Double.NEGATIVE_INFINITY;

  private long framesDropped = 0;

  /** One frame, already placed on the field. Held until a newer one replaces it. */
  private record Frame(
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
  }

  /**
   * @param cameraIndex the camera the detector publishes under, {@code CoprocessorModule.camera()}
   */
  public ObjectDetectionIOCoprocessor(int cameraIndex) {
    NetworkTable table =
        NetworkTableInstance.getDefault()
            .getTable(CoprocessorModule.OBJDETECT.tablePath(cameraIndex));
    sequenceSub = table.getIntegerTopic("sequence").subscribe(0);
    sequenceBeginSub = table.getIntegerTopic("sequenceBegin").subscribe(0);
    uSub = table.getDoubleArrayTopic("u").subscribe(new double[0]);
    vSub = table.getDoubleArrayTopic("v").subscribe(new double[0]);
    frameWidthSub = table.getIntegerTopic("frameWidth").subscribe(0);
    frameHeightSub = table.getIntegerTopic("frameHeight").subscribe(0);
    frameTimestampSub = table.getDoubleTopic("frameTimestamp").subscribe(Double.NaN);
    latencyMsSub = table.getDoubleTopic("latencyMs").subscribe(0.0);
    connectedSub = table.getBooleanTopic("connected").subscribe(false);
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    long sequence = sequenceSub.get();
    if (sequence != lastSequence) {
      readFrame(sequence);
    }

    // Judged on arrival, not capture. A pipeline with most of a second of latency still delivers
    // a frame every few loops, and judging it on capture would call it disconnected between them.
    double now = Timer.getTimestamp();
    inputs.connected =
        connectedSub.get() && now - lastArrival <= ObjectDetectionConstants.FRAME_TIMEOUT_SEC;
    inputs.framesDropped = framesDropped;
    inputs.frameTimestamp = Double.isFinite(latest.timestamp()) ? latest.timestamp() : 0.0;
    inputs.cameraPose = latest.cameraPose();
    inputs.latencySeconds = latest.latencySeconds();
    inputs.ballPositions = latest.balls();
    inputs.pixelU = latest.pixelU();
    inputs.pixelV = latest.pixelV();
  }

  /**
   * Reads one frame and places it, if it is whole and recent enough.
   *
   * <p>The coprocessor writes {@code sequenceBegin} first and {@code sequence} last. Having read
   * {@code sequence}, if {@code sequenceBegin} still matches after everything else has been read,
   * no newer frame started arriving in between and every array belongs to this frame. If it does
   * not match, the next loop tries again with the newer frame; they arrive a tenth of a second
   * apart, so nothing is lost by waiting.
   */
  private void readFrame(long sequence) {
    double[] u = uSub.get();
    double[] v = vSub.get();
    long width = frameWidthSub.get();
    long height = frameHeightSub.get();
    double stampedCapture = frameTimestampSub.get();
    double latencyMs = latencyMsSub.get();
    boolean sourceConnected = connectedSub.get();
    if (sequenceBeginSub.get() != sequence) {
      return; // torn: a newer frame was arriving while this one was read. Retried next loop.
    }
    lastSequence = sequence;

    // Published while the coprocessor waits on its camera. There is no image behind it, so it
    // says nothing about which balls are on the field.
    if (!sourceConnected || u.length != v.length) {
      return;
    }

    double now = Timer.getTimestamp();
    lastArrival = now;
    double capture = Double.isFinite(stampedCapture) ? stampedCapture : now - latencyMs / 1000.0;
    if (now - capture > ObjectDetectionConstants.MAX_FRAME_AGE_SEC) {
      framesDropped++;
      return;
    }
    Optional<Pose2d> robotAtCapture = RobotState.getInstance().getEstimatedPoseAt(capture);
    if (robotAtCapture.isEmpty()) {
      framesDropped++;
      return;
    }
    Pose3d camera =
        new Pose3d(robotAtCapture.get()).transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

    CameraIntrinsics lens =
        width > 0 && height > 0
            ? ObjectDetectionConstants.CAMERA_INTRINSICS.scaledTo((int) width, (int) height)
            : ObjectDetectionConstants.CAMERA_INTRINSICS;

    List<Translation2d> balls = new ArrayList<>(u.length);
    for (int i = 0; i < u.length; i++) {
      projectToBallPlane(camera, lens.pixelToRay(u[i], v[i]))
          .filter(ObjectDetectionIOCoprocessor::isOnField)
          .ifPresent(ball -> balls.add(ball.toTranslation2d()));
    }

    latest =
        new Frame(
            capture,
            now - capture,
            camera,
            balls.toArray(Translation2d[]::new),
            u.clone(),
            v.clone());
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
