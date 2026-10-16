package frc.robot.subsystems.object_detection;

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

/**
 * Balls found by the coprocessor's detector, read off NetworkTables and placed on the field.
 *
 * <p>The coprocessor reports where each ball's box sits in the image and when the image was taken,
 * and nothing that needs robot state. Everything that turns a pixel into a field position happens
 * on the robot. The lens in {@link ObjectDetectionConstants#CAMERA_INTRINSICS}, warp included,
 * turns each box centre into a ray, and {@link BallPlacement} follows the ray down to the plane a
 * resting ball's centre sits on, from the robot's pose at the instant the frame was captured, not
 * the instant it arrived. {@link ObjectDetectionIOLimelight} places the robot's real detections
 * through the same code.
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
  private BallPlacement.Frame latest = BallPlacement.Frame.NONE;

  /** Robot time a frame from a connected camera last arrived, whether or not it was used. */
  private double lastArrival = Double.NEGATIVE_INFINITY;

  private long framesDropped = 0;

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
    latest.writeTo(inputs);
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

    CameraIntrinsics lens =
        width > 0 && height > 0
            ? ObjectDetectionConstants.CAMERA_INTRINSICS.scaledTo((int) width, (int) height)
            : ObjectDetectionConstants.CAMERA_INTRINSICS;
    Translation3d[] rays = new Translation3d[u.length];
    for (int i = 0; i < u.length; i++) {
      rays[i] = lens.pixelToRay(u[i], v[i]);
    }

    BallPlacement.place(
            capture,
            now,
            rays,
            u.clone(),
            v.clone(),
            timestamp -> RobotState.getInstance().getEstimatedPoseAt(timestamp))
        .ifPresentOrElse(frame -> latest = frame, () -> framesDropped++);
  }
}
