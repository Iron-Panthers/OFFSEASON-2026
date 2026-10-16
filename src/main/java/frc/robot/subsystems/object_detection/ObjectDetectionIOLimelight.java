package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

/**
 * Balls found by a Limelight 3's neural detector, read off NetworkTables and placed on the field.
 *
 * <p>The robot's counterpart to {@link ObjectDetectionIOCoprocessor}, and as close to it as a
 * Limelight allows. The detector itself cannot be the same program: a Limelight runs its own neural
 * pipeline on the Coral, and its Python SnapScripts get OpenCV and numpy but no Torch, so the
 * coprocessor's Python does not run there. What is shared is everything after the detector. Each
 * detection becomes a bearing, and {@link BallPlacement} places it exactly as it places the
 * simulated detector's, from the pose the robot had when the frame was captured.
 *
 * <p>The Limelight reports each box as angles from the principal pixel ({@code txnc} and {@code
 * tync}) rather than as a pixel, so the bearing comes straight from those and no lens model is
 * involved here. The Limelight works those angles out through its own intrinsics, which are a
 * pinhole until it has been through its ChArUco calibration, after which they also undo the warp.
 *
 * <p>Read from the Limelight's table:
 *
 * <ul>
 *   <li>{@code hb}, a heartbeat that ticks once per processed frame
 *   <li>{@code rawdetections}, {@value #VALUES_PER_DETECTION} numbers per box: class, {@code txnc},
 *       {@code tync}, area, then four corners
 *   <li>{@code tl} and {@code cl}, pipeline and capture latency in milliseconds
 * </ul>
 */
public class ObjectDetectionIOLimelight implements ObjectDetectionIO {
  /** Numbers per box in {@code rawdetections}: class, txnc, tync, area, then four corner pairs. */
  static final int VALUES_PER_DETECTION = 12;

  private final DoubleSubscriber heartbeatSub;
  private final DoubleArraySubscriber rawDetectionsSub;
  private final DoubleSubscriber pipelineLatencySub;
  private final DoubleSubscriber captureLatencySub;
  private final DoubleFunction<Optional<Pose2d>> robotPoseAt;
  private final DoubleSupplier clock;

  private double lastHeartbeat = Double.NaN;
  private BallPlacement.Frame latest = BallPlacement.Frame.NONE;

  /** Robot time the Limelight last delivered a frame, whether or not it was used. */
  private double lastArrival = Double.NEGATIVE_INFINITY;

  private long framesDropped = 0;

  /**
   * @param name the Limelight's NetworkTables name, {@code limelight} unless it was renamed
   */
  public ObjectDetectionIOLimelight(String name) {
    this(
        NetworkTableInstance.getDefault(),
        name,
        timestamp -> RobotState.getInstance().getEstimatedPoseAt(timestamp),
        Timer::getTimestamp);
  }

  /**
   * @param nt the NetworkTables instance the Limelight publishes to
   * @param name the Limelight's NetworkTables name
   * @param robotPoseAt the pose estimator's history, {@code RobotState.getEstimatedPoseAt}
   * @param clock robot time now, in the same clock as the pose history
   */
  ObjectDetectionIOLimelight(
      NetworkTableInstance nt,
      String name,
      DoubleFunction<Optional<Pose2d>> robotPoseAt,
      DoubleSupplier clock) {
    NetworkTable table = nt.getTable(name);
    heartbeatSub = table.getDoubleTopic("hb").subscribe(0.0);
    rawDetectionsSub = table.getDoubleArrayTopic("rawdetections").subscribe(new double[0]);
    pipelineLatencySub = table.getDoubleTopic("tl").subscribe(0.0);
    captureLatencySub = table.getDoubleTopic("cl").subscribe(0.0);
    this.robotPoseAt = robotPoseAt;
    this.clock = clock;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    // A new heartbeat is a new frame, whatever it found. Going by the heartbeat rather than by the
    // detections changing is what lets an empty frame clear the balls the robot just ate: the same
    // empty result frame after frame is a duplicate, and NetworkTables does not resend those.
    TimestampedDouble heartbeat = heartbeatSub.getAtomic();
    if (heartbeat.timestamp != 0 && heartbeat.value != lastHeartbeat) {
      lastHeartbeat = heartbeat.value;
      // Arrival is when NetworkTables received the heartbeat, in microseconds of robot time, which
      // is the clock the pose history keeps. Reading it at the start of the next loop instead
      // would make every capture time up to a loop late.
      readFrame(heartbeat.timestamp / 1e6);
    }

    double now = clock.getAsDouble();
    inputs.connected = now - lastArrival <= ObjectDetectionConstants.FRAME_TIMEOUT_SEC;
    inputs.framesDropped = framesDropped;
    latest.writeTo(inputs);
  }

  /**
   * Reads the frame the heartbeat announced and places it, if it is recent enough.
   *
   * <p>The heartbeat and the detections are separate topics, so in principle the heartbeat can land
   * a frame before its detections do, and the previous frame's boxes are then placed with this
   * frame's timing. The Limelight sends a whole frame's values together, which makes that rare, and
   * it costs at most one frame of timing on one frame.
   */
  private void readFrame(double arrival) {
    lastArrival = arrival;
    double[] raw = rawDetectionsSub.get();
    double latencySeconds = (pipelineLatencySub.get() + captureLatencySub.get()) / 1000.0;
    if (raw.length % VALUES_PER_DETECTION != 0) {
      framesDropped++; // not whole boxes, so not something to read positions out of
      return;
    }

    List<Translation3d> rays = new ArrayList<>();
    for (int i = 0; i < raw.length; i += VALUES_PER_DETECTION) {
      int classId = (int) raw[i];
      if (ObjectDetectionConstants.LIMELIGHT_FUEL_CLASS_ID >= 0
          && classId != ObjectDetectionConstants.LIMELIGHT_FUEL_CLASS_ID) {
        continue;
      }
      rays.add(rayFromAngles(raw[i + 1], raw[i + 2]));
    }

    // Where the simulated camera would have drawn each box, so a log from the robot and one from
    // the simulation can be read side by side. Only for the log; placement uses the rays.
    double[] u = new double[rays.size()];
    double[] v = new double[rays.size()];
    for (int i = 0; i < rays.size(); i++) {
      Optional<Translation2d> pixel =
          ObjectDetectionConstants.CAMERA_INTRINSICS.rayToPixel(rays.get(i));
      u[i] = pixel.map(Translation2d::getX).orElse(Double.NaN);
      v[i] = pixel.map(Translation2d::getY).orElse(Double.NaN);
    }

    BallPlacement.place(
            arrival - latencySeconds,
            clock.getAsDouble(),
            rays.toArray(Translation3d[]::new),
            u,
            v,
            robotPoseAt)
        .ifPresentOrElse(frame -> latest = frame, () -> framesDropped++);
  }

  /**
   * The direction a Limelight's angles point along, in the WPILib camera frame.
   *
   * <p>Limelight angles are positive right and up, and each is the angle of the pinhole ray's
   * projection onto its own axis, so the ray through the box is {@code (1, -tan tx, tan ty)}. The
   * camera frame has Y to the left, hence the sign on the first.
   *
   * @param txncDegrees horizontal angle from the principal pixel, positive right
   * @param tyncDegrees vertical angle from the principal pixel, positive up
   */
  static Translation3d rayFromAngles(double txncDegrees, double tyncDegrees) {
    return new Translation3d(
        1.0, -Math.tan(Math.toRadians(txncDegrees)), Math.tan(Math.toRadians(tyncDegrees)));
  }
}
