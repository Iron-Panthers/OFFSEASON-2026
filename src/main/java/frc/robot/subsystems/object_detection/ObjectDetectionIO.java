package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  class ObjectDetectionIOInputs {
    public boolean connected = false;

    /**
     * Field relative positions of every ball in the latest frame. Repeated each loop until a newer
     * frame arrives, so compare {@link #frameTimestamp} to tell a new frame from a held one.
     */
    public Translation2d[] ballPositions = new Translation2d[0];

    /** Robot time the latest frame's scene was captured at. Zero until the first frame. */
    public double frameTimestamp = 0.0;

    /**
     * Where the camera was, field relative, at {@link #frameTimestamp}. Which balls a frame could
     * have seen depends on this pose, not on where the robot is by the time the frame arrives.
     */
    public Pose3d cameraPose = Pose3d.kZero;

    /** Capture to arrival on the robot, for the latest frame. */
    public double latencySeconds = 0.0;

    /**
     * Box centres in pixels, exactly as the detector reported them before any projection or
     * filtering. Empty for implementations with no image.
     */
    public double[] pixelU = new double[0];

    public double[] pixelV = new double[0];

    /**
     * Frames that arrived whole but were not used, because they were too old to place or came from
     * before the pose history begins. A running total.
     */
    public long framesDropped = 0;
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
