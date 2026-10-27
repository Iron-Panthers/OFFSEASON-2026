package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  class ObjectDetectionIOInputs {
    public boolean connected = false;

    /** Field-relative center of each color blob the camera can currently see. */
    public Translation2d[] blobPositions = new Translation2d[0];

    /** Estimated number of balls in each corresponding blob. */
    public int[] blobBallCounts = new int[0];
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
