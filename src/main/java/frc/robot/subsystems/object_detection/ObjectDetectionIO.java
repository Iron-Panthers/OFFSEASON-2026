package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
  @AutoLog
  class ObjectDetectionIOInputs {
    public boolean connected = false;

    /** Field relative positions of every ball the camera can currently see. */
    public Translation2d[] ballPositions = new Translation2d[0];
  }

  default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
