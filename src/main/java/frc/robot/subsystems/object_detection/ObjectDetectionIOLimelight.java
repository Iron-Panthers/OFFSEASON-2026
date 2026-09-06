package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Translation2d;

/** Real hardware implementation. Not wired to a Limelight yet — reports nothing. */
public class ObjectDetectionIOLimelight implements ObjectDetectionIO {
  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    inputs.connected = false;
    inputs.ballPositions = new Translation2d[0];
  }
}
