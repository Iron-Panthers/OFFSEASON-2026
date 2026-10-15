package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotSimState;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;

/**
 * Reports the fuel that falls inside the camera frustum, using the ground truth sim pose.
 *
 * <p>A perfect, zero-latency camera with no image behind it. It is what the simulation uses unless
 * {@code -Pcoproc=objdetect} runs the real detector against a rendered camera, in which case {@link
 * ObjectDetectionIOCoprocessor} takes over. Being cheap and deterministic is what makes it the
 * right one for A/B runs, and the reason it stays.
 */
public class ObjectDetectionIOSim implements ObjectDetectionIO {
  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    Pose3d cameraPose =
        RobotSimState.getInstance()
            .getRobotFramePose3d()
            .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

    List<Translation2d> visible = new ArrayList<>();
    for (Translation3d ball : RobotSimState.getInstance().getFuelSim().getFuelPositions()) {
      if (isVisible(cameraPose, ball)) {
        visible.add(ball.toTranslation2d());
      }
    }

    inputs.connected = true;
    inputs.ballPositions = visible.toArray(Translation2d[]::new);
    // Every loop is a fresh frame, seen from where the robot believes it is right now.
    inputs.frameTimestamp = Timer.getTimestamp();
    inputs.cameraPose =
        new Pose3d(RobotState.getInstance().getEstimatedPose())
            .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);
    inputs.latencySeconds = 0.0;
  }

  /** True when the ball is in front of the camera, inside both FOV cones and within range. */
  static boolean isVisible(Pose3d cameraPose, Translation3d ball) {
    Translation3d relative =
        ball.minus(cameraPose.getTranslation()).rotateBy(cameraPose.getRotation().unaryMinus());

    if (relative.getX() <= 0.0) return false;

    double range = relative.getNorm();
    if (range < ObjectDetectionConstants.MIN_RANGE_M
        || range > ObjectDetectionConstants.MAX_RANGE_M) {
      return false;
    }

    double yaw = Math.atan2(relative.getY(), relative.getX());
    double pitch = Math.atan2(-relative.getZ(), relative.getX());
    return Math.abs(yaw) <= ObjectDetectionConstants.HORIZONTAL_FOV_RAD / 2.0
        && Math.abs(pitch) <= ObjectDetectionConstants.VERTICAL_FOV_RAD / 2.0;
  }
}
