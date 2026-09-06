package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotSimState;
import java.util.ArrayList;
import java.util.List;

/** Reports the fuel that falls inside the camera frustum, using the ground truth sim pose. */
public class ObjectDetectionIOSim implements ObjectDetectionIO {
  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    Pose3d cameraPose =
        RobotSimState.getInstance()
            .getRobotPose3d()
            .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

    List<Translation2d> visible = new ArrayList<>();
    for (Translation3d ball : RobotSimState.getInstance().getFuelSim().getFuelPositions()) {
      if (isVisible(cameraPose, ball)) {
        visible.add(ball.toTranslation2d());
      }
    }

    inputs.connected = true;
    inputs.ballPositions = visible.toArray(Translation2d[]::new);
  }

  /** True when the ball is in front of the camera, inside both FOV cones and within range. */
  private boolean isVisible(Pose3d cameraPose, Translation3d ball) {
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
