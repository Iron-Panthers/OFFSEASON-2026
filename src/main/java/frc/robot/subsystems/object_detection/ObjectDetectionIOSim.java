package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotSimState;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Reports visible fuel as pre-clustered blobs, mirroring what the real Limelight produces. */
public class ObjectDetectionIOSim implements ObjectDetectionIO {
  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    Pose3d cameraPose =
        RobotSimState.getInstance()
            .getRobotPose3d()
            .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

    // Bin visible balls into grid cells, then report each non-empty cell as one blob.
    Map<Long, List<Translation2d>> cells = new HashMap<>();
    for (Translation3d ball : RobotSimState.getInstance().getFuelSim().getFuelPositions()) {
      if (isVisible(cameraPose, ball)) {
        Translation2d pos = ball.toTranslation2d();
        cells.computeIfAbsent(cellKey(pos), k -> new ArrayList<>()).add(pos);
      }
    }

    List<Translation2d> positions = new ArrayList<>();
    List<Integer> counts = new ArrayList<>();
    for (List<Translation2d> cellBalls : cells.values()) {
      Translation2d sum = new Translation2d();
      for (Translation2d b : cellBalls) sum = sum.plus(b);
      positions.add(sum.div(cellBalls.size()));
      counts.add(cellBalls.size());
    }

    inputs.connected = true;
    inputs.blobPositions = positions.toArray(Translation2d[]::new);
    inputs.blobBallCounts = counts.stream().mapToInt(Integer::intValue).toArray();
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

  private long cellKey(Translation2d point) {
    double cell = ObjectDetectionConstants.CLUSTER_CELL_SIZE_M;
    return ((long) Math.floor(point.getX() / cell) << 32) ^ (long) Math.floor(point.getY() / cell);
  }
}
