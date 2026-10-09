package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.List;

public class ObjectDetectionIOLimelight implements ObjectDetectionIO {
  private final NetworkTable table;

  public ObjectDetectionIOLimelight() {
    this("limelight");
  }

  public ObjectDetectionIOLimelight(String name) {
    table = NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    // Limelight increments this every frame — if it stops changing the camera is offline.
    long heartbeat = table.getEntry("hb").getInteger(-1);
    inputs.connected = heartbeat >= 0;

    if (!inputs.connected) {
      inputs.ballPositions = new Translation2d[0];
      return;
    }

    Pose3d cameraPose =
        new Pose3d(RobotState.getInstance().getEstimatedPose())
            .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

    // llpython is written by our Python pipeline: [count, cx0, cy0, w0, h0, cx1, cy1, w1, h1, ...]
    // Pixel coordinates are origin top-left; we convert to Limelight-style angles for ray-casting.
    double[] llpython = table.getEntry("llpython").getDoubleArray(new double[0]);

    List<Translation2d> positions = new ArrayList<>();

    if (llpython.length >= 1) {
      int count = (int) llpython[0];
      double halfW = ObjectDetectionConstants.IMAGE_WIDTH_PX / 2.0;
      double halfH = ObjectDetectionConstants.IMAGE_HEIGHT_PX / 2.0;
      double hHalfFovDeg = Math.toDegrees(ObjectDetectionConstants.HORIZONTAL_FOV_RAD / 2.0);
      double vHalfFovDeg = Math.toDegrees(ObjectDetectionConstants.VERTICAL_FOV_RAD / 2.0);

      for (int i = 0; i < count; i++) {
        int base = 1 + i * 4;
        if (base + 1 >= llpython.length) break;

        double cx = llpython[base];
        double cy = llpython[base + 1];

        // tx: positive right; ty: positive up (Limelight convention — image y is inverted).
        double txDeg = (cx - halfW) / halfW * hHalfFovDeg;
        double tyDeg = -(cy - halfH) / halfH * vHalfFovDeg;

        Translation2d pos = projectToFloor(cameraPose, txDeg, tyDeg);
        if (pos != null) positions.add(pos);
      }
    }

    inputs.ballPositions = positions.toArray(Translation2d[]::new);
  }

  /**
   * Ray-casts a Limelight angular detection to the floor and returns the field-relative position,
   * or null if the ray points away from the floor or falls outside the valid detection range.
   *
   * <p>tx is positive to the right, ty is positive upward (standard Limelight convention).
   */
  private Translation2d projectToFloor(Pose3d cameraPose, double txDeg, double tyDeg) {
    double txRad = Math.toRadians(txDeg);
    double tyRad = Math.toRadians(tyDeg);

    // Direction in WPILib camera frame: +X forward, +Y left, +Z up.
    // Positive tx means right → negative Y; positive ty means up → positive Z.
    Translation3d dirCam =
        new Translation3d(
            Math.cos(tyRad) * Math.cos(txRad), -Math.cos(tyRad) * Math.sin(txRad), Math.sin(tyRad));

    Translation3d dirField = dirCam.rotateBy(cameraPose.getRotation());

    // Ball center sits one radius above the floor.
    double targetZ = ObjectDetectionConstants.BALL_RADIUS_M;
    double dz = dirField.getZ();
    if (dz >= 0.0) return null; // Ray points upward — no floor intersection.

    double t = (targetZ - cameraPose.getZ()) / dz;
    if (t <= 0.0) return null;

    double fx = cameraPose.getX() + t * dirField.getX();
    double fy = cameraPose.getY() + t * dirField.getY();
    Translation2d pos = new Translation2d(fx, fy);

    double range = cameraPose.getTranslation().toTranslation2d().getDistance(pos);
    if (range < ObjectDetectionConstants.MIN_RANGE_M
        || range > ObjectDetectionConstants.MAX_RANGE_M) {
      return null;
    }

    return pos;
  }
}
