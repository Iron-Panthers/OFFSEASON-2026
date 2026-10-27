package frc.robot.subsystems.object_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;

/**
 * Reads the primary color blob from the Limelight's standard NT entries (tv/tx/ty/ta), projects it
 * onto the floor using the camera mount geometry, and estimates the number of balls it contains
 * from the blob's angular area vs. the angular size of one ball at that distance.
 *
 * <p>The Limelight color-blob pipeline only exposes one target at a time via these keys. To support
 * multiple simultaneous blobs, extend this class to parse the Limelight JSON output ("json" NT key)
 * and iterate over the results array.
 */
public class ObjectDetectionIOLimelight implements ObjectDetectionIO {
  private final String tableName;

  public ObjectDetectionIOLimelight() {
    this("limelight");
  }

  public ObjectDetectionIOLimelight(String tableName) {
    this.tableName = tableName;
  }

  @Override
  public void updateInputs(ObjectDetectionIOInputs inputs) {
    var table = NetworkTableInstance.getDefault().getTable(tableName);
    inputs.connected = true;

    if (table.getEntry("tv").getDouble(0.0) < 1.0) {
      inputs.blobPositions = new Translation2d[0];
      inputs.blobBallCounts = new int[0];
      return;
    }

    double txDeg = table.getEntry("tx").getDouble(0.0);
    double tyDeg = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);

    // Camera pose in world frame, derived from the current robot pose estimate.
    Pose3d cameraPose =
        new Pose3d(RobotState.getInstance().getEstimatedPose())
            .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

    // Unit ray from camera toward the blob in camera frame (x=forward, y=left, z=up).
    // tx positive = right = negative-y; ty positive = up = positive-z.
    double txRad = Math.toRadians(txDeg);
    double tyRad = Math.toRadians(tyDeg);
    Translation3d worldDir =
        new Translation3d(
                Math.cos(tyRad) * Math.cos(txRad),
                -Math.cos(tyRad) * Math.sin(txRad),
                Math.sin(tyRad))
            .rotateBy(cameraPose.getRotation());

    // Intersect the ray with the floor plane at z = BALL_RADIUS_M.
    double dz = worldDir.getZ();
    if (dz >= 0) {
      // Ray points up — cannot hit the floor.
      inputs.blobPositions = new Translation2d[0];
      inputs.blobBallCounts = new int[0];
      return;
    }
    double t = (ObjectDetectionConstants.BALL_RADIUS_M - cameraPose.getZ()) / dz;
    if (t <= 0) {
      inputs.blobPositions = new Translation2d[0];
      inputs.blobBallCounts = new int[0];
      return;
    }

    Translation2d fieldPos =
        new Translation2d(
            cameraPose.getX() + t * worldDir.getX(), cameraPose.getY() + t * worldDir.getY());

    // Angular area of one ball (radius r) at distance t: π·r²/t²
    double r = ObjectDetectionConstants.BALL_RADIUS_M;
    double ballAngularArea = Math.PI * r * r / (t * t);

    // Angular area of the blob as a fraction of the full FOV solid angle.
    double blobAngularArea =
        (ta / 100.0)
            * ObjectDetectionConstants.HORIZONTAL_FOV_RAD
            * ObjectDetectionConstants.VERTICAL_FOV_RAD;

    int estimatedCount = Math.max(1, (int) Math.round(blobAngularArea / ballAngularArea));

    inputs.blobPositions = new Translation2d[] {fieldPos};
    inputs.blobBallCounts = new int[] {estimatedCount};
  }
}
