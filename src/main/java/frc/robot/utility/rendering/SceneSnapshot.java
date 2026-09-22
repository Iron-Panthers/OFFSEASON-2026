package frc.robot.utility.rendering;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;

/**
 * Everything about a single simulation instant that the renderer needs, copied out of the
 * simulation and then never touched again.
 *
 * <p>The renderer runs on its own threads at its own rate, far slower than the 20 ms robot loop.
 * Handing it live simulation objects would mean either locking the loop behind a frame or reading
 * fuel positions that move underneath the ray tracer mid-frame. Publishing an immutable snapshot
 * instead keeps the loop free and makes every frame internally consistent.
 *
 * @param timestampSeconds simulation time this state was captured at
 * @param robotPose ego robot pose in field coordinates
 * @param opponents other robots on the field
 * @param fuelCenters xyz per ball, packed, in field coordinates
 * @param fuelCount number of valid balls in {@code fuelCenters}
 */
record SceneSnapshot(
    double timestampSeconds,
    Pose3d robotPose,
    List<Pose3d> opponents,
    float[] fuelCenters,
    int fuelCount) {

  /** An empty field, used before the simulation has published anything. */
  static SceneSnapshot empty() {
    return new SceneSnapshot(0, Pose3d.kZero, List.of(), new float[0], 0);
  }

  /**
   * Packs live simulation state into a snapshot.
   *
   * @param fuel ball centres straight from {@code FuelSim.getFuelPositions()}
   * @param obstacles opponent poses from {@code RobotSimState.getObstaclePositions()}
   */
  static SceneSnapshot of(
      double timestampSeconds, Pose3d robotPose, List<Pose2d> obstacles, Translation3d[] fuel) {
    float[] centers = new float[fuel.length * 3];
    for (int i = 0; i < fuel.length; i++) {
      centers[i * 3] = (float) fuel[i].getX();
      centers[i * 3 + 1] = (float) fuel[i].getY();
      centers[i * 3 + 2] = (float) fuel[i].getZ();
    }
    List<Pose3d> lifted =
        obstacles.stream()
            .map(
                pose ->
                    new Pose3d(
                        new Translation3d(pose.getX(), pose.getY(), 0),
                        new Rotation3d(0, 0, pose.getRotation().getRadians())))
            .toList();
    return new SceneSnapshot(timestampSeconds, robotPose, lifted, centers, fuel.length);
  }
}
