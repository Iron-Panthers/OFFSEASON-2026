package frc.robot.subsystems.object_detection;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Rear mounted ball camera, on the same side as the intake. Groups the detected balls into clusters
 * and turns the best of them into a pickup path.
 */
public class ObjectDetection extends SubsystemBase {
  /** A group of balls close enough together to be swept up in one pass. */
  public record Cluster(Translation2d center, int ballCount) {}

  /** One grid cell's most recent observation, kept after the camera has looked away. */
  private record PooledCell(Translation2d center, int ballCount, double timestamp) {}

  private final ObjectDetectionIO io;
  private final ObjectDetectionIOInputsAutoLogged inputs = new ObjectDetectionIOInputsAutoLogged();

  private final Map<Long, PooledCell> pool = new HashMap<>();
  private List<Cluster> clusters = List.of();

  public ObjectDetection(ObjectDetectionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ObjectDetection", inputs);

    updatePool(inputs.ballPositions);
    clusters = pool.values().stream().map(c -> new Cluster(c.center(), c.ballCount())).toList();

    Logger.recordOutput("ObjectDetection/Camera Pose", getCameraPose());
    Logger.recordOutput("ObjectDetection/Cluster Count", clusters.size());
    Logger.recordOutput("ObjectDetection/Visible Ball Count", inputs.ballPositions.length);
    Logger.recordOutput(
        "ObjectDetection/Cluster Centers",
        clusters.stream().map(Cluster::center).toArray(Translation2d[]::new));
    Logger.recordOutput(
        "ObjectDetection/Cluster Ball Counts",
        clusters.stream().mapToLong(Cluster::ballCount).toArray());
  }

  /**
   * Camera position in world space, for lining detections up against the field in AdvantageScope.
   */
  public Pose3d getCameraPose() {
    return new Pose3d(RobotState.getInstance().getEstimatedPose())
        .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);
  }

  /** Every cluster the camera has seen recently, not just the ones in view this frame. */
  public List<Cluster> getClusters() {
    return clusters;
  }

  /** Forgets every pooled observation. */
  public void resetPool() {
    pool.clear();
  }

  /**
   * Folds this frame's detections into the pool. Cells currently in view are replaced with fresh
   * data (so a cluster the robot just ate stops being a target), cells outside the view keep their
   * last known value, and anything not seen for a while expires. This is what lets clusters spotted
   * on the drive to the observing pose — including the back corners it cannot see from there —
   * still count when the pickup path is generated.
   */
  private void updatePool(Translation2d[] balls) {
    double now = Timer.getFPGATimestamp();
    Pose2d camera = getCameraPose().toPose2d();

    pool.values()
        .removeIf(cell -> now - cell.timestamp() > ObjectDetectionConstants.POOL_MEMORY_SEC);
    pool.values().removeIf(cell -> inView(cell.center(), camera));

    for (List<Translation2d> cellBalls : binByCell(balls).values()) {
      if (cellBalls.size() < ObjectDetectionConstants.MIN_CLUSTER_SIZE) continue;
      Translation2d sum = new Translation2d();
      for (Translation2d ball : cellBalls) {
        sum = sum.plus(ball);
      }
      Translation2d center = sum.div(cellBalls.size());
      pool.put(cellKey(center), new PooledCell(center, cellBalls.size(), now));
    }
  }

  /** Whether a point falls inside the camera's horizontal view, flattened to 2d. */
  private boolean inView(Translation2d point, Pose2d camera) {
    Translation2d relative =
        point.minus(camera.getTranslation()).rotateBy(camera.getRotation().unaryMinus());
    double range = relative.getNorm();
    return relative.getX() > 0
        && range >= ObjectDetectionConstants.MIN_RANGE_M
        && range <= ObjectDetectionConstants.MAX_RANGE_M
        && Math.abs(Math.atan2(relative.getY(), relative.getX()))
            <= ObjectDetectionConstants.HORIZONTAL_FOV_RAD / 2.0;
  }

  private long cellKey(Translation2d point) {
    double cell = ObjectDetectionConstants.CLUSTER_CELL_SIZE_M;
    return ((long) Math.floor(point.getX() / cell) << 32) ^ (long) Math.floor(point.getY() / cell);
  }

  private Map<Long, List<Translation2d>> binByCell(Translation2d[] balls) {
    Map<Long, List<Translation2d>> cells = new HashMap<>();
    for (Translation2d ball : balls) {
      cells.computeIfAbsent(cellKey(ball), k -> new ArrayList<>()).add(ball);
    }
    return cells;
  }

  /**
   * Whether a cluster sits in the neutral strip between the two hubs. Fuel past a hub is in an
   * alliance's own end of the field, and the straight sweep has no way around the obstacles there.
   */
  private boolean inPickupZone(Translation2d center) {
    return center.getX() >= ObjectDetectionConstants.PICKUP_ZONE_MIN_X
        && center.getX() <= ObjectDetectionConstants.PICKUP_ZONE_MAX_X;
  }

  /**
   * Picks the clusters to sweep, best first, until {@code ballGoal} balls are covered or the known
   * clusters run out.
   *
   * <p>A true shortest tour is NP-hard and the ball field changes every loop, so this uses a greedy
   * walk: from the current position, repeatedly take the best {@code ballCount / cost} cluster,
   * where cost charges for distance and for how far the robot has to turn.
   */
  private List<Translation2d> chooseStops(Pose2d startPose, int ballGoal) {
    List<Cluster> remaining = new ArrayList<>(clusters);
    List<Translation2d> stops = new ArrayList<>();
    Translation2d position = startPose.getTranslation();
    int ballsCovered = 0;

    // The intake and camera face out the back, so that is the way the robot sweeps first.
    Rotation2d heading = startPose.getRotation().rotateBy(Rotation2d.kPi);

    while (ballsCovered < ballGoal && !remaining.isEmpty()) {
      Cluster best = null;
      double bestScore = 0.0;
      for (Cluster candidate : remaining) {
        if (!inPickupZone(candidate.center())) continue;
        Translation2d leg = candidate.center().minus(position);
        double distance = leg.getNorm();
        if (distance < ObjectDetectionConstants.CLUSTER_SKIP_RADIUS_M) continue;
        double turn = Math.abs(leg.getAngle().minus(heading).getRadians()) / Math.PI;
        double cost =
            distance
                + turn * ObjectDetectionConstants.CLUSTER_TURN_PENALTY_M
                + ObjectDetectionConstants.CLUSTER_DISTANCE_BIAS_M;
        double score = candidate.ballCount() / cost;
        if (score > bestScore) {
          bestScore = score;
          best = candidate;
        }
      }
      if (best == null) break;

      heading = best.center().minus(position).getAngle();
      remaining.remove(best);
      stops.add(best.center());
      position = best.center();
      ballsCovered += best.ballCount();
    }

    Logger.recordOutput("ObjectDetection/Pickup Stops", stops.toArray(Translation2d[]::new));
    Logger.recordOutput("ObjectDetection/Pickup Balls Covered", ballsCovered);
    return stops;
  }

  /**
   * Sweeps through the chosen clusters. The intake is on the back of the robot, so every holonomic
   * rotation points opposite the direction of travel, and the path carries on past the last cluster
   * rather than decelerating to a stop on top of it.
   *
   * @return a command that does nothing when no cluster is worth driving to
   */
  public Command getPickupCommand(Pose2d startPose, int ballGoal) {
    List<Translation2d> stops = chooseStops(startPose, ballGoal);
    if (stops.isEmpty()) return Commands.none();

    List<Translation2d> points = new ArrayList<>();
    points.add(startPose.getTranslation());
    points.addAll(stops);

    Translation2d last = points.get(points.size() - 1);
    Rotation2d runOut = last.minus(points.get(points.size() - 2)).getAngle();
    points.add(last.plus(new Translation2d(ObjectDetectionConstants.FOLLOW_THROUGH_M, runOut)));

    // Each waypoint's rotation is the bezier tangent, so point it along the leg of travel.
    List<Pose2d> waypointPoses = new ArrayList<>();
    List<RotationTarget> rotationTargets = new ArrayList<>();
    for (int i = 0; i < points.size(); i++) {
      Rotation2d heading =
          i < points.size() - 1
              ? points.get(i + 1).minus(points.get(i)).getAngle()
              : points.get(i).minus(points.get(i - 1)).getAngle();
      waypointPoses.add(new Pose2d(points.get(i), heading));
      // The end rotation comes from the goal end state, so the last waypoint gets no target.
      if (i > 0 && i < points.size() - 1) {
        rotationTargets.add(new RotationTarget(i, heading.rotateBy(Rotation2d.kPi)));
      }
    }

    Rotation2d endHeading = waypointPoses.get(waypointPoses.size() - 1).getRotation();
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(waypointPoses),
            rotationTargets,
            List.of(),
            List.of(),
            List.of(),
            ObjectDetectionConstants.PICKUP_PATH_CONSTRAINTS,
            new IdealStartingState(0.0, startPose.getRotation()),
            new GoalEndState(0.0, endHeading.rotateBy(Rotation2d.kPi)),
            false);
    // Detections are already field absolute, so the alliance flip must not be applied again.
    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }
}
