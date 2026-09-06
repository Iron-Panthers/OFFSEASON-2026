package frc.robot.subsystems.object_detection;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.swerve.DriveConstants;

public class ObjectDetectionConstants {
  /**
   * Camera mounted on the back of the robot, tilted down to see the floor. It faces the same way as
   * the intake so the robot approaches fuel on the side that can pick it up.
   */
  public static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(-0.30, 0.0, 0.60),
          new Rotation3d(0.0, Math.toRadians(20.0), Math.PI)); // positive pitch = down

  public static final double HORIZONTAL_FOV_RAD = Math.toRadians(63.3);
  public static final double VERTICAL_FOV_RAD = Math.toRadians(49.7);

  /** Range over which a ball on the floor is reported. */
  public static final double MIN_RANGE_M = 0.4;

  public static final double MAX_RANGE_M = 6.0;

  /** Radius of the game piece, used to place detections at floor height. */
  public static final double BALL_RADIUS_M = 0.075;

  /** Grid cell size for clustering — roughly the width the intake sweeps in one pass. */
  public static final double CLUSTER_CELL_SIZE_M = 0.8;

  /** How long a cluster is remembered after the camera last saw it. */
  public static final double POOL_MEMORY_SEC = 20.0;

  /** Clusters smaller than this are noise and are ignored. */
  public static final int MIN_CLUSTER_SIZE = 2;

  /**
   * Bias for the greedy cluster tour. Larger values make the robot willing to drive further for a
   * denser cluster, rather than settling for a small one underfoot.
   */
  public static final double CLUSTER_DISTANCE_BIAS_M = 4.0;

  /**
   * Turning charged as extra travel when ordering the tour. Without it the greedy walk zig-zags
   * back and forth across the pile. A full reversal costs this much, a straight leg costs nothing.
   */
  public static final double CLUSTER_TURN_PENALTY_M = 3.0;

  /**
   * Distance the path carries on past the last cluster, so the intake sweeps all the way through it
   * instead of decelerating to a stop on top of it.
   */
  public static final double FOLLOW_THROUGH_M = 1.0;

  /** A cluster this close to the robot is already being driven over, so it is skipped. */
  public static final double CLUSTER_SKIP_RADIUS_M = 0.3;

  /** Clusters are only targeted between the two hubs, the neutral strip where the fuel sits. */
  public static final double PICKUP_ZONE_MIN_X = DriveConstants.BLUE_HUB_ORIGIN.getX();

  public static final double PICKUP_ZONE_MAX_X = DriveConstants.RED_HUB_ORIGIN.getX();

  public static final PathConstraints PICKUP_PATH_CONSTRAINTS =
      new PathConstraints(3.0, 3.0, Math.toRadians(540), Math.toRadians(720));
}
