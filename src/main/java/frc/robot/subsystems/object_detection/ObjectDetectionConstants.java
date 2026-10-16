package frc.robot.subsystems.object_detection;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utility.CameraIntrinsics;

public class ObjectDetectionConstants {
  /**
   * Camera mounted on the back of the robot, tilted down to see the floor. It faces the same way as
   * the intake so the robot approaches fuel on the side that can pick it up.
   *
   * <p>On the robot this is the Limelight, and it has to be mounted where this says. Every ball is
   * placed by following its bearing down to the floor from here, so a camera a few centimetres
   * higher or a degree steeper than this misplaces every ball by the same proportion.
   */
  public static final Transform3d ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(-0.30, 0.0, 0.60),
          new Rotation3d(0.0, Math.toRadians(20.0), Math.PI)); // positive pitch = down

  /**
   * NetworkTables name of the Limelight running the fuel detector, as set in its web interface. A
   * Limelight is {@code limelight} until it is renamed.
   */
  public static final String LIMELIGHT_NAME = "limelight";

  /**
   * Class the Limelight's model reports fuel as. Negative keeps every class, which is what a
   * single-class fuel model wants.
   */
  public static final int LIMELIGHT_FUEL_CLASS_ID = -1;

  /**
   * Horizontal field of view of a Limelight 3, from Limelight's hardware comparison. The camera
   * below is modelled on it, so the simulation sees what the robot's Limelight sees.
   */
  public static final double LIMELIGHT_3_HORIZONTAL_FOV_DEGREES = 62.5;

  /**
   * Frame size the camera is modelled at. A Limelight 3's 640x480 scaled by three quarters, which
   * keeps its shape and field of view and saves the renderer 44% of the pixels. Robot code never
   * depends on it, because the Limelight reports angles rather than pixels. The renderer draws this
   * camera at exactly this size, independent of {@code -Prender.width}, because it is a different
   * sensor from the vision cameras.
   */
  public static final int CAMERA_WIDTH_PX = 480;

  public static final int CAMERA_HEIGHT_PX = 360;

  /**
   * Focal lengths in pixels at {@link #CAMERA_WIDTH_PX} x {@link #CAMERA_HEIGHT_PX}, from the
   * Limelight 3's horizontal field of view with square pixels. That puts the vertical field of view
   * at 48.9 degrees, which is the datasheet's other figure.
   */
  public static final double CAMERA_FX_PX =
      (CAMERA_WIDTH_PX / 2.0) / Math.tan(Math.toRadians(LIMELIGHT_3_HORIZONTAL_FOV_DEGREES) / 2.0);

  public static final double CAMERA_FY_PX = CAMERA_FX_PX;

  /** Principal point in pixels, where the optical axis meets the sensor. */
  public static final double CAMERA_CX_PX = CAMERA_WIDTH_PX / 2.0;

  public static final double CAMERA_CY_PX = CAMERA_HEIGHT_PX / 2.0;

  /**
   * Radial lens warp, in {@link CameraIntrinsics}'s undistorting convention (not OpenCV's).
   *
   * <p>Zero, because that is how the Limelight models its own lens when it turns a box into the
   * angles {@link ObjectDetectionIOLimelight} reads, until it has been through Limelight's ChArUco
   * calibration. The renderer drawing the same pinhole keeps the simulated detector reporting balls
   * at the bearings the real one would. A measured warp can go here, refit to this convention, to
   * make the rendered image look like the real lens. The renderer bends its rays through these and
   * {@link ObjectDetectionIOCoprocessor} unbends each detection through the same numbers, so
   * placement stays exact either way.
   */
  public static final double CAMERA_DISTORTION_K1 = 0.0;

  public static final double CAMERA_DISTORTION_K2 = 0.0;

  /** Everything above in one object, which is what the renderer and the IO actually take. */
  public static final CameraIntrinsics CAMERA_INTRINSICS =
      new CameraIntrinsics(
          CAMERA_WIDTH_PX,
          CAMERA_HEIGHT_PX,
          CAMERA_FX_PX,
          CAMERA_FY_PX,
          CAMERA_CX_PX,
          CAMERA_CY_PX,
          CAMERA_DISTORTION_K1,
          CAMERA_DISTORTION_K2);

  /**
   * Field of view the camera actually sees, warp included: 62.5 by 48.9 degrees, a Limelight 3's.
   *
   * <p>Derived from the intrinsics rather than written down beside them. A hand-entered figure
   * would be a second description of the lens to keep in step, and when it drifts the ground-truth
   * sim and the pool's in-view check disagree with the rendered camera about which balls are in
   * frame.
   */
  public static final double HORIZONTAL_FOV_RAD = CAMERA_INTRINSICS.horizontalFovRad();

  public static final double VERTICAL_FOV_RAD = CAMERA_INTRINSICS.verticalFovRad();

  /**
   * Pinhole corner-to-corner angle, warp ignored. The coprocessor derives its focal length from the
   * frame diagonal and this number, so it has to be the pinhole figure for the two to agree on the
   * focal length.
   */
  public static final double CAMERA_DIAGONAL_FOV_DEGREES =
      CAMERA_INTRINSICS.pinholeDiagonalFovDegrees();

  /**
   * A frame older than this when it reaches the robot is dropped. The pose estimator keeps 1.5 s of
   * history and clamps anything older to its oldest entry, so a frame past that would be placed
   * using the wrong pose without complaint. This leaves a tenth of a second of margin.
   *
   * <p>As tight as it can safely be, not tighter, because the simulated pipeline is slow. With the
   * renderer and the detector sharing one machine, capture to arrival averages about 0.75 s with a
   * long tail, and a 1.0 s limit was throwing away frames the pose history could still place.
   */
  public static final double MAX_FRAME_AGE_SEC = 1.4;

  /** No frame has arrived for this long and the detector is reported disconnected. */
  public static final double FRAME_TIMEOUT_SEC = 1.0;

  /**
   * Detections projected further than this outside the field are dropped. Fuel cannot be there, so
   * a projection landing there is a box on something that is not a ball on the floor.
   */
  public static final double FIELD_BOUNDS_TOLERANCE_M = 0.2;

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

  /**
   * Keeps planned waypoints inside the field. Balls roll into the perimeter, so without this the
   * tour aims the robot at points it cannot physically occupy and it grinds along the wall.
   */
  public static final double FIELD_MARGIN_M = 0.6;

  /**
   * Stops chained into one generated path. Balls scatter when the robot drives through them, so a
   * short path that is regenerated often tracks the pile better than one long committed tour.
   */
  public static final int MAX_STOPS_PER_PATH = 2;

  /** Speed carried through an intermediate stop, so the sweep does not stop on every cluster. */
  public static final double PICKUP_TRANSIT_VEL = 2.0;

  public static final PathConstraints PICKUP_PATH_CONSTRAINTS =
      new PathConstraints(3.0, 3.0, Math.toRadians(540), Math.toRadians(720));
}
