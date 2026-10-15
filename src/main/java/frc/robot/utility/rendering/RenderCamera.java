package frc.robot.utility.rendering;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utility.CameraIntrinsics;

/**
 * Maps pixels to world-space rays for one camera on the robot.
 *
 * <p>Intrinsics follow the OV9281 modelling already in {@link
 * frc.robot.subsystems.vision.VisionConstants}, so a frame rendered here frames the same scene the
 * existing PhotonVision simulation would. Extrinsics come from the same {@code CAMERA_TRANSFORM}
 * table the real robot is calibrated into, which means a rendered frame and a real one from that
 * camera are directly comparable.
 *
 * <p>Lens distortion is applied on the way out rather than corrected on the way in. Rendering a
 * pinhole image and then warping it resamples and softens the result; bending the ray instead
 * produces the lens distortion at full resolution, which matters because distortion is exactly the
 * thing a vision pipeline has to undo.
 */
final class RenderCamera {

  /**
   * Brown-Conrady radial terms for the vision cameras: a wide M12 lens on a 1/4 inch sensor, in
   * {@link CameraIntrinsics}'s undistorting convention.
   *
   * <p>Representative rather than calibrated, matching the spirit of the sim camera constants: the
   * point is that a pipeline developed against these frames has to cope with distortion at all. The
   * object-detection camera does not use these; it carries its own in {@code
   * ObjectDetectionConstants.CAMERA_INTRINSICS}.
   */
  static final double DISTORTION_K1 = -0.105;

  static final double DISTORTION_K2 = 0.021;

  private final CameraIntrinsics intrinsics;

  private double originX;
  private double originY;
  private double originZ;

  /** Camera basis in world space: forward down the optical axis, plus right and up. */
  private final double[] forward = new double[3];

  private final double[] right = new double[3];
  private final double[] up = new double[3];

  /** A vision camera: square pixels from a diagonal field of view, with the default warp. */
  RenderCamera(int width, int height, double diagonalFovDegrees) {
    this(
        CameraIntrinsics.fromDiagonalFov(
            width, height, diagonalFovDegrees, DISTORTION_K1, DISTORTION_K2));
  }

  /** A camera with its own calibrated lens, rendered at the resolution it was calibrated at. */
  RenderCamera(CameraIntrinsics intrinsics) {
    this.intrinsics = intrinsics;
  }

  int width() {
    return intrinsics.width();
  }

  int height() {
    return intrinsics.height();
  }

  CameraIntrinsics intrinsics() {
    return intrinsics;
  }

  double originX() {
    return originX;
  }

  double originY() {
    return originY;
  }

  double originZ() {
    return originZ;
  }

  /**
   * Places the camera for a frame.
   *
   * @param robotPose robot pose in field coordinates
   * @param mounting robot-to-camera transform, straight from the vision constants
   */
  void place(Pose3d robotPose, Transform3d mounting) {
    Pose3d camera = robotPose.transformBy(mounting);
    originX = camera.getX();
    originY = camera.getY();
    originZ = camera.getZ();

    // WPILib camera convention: +X out of the lens, +Y to the left, +Z up.
    Rotation3d rotation = camera.getRotation();
    rotate(rotation, 1, 0, 0, forward);
    rotate(rotation, 0, -1, 0, right);
    rotate(rotation, 0, 0, 1, up);
  }

  private static void rotate(Rotation3d rotation, double x, double y, double z, double[] out) {
    var rotated = new edu.wpi.first.math.geometry.Translation3d(x, y, z).rotateBy(rotation);
    out[0] = rotated.getX();
    out[1] = rotated.getY();
    out[2] = rotated.getZ();
  }

  /**
   * Builds the world-space direction for a point on the sensor.
   *
   * @param pixelX horizontal sample position in pixels, may be fractional for anti-aliasing
   * @param pixelY vertical sample position in pixels
   * @param out receives a unit direction
   */
  void ray(double pixelX, double pixelY, float[] out) {
    // Normalised image coordinates, +a right and +b down, exactly as CameraIntrinsics defines them.
    double a = (pixelX - intrinsics.cx()) / intrinsics.fx();
    double b = (pixelY - intrinsics.cy()) / intrinsics.fy();

    // A real lens maps the incoming angle to a radius that grows non-linearly, so scale the
    // direction by the radial polynomial before turning it into a ray. The polynomial is the lens's
    // own, so robot code undoing it through the same intrinsics lands on this exact ray.
    double scale = intrinsics.undistortionScale(a * a + b * b);
    double nx = a * scale;
    double ny = -b * scale;

    double dx = forward[0] + right[0] * nx + up[0] * ny;
    double dy = forward[1] + right[1] * nx + up[1] * ny;
    double dz = forward[2] + right[2] * nx + up[2] * ny;
    double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
    out[0] = (float) (dx / length);
    out[1] = (float) (dy / length);
    out[2] = (float) (dz / length);
  }
}
