package frc.robot.utility.rendering;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

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
 * produces the barrel distortion at full resolution, which matters because distortion is exactly
 * the thing a vision pipeline has to undo.
 */
final class RenderCamera {

  /**
   * Brown-Conrady radial terms for a wide M12 lens on a 1/4 inch sensor.
   *
   * <p>Representative rather than calibrated, matching the spirit of the sim camera constants: the
   * point is that a pipeline developed against these frames has to cope with distortion at all.
   */
  private static final double DISTORTION_K1 = -0.105;

  private static final double DISTORTION_K2 = 0.021;

  private final int width;
  private final int height;

  /** Tangent of half the horizontal and vertical field of view at the image edge. */
  private final double tanHalfHorizontal;

  private final double tanHalfVertical;

  private double originX;
  private double originY;
  private double originZ;

  /** Camera basis in world space: forward down the optical axis, plus right and up. */
  private final double[] forward = new double[3];

  private final double[] right = new double[3];
  private final double[] up = new double[3];

  RenderCamera(int width, int height, double diagonalFovDegrees) {
    this.width = width;
    this.height = height;

    // Split the diagonal field of view between the axes by the sensor aspect ratio.
    double diagonal = Math.hypot(width, height);
    double tanHalfDiagonal = Math.tan(Math.toRadians(diagonalFovDegrees) / 2.0);
    this.tanHalfHorizontal = tanHalfDiagonal * width / diagonal;
    this.tanHalfVertical = tanHalfDiagonal * height / diagonal;
  }

  int width() {
    return width;
  }

  int height() {
    return height;
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
    // Normalised image coordinates, centred, +x right and +y up.
    double nx = (2.0 * pixelX / width - 1.0) * tanHalfHorizontal;
    double ny = (1.0 - 2.0 * pixelY / height) * tanHalfVertical;

    // A real lens maps the incoming angle to a radius that grows non-linearly, so scale the
    // direction by the radial polynomial before turning it into a ray.
    double radiusSquared = nx * nx + ny * ny;
    double scale =
        1.0 + DISTORTION_K1 * radiusSquared + DISTORTION_K2 * radiusSquared * radiusSquared;
    nx *= scale;
    ny *= scale;

    double dx = forward[0] + right[0] * nx + up[0] * ny;
    double dy = forward[1] + right[1] * nx + up[1] * ny;
    double dz = forward[2] + right[2] * nx + up[2] * ny;
    double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
    out[0] = (float) (dx / length);
    out[1] = (float) (dy / length);
    out[2] = (float) (dz / length);
  }
}
