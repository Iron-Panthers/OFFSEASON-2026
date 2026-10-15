package frc.robot.utility;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;

/**
 * One camera's lens: pinhole focal lengths and principal point at a stated resolution, plus the
 * radial warp the lens adds on top.
 *
 * <p>This is the single description of the lens shared by the renderer, which bends every ray it
 * traces through it, and by robot code, which runs the same mapping to turn a detection's pixel
 * back into a direction. Both go through {@link #undistortionScale}, so a pixel maps back onto
 * exactly the ray the renderer drew it along, and changing a constant changes both sides at once.
 *
 * <h2>The warp</h2>
 *
 * <p>Brown-Conrady radial terms, written in the undistorting direction:
 *
 * <pre>
 *   undistorted = distorted * (1 + k1 r^2 + k2 r^4),   r = |distorted|
 * </pre>
 *
 * where both sides are normalised image coordinates, {@code (pixel - principal point) / focal
 * length}. This is the direction both users need: the renderer goes from a pixel to a ray once per
 * sample, and robot code goes from a detection's pixel to a ray once per ball, so neither has to
 * solve anything. The forward direction, {@link #rayToPixel}, is only needed for tests and is
 * solved by Newton's method.
 *
 * <p>It is the inverse of OpenCV's convention. A calibration exported from OpenCV or PhotonVision
 * describes the opposite mapping, so its k1 and k2 do not drop in here unchanged; they have to be
 * refit.
 *
 * <p>Pixel coordinates are the usual image convention: {@code u} right, {@code v} down, origin at
 * the top-left corner of the top-left pixel, so the image spans {@code [0, width]} by {@code [0,
 * height]}. Directions are in the WPILib camera frame: X out of the lens, Y left, Z up.
 *
 * @param width frame width these intrinsics were calibrated at, in pixels
 * @param height frame height these intrinsics were calibrated at, in pixels
 * @param fx horizontal focal length, in pixels
 * @param fy vertical focal length, in pixels
 * @param cx principal point, pixels from the left edge
 * @param cy principal point, pixels from the top edge
 * @param k1 second-order radial warp term
 * @param k2 fourth-order radial warp term
 */
public record CameraIntrinsics(
    int width, int height, double fx, double fy, double cx, double cy, double k1, double k2) {

  /** Newton iterations for {@link #rayToPixel}. It converges to machine precision in four. */
  private static final int INVERSION_ITERATIONS = 8;

  /**
   * Square pixels and a centred principal point, from the one number a datasheet gives.
   *
   * <p>The diagonal field of view is divided between the axes by the frame's aspect ratio, which is
   * what the vision cameras have always been rendered with.
   */
  public static CameraIntrinsics fromDiagonalFov(
      int width, int height, double diagonalFovDegrees, double k1, double k2) {
    double focal =
        (Math.hypot(width, height) / 2.0) / Math.tan(Math.toRadians(diagonalFovDegrees) / 2.0);
    return new CameraIntrinsics(width, height, focal, focal, width / 2.0, height / 2.0, k1, k2);
  }

  /**
   * The same optics at a different resolution.
   *
   * <p>A stream that does not arrive at the calibrated size still has the same lens, and using the
   * calibrated focal length against a different frame size biases every ray, silently.
   */
  public CameraIntrinsics scaledTo(int newWidth, int newHeight) {
    if (newWidth == width && newHeight == height) {
      return this;
    }
    double sx = (double) newWidth / width;
    double sy = (double) newHeight / height;
    return new CameraIntrinsics(newWidth, newHeight, fx * sx, fy * sy, cx * sx, cy * sy, k1, k2);
  }

  /**
   * How much the warp scales a normalised image point at squared radius {@code r2}.
   *
   * <p>The single place the warp polynomial lives. The renderer calls it once per ray sample, so it
   * takes and returns plain doubles rather than allocating.
   */
  public double undistortionScale(double r2) {
    return 1.0 + k1 * r2 + k2 * r2 * r2;
  }

  /**
   * The direction a pixel looks along, lens warp removed.
   *
   * @param u horizontal pixel coordinate, may be fractional
   * @param v vertical pixel coordinate, may be fractional
   * @return a unit vector in the WPILib camera frame
   */
  public Translation3d pixelToRay(double u, double v) {
    double a = (u - cx) / fx;
    double b = (v - cy) / fy;
    double scale = undistortionScale(a * a + b * b);
    // Image right is camera -Y, image down is camera -Z.
    Translation3d ray = new Translation3d(1.0, -a * scale, -b * scale);
    return ray.div(ray.getNorm());
  }

  /**
   * Where a camera-frame direction lands in the image, lens warp included.
   *
   * <p>The inverse of {@link #pixelToRay}. The warp has no closed-form inverse, so the distorted
   * radius is found by Newton's method on {@code rd (1 + k1 rd^2 + k2 rd^4) = ru}.
   *
   * @param direction any vector in the WPILib camera frame; its length does not matter
   * @return the pixel, which may lie outside the frame, or empty when the direction is behind the
   *     camera or so far off axis that the warp folds back on itself
   */
  public Optional<Translation2d> rayToPixel(Translation3d direction) {
    if (direction.getX() <= 0.0) {
      return Optional.empty();
    }
    double au = -direction.getY() / direction.getX();
    double bu = -direction.getZ() / direction.getX();
    double ru = Math.hypot(au, bu);
    if (ru == 0.0) {
      return Optional.of(new Translation2d(cx, cy));
    }

    double rd = ru;
    for (int i = 0; i < INVERSION_ITERATIONS; i++) {
      double r2 = rd * rd;
      double residual = rd * undistortionScale(r2) - ru;
      double slope = 1.0 + 3.0 * k1 * r2 + 5.0 * k2 * r2 * r2;
      if (slope <= 0.0) {
        return Optional.empty(); // past the fold, where two radii map to one ray
      }
      rd -= residual / slope;
    }

    double shrink = rd / ru;
    return Optional.of(new Translation2d(cx + fx * au * shrink, cy + fy * bu * shrink));
  }

  /**
   * The angle actually seen edge to edge along the principal row, lens warp included.
   *
   * <p>Differs from the pinhole figure {@code 2 atan(width / 2 fx)} by however much the warp bends
   * the edge rays. Anything asking "is this ball in frame" needs this one.
   */
  public double horizontalFovRad() {
    return edgeAngle(0.0, cy) + edgeAngle(width, cy);
  }

  /** As {@link #horizontalFovRad}, down the principal column. */
  public double verticalFovRad() {
    return edgeAngle(cx, 0.0) + edgeAngle(cx, height);
  }

  /**
   * The pinhole diagonal field of view, warp ignored.
   *
   * <p>What a consumer that only models a pinhole with square pixels needs in order to arrive at
   * the same focal length as this lens. The coprocessor is one: it derives its focal length from
   * the frame diagonal and this angle.
   */
  public double pinholeDiagonalFovDegrees() {
    double focal = Math.sqrt(fx * fy);
    return Math.toDegrees(2.0 * Math.atan((Math.hypot(width, height) / 2.0) / focal));
  }

  /** Angle between the optical axis and the ray through a pixel. */
  private double edgeAngle(double u, double v) {
    return Math.acos(pixelToRay(u, v).getX());
  }
}
