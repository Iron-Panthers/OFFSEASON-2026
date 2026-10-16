package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.object_detection.ObjectDetectionConstants;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * The lens model shared by the renderer and robot code.
 *
 * <p>The round trip is what matters: the renderer goes pixel to ray, the forward direction here
 * goes ray to pixel, and a test that builds a pixel from a known ball position is only honest if
 * the two are exact inverses.
 */
class CameraIntrinsicsTest {
  private static final CameraIntrinsics LENS = ObjectDetectionConstants.CAMERA_INTRINSICS;

  /**
   * A lens with real barrel warp, the kind a wide M12 has. The detection camera is modelled as a
   * pinhole, like the Limelight it stands in for, so the warp is exercised here instead.
   */
  private static final CameraIntrinsics WARPED =
      new CameraIntrinsics(480, 360, 374.0, 374.0, 240.0, 180.0, -0.105, 0.021);

  @Test
  void pixelToRayAndBackIsExactAcrossTheWholeFrame() {
    for (CameraIntrinsics lens : new CameraIntrinsics[] {LENS, WARPED}) {
      for (double u = 0; u <= lens.width(); u += lens.width() / 12.0) {
        for (double v = 0; v <= lens.height(); v += lens.height() / 9.0) {
          Optional<Translation2d> back = lens.rayToPixel(lens.pixelToRay(u, v));
          assertTrue(back.isPresent(), "no inverse at " + u + ", " + v);
          assertEquals(u, back.get().getX(), 1e-9, "u at " + u + ", " + v);
          assertEquals(v, back.get().getY(), 1e-9, "v at " + u + ", " + v);
        }
      }
    }
  }

  @Test
  void raysFollowTheWpilibCameraFrame() {
    // Right of centre looks to the camera's right, which is -Y. Below centre looks down, -Z. A
    // sign slip here mirrors every detection and still produces plausible-looking positions.
    Translation3d right = LENS.pixelToRay(LENS.width(), LENS.cy());
    Translation3d below = LENS.pixelToRay(LENS.cx(), LENS.height());
    assertTrue(right.getY() < 0.0 && Math.abs(right.getZ()) < 1e-12);
    assertTrue(below.getZ() < 0.0 && Math.abs(below.getY()) < 1e-12);
    assertEquals(1.0, LENS.pixelToRay(LENS.cx(), LENS.cy()).getX(), 1e-12);
  }

  @Test
  void theWarpBendsEdgeRaysAwayFromThePinholeOnes() {
    // With the warp the edge pixel looks along a different angle from the pinhole one. If this
    // ever reads equal, the warp has silently stopped being applied.
    double pinholeHalfAngle = Math.atan((WARPED.width() / 2.0) / WARPED.fx());
    double actualHalfAngle = WARPED.horizontalFovRad() / 2.0;
    assertTrue(Math.abs(pinholeHalfAngle - actualHalfAngle) > Math.toRadians(0.5));
  }

  @Test
  void theDetectionCameraSeesWhatALimelight3Sees() {
    // Limelight's hardware comparison gives a Limelight 3 as 62.5 by 48.9 degrees. The simulated
    // camera, the ground-truth sim and the pool's in-view check all have to agree with it.
    assertEquals(62.5, Math.toDegrees(ObjectDetectionConstants.HORIZONTAL_FOV_RAD), 0.05);
    assertEquals(48.9, Math.toDegrees(ObjectDetectionConstants.VERTICAL_FOV_RAD), 0.05);
  }

  @Test
  void aDifferentResolutionOfTheSameLensLooksTheSameWay() {
    CameraIntrinsics doubled = WARPED.scaledTo(WARPED.width() * 2, WARPED.height() * 2);
    Translation3d original = WARPED.pixelToRay(37.0, 301.5);
    Translation3d scaled = doubled.pixelToRay(74.0, 603.0);
    assertEquals(0.0, original.minus(scaled).getNorm(), 1e-12);
  }

  @Test
  void behindTheCameraHasNoPixel() {
    assertTrue(LENS.rayToPixel(new Translation3d(-1.0, 0.1, 0.1)).isEmpty());
  }
}
