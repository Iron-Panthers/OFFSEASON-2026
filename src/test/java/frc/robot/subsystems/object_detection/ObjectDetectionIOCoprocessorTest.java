package frc.robot.subsystems.object_detection;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utility.CameraIntrinsics;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * Turning a detection's pixel back into a ball on the field.
 *
 * <p>Each case starts from a known ball, projects it into the image through the lens model, and
 * checks the IO's geometry recovers it. The pixel is the projection of the ball's centre, not a box
 * centre, so this pins the geometry and leaves the detector's own box error out of it.
 */
class ObjectDetectionIOCoprocessorTest {
  private static final CameraIntrinsics LENS = ObjectDetectionConstants.CAMERA_INTRINSICS;
  private static final double R = ObjectDetectionConstants.BALL_RADIUS_M;

  private static final Pose3d CAMERA =
      new Pose3d(new Pose2d(6.1, 0.8, Rotation2d.fromDegrees(35.0)))
          .transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

  /** Pixel a resting ball's centre lands on, or empty if it is out of frame. */
  private static Optional<Translation2d> pixelOf(Translation3d ball) {
    Translation3d inCamera =
        ball.minus(CAMERA.getTranslation()).rotateBy(CAMERA.getRotation().unaryMinus());
    return LENS.rayToPixel(inCamera)
        .filter(
            p ->
                p.getX() >= 0
                    && p.getX() <= LENS.width()
                    && p.getY() >= 0
                    && p.getY() <= LENS.height());
  }

  @Test
  void recoversBallsAcrossTheFrameIncludingTheWarpedEdges() {
    int checked = 0;
    // A sweep of floor points out along the camera's heading, where it looks.
    Rotation2d heading = new Rotation2d(CAMERA.getRotation().getZ());
    for (double forward = 0.6; forward <= 5.5; forward += 0.35) {
      for (double lateral = -3.0; lateral <= 3.0; lateral += 0.3) {
        Translation2d floor =
            CAMERA
                .getTranslation()
                .toTranslation2d()
                .plus(new Translation2d(forward, lateral).rotateBy(heading));
        Translation3d ball = new Translation3d(floor.getX(), floor.getY(), R);
        Optional<Translation2d> pixel = pixelOf(ball);
        if (pixel.isEmpty()) {
          continue;
        }
        Optional<Translation3d> placed =
            ObjectDetectionIOCoprocessor.projectToBallPlane(
                CAMERA, LENS.pixelToRay(pixel.get().getX(), pixel.get().getY()));
        if (placed.isEmpty()) {
          continue; // outside the reported range
        }
        assertEquals(0.0, placed.get().minus(ball).getNorm(), 1e-6, "ball at " + ball);
        checked++;
      }
    }
    assertTrue(checked > 50, "the sweep should cover much of the frame, covered " + checked);
  }

  @Test
  void ignoringTheWarpMisplacesBallsNearTheEdge() {
    // What this whole arrangement exists to prevent. A pinhole model reading the rendered frame
    // puts a ball out towards the side of the image well away from where it is. Above centre, so
    // the ball is a few metres out, where a small bearing error becomes a large range error.
    CameraIntrinsics pinhole =
        new CameraIntrinsics(
            LENS.width(), LENS.height(), LENS.fx(), LENS.fy(), LENS.cx(), LENS.cy(), 0.0, 0.0);
    double u = LENS.width() * 0.95;
    double v = LENS.height() * 0.30;
    Translation3d truth =
        ObjectDetectionIOCoprocessor.projectToBallPlane(CAMERA, LENS.pixelToRay(u, v))
            .orElseThrow();
    Translation3d naive =
        ObjectDetectionIOCoprocessor.projectToBallPlane(CAMERA, pinhole.pixelToRay(u, v))
            .orElseThrow();
    assertTrue(
        naive.minus(truth).getNorm() > 0.10,
        "expected the warp to matter by more than 10 cm, got " + naive.minus(truth).getNorm());
  }

  @Test
  void aRayAboveTheHorizonIsNotABall() {
    assertTrue(
        ObjectDetectionIOCoprocessor.projectToBallPlane(CAMERA, LENS.pixelToRay(LENS.cx(), 0.0))
            .isEmpty());
  }

  @Test
  void aBallBeyondReportedRangeIsDropped() {
    // A ray just below the horizon does meet the floor, but tens of metres out, far beyond where
    // the
    // detector is trusted.
    Translation3d nearHorizon = new Translation3d(1.0, 0.0, -0.01);
    Translation3d cameraFrame = nearHorizon.rotateBy(CAMERA.getRotation().unaryMinus());
    assertTrue(ObjectDetectionIOCoprocessor.projectToBallPlane(CAMERA, cameraFrame).isEmpty());
  }
}
