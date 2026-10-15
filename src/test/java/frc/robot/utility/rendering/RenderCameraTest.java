package frc.robot.utility.rendering;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.object_detection.ObjectDetectionConstants;
import frc.robot.subsystems.vision.VisionConstants;
import org.junit.jupiter.api.Test;

/**
 * The renderer and robot code have to agree on where a pixel looks, or every detection is placed
 * somewhere other than where the renderer drew it.
 */
class RenderCameraTest {

  @Test
  void theRenderedRayIsTheOneRobotCodeReconstructs() {
    // An arbitrary robot pose, so the camera basis is exercised in every axis rather than only at
    // the identity where sign errors cancel.
    Pose3d robot = new Pose3d(3.2, 5.1, 0.0, new Rotation3d(0.02, -0.03, 2.4));
    RenderCamera camera = new RenderCamera(ObjectDetectionConstants.CAMERA_INTRINSICS);
    camera.place(robot, ObjectDetectionConstants.ROBOT_TO_CAMERA);
    Pose3d cameraPose = robot.transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

    float[] rendered = new float[3];
    for (double u = 0.5; u < camera.width(); u += 47.0) {
      for (double v = 0.5; v < camera.height(); v += 31.0) {
        camera.ray(u, v, rendered);
        Translation3d reconstructed =
            ObjectDetectionConstants.CAMERA_INTRINSICS
                .pixelToRay(u, v)
                .rotateBy(cameraPose.getRotation());
        assertEquals(reconstructed.getX(), rendered[0], 1e-6, "x at " + u + ", " + v);
        assertEquals(reconstructed.getY(), rendered[1], 1e-6, "y at " + u + ", " + v);
        assertEquals(reconstructed.getZ(), rendered[2], 1e-6, "z at " + u + ", " + v);
      }
    }
  }

  @Test
  void visionCamerasRenderExactlyAsBefore() {
    // The vision cameras moved onto the shared lens model. This is the formula they were rendered
    // with before it, which a tag pipeline developed against the old frames depends on.
    int width = 480;
    int height = 300;
    double diagonal = Math.hypot(width, height);
    double tanHalfDiagonal =
        Math.tan(Math.toRadians(VisionConstants.SIM_CAMERA_FOV_DIAGONAL_DEGREES) / 2.0);
    double tanHalfHorizontal = tanHalfDiagonal * width / diagonal;
    double tanHalfVertical = tanHalfDiagonal * height / diagonal;

    RenderCamera camera =
        new RenderCamera(width, height, VisionConstants.SIM_CAMERA_FOV_DIAGONAL_DEGREES);
    camera.place(Pose3d.kZero, new Transform3d());

    float[] rendered = new float[3];
    for (double u = 0.25; u < width; u += 53.0) {
      for (double v = 0.75; v < height; v += 29.0) {
        double nx = (2.0 * u / width - 1.0) * tanHalfHorizontal;
        double ny = (1.0 - 2.0 * v / height) * tanHalfVertical;
        double r2 = nx * nx + ny * ny;
        double scale = 1.0 + RenderCamera.DISTORTION_K1 * r2 + RenderCamera.DISTORTION_K2 * r2 * r2;
        Translation3d old = new Translation3d(1.0, -nx * scale, ny * scale);
        old = old.div(old.getNorm());

        camera.ray(u, v, rendered);
        assertEquals(old.getX(), rendered[0], 1e-6);
        assertEquals(old.getY(), rendered[1], 1e-6);
        assertEquals(old.getZ(), rendered[2], 1e-6);
      }
    }
  }
}
