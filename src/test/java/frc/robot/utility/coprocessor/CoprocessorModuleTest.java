package frc.robot.utility.coprocessor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.object_detection.ObjectDetectionConstants;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.utility.rendering.RenderingEngine;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * The module registry.
 *
 * <p>Process spawning is deliberately not tested here: it needs a Python install and a live
 * renderer, which belongs in a manual run rather than in the build. What is worth pinning is that a
 * typo fails loudly and that no two modules collide on a port.
 */
class CoprocessorModuleTest {

  @Test
  void lookupIsCaseInsensitive() {
    assertEquals(CoprocessorModule.OBJDETECT, CoprocessorModule.byId("objdetect"));
    assertEquals(CoprocessorModule.OBJDETECT, CoprocessorModule.byId("OBJDETECT"));
    assertEquals(CoprocessorModule.OBJDETECT, CoprocessorModule.byId("  ObjDetect  "));
  }

  @Test
  void unknownModuleNamesTheValidOnes() {
    // A typo that silently starts nothing is the worst outcome here, so the message has to say
    // what was actually available.
    IllegalArgumentException thrown =
        assertThrows(IllegalArgumentException.class, () -> CoprocessorModule.byId("objdetct"));
    assertTrue(thrown.getMessage().contains("objdetct"), "should quote what was asked for");
    assertTrue(thrown.getMessage().contains("objdetect"), "should list what exists");
  }

  @Test
  void everyModuleHasADistinctId() {
    Set<String> seen = new HashSet<>();
    for (CoprocessorModule module : CoprocessorModule.values()) {
      assertTrue(seen.add(module.id()), "duplicate module id: " + module.id());
    }
  }

  @Test
  void outputPortsDoNotCollideWithTheRenderer() {
    // The renderer takes 1191 upward, one port per camera, and PhotonVision's simulation already
    // holds 1181 upward. An annotated stream landing in either range binds nothing and the module
    // appears to start fine.
    for (CoprocessorModule module : CoprocessorModule.values()) {
      for (int camera = 0; camera < 8; camera++) {
        int port = module.outputPort(camera);
        assertTrue(
            port > RenderingEngine.BASE_PORT + 8,
            module.id() + " camera " + camera + " lands on " + port + ", inside the render range");
      }
    }
  }

  @Test
  void outputPortsAreUniquePerCamera() {
    Set<Integer> seen = new HashSet<>();
    for (int camera = 0; camera < 8; camera++) {
      assertTrue(
          seen.add(CoprocessorModule.OBJDETECT.outputPort(camera)),
          "two cameras share an output port");
    }
  }

  @Test
  void objectDetectionWatchesItsOwnCameraAndNotAVisionCamera() {
    // Not one of the vision cameras, and this is the point rather than an accident. Those are
    // pitched up to put AprilTags on walls in frame, which leaves them unable to see floor
    // anywhere near the robot; aimed at one of them the detector measurably finds nothing. The
    // object-detection camera is ObjectDetectionConstants.ROBOT_TO_CAMERA, tilted down at the
    // floor, and the renderer serves it immediately after the vision cameras.
    int expected = VisionConstants.CAMERA_TRANSFORM.length;
    assertEquals(expected, CoprocessorModule.OBJDETECT.defaultCamera());
    assertEquals(expected, RenderingEngine.objectDetectionCameraIndex());

    for (int vision = 0; vision < VisionConstants.CAMERA_TRANSFORM.length; vision++) {
      assertNotEquals(
          vision,
          CoprocessorModule.OBJDETECT.defaultCamera(),
          "the detector must not default to a vision camera");
    }
  }

  @Test
  void theObjectDetectionCameraActuallyLooksDown() {
    // The failure this guards is silent: flip the sign and every frame still renders, still
    // looks like a field, and simply contains no fuel. Positive pitch is nose down in WPILib's
    // Rotation3d, so this must stay positive.
    assertTrue(
        ObjectDetectionConstants.ROBOT_TO_CAMERA.getRotation().getY() > 0.0,
        "the object-detection camera must be pitched down to see fuel on the floor");
    assertTrue(
        ObjectDetectionConstants.ROBOT_TO_CAMERA.getZ() > ObjectDetectionConstants.BALL_RADIUS_M,
        "the camera must sit above a ball, or ground-plane ranging has no intersection");
  }

  @Test
  void theRenderedFieldOfViewMatchesTheCalibratedOne() {
    // The coprocessor derives its focal length from the diagonal figure, so it drifting away
    // from the horizontal and vertical ones biases every range the detector reports.
    double halfDiagonal = Math.toRadians(ObjectDetectionConstants.CAMERA_DIAGONAL_FOV_DEGREES) / 2;
    double expected =
        Math.hypot(
            Math.tan(ObjectDetectionConstants.HORIZONTAL_FOV_RAD / 2),
            Math.tan(ObjectDetectionConstants.VERTICAL_FOV_RAD / 2));
    assertEquals(expected, Math.tan(halfDiagonal), 1e-9);
  }

  @Test
  void idsListingIsNotEmpty() {
    assertNotEquals("", CoprocessorModule.ids());
    assertTrue(CoprocessorModule.ids().contains("objdetect"));
  }
}
