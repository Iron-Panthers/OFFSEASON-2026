package frc.robot.utility.coprocessor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
  void objectDetectionWatchesTheCameraServedOn1193() {
    // The third camera in CAMERA_TRANSFORM, which the renderer serves on 1191 + 2.
    assertEquals(2, CoprocessorModule.OBJDETECT.defaultCamera());
    assertEquals(1193, RenderingEngine.BASE_PORT + CoprocessorModule.OBJDETECT.defaultCamera());
  }

  @Test
  void idsListingIsNotEmpty() {
    assertNotEquals("", CoprocessorModule.ids());
    assertTrue(CoprocessorModule.ids().contains("objdetect"));
  }
}
