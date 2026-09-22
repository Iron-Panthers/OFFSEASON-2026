package frc.robot.utility.rendering;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import java.nio.file.Path;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Checks that the vendored field export really lands where the rest of the robot code thinks the
 * field is.
 *
 * <p>Everything downstream of this is a rendering detail. If the geometry is in the wrong place, or
 * the AprilTag panels disagree with the layout the pose estimator uses, then frames produced by the
 * renderer would train and validate vision code against a field that does not exist.
 */
class FieldSceneTest {

  private static final Path FIELD_DIRECTORY =
      Path.of("advantage_scope_files", "Field3d_2026FRCFieldV2");
  private static final Path TAG_DIRECTORY = Path.of("advantage_scope_files", "AprilTag_36h11");
  private static final Path CACHE_DIRECTORY = Path.of("build", "render-cache");

  private static AprilTagFieldLayout layout;
  private static FieldScene scene;

  @BeforeAll
  static void loadOnce() throws Exception {
    layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    scene = FieldScene.load(FIELD_DIRECTORY, TAG_DIRECTORY, layout, CACHE_DIRECTORY);
  }

  @Test
  void staged_fuel_is_stripped_from_the_field_mesh() {
    // The export bakes 455 balls into the field at 295 488 triangles. Live fuel replaces them, so
    // none of that geometry should survive into the soup.
    assertTrue(
        scene.soup.triangleCount > 3_500_000,
        "expected the bulk of the field to load, got " + scene.soup.triangleCount);
    assertTrue(
        scene.soup.triangleCount < 4_100_000,
        "staged fuel looks like it is still in the mesh: " + scene.soup.triangleCount);
  }

  @Test
  void geometry_sits_inside_the_wpilib_field_volume() {
    float minX = Float.POSITIVE_INFINITY;
    float maxX = Float.NEGATIVE_INFINITY;
    float minY = Float.POSITIVE_INFINITY;
    float maxY = Float.NEGATIVE_INFINITY;
    float minZ = Float.POSITIVE_INFINITY;
    float maxZ = Float.NEGATIVE_INFINITY;
    for (int v = 0; v < scene.soup.positions.length; v += 3) {
      minX = Math.min(minX, scene.soup.positions[v]);
      maxX = Math.max(maxX, scene.soup.positions[v]);
      minY = Math.min(minY, scene.soup.positions[v + 1]);
      maxY = Math.max(maxY, scene.soup.positions[v + 1]);
      minZ = Math.min(minZ, scene.soup.positions[v + 2]);
      maxZ = Math.max(maxZ, scene.soup.positions[v + 2]);
    }

    // The carpet runs well past the playing area on both axes, so the model is wider than the
    // field, but it must be centred on it and sit on z = 0.
    double centreX = (minX + maxX) / 2;
    double centreY = (minY + maxY) / 2;
    assertEquals(layout.getFieldLength() / 2, centreX, 0.05, "field is not centred along x");
    assertEquals(layout.getFieldWidth() / 2, centreY, 0.05, "field is not centred along y");
    assertEquals(0.0, minZ, 0.02, "the carpet should rest on the floor plane");
    assertTrue(maxZ > 2.5 && maxZ < 4.0, "unexpected field height " + maxZ);
  }

  @Test
  void every_tag_panel_agrees_with_the_field_layout() {
    assertTrue(
        scene.tagAlignmentErrorMeters < 0.02,
        "worst tag panel is "
            + String.format("%.4f", scene.tagAlignmentErrorMeters)
            + " m from the layout pose");
  }

  @Test
  void all_thirty_two_tags_are_present_and_decalled() {
    boolean[] seen = new boolean[33];
    for (Surface surface : scene.soup.surfaces) {
      if (surface.decal() != null) {
        seen[surface.decal().id()] = true;
      }
    }
    for (int id = 1; id <= 32; id++) {
      assertTrue(seen[id], "no decal was built for tag " + id);
    }
  }

  /**
   * Open carpet on the blue half, clear of the hub at midfield and of the trench along the walls.
   * Verified by probing a grid across the field.
   */
  private static final float OPEN_CARPET_X = 2.5f;

  private static final float OPEN_CARPET_Y = 2.0f;

  @Test
  void a_ray_dropped_from_above_open_carpet_lands_on_the_floor() {
    Bvh.Scratch scratch = new Bvh.Scratch();
    assertTrue(
        scene.bvh.intersect(
            scene.soup, OPEN_CARPET_X, OPEN_CARPET_Y, 3.0f, 0, 0, -1, 10f, scratch),
        "nothing under an open patch of the field");
    assertEquals(
        3.0, scratch.hit.distance, 0.05, "the first surface below should be the carpet at z = 0");
  }

  @Test
  void looking_at_a_tag_hits_its_decal() {
    // Tag 1 sits on the blue side wall. Stand a metre out along its own normal and look back.
    Pose3d tag = layout.getTagPose(1).orElseThrow();
    var normal = tag.getRotation().getZ();
    float standX = (float) (tag.getX() + Math.cos(normal));
    float standY = (float) (tag.getY() + Math.sin(normal));
    float standZ = (float) tag.getZ();

    float dx = (float) (tag.getX() - standX);
    float dy = (float) (tag.getY() - standY);
    float length = (float) Math.hypot(dx, dy);

    Bvh.Scratch scratch = new Bvh.Scratch();
    assertTrue(
        scene.bvh.intersect(scene.soup, standX, standY, standZ, dx / length, dy / length, 0, 5f, scratch),
        "no geometry between the viewpoint and tag 1");

    Surface surface = scene.soup.surfaces.get(scene.soup.surfaceIds[scratch.hit.triangle]);
    assertNotNull(surface.decal(), "the first surface in front of tag 1 is not the tag panel");
    assertEquals(1, surface.decal().id());
  }

  @Test
  void carpet_is_not_rendered_as_polished_metal() {
    // The exporter marks every material metallic. If that leaks through, the floor mirrors the
    // ceiling and the whole frame reads as CG.
    Bvh.Scratch scratch = new Bvh.Scratch();
    scene.bvh.intersect(scene.soup, OPEN_CARPET_X, OPEN_CARPET_Y, 3.0f, 0, 0, -1, 10f, scratch);

    Surface carpet = scene.soup.surfaces.get(scene.soup.surfaceIds[scratch.hit.triangle]);
    assertEquals(0f, carpet.metallic(), "carpet must be a dielectric");
    assertTrue(carpet.roughness() > 0.8f, "carpet must be rough, got " + carpet.roughness());
    assertEquals(Surface.Pattern.CARPET, carpet.pattern(), "carpet needs its procedural weave");
  }
}
