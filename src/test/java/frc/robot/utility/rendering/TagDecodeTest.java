package frc.robot.utility.rendering;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionConstants;
import java.awt.image.BufferedImage;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Runs the real AprilTag detector over rendered frames.
 *
 * <p>This is the test that decides whether any of the rest of this is worth having. A frame can
 * look convincing and still be useless: if the tag bitmap is mirrored, rotated, scaled wrong, or
 * placed on the wrong face of its panel, the picture looks fine and every pipeline built on it is
 * wrong. Asking WPILib's own detector to read the tag back and report the ID the field layout says
 * should be there checks the whole chain at once, from the glTF coordinate change through the decal
 * basis to the lens distortion.
 */
class TagDecodeTest {

  // Rendered at the same resolution and field of view the simulated cameras already use, so a
  // detection result here says something about the real pipeline rather than about a made-up one.
  private static final int WIDTH = VisionConstants.SIM_CAMERA_WIDTH_PX;

  private static final int HEIGHT = VisionConstants.SIM_CAMERA_HEIGHT_PX;

  private static final double FOV_DEGREES = VisionConstants.SIM_CAMERA_FOV_DIAGONAL_DEGREES;

  private static FieldScene scene;
  private static ExecutorService workers;
  private static Renderer renderer;
  private static AprilTagFieldLayout layout;

  @BeforeAll
  static void setUp() throws Exception {
    // The detector takes an OpenCV Mat, and both are native. GradleRIO extracts the natives for
    // the test JVM but only wires up the WPILib ones, so the OpenCV Java binding is loaded by hand.
    HAL.initialize(500, 0);
    loadOpenCv();

    layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    scene =
        FieldScene.load(
            Path.of("advantage_scope_files", "Field3d_2026FRCFieldV2"),
            Path.of("advantage_scope_files", "AprilTag_36h11"),
            layout,
            Path.of("build", "render-cache"));
    int threads = Math.max(1, Runtime.getRuntime().availableProcessors());
    workers = Executors.newFixedThreadPool(threads);
    renderer =
        new Renderer(
            scene, ArenaLighting.forField(scene.fieldLength, scene.fieldWidth), workers, threads);
  }

  /**
   * Loads the OpenCV Java binding from wherever GradleRIO extracted it.
   *
   * <p>Found by search rather than by name because the file carries the OpenCV version, and pinning
   * that here would turn a routine WPILib bump into a mysterious test failure.
   */
  private static void loadOpenCv() throws Exception {
    Path jniDirectory = Path.of("build", "jni", "release");
    try (var files = Files.list(jniDirectory)) {
      Path binding =
          files
              .filter(file -> file.getFileName().toString().startsWith("opencv_java"))
              .findFirst()
              .orElseThrow(
                  () ->
                      new IllegalStateException(
                          "No OpenCV Java binding under " + jniDirectory.toAbsolutePath()));
      System.load(binding.toAbsolutePath().toString());
    }
  }

  @AfterAll
  static void tearDown() {
    workers.shutdownNow();
    HAL.shutdown();
  }

  /** Renders a camera standing {@code range} metres out from a tag, square to its face. */
  private BufferedImage lookAtTag(int tagId, double range) {
    Pose3d tag = layout.getTagPose(tagId).orElseThrow();
    double facing = tag.getRotation().getZ();

    Pose3d robot =
        new Pose3d(
            new Translation3d(
                tag.getX() + Math.cos(facing) * range, tag.getY() + Math.sin(facing) * range, 0),
            new Rotation3d(0, 0, facing + Math.PI));
    // Put the lens at the tag's own height so the tag lands in the middle of the frame.
    Transform3d mounting =
        new Transform3d(new Translation3d(0.0, 0, tag.getZ()), new Rotation3d(0, 0, 0));

    RenderCamera camera = new RenderCamera(WIDTH, HEIGHT, FOV_DEGREES);
    camera.place(robot, mounting);

    // Two samples plus the denoiser, which is what the live streams run at. Rendering these
    // checks at a higher quality than the engine actually delivers would make them pass on frames
    // nobody will ever see.
    RenderSettings settings =
        RenderSettings.defaults().withResolution(WIDTH, HEIGHT).withSamplesPerPixel(2);
    SceneSnapshot snapshot = new SceneSnapshot(0, robot, List.of(), new float[0], 0);
    Renderer.Frame frame =
        renderer.render(camera, settings, DynamicScene.from(snapshot, null, null), 0);
    Denoiser.apply(frame, 3);
    return new Film().develop(frame, settings.sensorGain());
  }

  /** Feeds an image to the detector as the 8 bit greyscale Mat it expects. */
  private static AprilTagDetection[] detect(BufferedImage image) {
    int width = image.getWidth();
    int height = image.getHeight();
    byte[] grey = new byte[width * height];
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int rgb = image.getRGB(x, y);
        int luminance =
            (((rgb >> 16) & 0xFF) * 54 + ((rgb >> 8) & 0xFF) * 183 + (rgb & 0xFF) * 19) >> 8;
        grey[y * width + x] = (byte) luminance;
      }
    }

    Mat mat = new Mat(height, width, CvType.CV_8UC1);
    mat.put(0, 0, grey);
    try (AprilTagDetector detector = new AprilTagDetector()) {
      detector.addFamily("tag36h11");
      AprilTagDetector.Config config = detector.getConfig();
      // The real pipeline decimates for speed; this test is about whether the tag is correct, so
      // give the detector the full image and let it fail on the merits.
      config.quadDecimate = 1;
      detector.setConfig(config);
      return detector.detect(mat);
    } finally {
      mat.release();
    }
  }

  @Test
  void a_rendered_tag_decodes_to_the_id_the_field_layout_expects() {
    // Tag 1 is on the blue side wall at 0.889 m, a tag a real camera sees constantly.
    AprilTagDetection[] detections = detect(lookAtTag(1, 2.0));

    assertTrue(detections.length > 0, "the detector found no tag at all in a rendered frame");
    AprilTagDetection best = detections[0];
    for (AprilTagDetection detection : detections) {
      if (detection.getDecisionMargin() > best.getDecisionMargin()) {
        best = detection;
      }
    }
    System.out.printf(
        "DECODE tag %d  margin %.1f  hamming %d%n",
        best.getId(), best.getDecisionMargin(), best.getHamming());

    assertEquals(1, best.getId(), "the rendered tag decoded as a different tag");
    assertEquals(0, best.getHamming(), "the tag decoded only after bit correction");
    assertTrue(
        best.getDecisionMargin() > 30,
        "decision margin " + best.getDecisionMargin() + " is too weak to trust");
  }

  @Test
  void tags_across_the_field_all_decode_correctly() {
    // A mirrored or transposed decal basis still decodes for symmetric patterns, so spot check
    // tags facing all four ways: the two end walls and both sides of the hub.
    int[] ids = {1, 13, 20, 29};
    for (int id : ids) {
      AprilTagDetection[] detections = detect(lookAtTag(id, 1.6));
      boolean found = false;
      for (AprilTagDetection detection : detections) {
        if (detection.getId() == id && detection.getHamming() == 0) {
          found = true;
          System.out.printf("DECODE tag %d margin %.1f%n", id, detection.getDecisionMargin());
        }
      }
      assertTrue(found, "tag " + id + " did not decode from a rendered frame");
    }
  }

  @Test
  void a_tag_stays_detectable_out_to_shooting_range() {
    // Tag 13 sits on the far end wall, so there is clear field in front of it to back away down.
    //
    // Five metres is the honest limit for this camera, and it is an optical one rather than a
    // rendering artefact: a 6.5 inch tag subtends about 38 px at 5 m through a 70 degree lens on a
    // 1280 px sensor, and both a higher sample count and a higher resolution give the same
    // decision margin. Past about 5.5 m the trench along the side wall starts cutting the sight
    // line to this particular tag, which is a property of the field, not of the renderer.
    for (double range : new double[] {2.0, 3.0, 4.0, 5.0}) {
      AprilTagDetection[] detections = detect(lookAtTag(13, range));
      boolean found = false;
      for (AprilTagDetection detection : detections) {
        if (detection.getId() == 13 && detection.getHamming() == 0) {
          found = true;
          System.out.printf(
              "DECODE tag 13 at %.0f m  margin %.1f%n", range, detection.getDecisionMargin());
        }
      }
      assertTrue(found, "tag 13 was not detectable at " + range + " m");
    }
  }
}
