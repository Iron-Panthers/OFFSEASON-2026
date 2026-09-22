package frc.robot.utility.rendering;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.awt.image.BufferedImage;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import javax.imageio.ImageIO;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Renders a few fixed viewpoints and checks the result looks like a photograph of a field rather
 * than like a bug.
 *
 * <p>Also writes the frames to {@code build/render-preview} so they can be looked at. An automated
 * check can tell you the image is not black; only a person can tell you it looks real.
 */
class RenderPreviewTest {

  private static final Path OUTPUT = Path.of("build", "render-preview");
  private static final int WIDTH = 480;
  private static final int HEIGHT = 360;

  private static FieldScene scene;
  private static ArenaLighting lighting;
  private static ExecutorService workers;
  private static Renderer renderer;
  private static AprilTagFieldLayout layout;

  @BeforeAll
  static void setUp() throws Exception {
    layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    scene =
        FieldScene.load(
            Path.of("advantage_scope_files", "Field3d_2026FRCFieldV2"),
            Path.of("advantage_scope_files", "AprilTag_36h11"),
            layout,
            Path.of("build", "render-cache"));
    lighting = ArenaLighting.forField(scene.fieldLength, scene.fieldWidth);
    int threads = Math.max(1, Runtime.getRuntime().availableProcessors());
    workers = Executors.newFixedThreadPool(threads);
    renderer = new Renderer(scene, lighting, workers, threads);
    Files.createDirectories(OUTPUT);
  }

  @AfterAll
  static void tearDown() {
    workers.shutdownNow();
  }

  /** Scatters fuel across the blue half, roughly where it sits at the start of a match. */
  private static SceneSnapshot snapshotWithFuel(Pose3d robotPose) {
    List<Translation3d> fuel = new ArrayList<>();
    java.util.Random random = new java.util.Random(7);
    for (int i = 0; i < 120; i++) {
      fuel.add(
          new Translation3d(
              1.5 + random.nextDouble() * 6.0,
              0.8 + random.nextDouble() * 6.4,
              SphereIndex.FUEL_RADIUS));
    }
    return SceneSnapshot.of(0.0, robotPose, List.of(), fuel.toArray(new Translation3d[0]));
  }

  private BufferedImage renderFrom(String name, Pose3d robotPose, Transform3d mounting)
      throws Exception {
    RenderSettings settings =
        RenderSettings.defaults().withResolution(WIDTH, HEIGHT).withSamplesPerPixel(4);

    RenderCamera camera = new RenderCamera(WIDTH, HEIGHT, 70.0);
    camera.place(robotPose, mounting);

    SceneSnapshot snapshot = snapshotWithFuel(robotPose);
    DynamicScene dynamic = DynamicScene.from(snapshot, null, null);

    long started = System.nanoTime();
    Renderer.Frame frame = renderer.render(camera, settings, dynamic, 0);
    long traced = System.nanoTime();
    if (settings.denoise()) {
      Denoiser.apply(frame, 3);
    }
    BufferedImage image = new Film().develop(frame, settings.sensorGain());
    long finished = System.nanoTime();

    System.out.printf(
        "RENDER %-18s trace %5.0f ms  post %4.0f ms  (%dx%d, %d spp)%n",
        name,
        (traced - started) / 1e6,
        (finished - traced) / 1e6,
        WIDTH,
        HEIGHT,
        settings.samplesPerPixel());
    ImageIO.write(image, "png", OUTPUT.resolve(name + ".png").toFile());
    return image;
  }

  @Test
  void renders_the_field_from_a_camera_on_the_robot() throws Exception {
    // Open carpet on the blue half looking down the length of the field. Not the centre line:
    // the Outpost sits right there, and a camera two metres from a flat wall shows nothing.
    Pose3d robot = new Pose3d(new Translation3d(2.5, 2.0, 0), new Rotation3d(0, 0, 0));
    Transform3d mounting =
        new Transform3d(new Translation3d(0.33493633, 0, 0.422076702), new Rotation3d(0, -0.09, 0));

    BufferedImage image = renderFrom("downfield", robot, mounting);
    assertImageIsPlausible(image, "downfield");
  }

  @Test
  void renders_a_close_view_of_fuel_on_the_carpet() throws Exception {
    // Low and close, the view a ball detector actually works from.
    Pose3d robot = new Pose3d(new Translation3d(2.2, 3.4, 0), new Rotation3d(0, 0, 0.35));
    Transform3d mounting =
        new Transform3d(new Translation3d(0.33, 0, 0.42), new Rotation3d(0, 0.38, 0));

    BufferedImage image = renderFrom("fuel-closeup", robot, mounting);
    assertImageIsPlausible(image, "fuel-closeup");
  }

  @Test
  void renders_an_apriltag_head_on() throws Exception {
    // Two metres out from tag 1, square to its face, which is where a tag pipeline lives.
    Pose3d tag = layout.getTagPose(1).orElseThrow();
    double facing = tag.getRotation().getZ();
    Pose3d robot =
        new Pose3d(
            new Translation3d(
                tag.getX() + Math.cos(facing) * 2.0, tag.getY() + Math.sin(facing) * 2.0, 0),
            new Rotation3d(0, 0, facing + Math.PI));
    Transform3d mounting =
        new Transform3d(new Translation3d(0.3, 0, 0.889), new Rotation3d(0, 0, 0));

    BufferedImage image = renderFrom("apriltag", robot, mounting);
    assertImageIsPlausible(image, "apriltag");
  }

  /**
   * Catches the failures that actually happen: an all-black frame from a broken camera basis, an
   * all-white one from runaway exposure, and a flat one from geometry that never got hit.
   */
  private static void assertImageIsPlausible(BufferedImage image, String name) {
    long total = 0;
    int minimum = 255;
    int maximum = 0;
    int[] histogram = new int[256];

    for (int y = 0; y < image.getHeight(); y++) {
      for (int x = 0; x < image.getWidth(); x++) {
        int rgb = image.getRGB(x, y);
        int luminance =
            (((rgb >> 16) & 0xFF) * 54 + ((rgb >> 8) & 0xFF) * 183 + (rgb & 0xFF) * 19) >> 8;
        total += luminance;
        minimum = Math.min(minimum, luminance);
        maximum = Math.max(maximum, luminance);
        histogram[luminance]++;
      }
    }

    double mean = (double) total / (image.getWidth() * image.getHeight());
    int occupiedBuckets = 0;
    for (int count : histogram) {
      if (count > 0) {
        occupiedBuckets++;
      }
    }

    System.out.printf(
        "RENDER %-18s mean %.1f  range %d..%d  distinct levels %d%n",
        name, mean, minimum, maximum, occupiedBuckets);

    assertTrue(mean > 12 && mean < 235, name + " is not exposed sensibly, mean " + mean);
    assertTrue(maximum - minimum > 60, name + " has almost no contrast");
    assertTrue(occupiedBuckets > 40, name + " only uses " + occupiedBuckets + " brightness levels");
  }
}
