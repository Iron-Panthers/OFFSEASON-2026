package frc.robot.utility.rendering;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import javax.imageio.ImageIO;
import org.junit.jupiter.api.Test;

/** Fast mode against the path tracer at shipping defaults, across core counts. */
class FastModeBenchTmp {

  @Test
  void bench() throws Exception {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    FieldScene scene =
        FieldScene.load(
            Path.of("advantage_scope_files", "Field3d_2026FRCFieldV2"),
            Path.of("advantage_scope_files", "AprilTag_36h11"),
            layout,
            Path.of("build", "render-cache"));
    FieldScene lean =
        FieldScene.load(
            Path.of("advantage_scope_files", "Field3d_2026FRCFieldV2"),
            Path.of("advantage_scope_files", "AprilTag_36h11"),
            layout,
            Path.of("build", "render-cache"),
            false);
    System.out.printf(
        "BENCH triangles: full %,d  lean %,d (%.0f%% removed)%n",
        scene.soup.triangleCount,
        lean.soup.triangleCount,
        100.0 * (scene.soup.triangleCount - lean.soup.triangleCount) / scene.soup.triangleCount);

    ArenaLighting lighting = ArenaLighting.forField(scene.fieldLength, scene.fieldWidth);

    long t0 = System.nanoTime();
    IrradianceVolume volume =
        IrradianceVolume.bakeOrLoad(
            scene, lighting, 0.20f, Path.of("build", "render-cache"), "bench-v2");
    System.out.printf(
        "BENCH volume %,d cells, %.1f s%n", volume.cellCount(), (System.nanoTime() - t0) / 1e9);

    Pose3d robot = new Pose3d(new Translation3d(2.2, 3.4, 0), new Rotation3d(0, 0, 0.35));
    Transform3d mount =
        new Transform3d(new Translation3d(0.33, 0, 0.42), new Rotation3d(0, 0.38, 0));
    float[] centers = new float[60 * 3];
    Random r = new Random(7);
    for (int i = 0; i < 60; i++) {
      centers[i * 3] = (float) (1.5 + r.nextDouble() * 6.0);
      centers[i * 3 + 1] = (float) (0.8 + r.nextDouble() * 6.4);
      centers[i * 3 + 2] = 0.075f;
    }
    DynamicScene dyn =
        DynamicScene.from(new SceneSnapshot(0, robot, List.of(), centers, 60), null, null);
    Path out = Path.of("build", "render-preview");
    Files.createDirectories(out);

    IrradianceVolume leanVolume =
        IrradianceVolume.bakeOrLoad(
            lean, lighting, 0.20f, Path.of("build", "render-cache"), "bench-lean-v2");

    RenderSettings fast = RenderSettings.fastDefaults();
    RenderSettings fastOne = fast.withSamplesPerPixel(1);
    RenderSettings high = RenderSettings.highDefaults();

    for (int threads : new int[] {4, 8, Runtime.getRuntime().availableProcessors()}) {
      ExecutorService pool = Executors.newFixedThreadPool(threads);
      System.out.printf("--- %d threads ---%n", threads);

      for (Object[] c :
          new Object[][] {
            {"fast-2spp", fast, volume, scene},
            {"fast-1spp", fastOne, volume, scene},
            {"lean-2spp", fast, leanVolume, lean},
            {"lean-1spp", fastOne, leanVolume, lean},
            {"high", high, null, scene},
          }) {
        String name = (String) c[0];
        RenderSettings settings = (RenderSettings) c[1];
        IrradianceVolume useVolume = (IrradianceVolume) c[2];
        FieldScene useScene = (FieldScene) c[3];

        Renderer renderer = new Renderer(useScene, lighting, pool, threads, useVolume);
        RenderCamera cam = new RenderCamera(settings.width(), settings.height(), 70.0);
        cam.place(robot, mount);

        for (int i = 0; i < 4; i++) {
          renderer.render(cam, settings, dyn, i);
        }
        long best = Long.MAX_VALUE;
        for (int i = 0; i < 5; i++) {
          long s = System.nanoTime();
          Renderer.Frame f = renderer.render(cam, settings, dyn, i);
          if (settings.denoise()) {
            Denoiser.apply(f, 3);
          }
          new Film().develop(f, settings.sensorGain());
          best = Math.min(best, System.nanoTime() - s);
        }
        System.out.printf(
            "BENCH %-12s %dx%d  %6.0f ms  %5.1f FPS%n",
            name, settings.width(), settings.height(), best / 1e6, 1e9 / best);

        if (threads == 4) {
          Renderer.Frame frame = renderer.render(cam, settings, dyn, 0);
          if (settings.denoise()) {
            Denoiser.apply(frame, 3);
          }
          ImageIO.write(
              new Film().develop(frame, settings.sensorGain()),
              "png",
              out.resolve("mode-" + name + ".png").toFile());
        }
      }
      pool.shutdownNow();
    }
  }
}
