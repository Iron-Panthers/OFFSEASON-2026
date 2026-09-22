package frc.robot.utility.rendering;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionConstants;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.net.BindException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Renders what the robot's cameras would see during a match and serves each one as a motion JPEG
 * stream on localhost.
 *
 * <p>This is the entry point; everything else in the package supports it. Start it once in
 * simulation, call {@link #update} from the periodic loop, and each camera in {@link
 * VisionConstants#CAMERA_TRANSFORM} appears on its own port starting at {@value #BASE_PORT},
 * following PhotonVision's convention so existing tooling can consume them unchanged.
 *
 * <p>Simulation only, and deliberately so. Loading the field pulls in about four million triangles
 * and the developing stage needs {@code java.awt}; neither belongs anywhere near a roboRIO. {@link
 * #start} refuses to run outside {@link Constants.Mode#SIM}, so the classes are never even loaded
 * on real hardware.
 *
 * <h2>Threading</h2>
 *
 * <p>The robot loop only ever publishes an immutable {@link SceneSnapshot} into an atomic
 * reference, which costs it a few microseconds. Rendering happens on its own threads at whatever
 * rate it can manage, typically a few frames a second, and simply uses the most recent snapshot.
 * The 20 ms loop is never blocked waiting on a frame, and a frame is never torn across two
 * simulation instants.
 *
 * <h2>Enabling</h2>
 *
 * <pre>
 *   ./gradlew simulateJava -Prender
 *   ./gradlew simulateJava -Prender -Prender.width=1280 -Prender.height=800 -Prender.spp=4
 * </pre>
 */
public final class RenderingEngine {

  /**
   * First port served, one camera per port upward from here.
   *
   * <p>PhotonVision's own convention starts at 1181, and this deliberately does not: the
   * PhotonVision simulation already running in this process binds 1181 upward for its own camera
   * streams, so sharing the base means the renderer loses the race and never starts. 1191 keeps the
   * same one-port-per-camera shape without the collision. Override with {@code -Prender.port}.
   */
  public static final int BASE_PORT = 1191;

  /** Ports to try past the requested one before giving up on a camera. */
  private static final int PORT_SEARCH_RANGE = 16;

  private static final Path FIELD_ASSETS =
      Path.of("advantage_scope_files", "Field3d_2026FRCFieldV2");
  private static final Path TAG_ASSETS = Path.of("advantage_scope_files", "AprilTag_36h11");
  private static final Path ROBOT_ASSET =
      Path.of("advantage_scope_files", "Robot_2026FRC", "model.glb");
  private static final Path CACHE = Path.of("build", "render-cache");

  /**
   * Grid spacing for the baked static lighting.
   *
   * <p>Coarse on purpose. The truss casts penumbrae around half a metre wide, so there is no shadow
   * on this field sharp enough for a finer grid to resolve.
   */
  private static final float VOLUME_CELL_SIZE_METERS = 0.20f;

  private static RenderingEngine instance;

  private final RenderSettings settings;
  private final AtomicReference<SceneSnapshot> published =
      new AtomicReference<>(SceneSnapshot.empty());

  private FieldScene scene;
  private ArenaLighting lighting;
  private Renderer renderer;
  private RobotModel robotModel;
  private ExecutorService workers;
  private Thread renderLoop;
  private final List<CameraStream> cameras = new ArrayList<>();

  private volatile boolean running;
  private volatile double lastFrameSeconds;
  private volatile long framesRendered;

  /** One camera: its mounting on the robot, its ray generator and its HTTP endpoint. */
  private record CameraStream(
      int index, Transform3d mounting, RenderCamera camera, MjpegServer server, Film film) {}

  private RenderingEngine(RenderSettings settings) {
    this.settings = settings;
  }

  /**
   * Starts the engine if this run asked for it.
   *
   * <p>Safe to call unconditionally: it returns without doing anything outside simulation, or when
   * {@code -Drender.enabled} was not set.
   *
   * @return the running engine, or null if rendering is not enabled for this run
   */
  public static synchronized RenderingEngine startIfEnabled() {
    if (instance != null) {
      return instance;
    }
    if (Constants.getRobotMode() != Constants.Mode.SIM) {
      return null;
    }
    if (!Boolean.getBoolean("render.enabled")) {
      return null;
    }

    boolean high = "high".equalsIgnoreCase(System.getProperty("render.mode", "fast"));
    RenderSettings base = high ? RenderSettings.highDefaults() : RenderSettings.fastDefaults();

    RenderSettings settings =
        new RenderSettings(
            Integer.getInteger("render.width", base.width()),
            Integer.getInteger("render.height", base.height()),
            Integer.getInteger("render.spp", base.samplesPerPixel()),
            Integer.getInteger("render.bounces", base.diffuseBounces()),
            Integer.getInteger("render.transparency", base.transparencyDepth()),
            booleanProperty("render.denoise", base.denoise()),
            floatProperty("render.gain", base.sensorGain()),
            floatProperty("render.jpeg", base.jpegQuality()),
            base.quality());

    RenderingEngine engine = new RenderingEngine(settings);
    try {
      engine.start();
    } catch (IOException | RuntimeException failed) {
      System.err.println("[Rendering] Disabled: " + failed.getMessage());
      failed.printStackTrace();
      engine.stop();
      return null;
    }
    instance = engine;
    return engine;
  }

  /**
   * @return the running engine, or null if rendering is not enabled
   */
  public static synchronized RenderingEngine getInstance() {
    return instance;
  }

  private void start() throws IOException {
    long began = System.nanoTime();
    scene =
        FieldScene.load(FIELD_ASSETS, TAG_ASSETS, VisionConstants.APRIL_TAG_FIELD_LAYOUT, CACHE);
    lighting = ArenaLighting.forField(scene.fieldLength, scene.fieldWidth);

    if (Files.isRegularFile(ROBOT_ASSET)) {
      robotModel = RobotModel.load(ROBOT_ASSET);
    } else {
      System.out.println("[Rendering] No robot model at " + ROBOT_ASSET + "; field only.");
    }

    // Leave a core for the simulation itself, which still has to hit its 20 ms deadline.
    int threads = Math.max(1, Runtime.getRuntime().availableProcessors() - 1);
    workers =
        Executors.newFixedThreadPool(
            threads,
            runnable -> {
              Thread thread = new Thread(runnable, "RenderWorker");
              thread.setDaemon(true);
              // Below the simulation, so a slow frame never costs the robot loop a tick.
              thread.setPriority(Thread.NORM_PRIORITY - 2);
              return thread;
            });
    IrradianceVolume volume = null;
    if (settings.quality() == RenderSettings.Quality.FAST) {
      long bakeStarted = System.nanoTime();
      volume =
          IrradianceVolume.bakeOrLoad(
              scene, lighting, VOLUME_CELL_SIZE_METERS, CACHE, volumeCacheKey(scene));
      System.out.printf(
          "[Rendering] Baked static lighting: %,d cells at %.2f m in %.1f s%n",
          volume.cellCount(), volume.cellSize(), (System.nanoTime() - bakeStarted) / 1e9);
    }
    renderer = new Renderer(scene, lighting, workers, threads, volume);

    Transform3d[] mountings = VisionConstants.CAMERA_TRANSFORM;
    int nextPort = Integer.getInteger("render.port", BASE_PORT);
    for (int i = 0; i < mountings.length; i++) {
      MjpegServer server = startOnFirstFreePort("Camera " + i, nextPort);
      nextPort = server.port() + 1;
      cameras.add(
          new CameraStream(
              i,
              mountings[i],
              new RenderCamera(
                  settings.width(),
                  settings.height(),
                  VisionConstants.SIM_CAMERA_FOV_DIAGONAL_DEGREES),
              server,
              new Film()));
    }

    running = true;
    renderLoop = new Thread(this::renderForever, "RenderLoop");
    renderLoop.setDaemon(true);
    renderLoop.setPriority(Thread.NORM_PRIORITY - 2);
    renderLoop.start();

    System.out.printf(
        "[Rendering] %,d triangles, %d tag panels within %.1f mm of the layout, ready in %.1f s%n",
        scene.soup.triangleCount,
        (int) scene.soup.surfaces.stream().filter(s -> s.decal() != null).count(),
        scene.tagAlignmentErrorMeters * 1000,
        (System.nanoTime() - began) / 1e9);
    for (CameraStream camera : cameras) {
      System.out.printf(
          "[Rendering] %s -> http://localhost:%d/  (%dx%d, %d spp, %s)%n",
          camera.server().name(),
          camera.server().port(),
          settings.width(),
          settings.height(),
          settings.samplesPerPixel(),
          settings.quality());
    }
  }

  /**
   * Publishes the current simulation state for the renderer to pick up.
   *
   * <p>Call once per loop from {@code simulationPeriodic}. Copies a few hundred ball positions and
   * returns; it does no rendering work itself.
   *
   * @param robotPose ego robot pose in field coordinates
   * @param obstacles other robots on the field
   * @param fuel ball centres, straight from {@code FuelSim.getFuelPositions()}
   */
  public void update(
      double timestampSeconds, Pose3d robotPose, List<Pose2d> obstacles, Translation3d[] fuel) {
    if (!running) {
      return;
    }
    published.set(SceneSnapshot.of(timestampSeconds, robotPose, obstacles, fuel));
  }

  /**
   * @return seconds the last frame took, for logging how much the render is keeping up
   */
  public double lastFrameSeconds() {
    return lastFrameSeconds;
  }

  public long framesRendered() {
    return framesRendered;
  }

  /** Shuts down the streams and the render threads. */
  public synchronized void stop() {
    running = false;
    if (renderLoop != null) {
      renderLoop.interrupt();
    }
    for (CameraStream camera : cameras) {
      camera.server().stop();
    }
    cameras.clear();
    if (workers != null) {
      workers.shutdownNow();
    }
    if (instance == this) {
      instance = null;
    }
  }

  /**
   * Renders cameras round robin for as long as the engine is running.
   *
   * <p>Only cameras somebody is actually asking for get rendered, whether that is a live stream
   * client or a recent snapshot request. With three cameras and one browser tab open that is the
   * difference between a frame every second and a frame every three.
   */
  private void renderForever() {
    long frameIndex = 0;
    SceneSnapshot lastSnapshot = null;
    DynamicScene dynamic = null;

    while (running && !Thread.currentThread().isInterrupted()) {
      boolean renderedSomething = false;

      for (CameraStream camera : cameras) {
        if (!running) {
          break;
        }
        if (!camera.server().isWanted()) {
          continue;
        }

        SceneSnapshot snapshot = published.get();
        if (snapshot != lastSnapshot || dynamic == null) {
          // The balls move every loop, so their index is rebuilt per snapshot but shared by all
          // the cameras looking at that same instant.
          dynamic = DynamicScene.from(snapshot, robotModel, robotModel);
          lastSnapshot = snapshot;
        }

        long began = System.nanoTime();
        try {
          camera.camera().place(snapshot.robotPose(), camera.mounting());
          Renderer.Frame frame = renderer.render(camera.camera(), settings, dynamic, frameIndex++);
          if (settings.denoise()) {
            Denoiser.apply(frame, 3);
          }
          BufferedImage image = camera.film().develop(frame, settings.sensorGain());
          camera.server().publish(image, settings.jpegQuality());

          lastFrameSeconds = (System.nanoTime() - began) / 1e9;
          framesRendered++;
          renderedSomething = true;
        } catch (IOException | RuntimeException failed) {
          if (running) {
            System.err.println("[Rendering] Frame failed: " + failed);
          }
        }
      }

      if (!renderedSomething) {
        // Nobody is watching. Idle cheaply rather than spinning.
        try {
          Thread.sleep(100);
        } catch (InterruptedException interrupted) {
          Thread.currentThread().interrupt();
          return;
        }
      }
    }
  }

  /**
   * Binds a camera to the first port that is actually free.
   *
   * <p>Simulation runs leave sockets in TIME_WAIT, and other tools on a development machine take
   * ports unpredictably, so a fixed port turns into an intermittent failure to start. The port each
   * camera settled on is printed at startup.
   */
  private static MjpegServer startOnFirstFreePort(String name, int firstPort) throws IOException {
    IOException lastFailure = null;
    for (int port = firstPort; port < firstPort + PORT_SEARCH_RANGE; port++) {
      try {
        return MjpegServer.start(name, port);
      } catch (BindException taken) {
        lastFailure = taken;
      }
    }
    throw new IOException(
        "No free port for "
            + name
            + " in "
            + firstPort
            + ".."
            + (firstPort + PORT_SEARCH_RANGE - 1),
        lastFailure);
  }

  /**
   * Identifies the baked lighting, so a stale volume is never reused.
   *
   * <p>Keyed on the geometry and on the classes that define the light rig and the bake, which is
   * what changes when someone retunes the lighting.
   */
  private static String volumeCacheKey(FieldScene scene) {
    long hash = scene.soup.triangleCount * 0x9E3779B97F4A7C15L;
    hash ^= Float.floatToIntBits(VOLUME_CELL_SIZE_METERS) * 0x85EBCA6BL;
    hash ^= Double.doubleToLongBits(scene.tagAlignmentErrorMeters);
    for (Class<?> type : new Class<?>[] {ArenaLighting.class, IrradianceVolume.class}) {
      hash = hash * 31 + classHash(type);
    }
    return Long.toHexString(hash);
  }

  /** Hashes a compiled class, so editing the light rig rebakes rather than reusing old light. */
  private static long classHash(Class<?> type) {
    String resource = type.getName().replace('.', '/') + ".class";
    try (java.io.InputStream stream = type.getClassLoader().getResourceAsStream(resource)) {
      if (stream == null) {
        return type.getName().hashCode();
      }
      long hash = 1125899906842597L;
      for (byte value : stream.readAllBytes()) {
        hash = hash * 31 + value;
      }
      return hash;
    } catch (IOException unreadable) {
      return type.getName().hashCode();
    }
  }

  private static boolean booleanProperty(String key, boolean fallback) {
    String value = System.getProperty(key);
    return value == null ? fallback : Boolean.parseBoolean(value);
  }

  private static float floatProperty(String key, float fallback) {
    String value = System.getProperty(key);
    if (value == null) {
      return fallback;
    }
    try {
      return Float.parseFloat(value);
    } catch (NumberFormatException notANumber) {
      return fallback;
    }
  }
}
