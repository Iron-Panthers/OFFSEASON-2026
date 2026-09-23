package frc.robot.utility.coprocessor;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.subsystems.object_detection.ObjectDetectionConstants;
import frc.robot.utility.rendering.RenderingEngine;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Runs coprocessor code beside the simulation, as its own process.
 *
 * <p>The point is that the thing being run is the real coprocessor program, not a simulation of
 * one. The same Python that ships to the Rubik Pi is launched here with {@code --source} pointed at
 * a rendered MJPEG stream instead of a camera, and it publishes to the simulation's NetworkTables
 * exactly as it would to the robot's. Nothing on the Python side knows the difference.
 *
 * <p>Supervising it from inside the simulation rather than from Gradle is what makes the stream URL
 * correct: {@link RenderingEngine} walks upward from its base port when something already holds it,
 * so the port a camera actually landed on is only knowable in this process.
 *
 * <p>Simulation only, and fails soft. A machine without Python, or without the detector's
 * dependencies installed, prints why and runs the simulation anyway.
 */
public final class CoprocessorManager {

  /** NT4 server port. In simulation the robot program is the server, on loopback. */
  private static final int NT_PORT = 5810;

  /** Where the Python lives, relative to the project root that Gradle runs us from. */
  private static final Path PACKAGE_ROOT = Path.of("coprocessor");

  private static CoprocessorManager instance;

  private final List<Running> processes = new ArrayList<>();

  private record Running(CoprocessorModule module, Process process) {}

  private CoprocessorManager() {}

  /**
   * Starts every module named by {@code -Dcoproc.modules}, if this run asked for any.
   *
   * <p>Safe to call unconditionally. Must be called after {@link RenderingEngine#startIfEnabled()},
   * which is what knows the stream URLs.
   *
   * @return the manager, or null if no modules were requested or none could be started
   */
  public static synchronized CoprocessorManager startIfEnabled() {
    if (instance != null) {
      return instance;
    }
    if (Constants.getRobotMode() != Constants.Mode.SIM) {
      return null;
    }
    String requested = System.getProperty("coproc.modules", "").trim();
    if (requested.isEmpty()) {
      return null;
    }

    CoprocessorManager manager = new CoprocessorManager();
    for (String name : requested.split(",")) {
      if (name.isBlank()) {
        continue;
      }
      try {
        manager.launch(CoprocessorModule.byId(name));
      } catch (IllegalArgumentException | IOException failed) {
        System.err.println("[Coprocessor] " + name.trim() + " disabled: " + failed.getMessage());
      }
    }

    if (manager.processes.isEmpty()) {
      return null;
    }
    Runtime.getRuntime().addShutdownHook(new Thread(manager::stop, "CoprocessorShutdown"));
    instance = manager;
    return manager;
  }

  /**
   * @return the running manager, or null if no coprocessor modules are running
   */
  public static synchronized CoprocessorManager getInstance() {
    return instance;
  }

  private void launch(CoprocessorModule module) throws IOException {
    int camera = Integer.getInteger("coproc." + module.id() + ".camera", module.defaultCamera());
    String source = resolveSource(module, camera);

    Path root = PACKAGE_ROOT.toAbsolutePath();
    if (!Files.isDirectory(root)) {
      throw new IOException("no coprocessor package at " + root);
    }

    List<String> command = new ArrayList<>();
    command.add(resolvePython(root));
    command.add("-u"); // unbuffered, or its output only appears when the process dies
    command.add("-m");
    command.add("coproc");
    command.add(module.id());
    command.add("--source");
    command.add(source);
    command.add("--camera");
    command.add(Integer.toString(camera));
    command.add("--nt");
    command.add("localhost");
    command.add("--nt-port");
    command.add(Integer.toString(NT_PORT));
    command.add("--output-port");
    command.add(Integer.toString(module.outputPort(camera)));

    RenderingEngine rendering = RenderingEngine.getInstance();
    if (rendering != null) {
      // The detector derives its focal length from the frame size, so a mismatch here biases
      // every range it reports. Take the resolution from the renderer rather than assuming.
      command.add("--width");
      command.add(Integer.toString(rendering.frameWidth()));
      command.add("--height");
      command.add(Integer.toString(rendering.frameHeight()));
      // Focal length follows from the frame size and this angle. The object-detection camera has
      // its own optics, so taking the vision cameras' figure here would bias every range.
      command.add("--fov");
      command.add(Double.toString(rendering.cameraFovDegrees(camera)));
    }
    command.addAll(mountingOptions(module, camera));
    command.addAll(moduleOptions(module));

    Process process =
        new ProcessBuilder(command).directory(root.toFile()).redirectErrorStream(true).start();

    processes.add(new Running(module, process));
    pump(module, process);

    System.out.printf(
        "[Coprocessor] %s -> %s, annotated on http://localhost:%d/%n",
        module.id(), source, module.outputPort(camera));
  }

  /**
   * Where this module reads frames from.
   *
   * <p>An explicit source override wins, so the same launcher can be pointed at a real camera or a
   * recorded stream. Otherwise it comes from the renderer, and the absence of the renderer is an
   * error rather than a default: a detector pointed at a port nothing is serving looks identical to
   * one that is simply finding nothing.
   */
  private String resolveSource(CoprocessorModule module, int camera) throws IOException {
    String override = System.getProperty("coproc." + module.id() + ".source");
    if (override != null && !override.isBlank()) {
      return override;
    }

    RenderingEngine rendering = RenderingEngine.getInstance();
    if (rendering == null) {
      throw new IOException(
          "no camera stream to read. Add -Prender, or set -Pcoproc."
              + module.id()
              + ".source=<url>");
    }
    String url = rendering.streamUrl(camera);
    if (url == null) {
      throw new IOException(
          "camera "
              + camera
              + " is not being rendered; "
              + rendering.cameraCount()
              + " camera(s) available");
    }
    return url;
  }

  /** Module-specific options, passed through from the matching system properties. */
  private List<String> moduleOptions(CoprocessorModule module) {
    List<String> options = new ArrayList<>();
    if (module == CoprocessorModule.OBJDETECT) {
      addIfSet(options, "--model", "coproc.objdetect.model");
      addIfSet(options, "--conf", "coproc.objdetect.conf");
      addIfSet(options, "--classes", "coproc.objdetect.classes");
      addIfSet(options, "--radius", "coproc.objdetect.radius");
      addIfSet(options, "--device", "coproc.objdetect.device");
      addIfSet(options, "--imgsz", "coproc.objdetect.imgsz");
      addIfSet(options, "--disagreement", "coproc.objdetect.disagreement");
    }
    return options;
  }

  /**
   * How the camera sits above the floor, which is what turns a bearing into a range.
   *
   * <p>Only supplied for the object-detection camera, and only because a ball resting on the floor
   * has a known centre height. Ranging off that rather than off the apparent size of the box is the
   * difference between an error that stays inside ten percent and one that silently halves when two
   * balls share a box. Yaw is deliberately not passed: it cannot affect where a ray meets the
   * floor, and passing it would only create something else to keep in step.
   *
   * <p>Nothing is passed for any other camera, and the detector falls back to apparent size alone
   * rather than ranging off a floor plane that means nothing to it.
   */
  private List<String> mountingOptions(CoprocessorModule module, int camera) {
    List<String> options = new ArrayList<>();
    if (module != CoprocessorModule.OBJDETECT
        || camera != RenderingEngine.objectDetectionCameraIndex()) {
      return options;
    }
    Transform3d mounting = ObjectDetectionConstants.ROBOT_TO_CAMERA;
    options.add("--camera-height");
    options.add(Double.toString(mounting.getZ()));
    options.add("--camera-pitch");
    options.add(Double.toString(Math.toDegrees(mounting.getRotation().getY())));
    options.add("--camera-roll");
    options.add(Double.toString(Math.toDegrees(mounting.getRotation().getX())));
    return options;
  }

  private static void addIfSet(List<String> options, String flag, String property) {
    String value = System.getProperty(property);
    if (value != null && !value.isBlank()) {
      options.add(flag);
      options.add(value);
    }
  }

  /**
   * Picks an interpreter: an explicit one, then the package's own virtualenv, then the platform
   * default.
   */
  private static String resolvePython(Path root) {
    String explicit = System.getProperty("coproc.python");
    if (explicit != null && !explicit.isBlank()) {
      return explicit;
    }

    boolean windows = System.getProperty("os.name", "").toLowerCase().contains("win");
    Path venv = root.resolve(".venv").resolve(windows ? "Scripts" : "bin");
    Path inVenv = venv.resolve(windows ? "python.exe" : "python");
    if (Files.isRegularFile(inVenv)) {
      return inVenv.toString();
    }
    return windows ? "python" : "python3";
  }

  /**
   * Forwards the process's output into the simulation console, tagged.
   *
   * <p>Without this a Python traceback goes nowhere and the module just appears not to work.
   */
  private void pump(CoprocessorModule module, Process process) {
    Thread pump =
        new Thread(
            () -> {
              try (BufferedReader reader =
                  new BufferedReader(
                      new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8))) {
                String line;
                while ((line = reader.readLine()) != null) {
                  // The module tags its own status lines, which is what a coprocessor's journal
                  // shows when it runs for real. Only tag what arrives untagged -- a traceback,
                  // say -- so lines are attributed without being labelled twice.
                  System.out.println(line.startsWith("[") ? line : "[" + module.id() + "] " + line);
                }
              } catch (IOException closed) {
                // The process exiting is the normal way this ends.
              }
              if (process.isAlive()) {
                return;
              }
              int code = process.exitValue();
              if (code != 0) {
                System.err.printf("[Coprocessor] %s exited with %d%n", module.id(), code);
              }
            },
            "Coprocessor-" + module.id());
    pump.setDaemon(true);
    pump.start();
  }

  /** Stops every module. Idempotent. */
  public synchronized void stop() {
    for (Running running : processes) {
      // Kill descendants first. Python spawns children for the model runtime, and on Windows
      // destroying only the parent leaves them holding the output port and the GPU.
      running.process().descendants().forEach(ProcessHandle::destroyForcibly);
      running.process().destroyForcibly();
    }
    processes.clear();
    if (instance == this) {
      instance = null;
    }
  }

  /**
   * @return how many module processes are still alive
   */
  public int running() {
    return (int) processes.stream().filter(entry -> entry.process().isAlive()).count();
  }
}
