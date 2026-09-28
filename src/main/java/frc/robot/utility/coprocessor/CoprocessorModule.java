package frc.robot.utility.coprocessor;

import frc.robot.utility.rendering.RenderingEngine;

/**
 * A coprocessor module the simulation knows how to launch.
 *
 * <p>The Python side has its own registry in {@code coproc/__main__.py}; this is the half that
 * decides which camera a module watches and where its annotated output lands. Adding a module means
 * one entry here and one there.
 */
public enum CoprocessorModule {
  /**
   * YOLO ball detection.
   *
   * <p>Defaults to the object-detection camera, which is not one of the vision cameras: it is
   * {@code ObjectDetectionConstants.ROBOT_TO_CAMERA}, mounted low and tilted down at the floor
   * beside the intake. The vision cameras are pitched <em>up</em> to see tags on walls, which
   * leaves them unable to see floor anywhere near the robot, so pointing the detector at one of
   * them finds nothing at all.
   */
  OBJDETECT("objdetect", RenderingEngine.objectDetectionCameraIndex());

  /** First port for annotated output. Clear of the renderer at 1191+ and PhotonVision at 1181+. */
  public static final int OUTPUT_BASE_PORT = 1291;

  private final String id;
  private final int defaultCamera;

  CoprocessorModule(String id, int defaultCamera) {
    this.id = id;
    this.defaultCamera = defaultCamera;
  }

  /**
   * @return the name used on the command line, in the NetworkTables path and in log prefixes
   */
  public String id() {
    return id;
  }

  /**
   * @return the camera this module watches unless told otherwise
   */
  public int defaultCamera() {
    return defaultCamera;
  }

  /**
   * @return the camera this module watches this run: {@code -Pcoproc.<id>.camera} if given, else
   *     {@link #defaultCamera()}
   */
  public int camera() {
    return Integer.getInteger("coproc." + id + ".camera", defaultCamera);
  }

  /**
   * Whether this run asked for the module with {@code -Pcoproc}.
   *
   * <p>Asked for, not running: robot code is constructed before the simulation starts any module,
   * and a module that then fails to start should look like a coprocessor reporting nothing rather
   * than silently fall back to something else.
   */
  public boolean isRequested() {
    for (String name : System.getProperty("coproc.modules", "").split(",")) {
      if (name.trim().equalsIgnoreCase(id)) {
        return true;
      }
    }
    return false;
  }

  /**
   * @return the NetworkTables table this module publishes a camera's detections under
   */
  public String tablePath(int cameraIndex) {
    return "/coprocessor/" + id + "/cam" + cameraIndex;
  }

  /**
   * @return the port this module serves its annotated stream on
   */
  public int outputPort(int cameraIndex) {
    return OUTPUT_BASE_PORT + cameraIndex;
  }

  /**
   * Looks up a module by the name given on the command line.
   *
   * @param name case-insensitive module id
   * @return the module
   * @throws IllegalArgumentException naming the valid options, since the alternative is a typo that
   *     silently starts nothing
   */
  public static CoprocessorModule byId(String name) {
    for (CoprocessorModule module : values()) {
      if (module.id.equalsIgnoreCase(name.trim())) {
        return module;
      }
    }
    throw new IllegalArgumentException(
        "Unknown coprocessor module '" + name + "'; known: " + ids());
  }

  /**
   * @return every module id, comma separated, for error messages and help text
   */
  public static String ids() {
    StringBuilder joined = new StringBuilder();
    for (CoprocessorModule module : values()) {
      if (joined.length() > 0) {
        joined.append(", ");
      }
      joined.append(module.id);
    }
    return joined.toString();
  }
}
