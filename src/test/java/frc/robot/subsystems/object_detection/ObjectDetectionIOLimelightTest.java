package frc.robot.subsystems.object_detection;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.object_detection.ObjectDetectionIO.ObjectDetectionIOInputs;
import frc.robot.utility.CameraIntrinsics;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Reading a Limelight's detections off NetworkTables and placing them on the field.
 *
 * <p>Each case publishes what a Limelight publishes, on a private NetworkTables instance, for balls
 * at known positions, and checks the IO puts them back where they are. The angles are worked out
 * the way the Limelight works them out, per axis through a pinhole, so this pins the IO's reading
 * of them and leaves the detector's own box error out of it.
 */
class ObjectDetectionIOLimelightTest {
  private static final String NAME = "limelight-test";
  private static final double R = ObjectDetectionConstants.BALL_RADIUS_M;
  private static final CameraIntrinsics LENS = ObjectDetectionConstants.CAMERA_INTRINSICS;

  /**
   * The robot holds still mid-field, looking across open carpet, so no ball falls off the field.
   */
  private static final Pose2d ROBOT = new Pose2d(8.0, 4.0, Rotation2d.fromDegrees(160.0));

  private static final Pose3d CAMERA =
      new Pose3d(ROBOT).transformBy(ObjectDetectionConstants.ROBOT_TO_CAMERA);

  private NetworkTableInstance nt;
  private DoublePublisher heartbeat;
  private DoubleArrayPublisher rawDetections;
  private DoublePublisher pipelineLatency;
  private DoublePublisher captureLatency;
  private ObjectDetectionIOLimelight io;
  private final ObjectDetectionIOInputs inputs = new ObjectDetectionIOInputs();

  /** The test's clock, in seconds of robot time. */
  private double now = 0.0;

  /** Every time the IO asked the pose history about. */
  private final List<Double> poseRequests = new ArrayList<>();

  @BeforeEach
  void setUp() {
    nt = NetworkTableInstance.create();
    NetworkTable table = nt.getTable(NAME);
    heartbeat = table.getDoubleTopic("hb").publish();
    rawDetections = table.getDoubleArrayTopic("rawdetections").publish();
    pipelineLatency = table.getDoubleTopic("tl").publish();
    captureLatency = table.getDoubleTopic("cl").publish();
    io =
        new ObjectDetectionIOLimelight(
            nt,
            NAME,
            timestamp -> {
              poseRequests.add(timestamp);
              return Optional.of(ROBOT);
            },
            () -> now);
  }

  @AfterEach
  void tearDown() {
    nt.close();
  }

  /** The direction from the camera to a ball, in the camera frame. */
  private static Translation3d inCamera(Translation3d ball) {
    return ball.minus(CAMERA.getTranslation()).rotateBy(CAMERA.getRotation().unaryMinus());
  }

  /** {@code rawdetections} for these balls, as the Limelight lays it out. */
  private static double[] rawDetectionsOf(List<Translation3d> balls) {
    double[] raw = new double[balls.size() * ObjectDetectionIOLimelight.VALUES_PER_DETECTION];
    for (int i = 0; i < balls.size(); i++) {
      Translation3d d = inCamera(balls.get(i));
      int base = i * ObjectDetectionIOLimelight.VALUES_PER_DETECTION;
      raw[base] = 0; // class
      raw[base + 1] = Math.toDegrees(Math.atan(-d.getY() / d.getX())); // txnc, right positive
      raw[base + 2] = Math.toDegrees(Math.atan(d.getZ() / d.getX())); // tync, up positive
      raw[base + 3] = 0.5; // area; corners stay zero, the IO does not read them
    }
    return raw;
  }

  /** Publishes one frame the way a Limelight does, arriving at {@code arrivalSeconds}. */
  private void publishFrame(double beat, double arrivalSeconds, double latencyMs, double[] raw) {
    long time = (long) (arrivalSeconds * 1e6);
    rawDetections.set(raw, time);
    pipelineLatency.set(latencyMs * 0.6, time);
    captureLatency.set(latencyMs * 0.4, time);
    heartbeat.set(beat, time);
  }

  /** Resting balls the camera can see, out along its heading, each inside the frame and range. */
  private static List<Translation3d> ballsInView() {
    List<Translation3d> balls = new ArrayList<>();
    Rotation2d heading = new Rotation2d(CAMERA.getRotation().getZ());
    for (double forward = 0.8; forward <= 4.5; forward += 0.45) {
      for (double lateral = -2.4; lateral <= 2.4; lateral += 0.4) {
        Translation2d floor =
            CAMERA
                .getTranslation()
                .toTranslation2d()
                .plus(new Translation2d(forward, lateral).rotateBy(heading));
        Translation3d ball = new Translation3d(floor.getX(), floor.getY(), R);
        boolean inFrame =
            LENS.rayToPixel(inCamera(ball))
                .filter(
                    p ->
                        p.getX() >= 0
                            && p.getX() <= LENS.width()
                            && p.getY() >= 0
                            && p.getY() <= LENS.height())
                .isPresent();
        if (inFrame && BallPlacement.projectToBallPlane(CAMERA, inCamera(ball)).isPresent()) {
          balls.add(ball);
        }
      }
    }
    return balls;
  }

  @Test
  void placesEachBallWhereItIs() {
    List<Translation3d> balls = ballsInView();
    assertTrue(balls.size() > 30, "the sweep should cover much of the frame, got " + balls.size());

    publishFrame(1, 10.0, 40.0, rawDetectionsOf(balls));
    now = 10.01;
    io.updateInputs(inputs);

    assertEquals(balls.size(), inputs.ballPositions.length);
    for (int i = 0; i < balls.size(); i++) {
      assertEquals(
          0.0,
          inputs.ballPositions[i].minus(balls.get(i).toTranslation2d()).getNorm(),
          1e-6,
          "ball at " + balls.get(i));
    }
  }

  @Test
  void aBallLandsWhereTheSimulatedDetectorPutsIt() {
    // The point of the whole arrangement. In simulation the coprocessor reports the pixel the
    // renderer drew a ball at and the robot reads it through the lens model. On the robot the
    // Limelight reports angles. The same ball has to come out in the same place either way, and
    // the pixel logged for it has to be the one the renderer would have drawn.
    List<Translation3d> balls = ballsInView();
    publishFrame(1, 10.0, 40.0, rawDetectionsOf(balls));
    now = 10.01;
    io.updateInputs(inputs);

    for (int i = 0; i < balls.size(); i++) {
      Translation2d rendered = LENS.rayToPixel(inCamera(balls.get(i))).orElseThrow();
      Translation3d simulated =
          BallPlacement.projectToBallPlane(
                  CAMERA, LENS.pixelToRay(rendered.getX(), rendered.getY()))
              .orElseThrow();
      assertEquals(
          0.0,
          inputs.ballPositions[i].minus(simulated.toTranslation2d()).getNorm(),
          1e-9,
          "ball at " + balls.get(i));
      assertEquals(rendered.getX(), inputs.pixelU[i], 1e-6);
      assertEquals(rendered.getY(), inputs.pixelV[i], 1e-6);
    }
  }

  @Test
  void limelightAnglesFollowItsSigns() {
    // Right is -Y in the camera frame and up is +Z. A sign slip here mirrors every ball across the
    // camera's heading and still produces plausible-looking positions.
    Translation3d right = ObjectDetectionIOLimelight.rayFromAngles(10.0, 0.0);
    Translation3d down = ObjectDetectionIOLimelight.rayFromAngles(0.0, -10.0);
    assertTrue(right.getY() < 0.0 && right.getZ() == 0.0);
    assertTrue(down.getZ() < 0.0 && down.getY() == 0.0);
  }

  @Test
  void eachFrameIsPlacedWithThePoseAtCapture() {
    publishFrame(1, 10.0, 50.0, rawDetectionsOf(ballsInView()));
    now = 10.02;
    io.updateInputs(inputs);

    assertEquals(1, poseRequests.size());
    assertEquals(9.95, poseRequests.get(0), 1e-9, "arrival less the pipeline and capture latency");
    assertEquals(9.95, inputs.frameTimestamp, 1e-9);
    assertEquals(10.02 - 9.95, inputs.latencySeconds, 1e-9);
    assertEquals(0.0, inputs.cameraPose.minus(CAMERA).getTranslation().getNorm(), 1e-9);
  }

  @Test
  void theSameHeartbeatIsTheSameFrame() {
    publishFrame(1, 10.0, 40.0, rawDetectionsOf(ballsInView()));
    now = 10.01;
    io.updateInputs(inputs);
    Translation2d[] first = inputs.ballPositions;

    // Detections changing without the heartbeat moving are the next frame partly arrived, not a
    // frame to place yet.
    rawDetections.set(new double[0], 10_015_000L);
    now = 10.02;
    io.updateInputs(inputs);
    assertArrayEquals(first, inputs.ballPositions);
    assertEquals(1, poseRequests.size());
  }

  @Test
  void anEmptyFrameClearsTheBalls() {
    publishFrame(1, 10.0, 40.0, rawDetectionsOf(ballsInView()));
    now = 10.01;
    io.updateInputs(inputs);
    assertTrue(inputs.ballPositions.length > 0);

    publishFrame(2, 10.05, 40.0, new double[0]);
    now = 10.06;
    io.updateInputs(inputs);
    assertEquals(0, inputs.ballPositions.length);
    assertEquals(10.01, inputs.frameTimestamp, 1e-9);

    // The same empty result again is a duplicate NetworkTables does not resend. It is still a new
    // frame, and still has to count as one, or the pool would stop clearing what is in view.
    publishFrame(3, 10.10, 40.0, new double[0]);
    now = 10.11;
    io.updateInputs(inputs);
    assertEquals(10.06, inputs.frameTimestamp, 1e-9);
  }

  @Test
  void aFrameTooOldToPlaceIsDropped() {
    double tooOldMs = (ObjectDetectionConstants.MAX_FRAME_AGE_SEC + 0.1) * 1000.0;
    publishFrame(1, 10.0, tooOldMs, rawDetectionsOf(ballsInView()));
    now = 10.01;
    io.updateInputs(inputs);
    assertEquals(1, inputs.framesDropped);
    assertEquals(0, inputs.ballPositions.length);
    assertEquals(0.0, inputs.frameTimestamp);
  }

  @Test
  void anArrayThatIsNotWholeBoxesIsDropped() {
    publishFrame(1, 10.0, 40.0, new double[ObjectDetectionIOLimelight.VALUES_PER_DETECTION + 1]);
    now = 10.01;
    io.updateInputs(inputs);
    assertEquals(1, inputs.framesDropped);
    assertEquals(0, inputs.ballPositions.length);
  }

  @Test
  void connectedOnlyWhileFramesKeepArriving() {
    now = 5.0;
    io.updateInputs(inputs);
    assertFalse(inputs.connected, "nothing published yet");

    publishFrame(1, 10.0, 40.0, new double[0]);
    now = 10.5;
    io.updateInputs(inputs);
    assertTrue(inputs.connected);

    now = 10.0 + ObjectDetectionConstants.FRAME_TIMEOUT_SEC + 0.1;
    io.updateInputs(inputs);
    assertFalse(inputs.connected, "the heartbeat stopped");
  }
}
