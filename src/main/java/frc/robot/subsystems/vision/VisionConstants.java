package frc.robot.subsystems.vision;

import static frc.robot.Constants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class VisionConstants {
  public static final double AMBIGUITY_CUTOFF = 0.1;
  public static final double Z_ERROR_CUTOFF = 0.5;

  public static final Matrix<N3, N1> VISION_STATE_STD_DEVS =
      VecBuilder.fill(0.1, 0.1, 0.1); // not real values, copy and psated :)

  // index 0 -> arducam-1, etc
  public static final Transform3d[] CAMERA_TRANSFORM =
      switch (getRobotType()) {
          // SIM shares COMP's cameras. The SIM arm used to declare FIVE cameras in entirely
          // different places -- two of them rear-facing -- while RobotContainer wired up only
          // one of them, index 3, a camera pointing backwards. The simulation was therefore
          // localising off a single rear camera the real robot does not have.
        case COMP, SIM -> new Transform3d[] {
          // new Transform3d(new Translation3d(), new Rotation3d())
          // arducam-7 (front in rollers)
          new Transform3d(
              new Translation3d(0.33493633, 0, 0.422076702),
              new Rotation3d(0.00776866, -0.5635942421, 0)),
          new Transform3d(
              new Translation3d(0.373037, -0.295926, 0.485082),
              new Rotation3d(0, (Math.toRadians(-11)), (Math.toRadians(-90)))),
          new Transform3d(
              new Translation3d(0.373037, 0.295926, 0.485082),
              new Rotation3d(0, (Math.toRadians(-11)), (Math.toRadians(90)))),
          // // arducam-6 (front)
          // new Transform3d(new Translation3d(0.22860929920064077, 0.2077131830328219,
          // 0.4409522926695345), new Rotation3d(0.022664911373188813, -0.47667215401543667,
          // 0.005354613028298594))
          // arducam-8 (back)
          //   new Transform3d(
          //       new Translation3d(0.3245, -0.2707, 0.4556), new Rotation3d(0, -0.3816,
          // 3.1275))
          // new Transform3d(
          //     new Translation3d(0.32229575747678485, 0.2693020732473436, 0.4564431972611209),
          //     new Rotation3d(0.002536819350520349, -0.3801061008791202, -0.010367739955583455))
        };
        case VISION -> new Transform3d[] {
          // arducam-1 (front left)
          // new Transform3d(new Translation3d(), new Rotation3d()),
          new Transform3d(
              new Translation3d(
                  0.4219891379261578 - Units.inchesToMeters(4),
                  0.10887153688779888,
                  0.48291456142117706),
              new Rotation3d(0.05809703484961512, -0.25210328052613, -0.15674754278844813)),
          // arducam-2 (front center)
          new Transform3d(
              new Translation3d(
                  0.42331372439056836 - Units.inchesToMeters(4),
                  -0.13898825541229506,
                  0.4727338620375543),
              new Rotation3d(-0.07349738757716055, -0.2743445020047346, 0.22780794700963716))
          // new Transform3d(new Translation3d(), new Rotation3d())
          // arducam-3 (front right)
          // new Transform3d(
          //     0.299, -0.2744, 0.3464, new Rotation3d(0, -Math.toRadians(35),
          // -Math.toRadians(55))),
          // // arducam-4 (back right)
          // new Transform3d(
          //     -0.17, -0.298, 0.3651, new Rotation3d(0, 0, Math.PI - Math.toRadians(12))),
          // // arducam-5 (back left)
          // new Transform3d(-0.17, 0.298, 0.3651, new Rotation3d(0, 0, -Math.PI +
          // Math.toRadians(12)))
        };
        case ALPHA -> new Transform3d[] {
          // arducam-1 (Lower intake)
          new Transform3d(
              -0.305,
              -0.102,
              0.159,
              new Rotation3d(
                  Math.toRadians(17.259),
                  Math.toRadians(-35.296 + 4),
                  Math.toRadians(-36.069 - 180 - 7))),
          // arducam-2 (Upper intake)
          new Transform3d(
              -0.181,
              0.243,
              0.249,
              new Rotation3d(
                  Math.toRadians(5.739), Math.toRadians(-19.623), Math.toRadians(34.632 - 180)))
        };
        default -> new Transform3d[0];
      };

  /**
   * Simulated camera model.
   *
   * <p>{@code new SimCameraProperties()} is PhotonVision's {@code PERFECT_90DEG}: a 960x720 camera
   * with <b>zero</b> calibration error and <b>zero</b> latency. The simulated robot therefore
   * localised perfectly at any range, which is the one thing real vision never does.
   *
   * <p>The important part is that the noise is injected <b>in pixels, on the detected tag
   * corners</b>. A tag at 2 m spans many pixels and a tag at 8 m spans few, so the same pixel error
   * produces a pose error that grows with distance on its own. Accuracy falling off with range is
   * then a consequence of the optics rather than a curve someone tuned.
   *
   * <p><b>Measured from five real matches:</b> {@link #SIM_MAX_SIGHT_RANGE_METERS}. Per-camera
   * maximum observed target distance ran 3.56-4.55 m for camera 0 (the one in the rollers, pitched
   * down) and 7.11-10.13 m for the two side cameras.
   *
   * <p><b>Not measured, and worth revisiting:</b> resolution, field of view, latency, frame rate
   * and calibration error. These are representative values for the Arducam OV9281 the robot runs,
   * not values read off this robot's calibration. They cannot be measured from the logs, because
   * {@code VisionIOInputs.observations} is never written to the log -- only tag IDs and average
   * distance are -- so there is no recorded vision pose to compare against.
   */
  public static final int SIM_CAMERA_WIDTH_PX = 1280;

  public static final int SIM_CAMERA_HEIGHT_PX = 800;

  public static final double SIM_CAMERA_FOV_DIAGONAL_DEGREES = 70.0;

  public static final double SIM_CAMERA_FPS = 40.0;

  public static final double SIM_CAMERA_LATENCY_MS = 25.0;

  public static final double SIM_CAMERA_LATENCY_STD_DEV_MS = 8.0;

  /**
   * Average and standard deviation of corner detection error, in pixels.
   *
   * <p>Calibrated against the one measurement of vision accuracy that does not depend on the pose
   * estimator: when two cameras report a pose on the SAME loop they are looking at the same robot
   * at the same instant, so their disagreement is vision error and nothing else. Measured on q54
   * with {@code scripts/drive_vision_fidelity.py}:
   *
   * <table>
   * <tr><th>target distance</th><th>real</th><th>simulated at 0.25 px</th></tr>
   * <tr><td>2-3 m</td><td>0.073 m</td><td>0.030 m</td></tr>
   * <tr><td>3-4 m</td><td>0.084 m</td><td>0.039 m</td></tr>
   * <tr><td>4-5 m</td><td>0.207 m</td><td>0.026 m</td></tr>
   * </table>
   *
   * <p>0.25 px is what a good calibration achieves on a bench. A camera bolted to a robot being
   * driven hard does considerably worse -- vibration, focus, tag ambiguity at oblique angles -- and
   * the measurement above says by roughly a factor of three.
   */
  public static final double SIM_CAMERA_CALIB_ERROR_PX = 0.75;

  public static final double SIM_CAMERA_CALIB_ERROR_STD_DEV_PX = 0.25;

  /**
   * Beyond this a tag is not detected at all.
   *
   * <p>10.2 m, just past the 10.13 m furthest target any camera reported across q54, q93, q14, q103
   * and q64. Without a limit the simulation happily reads tags across the whole field.
   */
  public static final double SIM_MAX_SIGHT_RANGE_METERS = 10.2;

  /**
   * Smallest fraction of the image a tag may occupy and still be detected, in percent.
   *
   * <p>Guards the same failure from the other side: a tag seen edge-on at range covers a handful of
   * pixels and a real pipeline will not resolve its corners.
   */
  public static final double SIM_MIN_TARGET_AREA_PERCENT = 0.03;

  public static final List<TagCountDeviation> TAG_COUNT_DEVIATIONS =
      switch (getRobotType()) {
          // TODO: tune these?
        default -> List.of(
            // 1 tag
            new TagCountDeviation(
                new UnitDeviationParams(0.007329, 0, 0),
                new UnitDeviationParams(0.007329, 0, 0),
                new UnitDeviationParams(0.0166, 0, 0)),
            // 2 tag
            new TagCountDeviation(
                new UnitDeviationParams(0.00162493, 0, 0),
                new UnitDeviationParams(0.0010625, 0, 0)),
            // 3+ tag
            new TagCountDeviation(
                new UnitDeviationParams(0, 0.0, 0.001), new UnitDeviationParams(0, 0, 0.0001)));
      };

  public static final int[] IGNORE_TAGS = {};
  // public static final int[] IGNORE_TAGS = {}; // removed

  // Fixed AprilTag field layout initialization
  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

  static {
    // logic for dynamically setting the april tag field layout
    AprilTagFieldLayout defaultFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    List<AprilTag> aprilTags = defaultFieldLayout.getTags();
    // remove ignored tags
    aprilTags.removeIf(
        tag -> {
          for (int ignoreTag : IGNORE_TAGS) {
            if (tag.ID == ignoreTag) {
              return true;
            }
          }
          return false;
        });
    APRIL_TAG_FIELD_LAYOUT =
        new AprilTagFieldLayout(
            aprilTags, defaultFieldLayout.getFieldLength(), defaultFieldLayout.getFieldWidth());
  }

  public static record TagCountDeviation(
      UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
    protected Matrix<N3, N1> computeDeviation(double averageDistance) {
      return VecBuilder.fill(
          xParams.computeUnitDeviation(averageDistance),
          yParams.computeUnitDeviation(averageDistance),
          thetaParams.computeUnitDeviation(averageDistance));
    }

    public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
      this(xyParams, xyParams, thetaParams);
    }
  }

  public static record UnitDeviationParams(
      double distanceMultiplier, double eulerMultiplier, double constant) {
    private double computeUnitDeviation(double averageDistance) {
      return distanceMultiplier * averageDistance + constant;
    }
  }
}
