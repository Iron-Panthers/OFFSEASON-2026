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
        case COMP -> new Transform3d[] {
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
        case SIM -> new Transform3d[] {
          // arducam-1 (front left)
          new Transform3d(
              0.299, 0.2744, 0.3464, new Rotation3d(0, -Math.toRadians(35), Math.toRadians(55))),
          // arducam-2 (front center)
          new Transform3d(0.3017, 0, 0.3373, new Rotation3d(0, -Math.toRadians(35), 0)),
          // arducam-3 (front right)
          new Transform3d(
              0.299, -0.2744, 0.3464, new Rotation3d(0, -Math.toRadians(35), -Math.toRadians(55))),
          // arducam-4 (back right)
          new Transform3d(
              -0.17, -0.298, 0.3651, new Rotation3d(0, 0, Math.PI - Math.toRadians(12))),
          // arducam-5 (back left)
          new Transform3d(-0.17, 0.298, 0.3651, new Rotation3d(0, 0, -Math.PI + Math.toRadians(12)))
        };
        default -> new Transform3d[0];
      };

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
