package frc.robot;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.object_detection.ObjectDetectionConstants;

/** Tuning values for the SCORE/PICKUP full match auto state machine in {@link RobotState}. */
public class FullMatchAutoConstants {
  /** Time the shooter needs to empty a full hopper. */
  public static final double FULL_HOPPER_SHOOT_SEC = 2.0;

  /** Time given to spin the shooter up before the hopper is dumped. */
  public static final double SPIN_UP_SEC = 0.6;

  /** Balls worth of clusters to chase before returning to SCORE. */
  public static final int PICKUP_BALL_GOAL = 40;

  /** Middle of the field — observing poses aim the intake here. */
  public static final Translation2d FIELD_CENTER = new Translation2d(8.255, 4.02);

  /**
   * Shooting poses, blue frame. These are the end poses of the 4xT trench autos ("2xTT 2
   * Right/Left"), one per side of the field. {@code AlignToPoseCommand} applies the red flip.
   */
  public static final Pose2d[] SCORING_POSES = {
    new Pose2d(3.891, 0.823, Rotation2d.fromDegrees(77.005)),
    new Pose2d(3.891, 7.247, Rotation2d.fromDegrees(-77.005))
  };

  /**
   * Corners of the center of the field, blue frame. Each pose aims the camera at {@link
   * #FIELD_CENTER}, which also points the intake at the balls since the two share a side.
   */
  public static final Pose2d[] OBSERVING_POSES = {
    observingPose(6.135, 0.794),
    observingPose(6.135, 7.246),
    FlippingUtil.flipFieldPose(observingPose(6.135, 0.794)),
    FlippingUtil.flipFieldPose(observingPose(6.135, 7.246))
  };

  /** Heading that points the camera, not the robot front, at the middle of the field. */
  private static Pose2d observingPose(double x, double y) {
    Translation2d position = new Translation2d(x, y);
    Rotation2d cameraYaw =
        new Rotation2d(ObjectDetectionConstants.ROBOT_TO_CAMERA.getRotation().getZ());
    return new Pose2d(position, FIELD_CENTER.minus(position).getAngle().minus(cameraYaw));
  }
}
