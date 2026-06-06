package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class VisionTuningCommands {
  private static class TransformAverage {
    private double xSum = 0;
    private double ySum = 0;
    private double zSum = 0;
    private double pitchSum = 0;
    private double rollSum = 0;
    private double yawSum = 0;
    private int datapoints = 0;

    public TransformAverage() {}

    public void add(Transform3d transform) {
      xSum += transform.getX();
      ySum += transform.getY();
      zSum += transform.getZ();
      pitchSum += transform.getRotation().getY();
      rollSum += transform.getRotation().getX();
      yawSum += transform.getRotation().getZ();
      datapoints += 1;
    }

    public Transform3d getAverage() {
      if (datapoints == 0) return new Transform3d();
      return new Transform3d(
          new Translation3d(xSum / datapoints, ySum / datapoints, zSum / datapoints),
          new Rotation3d(rollSum / datapoints, pitchSum / datapoints, yawSum / datapoints));
    }
  }

  /** Adds the drive tuning commands to the auto chooser. */
  public static void addTuningCommandsToAutoChooser(
      Vision vision, LoggedDashboardChooser<Command> chooser) {
    // We may want to run this at a competition
    chooser.addOption(
        "TUNING | Vision Camera Position Measurement", measureCameraPositions(vision));
  }

  /** The transform of the calibration tag, relative to the robot base. */
  public static Transform3d heldTagTransform =
      new Transform3d(
          new Translation3d( //
              Units.inchesToMeters((29 + 12.75)), // Distance forward
              Units.inchesToMeters(0), // Distance left
              Units.inchesToMeters(44.25) // Distance up
              ),
          new Rotation3d(0., 0., Units.degreesToRadians(180)));

  public static Pose3d heldTagPose = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(10).get();

  // 481.387in

  public static Command measureCameraPositions(Vision vision) {
    TransformAverage[] averages = new TransformAverage[vision.getCameraCount()];
    return Commands.startRun(
            () -> {
              for (int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                averages[cameraIndex] = new TransformAverage();
              }
              System.out.println(
                  "********** Vision camera position measurement started. **********");
            },
            () -> {
              Pose3d[] poses = vision.getRobotTransforms();
              for (int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                if (poses[cameraIndex] == null) continue;
                averages[cameraIndex].add(heldTagPose.minus(poses[cameraIndex]));
              }
            },
            vision)
        .finallyDo(
            () -> {
              System.out.println(
                  "********** Vision camera position measurement results **********");
              Transform3d[] adjustedTransforms = new Transform3d[4];
              for (int cameraIndex = 0; cameraIndex < vision.getCameraCount(); cameraIndex++) {
                Transform3d averageTransform = averages[cameraIndex].getAverage();
                Transform3d transform = heldTagTransform.plus(averageTransform.inverse());
                adjustedTransforms[cameraIndex] = transform;

                System.out.print("Robot to camera " + cameraIndex + ": ");
                printTransform(transform);
              }

              //   Transform3d frontLeftTransform = adjustedTransforms[0];
              //   Transform3d frontRightTransform = adjustedTransforms[1];
              //   double frontWidth = frontLeftTransform.getY() - frontRightTransform.getY();
              //   frontLeftTransform =
              //       new Transform3d(
              //           new Translation3d(
              //               frontLeftTransform.getX(), frontWidth / 2,
              // frontLeftTransform.getZ()),
              //           frontLeftTransform.getRotation());
              //   frontRightTransform =
              //       new Transform3d(
              //           new Translation3d(
              //               frontRightTransform.getX(), -frontWidth / 2,
              // frontRightTransform.getZ()),
              //           frontRightTransform.getRotation());

              //   System.out.print("Robot to front left camera (centered Y compensated): ");
              //   printTransform(frontLeftTransform);
              //   System.out.print("Robot to front right camera (centered Y compensated): ");
              //   printTransform(frontRightTransform);
            });
  }

  private static void printTransform(Transform3d transform) {
    // new Transform3d(new Translation3d(x, y, z), new Rotation3d(roll, pitch, yaw))
    Rotation3d rotation = transform.getRotation();
    System.out.println(
        "`new Transform3d(new Translation3d("
            + transform.getX()
            + ", "
            + transform.getY()
            + ", "
            + transform.getZ()
            + "), new Rotation3d("
            + rotation.getX()
            + ", "
            + rotation.getY()
            + ", "
            + rotation.getZ()
            + "))`");
  }
}
