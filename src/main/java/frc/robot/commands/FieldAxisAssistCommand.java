package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.swerve.DriveConstants.CENTER_OF_FIELD;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class FieldAxisAssistCommand extends AxisAssistCommand {
  public FieldAxisAssistCommand(Drive swerve) {
    // Init our Axis Assist Command with the target position of the nearest wall/hub and the target
    // heading of the nearest 180 degree rotation.
    // When horizontal (closer to top/bottom wall), PID controls Y axis, driver controls X →
    // controlY = false
    // When vertical (closer to hub X position), PID controls X axis, driver controls Y → controlY =
    // true
    super(swerve, () -> getAxisPosition(), () -> getTargetHeading(), () -> !isHorizontal());
  }

  private static double TRANS_OFFSET = 12;
  private static double ROTATION_OFFSET = Math.toRadians(20);

  private static final double FIELD_WIDTH = 8.21;

  private static boolean closerToBlueHub() {
    return RobotState.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(DriveConstants.BLUE_HUB_ORIGIN.toTranslation2d())
        < RobotState.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(DriveConstants.RED_HUB_ORIGIN.toTranslation2d());
  }

  /**
   * Checks if the robot is closer to a horizontal wall (top/bottom, Y=0 or Y=FIELD_WIDTH) than to
   * the nearest hub X position.
   *
   * @return true if closer to a horizontal wall, false if closer to a hub X position
   */
  private static boolean isHorizontal() {
    double robotX = RobotState.getInstance().getEstimatedPose().getX();
    double robotY = RobotState.getInstance().getEstimatedPose().getY();

    // Distance to the nearest horizontal wall (Y = 0 or Y = FIELD_WIDTH)
    double distToHorizontalWall = Math.min(robotY, FIELD_WIDTH - robotY);

    // Distance to the nearest hub X position
    double distToHubX =
        Math.min(
            Math.abs(
                robotX
                    - (DriveConstants.BLUE_HUB_ORIGIN.getX()
                        + DriveConstants.HUB_WIDTH / 2
                        + Units.inchesToMeters(10))),
            Math.abs(
                robotX
                    - (DriveConstants.RED_HUB_ORIGIN.getX()
                        - DriveConstants.HUB_WIDTH / 2
                        - Units.inchesToMeters(10))));

    return distToHorizontalWall < distToHubX;
  }

  private static Rotation2d getTargetHeading() {
    double poseRadians = RobotState.getInstance().getEstimatedPose().getRotation().getRadians();

    double fieldTarget;
    if (isHorizontal()) {
      // For horizontal wall alignment, snap to PI/2 or 3PI/2 so the intake faces toward the wall
      fieldTarget = (poseRadians > Math.PI) ? 0 : Math.PI;
      double offset = ROTATION_OFFSET;
      if (RobotState.getInstance().getEstimatedPose().getY() > CENTER_OF_FIELD.getY()) {
        offset *= -1;
      }
      if (fieldTarget == 0) {
        offset *= -1;
      }
      fieldTarget += offset;
    } else {
      // For vertical hub alignment, snap to 0 or PI
      if (RobotState.isAllianceRed()) {
        fieldTarget = (poseRadians > 0 && poseRadians < Math.PI) ? -Math.PI / 2 : Math.PI / 2;
      } else {
        fieldTarget = (poseRadians > 0 && poseRadians < Math.PI) ? Math.PI / 2 : -Math.PI / 2;
      }
      double offset = ROTATION_OFFSET;

      if (fieldTarget == Math.PI / 2) {
        offset *= -1;
      }
      if (closerToBlueHub()) {
        offset *= -1;
      }
      if (!RobotState.isAllianceRed()) {
        offset *= -1;
      }
      fieldTarget += offset;
      return new Rotation2d(fieldTarget);
    }

    // The heading controller operates on fieldRelativeYaw (driver-relative).
    // On red alliance, fieldRelativeYaw is offset by ~180° from the odometry heading,
    // so we must flip the target to stay in the same frame.
    if (RobotState.isAllianceRed()) {
      fieldTarget += Math.PI;
    }
    return new Rotation2d(fieldTarget);
  }

  private static Distance getAxisPosition() {
    if (isHorizontal()) {
      // Align to the nearest horizontal wall (Y axis)
      double robotY = RobotState.getInstance().getEstimatedPose().getY();
      if (robotY < FIELD_WIDTH / 2) {
        // Closer to bottom wall (Y = 0)
        return Meters.of(DriveConstants.DRIVE_CONFIG.bumperWidthY() / 2 + Units.inchesToMeters(6));
      } else {
        // Closer to top wall (Y = FIELD_WIDTH)
        return Meters.of(
            FIELD_WIDTH - DriveConstants.DRIVE_CONFIG.bumperWidthY() / 2 - Units.inchesToMeters(6));
      }
    } else {
      // Align to the nearest hub X position
      if (closerToBlueHub()) {
        return Meters.of(
            DriveConstants.BLUE_HUB_ORIGIN.getX()
                + DriveConstants.DRIVE_CONFIG.bumperWidthX() / 2
                + DriveConstants.HUB_WIDTH
                + Units.inchesToMeters(TRANS_OFFSET));
      } else {
        return Meters.of(
            DriveConstants.RED_HUB_ORIGIN.getX()
                - DriveConstants.DRIVE_CONFIG.bumperWidthX() / 2
                - DriveConstants.HUB_WIDTH
                - Units.inchesToMeters(TRANS_OFFSET));
      }
    }
  }
}
