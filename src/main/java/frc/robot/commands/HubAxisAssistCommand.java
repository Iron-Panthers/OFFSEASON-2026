package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class HubAxisAssistCommand extends AxisAssistCommand {
  public HubAxisAssistCommand(Drive swerve) {
    super(swerve, () -> getAxisPosition(), () -> getTargetHeading(), () -> true);
  }

  private static double TRANS_OFFSET = 16;
  private static double ROTATION_OFFSET = Math.toRadians(20);

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

  public static Distance getAxisPosition() {
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

  public static Rotation2d getTargetHeading() {
    double poseRadians = RobotState.getInstance().getEstimatedPose().getRotation().getRadians();

    double fieldTarget = (poseRadians > -Math.PI / 2 && poseRadians < Math.PI / 2) ? Math.PI : 0;
    double offset = ROTATION_OFFSET;
    if (fieldTarget == 0) {
      offset *= -1;
    }
    if (closerToBlueHub()) {
      offset *= -1;
    }
    fieldTarget += offset;
    return new Rotation2d(fieldTarget);
  }
}
