package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DoubleArmMechanism extends SubsystemBase {
  private LoggedMechanismLigament2d arm1;
  private LoggedMechanismLigament2d arm2;

  private double targetX = 0.0;
  private double targetY = 0.0;
  private LoggedMechanism2d mech;

  public DoubleArmMechanism(double x, double y) {
    mech = new LoggedMechanism2d(4, 4);
    LoggedMechanismRoot2d root = mech.getRoot("root", 2, 2);

    arm1 = root.append(new LoggedMechanismLigament2d("arm1", 1, 0));
    arm2 = arm1.append(new LoggedMechanismLigament2d("arm2", 1, 0));

    setTargetPosition(x, y);
    calibrateDoubleArm(targetX, targetY);
  }

  public void setTargetPosition(double x, double y) {
    targetX = x;
    targetY = y;
  }

  public void calibrateDoubleArm(double x, double y) {
    double distance = Math.sqrt(x * x + y * y);
    
    double heading = Math.atan2(y, x);
    double elbowInterior = 2 * Math.asin(distance / 2);
    double baseAngle = (Math.PI - elbowInterior) / 2;

    double angle1 = heading - baseAngle;
    double angle2 = Math.PI - elbowInterior;

    setArm1Angle(angle1);
    setArm2Angle(angle2);
  }

  public void setArm1Angle(double angle) {
    arm1.setAngle(Math.toDegrees(angle));
  }

  public void setArm2Angle(double angle) {
    arm2.setAngle(Math.toDegrees(angle));
  }

  @Override
  public void periodic() {
    calibrateDoubleArm(targetX, targetY);
    Logger.recordOutput("DoubleArmMechanism/Mechanism", mech);
  }
}
