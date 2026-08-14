package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class DoubleArmMechanism implements Subsystem {
  private LoggedMechanismLigament2d arm1;
  private LoggedMechanismLigament2d arm2;

  private double targetX = 0.0;
  private double targetY = 0.0;
  private LoggedMechanism2d mech;

  public DoubleArmMechanism(double x, double y) {
    mech = new LoggedMechanism2d(0, 0);
    LoggedMechanismRoot2d root = mech.getRoot(null, 0, 0);

    arm1 = root.append(new LoggedMechanismLigament2d("arm1", 1, 0));
    arm2 = arm1.append(new LoggedMechanismLigament2d("arm2", 1, 0));

    setTargetPosition(x, y);
    calibrateDoubleArm(targetX, targetY);
    SmartDashboard.putData("Mech2d", mech);
  }

  public void setTargetPosition(double x, double y) {
    targetX = x;
    targetY = y;
  }

  public void calibrateDoubleArm(double x, double y) {
    double angle1 = Math.acos(-Math.sqrt(x * x + y * y) / 2);
    double angle2 = 2 * Math.asin(Math.sqrt(x * x + y * y) / 2);

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
