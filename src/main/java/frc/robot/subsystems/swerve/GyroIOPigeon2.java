package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(DriveConstants.GYRO_ID);
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.withMountPosePitch(
        DriveConstants.IS_GYRO_UPSIDEDOWN ? Degree.of(180) : Degree.of(0));
    config.MountPose.withMountPoseYaw(DriveConstants.GYRO_ROTATION_OFFSET.getDegrees());
    pigeon.getConfigurator().apply(config);
    pigeon.setYaw(0, 1.0);

    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    BaseStatusSignal.setUpdateFrequencyForAll(100, yaw, yawVelocity);

    pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}
