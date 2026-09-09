package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    // getMeasuredAngularVelocity() already returns rad/s, so the degreesToRadians() that used
    // to wrap this divided the value by 57.3. GyroIOPigeon2 needs that conversion because the
    // Pigeon reports deg/s; this line was copied from it without dropping it. Sim yaw rate
    // spanned +-0.128 rad/s against the real robot's -5.01..+4.15.
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
  }
}
