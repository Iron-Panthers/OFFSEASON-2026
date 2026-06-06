package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO moduleIO;
  private final int index;
  private double totalAmps = 0;

  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

  public Module(ModuleIO moduleIO, int index) {
    this.moduleIO = moduleIO;
    this.index = index;
  }

  public void updateInputs() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);
    totalAmps += (inputs.driveSupplyCurrent / 50);
    Logger.recordOutput("Swerve/Module" + index + "/Total Amp Seconds", totalAmps);
  }

  public void runToSetpoint(SwerveModuleState targetState) {
    targetState.optimize(getSteerHeading());
    targetState.cosineScale(getSteerHeading());
    moduleIO.runSteerPositionSetpoint(targetState.angle.getRadians());

    double driveVelocityRads =
        ((targetState.speedMetersPerSecond) / DriveConstants.DRIVE_CONFIG.wheelRadius());

    moduleIO.runDriveVelocitySetpoint(driveVelocityRads);

    Logger.recordOutput(
        "Swerve/Module" + index + "/Steer Setpoint", targetState.angle.getRadians());
    Logger.recordOutput(
        "Swerve/Module" + index + "/SteerError",
        targetState.angle.getRadians() - inputs.steerAbsolutePosition.getRadians());
    Logger.recordOutput("Swerve/Module" + index + "/DriveVelRadsScalar", driveVelocityRads);
  }

  public Rotation2d getSteerHeading() {
    return inputs.steerAbsolutePosition;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(inputs.drivePositionMeters, inputs.steerAbsolutePosition);
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(inputs.driveVelocityMetersPerSec, inputs.steerAbsolutePosition);
  }

  public void setDriveSupplyCurrentLimit(double currentLimitAmps) {
    moduleIO.setSupplyCurrentLimit(currentLimitAmps);
  }

  public void setNeutralMode(NeutralModeValue value) {
    moduleIO.setNeutralMode(value);
  }
}
