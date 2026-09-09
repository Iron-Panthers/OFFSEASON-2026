package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.DriveConstants.ModuleConfig;
import frc.robot.utility.PhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(ModuleConfig constants, SwerveModuleSimulation simulation) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(steerTalon, encoder));

    // Phoenix models supply current for the maple-sim-driven modules directly.
    // Eight motors (four drive, four steer) dominate the robot's power draw, so
    // omitting them would make the battery model far too optimistic.
    frc.robot.utility.SimBattery.getInstance()
        .register(
            () -> driveTalon.getSimState().getSupplyCurrent(),
            DriveConstants.DRIVE_CURRENT_LIMIT_AMPS);
    frc.robot.utility.SimBattery.getInstance()
        .register(
            () -> steerTalon.getSimState().getSupplyCurrent(),
            DriveConstants.STEER_CURRENT_LIMIT_AMPS);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);
  }
}
