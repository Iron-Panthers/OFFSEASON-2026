package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.DriveConstants.ModuleConfig;
import frc.robot.utility.PhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  /**
   * Scale on reported drive odometry, never the physics. maple-sim sets wheel speed exactly to
   * ground speed unless skidding, so without this simulated odometry is perfect. Bias is slip,
   * scatter is per-wheel tread variation; both fitted to autonomous in five real matches.
   */
  private static final double ODOMETRY_SCALE_BIAS = 1.05;

  private static final double[] ODOMETRY_SCALE_SCATTER = {0.053, -0.053, 0.027, -0.027};

  private final double odometryScale;

  public ModuleIOTalonFXSim(ModuleConfig constants, SwerveModuleSimulation simulation) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    // Fixed per module so runs are comparable.
    int index = 0;
    for (int i = 0; i < DriveConstants.MODULE_CONFIGS.length; i++) {
      if (DriveConstants.MODULE_CONFIGS[i].driveID() == constants.driveID()) {
        index = i;
        break;
      }
    }
    odometryScale = ODOMETRY_SCALE_BIAS * (1.0 + ODOMETRY_SCALE_SCATTER[index]);
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(steerTalon, encoder));

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

    inputs.drivePositionRads *= odometryScale;
    inputs.drivePositionMeters *= odometryScale;
    inputs.driveVelocityRadsPerSec *= odometryScale;
    inputs.driveVelocityMetersPerSec *= odometryScale;
  }
}
