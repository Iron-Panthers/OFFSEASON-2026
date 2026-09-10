package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.DriveConstants.ModuleConfig;
import frc.robot.utility.PhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  /**
   * Scale between what this module's encoder reports and how far the robot actually travelled.
   *
   * <p>maple-sim models skidding, but only skidding: while a module grips, it sets the wheel speed
   * to <em>exactly</em> the ground velocity projected onto the wheel. Simulated odometry is
   * therefore perfect except during a skid event, and a real robot's is never perfect.
   *
   * <p>Measured during AUTONOMOUS across five matches, which is the closest thing to a contact-free
   * sample -- no one is pushing the robot yet:
   *
   * <table>
   * <tr><th>metric</th><th>real (auto)</th><th>simulated (auto)</th></tr>
   * <tr><td>module disagreement, median</td><td>0.028-0.038</td><td>0.012</td></tr>
   * <tr><td>wheel distance / true distance</td><td>1.077-1.111</td><td>1.006</td></tr>
   * </table>
   *
   * <p>Two separate effects, and they need two separate numbers because the coefficient of friction
   * only produces one of them. Dropping it from 1.05 to maple-sim's lowest supported 0.65 moved
   * wheel-over-true from 1.006 to 1.037 but left disagreement at 0.012, because four wheels
   * skidding together still agree with each other. Real wheels disagree because they are not
   * identical.
   *
   * <ul>
   *   <li><b>bias</b> -- wheels always turn further than the robot travels, never less. Lumps
   *       together the slip maple-sim does not produce even at its lowest supported grip.
   *   <li><b>scatter</b> -- per-wheel tread wear and calibration. At a 1.97 in wheel, +/-3% is
   *       +/-0.06 in of effective radius, which is an ordinary amount of wear across four wheels.
   * </ul>
   *
   * <p>Applied to the REPORTED position and velocity only, never to the physics -- which is exactly
   * what slip is. The simulated robot really does go where maple-sim says; it just no longer knows
   * it precisely, so the pose estimator has to lean on vision the way the real one does.
   */
  private static final double ODOMETRY_SCALE_BIAS = 1.05;

  private static final double[] ODOMETRY_SCALE_SCATTER = {0.03, -0.03, 0.015, -0.015};

  private final double odometryScale;

  public ModuleIOTalonFXSim(ModuleConfig constants, SwerveModuleSimulation simulation) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    // Indexed by the module's position in DriveConstants.MODULE_CONFIGS, so each module keeps the
    // same error every run. Randomising it per run would make two runs of identical code
    // incomparable, and the whole point of this work is comparing runs.
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

    // Only the drive encoder is scaled. Steering knows where it points; it is the distance
    // travelled that a slipping wheel gets wrong.
    inputs.drivePositionRads *= odometryScale;
    inputs.drivePositionMeters *= odometryScale;
    inputs.driveVelocityRadsPerSec *= odometryScale;
    inputs.driveVelocityMetersPerSec *= odometryScale;
  }
}
