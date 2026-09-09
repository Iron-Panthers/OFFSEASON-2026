package frc.robot.subsystems.shooter.shooter_hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIO;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class ShooterHoodIOSim extends GenericSuperstructureIOSim implements ShooterHoodIO {

  private final SingleJointedArmSim shooterHoodSim;
  private final double reduction;

  public ShooterHoodIOSim() {
    super(
        ShooterHoodConstants.SHOOTER_HOOD_CONFIG.motorID(),
        ShooterHoodConstants.SHOOTER_HOOD_CONFIG.reduction());

    this.reduction = ShooterHoodConstants.SHOOTER_HOOD_CONFIG.reduction();

    shooterHoodSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            ShooterHoodConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
            ShooterHoodConstants.PHYSICAL_CONSTANTS.lengthMeters(),
            ShooterHoodConstants.PHYSICAL_CONSTANTS.minAngleRads(),
            ShooterHoodConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
            ShooterHoodConstants.PHYSICAL_CONSTANTS.simulatedGravity(),
            0);
    setOffset();
    setSlot0(
        ShooterHoodConstants.GAINS.kP(),
        ShooterHoodConstants.GAINS.kI(),
        ShooterHoodConstants.GAINS.kD(),
        ShooterHoodConstants.GAINS.kS(),
        ShooterHoodConstants.GAINS.kV(),
        ShooterHoodConstants.GAINS.kA(),
        ShooterHoodConstants.GAINS.kG(),
        ShooterHoodConstants.MOTION_MAGIC_CONFIG.accelerations(),
        ShooterHoodConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        ShooterHoodConstants.GRAVITY_TYPE);
    frc.robot.utility.SimBattery.getInstance()
        .register(() -> lastSupplyCurrentAmps, ShooterHoodConstants.SUPPLY_CURRENT_LIMIT);
  }

  /** Last computed supply current, published to SimBattery. */
  private double lastSupplyCurrentAmps = 0.0;

  @Override
  public void updateInputs(GenericSuperstructureIO.GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    appliedVoltage =
        frc.robot.utility.SimCurrentLimit.clampToSupplyLimit(
            appliedVoltage,
            shooterHoodSim.getVelocityRadPerSec(),
            reduction,
            DCMotor.getKrakenX60Foc(1),
            RobotController.getBatteryVoltage(),
            ShooterHoodConstants.SUPPLY_CURRENT_LIMIT);

    // Simulate the physics
    shooterHoodSim.setInputVoltage(appliedVoltage);
    shooterHoodSim.update(0.02);

    // Mechanism units, then up to rotor units for the sensor. The old code DIVIDED by the
    // reduction where the rack multiplied -- the two superstructure sims applied gearing in
    // opposite directions, so at most one could have been right.
    double mechanismRotations = shooterHoodSim.getAngleRads() / (2 * Math.PI);
    double mechanismRPS = shooterHoodSim.getVelocityRadPerSec() / (2 * Math.PI);

    talon.getSimState().setRawRotorPosition(mechanismRotations * reduction);
    talon.getSimState().setRotorVelocity(mechanismRPS * reduction);

    inputs.isConnected = true;
    inputs.positionRotations = mechanismRotations;
    inputs.velocityRotPerSec = mechanismRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.statorCurrent = shooterHoodSim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = talon.getSimState().getSupplyCurrent();
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
  }

  @Override
  public void setOffset() {
    shooterHoodSim.setState(0, 0);
  }
}
