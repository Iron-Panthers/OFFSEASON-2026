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
    super(ShooterHoodConstants.SHOOTER_HOOD_CONFIG.motorID());

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
  }

  @Override
  public void updateInputs(GenericSuperstructureIO.GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate the physics
    shooterHoodSim.setInputVoltage(appliedVoltage);
    shooterHoodSim.update(0.02);

    // Convert the position and velocity from meters to rotations for the TalonFX sensor
    double rotations = shooterHoodSim.getAngleRads() / (2 * Math.PI * reduction);
    double velocityRPS = shooterHoodSim.getVelocityRadPerSec() / (2 * Math.PI * reduction);

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.isConnected = true;
    inputs.positionRotations = rotations;
    inputs.velocityRotPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = talon.getSimState().getSupplyCurrent();
  }

  @Override
  public void setOffset() {
    shooterHoodSim.setState(0, 0);
  }
}
