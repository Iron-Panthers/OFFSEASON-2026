package frc.robot.subsystems.intake.intake_rack;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotSimState;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class IntakeRackIOSim extends GenericSuperstructureIOSim implements IntakeRackIO {

  private final ElevatorSim intakeRackSim;
  private final double reduction;

  public IntakeRackIOSim() {
    super(IntakeRackConstants.INTAKE_RACK_CONFIG.motorID());

    this.reduction = IntakeRackConstants.INTAKE_RACK_CONFIG.reduction();

    intakeRackSim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            reduction,
            IntakeRackConstants.PHYSICAL_CONSTANTS.massInKilograms(),
            IntakeRackConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            IntakeRackConstants.PHYSICAL_CONSTANTS.minExtensionMeters(),
            IntakeRackConstants.PHYSICAL_CONSTANTS.maxExtensionMeters(),
            IntakeRackConstants.PHYSICAL_CONSTANTS.simulateGravity(),
            0);
    setOffset();
    setSlot0(
        IntakeRackConstants.GAINS.kP(),
        IntakeRackConstants.GAINS.kI(),
        IntakeRackConstants.GAINS.kD(),
        IntakeRackConstants.GAINS.kS(),
        IntakeRackConstants.GAINS.kV(),
        IntakeRackConstants.GAINS.kA(),
        IntakeRackConstants.GAINS.kG(),
        IntakeRackConstants.MOTION_MAGIC_CONFIG.acceleration(),
        IntakeRackConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
        0,
        IntakeRackConstants.GRAVITY_TYPE);
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // Simulate physics
    intakeRackSim.setInputVoltage(appliedVoltage);
    intakeRackSim.update(0.02);

    // Convert position and velocity from meters to rotations for the
    // TalonFX sensor
    // Correct unit conversion: meters to rotations
    double rotations =
        intakeRackSim.getPositionMeters()
            / (2 * Math.PI * IntakeRackConstants.PHYSICAL_CONSTANTS.drumRadiusMeters())
            * reduction;

    // Correct unit conversion: meters/s to rotations/s
    double velocityRPS =
        intakeRackSim.getVelocityMetersPerSecond()
            / (2 * Math.PI * IntakeRackConstants.PHYSICAL_CONSTANTS.drumRadiusMeters())
            * reduction;

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    inputs.isConnected = true;
    inputs.positionRotations = rotations;
    inputs.velocityRotPerSec = velocityRPS;
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = 1.0; // Not simulated

    // update the Sim State to match if it is up or down
    if (rotations < .1) {
      RobotSimState.getInstance().setIntakeState(false);
    } else {
      RobotSimState.getInstance().setIntakeState(true);
    }
  }

  @Override
  public void setOffset() {
    intakeRackSim.setState(0, 0);
  }
}
