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
    super(
        IntakeRackConstants.INTAKE_RACK_CONFIG.motorID(),
        IntakeRackConstants.INTAKE_RACK_CONFIG.reduction());

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
    frc.robot.utility.SimBattery.getInstance()
        .register(() -> lastSupplyCurrentAmps, IntakeRackConstants.SUPPLY_CURRENT_LIMIT);
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

  /** Last computed supply current, published to SimBattery. */
  private double lastSupplyCurrentAmps = 0.0;

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVoltage = talon.getSimState().getMotorVoltage();

    // The superstructure sim never applies its TalonFX config, so the 27 A supply limit the
    // subsystem pushes down every loop reaches nothing. Real rack supply p95 is 27.96 A against
    // that limit -- it is clearly binding on the real robot.
    appliedVoltage =
        frc.robot.utility.SimCurrentLimit.clampToSupplyLimit(
            appliedVoltage,
            intakeRackSim.getVelocityMetersPerSecond()
                / IntakeRackConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            reduction,
            DCMotor.getKrakenX60Foc(1),
            RobotController.getBatteryVoltage(),
            IntakeRackConstants.SUPPLY_CURRENT_LIMIT);

    // Simulate physics
    intakeRackSim.setInputVoltage(appliedVoltage);
    intakeRackSim.update(0.02);

    // Mechanism units first, then convert up to rotor units for the sensor. The TalonFX sim
    // state wants ROTOR rotations; the subsystem and the real robot both work in MECHANISM
    // rotations. Reporting rotor units here was the other half of the unit mismatch.
    double mechanismRotations =
        intakeRackSim.getPositionMeters()
            / (2 * Math.PI * IntakeRackConstants.PHYSICAL_CONSTANTS.drumRadiusMeters());
    double mechanismRPS =
        intakeRackSim.getVelocityMetersPerSecond()
            / (2 * Math.PI * IntakeRackConstants.PHYSICAL_CONSTANTS.drumRadiusMeters());

    talon.getSimState().setRawRotorPosition(mechanismRotations * reduction);
    talon.getSimState().setRotorVelocity(mechanismRPS * reduction);

    inputs.isConnected = true;
    inputs.positionRotations = mechanismRotations;
    inputs.velocityRotPerSec = mechanismRPS;
    inputs.appliedVolts = appliedVoltage;
    // Was a hardcoded 1.0 A "not simulated", which meant the intake rack
    // contributed a constant fake load and could never show a real current spike.
    double availableVolts = RobotController.getBatteryVoltage();
    double statorAmps = intakeRackSim.getCurrentDrawAmps();
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVoltage) / availableVolts : 0.0;
    inputs.statorCurrent = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
    reportedSupplyCurrentAmps = inputs.supplyCurrentAmps;

    // update the Sim State to match if it is up or down
    if (mechanismRotations < .1) {
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
