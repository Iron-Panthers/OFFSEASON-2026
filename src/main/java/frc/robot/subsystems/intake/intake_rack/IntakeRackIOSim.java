package frc.robot.subsystems.intake.intake_rack;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotSimState;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class IntakeRackIOSim extends GenericSuperstructureIOSim implements IntakeRackIO {

  private final ElevatorSim intakeRackSim;
  private final double reduction;

  /** Holding load toward stow: median applied voltage while the real rack is deployed and still. */
  private static final double LOAD_VOLTS = 0.61;

  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

  public IntakeRackIOSim() {
    super(
        IntakeRackConstants.INTAKE_RACK_CONFIG.motorID(),
        IntakeRackConstants.INTAKE_RACK_CONFIG.reduction());

    this.reduction = IntakeRackConstants.INTAKE_RACK_CONFIG.reduction();

    intakeRackSim =
        new ElevatorSim(
            MOTOR,
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

    appliedVoltage =
        frc.robot.utility.SimCurrentLimit.clampToSupplyLimit(
            appliedVoltage,
            intakeRackSim.getVelocityMetersPerSecond()
                / IntakeRackConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            reduction,
            MOTOR,
            RobotController.getBatteryVoltage(),
            IntakeRackConstants.SUPPLY_CURRENT_LIMIT);

    intakeRackSim.setInputVoltage(
        frc.robot.utility.SimCurrentLimit.applyConstantLoad(appliedVoltage, LOAD_VOLTS));
    intakeRackSim.update(0.02);

    // Mechanism units, converted to rotor units for the sensor.
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
    double availableVolts = RobotController.getBatteryVoltage();
    // Against the commanded voltage, so holding the load draws current.
    double statorAmps =
        frc.robot.utility.SimCurrentLimit.statorAmps(
            appliedVoltage,
            intakeRackSim.getVelocityMetersPerSecond()
                / IntakeRackConstants.PHYSICAL_CONSTANTS.drumRadiusMeters(),
            reduction,
            MOTOR);
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
