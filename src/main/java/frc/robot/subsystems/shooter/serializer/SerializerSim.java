package frc.robot.subsystems.shooter.serializer;

import static frc.robot.subsystems.shooter.serializer.SerializerConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

public class SerializerSim extends GenericRollersIOSim {
  private final FlywheelSim serializerSim;

  public SerializerSim() {
    super(
        SERIALIZER_CONFIG.motorID(),
        CURRENT_LIMIT_AMPS,
        SERIALIZER_CONFIG.inverted(),
        SERIALIZER_CONFIG.brake(),
        SERIALIZER_CONFIG.reduction());
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    serializerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                PHYSICAL_CONSTANTS.momentOfIntertia(),
                SERIALIZER_CONFIG.reduction()),
            DCMotor.getKrakenX60Foc(1));
    frc.robot.utility.SimBattery.getInstance().register(() -> lastSupplyCurrentAmps);
  }

  /** Last computed supply current, published to SimBattery. */
  private double lastSupplyCurrentAmps = 0.0;

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVelocity = talon.getSimState().getMotorVoltage();

    // Simulate physics
    serializerSim.setInputVoltage(appliedVelocity);
    serializerSim.update(0.02);

    double rotations = 0; // can't really be simulated

    // Divides our angular velocity by our reduction
    double velocityRPS =
        serializerSim.getAngularVelocityRadPerSec() / SERIALIZER_CONFIG.reduction();
    // FIXME: Doesn't work when reduction is 1

    talon.getSimState().setRawRotorPosition(rotations);
    talon.getSimState().setRotorVelocity(velocityRPS);

    // appliedVelocity actually holds a voltage (getMotorVoltage). Publish it as
    // appliedVolts too so the sim log carries the same key the real logs do.
    // supplyCurrentAmps was a hardcoded 1.0 A "not simulated".
    double availableVolts = RobotController.getBatteryVoltage();
    double statorAmps = Math.abs(serializerSim.getCurrentDrawAmps());
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVelocity) / availableVolts : 0.0;

    inputs.connected = true;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVelocity = appliedVelocity;
    inputs.appliedVolts = appliedVelocity;
    inputs.statorCurrentAmps = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
  }
}
