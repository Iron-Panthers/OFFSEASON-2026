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
  }

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

    inputs.connected = true;
    inputs.velocityRadsPerSec = velocityRPS;
    inputs.appliedVelocity = appliedVelocity;
    inputs.supplyCurrentAmps = 1.0; // Not simulated
  }
}
