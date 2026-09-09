package frc.robot.subsystems.shooter.serializer;

import static frc.robot.subsystems.shooter.serializer.SerializerConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

public class SerializerSim extends GenericRollersIOSim {
  private final FlywheelSim serializerSim;
  private double rotorPositionRotations = 0.0;

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

    // Every other roller IOSim sets this; SerializerSim did not, which left the Talon's
    // output sign inconsistent with the physics it drives.
    talon.getSimState().Orientation =
        SERIALIZER_CONFIG.inverted()
            ? com.ctre.phoenix6.sim.ChassisReference.Clockwise_Positive
            : com.ctre.phoenix6.sim.ChassisReference.CounterClockwise_Positive;

    frc.robot.utility.SimBattery.getInstance()
        .register(() -> lastSupplyCurrentAmps, CURRENT_LIMIT_AMPS);
  }

  /** Last computed supply current, published to SimBattery. */
  private double lastSupplyCurrentAmps = 0.0;

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    // Update TalonFX state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

    double appliedVelocity = talon.getSimState().getMotorVoltage();

    // Enforce the supply limit on the voltage so the mechanism's torque is limited too.
    appliedVelocity =
        frc.robot.utility.SimCurrentLimit.clampToSupplyLimit(
            appliedVelocity,
            serializerSim.getAngularVelocityRadPerSec(),
            SERIALIZER_CONFIG.reduction(),
            DCMotor.getKrakenX60Foc(1),
            RobotController.getBatteryVoltage(),
            CURRENT_LIMIT_AMPS);

    // Simulate physics
    serializerSim.setInputVoltage(appliedVelocity);
    serializerSim.update(0.02);

    // Rotor velocity, not mechanism velocity: the Talon sim state expects rotor rot/s.
    // The old line divided rad/s by the reduction, conflating rad/s with rot/s AND applying
    // the gearing backwards, which left the serializer running in reverse all match
    // (AppliedVolts spanned [-3.36, 0.00] V against the real [0.00, 11.11] V).
    double mechanismRadPerSec = serializerSim.getAngularVelocityRadPerSec();
    double rotorRPS = mechanismRadPerSec / (2.0 * Math.PI) * SERIALIZER_CONFIG.reduction();
    rotorPositionRotations += rotorRPS * 0.02;

    talon.getSimState().setRawRotorPosition(rotorPositionRotations);
    talon.getSimState().setRotorVelocity(rotorRPS);

    // appliedVelocity actually holds a voltage (getMotorVoltage). Publish it as
    // appliedVolts too so the sim log carries the same key the real logs do.
    // supplyCurrentAmps was a hardcoded 1.0 A "not simulated".
    double availableVolts = RobotController.getBatteryVoltage();
    double statorAmps = serializerSim.getCurrentDrawAmps();
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVelocity) / availableVolts : 0.0;

    inputs.connected = true;
    // Report mechanism rad/s, matching GenericRollersIOTalonFX on the real robot.
    inputs.positionRads = rotorPositionRotations / SERIALIZER_CONFIG.reduction() * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = mechanismRadPerSec;
    // inputs.appliedVelocity is deliberately not set: GenericRollersIOTalonFX never populates
    // it, so the real log holds a constant 0.0 and any sim value scores as pure noise.
    inputs.appliedVolts = appliedVelocity;
    inputs.statorCurrentAmps = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
  }
}
