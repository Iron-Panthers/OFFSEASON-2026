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

  /**
   * Motors the plant models, and motors actually on the mechanism.
   *
   * <p>The plant lumps both serializer motors into one. The logged signals stay PER MOTOR, matching
   * {@code GenericRollersIOTalonFX}, which reads the leader Talon only; the battery and {@code
   * MotorOutputManager} see both.
   */
  private static final int PLANT_MOTORS = 1;

  private static final int PACK_MOTORS = 2;

  private static final DCMotor MOTORS = DCMotor.getKrakenX60Foc(PLANT_MOTORS);

  /**
   * Steady-state drag, in stator amps per motor per mechanism rad/s.
   *
   * <p>By far the largest of any mechanism: 21-24 A per motor at 118 rad/s, and the most consistent
   * -- every one of the five matches measured lands in 0.176-0.206. That is unsurprising for a
   * mechanism whose whole job is pushing game pieces against a wall, but it also cannot be
   * explained by back-EMF alone. The textbook {@code (V - backEmf)/R} predicts 2.3 A at the logged
   * 6.69 V and 118.5 rad/s, an order of magnitude under what the robot actually drew, so some of
   * this coefficient is standing in for a reduction or motor constant that is not quite right.
   * Calibrating on the measured current rather than on the constants makes the simulated draw come
   * out correct either way.
   *
   * <p>Calibrated against the median of five real matches, not against any one of them: the
   * coefficient is set so that {@code transient + coeff * simSpeed} lands on the median real mean
   * stator current, where {@code transient} is what the plant produces on its own (spin-ups,
   * braking) and was measured from a run at the steady-state coefficient. This mechanism produced
   * 1.89 A of transient on its own, so the coefficient carries almost the whole load.
   *
   * <p>It is therefore a lumped AVERAGE MATCH LOAD, not pure bearing friction. Unloaded steady
   * state measures 0.1984 (21-24 A per motor at 118 rad/s); the rest is work done on game pieces,
   * which nothing in the sim models. That is a fitted parameter and is labelled as one -- but it is
   * fitted to a five-log median of a directly measured physical quantity, not to a fit score.
   */
  private static final double DRAG_AMPS_PER_RAD_PER_SEC = 0.2237;

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
            MOTORS);

    // Every other roller IOSim sets this; SerializerSim did not, which left the Talon's
    // output sign inconsistent with the physics it drives.
    talon.getSimState().Orientation =
        SERIALIZER_CONFIG.inverted()
            ? com.ctre.phoenix6.sim.ChassisReference.Clockwise_Positive
            : com.ctre.phoenix6.sim.ChassisReference.CounterClockwise_Positive;

    frc.robot.utility.SimBattery.getInstance()
        .register(() -> lastSupplyCurrentAmps, CURRENT_LIMIT_AMPS * PACK_MOTORS);
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
            MOTORS,
            RobotController.getBatteryVoltage(),
            CURRENT_LIMIT_AMPS);

    // Simulate physics. FlywheelSim is frictionless, so the drag the real mechanism fights all
    // match has to be injected by hand: spend the voltage it costs, and the mechanism settles
    // where the real one does and coasts down at the real rate instead of freewheeling.
    double dragVolts =
        frc.robot.utility.SimCurrentLimit.dragVolts(
            serializerSim.getAngularVelocityRadPerSec(),
            MOTORS,
            DRAG_AMPS_PER_RAD_PER_SEC * PLANT_MOTORS);
    serializerSim.setInputVoltage(appliedVelocity - dragVolts);
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
    // Stator current computed directly rather than read back from the plant. FlywheelSim reports
    // zero at every steady state (it inverts its own plant to get back-EMF, so the two terms
    // cancel), which is exactly the current the drag above is there to create. Evaluated against
    // the COMMANDED voltage, not the post-drag one -- the difference between them is the drag.
    double statorAmps =
        frc.robot.utility.SimCurrentLimit.statorAmps(
                appliedVelocity, mechanismRadPerSec, SERIALIZER_CONFIG.reduction(), MOTORS)
            / PLANT_MOTORS;
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
    // The pack sees both serializer motors, not just the one the log reports.
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps * PACK_MOTORS;
    reportedSupplyCurrentAmps = lastSupplyCurrentAmps;
  }
}
