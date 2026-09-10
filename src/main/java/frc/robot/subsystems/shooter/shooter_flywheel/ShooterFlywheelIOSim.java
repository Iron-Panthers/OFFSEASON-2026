package frc.robot.subsystems.shooter.shooter_flywheel;

import static frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheelConstants.*;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

// TODO: likely have to update shooterflywheelsiosim -- adjust values + motors might be wrong

public class ShooterFlywheelIOSim extends GenericRollersIOSim implements ShooterFlywheelIO {

  private final FlywheelSim shooterFlywheelsSim;
  private final SimpleMotorFeedforward feedforward;
  private double rotorPositionRotations = 0.0;
  private double velocitySetpointRPS = 0.0;

  /**
   * Motors the plant models, and motors actually on the mechanism.
   *
   * <p>These differ where the plant lumps a multi-motor mechanism into one motor. The logged
   * signals are PER MOTOR, because {@code GenericRollersIOTalonFX} reads the leader Talon only, but
   * the battery and {@code MotorOutputManager} have to see every motor.
   */
  private static final int PLANT_MOTORS = 4;

  private static final int PACK_MOTORS = 4;

  private static final DCMotor MOTORS = DCMotor.getKrakenX60Foc(PLANT_MOTORS);

  /**
   * Steady-state drag, in stator amps per motor per mechanism rad/s.
   *
   * <p>Measured from real match logs: median stator current over samples where the mechanism was
   * spinning, powered, and not accelerating, divided by the median speed. Measured 3.1-3.8 A per
   * motor at 239 rad/s across q93/q14/q103/q64. q54 sat at 12.2 A and is excluded as an outlier --
   * no other match comes near it.
   *
   * <p>Modelled as viscous (through the origin) rather than Coulomb because every match runs this
   * mechanism at essentially one speed, so the two are indistinguishable from the data. Viscous is
   * the safer of the two: it goes to zero at rest instead of chattering there.
   *
   * <p>Calibrated against the median of five real matches, not against any one of them: the
   * coefficient is set so that {@code transient + coeff * simSpeed} lands on the median real mean
   * stator current, where {@code transient} is what the plant produces on its own (spin-ups,
   * braking) and was measured from a run at the steady-state coefficient. This mechanism produced
   * essentially no transient current of its own (0.49 A), so the coefficient carries nearly the
   * whole load.
   *
   * <p>It is therefore a lumped AVERAGE MATCH LOAD, not pure bearing friction. Unloaded steady
   * state measures 0.0145 (3.1-3.8 A per motor at 239 rad/s); q54 alone measured 0.051 and is
   * excluded; the rest is work done on game pieces, which nothing in the sim models. That is a
   * fitted parameter and is labelled as one -- but it is fitted to a five-log median of a directly
   * measured physical quantity, not to a fit score.
   */
  private static final double DRAG_AMPS_PER_RAD_PER_SEC = 0.0341;

  public ShooterFlywheelIOSim() {
    super(
        SHOOTER_FLYWHEEL_CONFIG.motorID1(),
        CURRENT_LIMIT_AMPS,
        SHOOTER_FLYWHEEL_CONFIG.inverted(),
        SHOOTER_FLYWHEEL_CONFIG.brake(),
        SHOOTER_FLYWHEEL_CONFIG.reduction());
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    // Create feedforward controller using configured gains
    feedforward = new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

    shooterFlywheelsSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(4),
                PHYSICAL_CONSTANTS.momentOfInertia(),
                SHOOTER_FLYWHEEL_CONFIG.reduction()),
            MOTORS);

    // Enable physics simulation for Phoenix
    var simState = talon.getSimState();
    simState.Orientation =
        SHOOTER_FLYWHEEL_CONFIG.inverted()
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    // lastSupplyCurrentAmps is the whole mechanism's, so the limit has to be too.
    frc.robot.utility.SimBattery.getInstance()
        .register(() -> lastSupplyCurrentAmps, CURRENT_LIMIT_AMPS * PACK_MOTORS);
  }

  /** Last computed supply current, published to SimBattery. */
  private double lastSupplyCurrentAmps = 0.0;

  @Override
  public void runVelocity(double velocity) {
    velocitySetpointRPS = velocity;
    super.runVelocity(velocity);
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    double currentVelocityRPS = shooterFlywheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    // Set TalonFX sim state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    talon.getSimState().setRawRotorPosition(rotorPositionRotations);
    talon.getSimState().setRotorVelocity(currentVelocityRPS * SHOOTER_FLYWHEEL_CONFIG.reduction());

    // Regulate ROTOR velocity, matching Phoenix VelocityVoltage on the real robot.
    // GenericRollersIOTalonFX never sets SensorToMechanismRatio, so the setpoint the subsystem
    // passes down is in rotor rot/s; comparing it against MECHANISM rot/s made the sim settle at
    // a different speed for the same command, which is why the SIM reduction constants had been
    // fudged to the reciprocal of the real ones to compensate.
    double rotorVelocityRPS = currentVelocityRPS * SHOOTER_FLYWHEEL_CONFIG.reduction();
    double feedforwardVoltage = feedforward.calculate(velocitySetpointRPS);
    double error = velocitySetpointRPS - rotorVelocityRPS;
    double proportionalVoltage = GAINS.kP() * error;
    double appliedVoltage = feedforwardVoltage + proportionalVoltage;
    // Clamp to the battery's ACTUAL voltage so sag reduces available torque.
    // The old hardcoded +/-12 made brownouts cosmetic: the pack could read 6 V
    // while the motor still behaved as though it had a full 12 V to work with.
    double availableVolts = RobotController.getBatteryVoltage();
    appliedVoltage = Math.max(-availableVolts, Math.min(availableVolts, appliedVoltage));
    if (coasting) {
      // Commanded to stop: coast, matching the Talon's NeutralOut on the real robot.
      appliedVoltage = 0.0;
    } else {
      // Enforce the supply limit on the VOLTAGE, not just on the reported current. Clamping
      // only the number would leave the mechanism accelerating as though unlimited; a real
      // current-limited motor also makes less torque. CURRENT_LIMIT_AMPS is per motor, and this
      // mechanism has 4.
      appliedVoltage =
          frc.robot.utility.SimCurrentLimit.clampToSupplyLimit(
              appliedVoltage,
              shooterFlywheelsSim.getAngularVelocityRadPerSec(),
              SHOOTER_FLYWHEEL_CONFIG.reduction(),
              MOTORS,
              availableVolts,
              CURRENT_LIMIT_AMPS * 4.0);
    }

    // Simulate physics. FlywheelSim is frictionless, so the drag the real mechanism fights all
    // match has to be injected by hand: spend the voltage it costs, and the mechanism settles
    // where the real one does and coasts down at the real rate instead of freewheeling.
    double dragVolts =
        frc.robot.utility.SimCurrentLimit.dragVolts(
            shooterFlywheelsSim.getAngularVelocityRadPerSec(),
            MOTORS,
            DRAG_AMPS_PER_RAD_PER_SEC * PLANT_MOTORS);
    shooterFlywheelsSim.setInputVoltage(appliedVoltage - dragVolts);
    shooterFlywheelsSim.update(0.02);

    // Update position tracking
    currentVelocityRPS = shooterFlywheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    rotorPositionRotations += currentVelocityRPS * 0.02;

    inputs.connected = true;
    inputs.positionRads = rotorPositionRotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = shooterFlywheelsSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    // Stator current computed directly rather than read back from the plant. FlywheelSim
    // reports zero at every steady state (it inverts its own plant to get back-EMF, so the two
    // terms cancel), which is exactly the current the drag above is there to create.
    //
    // Reported PER MOTOR, because GenericRollersIOTalonFX logs the leader Talon only while the
    // plant models the whole group. Supply current is lower than stator by roughly the duty
    // cycle, since the motor controller is a buck converter.
    double statorAmps =
        coasting
            ? 0.0
            : frc.robot.utility.SimCurrentLimit.statorAmps(
                    appliedVoltage,
                    shooterFlywheelsSim.getAngularVelocityRadPerSec(),
                    SHOOTER_FLYWHEEL_CONFIG.reduction(),
                    MOTORS)
                / PLANT_MOTORS;
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVoltage) / availableVolts : 0.0;
    inputs.statorCurrentAmps = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    // The pack sees every motor on the mechanism, not just the one the log reports.
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps * PACK_MOTORS;
    reportedSupplyCurrentAmps = lastSupplyCurrentAmps;
  }
}
