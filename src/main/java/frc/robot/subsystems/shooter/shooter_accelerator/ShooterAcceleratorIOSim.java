package frc.robot.subsystems.shooter.shooter_accelerator;

import static frc.robot.subsystems.shooter.shooter_accelerator.ShooterAcceleratorConstants.*;

import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.*;

// TODO: likely have to update shooterflywheelsiosim -- adjust values + motors might be wrong

public class ShooterAcceleratorIOSim extends GenericRollersIOSim implements ShooterAcceleratorIO {

  private final FlywheelSim shooterAcceleratorSim;
  private final SimpleMotorFeedforward feedforward;
  private double rotorPositionRotations = 0.0;
  private double velocitySetpointRPS = 0.0;

  public ShooterAcceleratorIOSim() {
    super(
        SHOOTER_ACCELERATOR_CONFIG.motorID1(),
        CURRENT_LIMIT_AMPS,
        SHOOTER_ACCELERATOR_CONFIG.inverted(),
        SHOOTER_ACCELERATOR_CONFIG.brake(),
        SHOOTER_ACCELERATOR_CONFIG.reduction());
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());
    // Create feedforward controller using configured gains
    feedforward = new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

    shooterAcceleratorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                PHYSICAL_CONSTANTS.momentOfInertia(),
                SHOOTER_ACCELERATOR_CONFIG.reduction()),
            DCMotor.getKrakenX60Foc(1));

    // Enable physics simulation for Phoenix
    var simState = talon.getSimState();
    simState.Orientation =
        SHOOTER_ACCELERATOR_CONFIG.inverted()
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    // The real robot drives this mechanism with 2 motors and registers each with
    // MotorOutputManager, so the pack sees 2x this current; the sim models one.
    frc.robot.utility.SimBattery.getInstance()
        .register(() -> lastSupplyCurrentAmps * 2, CURRENT_LIMIT_AMPS * 2.0);
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
    double currentVelocityRPS =
        shooterAcceleratorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    // Set TalonFX sim state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    talon.getSimState().setRawRotorPosition(rotorPositionRotations);
    talon.getSimState().setRotorVelocity(currentVelocityRPS);

    // Calculate applied voltage using feedforward + proportional feedback
    double feedforwardVoltage = feedforward.calculate(velocitySetpointRPS);
    double error = velocitySetpointRPS - currentVelocityRPS;
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
    }

    // Simulate physics
    shooterAcceleratorSim.setInputVoltage(appliedVoltage);
    shooterAcceleratorSim.update(0.02);

    // Update position tracking
    currentVelocityRPS = shooterAcceleratorSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    rotorPositionRotations += currentVelocityRPS * 0.02;

    inputs.connected = true;
    inputs.positionRads = rotorPositionRotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = shooterAcceleratorSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    // getCurrentDrawAmps() is stator current. Supply current is lower by roughly
    // the duty cycle, since the motor controller is a buck converter. Reporting
    // stator as supply overstates pack draw and would make the battery model sag
    // far harder than the real robot does.
    // Signed, not abs(): a negative draw is the mechanism back-driving and returning
    // energy. abs() booked every deceleration as consumption -- 47% of the omniwheel's
    // total error, and the real robot logs supply current down to -69.94 A.
    double statorAmps = shooterAcceleratorSim.getCurrentDrawAmps();
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVoltage) / availableVolts : 0.0;
    inputs.statorCurrentAmps = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
  }
}
