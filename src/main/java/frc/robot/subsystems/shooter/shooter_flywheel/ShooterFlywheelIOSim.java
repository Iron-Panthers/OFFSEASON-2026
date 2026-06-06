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
                DCMotor.getKrakenX60Foc(1),
                PHYSICAL_CONSTANTS.momentOfInertia(),
                SHOOTER_FLYWHEEL_CONFIG.reduction()),
            DCMotor.getKrakenX60Foc(1));

    // Enable physics simulation for Phoenix
    var simState = talon.getSimState();
    simState.Orientation =
        SHOOTER_FLYWHEEL_CONFIG.inverted()
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
  }

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
    talon.getSimState().setRotorVelocity(currentVelocityRPS);

    // Calculate applied voltage using feedforward + proportional feedback
    double feedforwardVoltage = feedforward.calculate(velocitySetpointRPS);
    double error = velocitySetpointRPS - currentVelocityRPS;
    double proportionalVoltage = GAINS.kP() * error;
    double appliedVoltage = feedforwardVoltage + proportionalVoltage;
    appliedVoltage = Math.max(-12, Math.min(12, appliedVoltage)); // Clamp to battery voltage

    // Simulate physics
    shooterFlywheelsSim.setInputVoltage(appliedVoltage);
    shooterFlywheelsSim.update(0.02);

    // Update position tracking
    currentVelocityRPS = shooterFlywheelsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    rotorPositionRotations += currentVelocityRPS * 0.02;

    inputs.connected = true;
    inputs.positionRads = rotorPositionRotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = shooterFlywheelsSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = Math.abs(shooterFlywheelsSim.getCurrentDrawAmps());
  }
}
