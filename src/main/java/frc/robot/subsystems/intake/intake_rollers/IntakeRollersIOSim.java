package frc.robot.subsystems.intake.intake_rollers;

import static frc.robot.subsystems.intake.intake_rollers.IntakeRollersConstants.*;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOSim;

public class IntakeRollersIOSim extends GenericRollersIOSim implements IntakeRollersIO {
  private final FlywheelSim intakeRollersSim;
  private final SimpleMotorFeedforward feedforward;
  private double rotorPositionRotations = 0.0;
  private double velocitySetpointRPS = 0.0;

  public IntakeRollersIOSim() {
    super(
        INTAKE_ROLLER_CONFIG.motorID(),
        CURRENT_LIMIT_AMPS,
        INTAKE_ROLLER_CONFIG.inverted(),
        INTAKE_ROLLER_CONFIG.brake(),
        INTAKE_ROLLER_CONFIG.reduction());
    super.setSlot0(GAINS.kP(), GAINS.kI(), GAINS.kD(), GAINS.kS(), GAINS.kV(), GAINS.kA());

    // Create feedforward controller using configured gains
    feedforward = new SimpleMotorFeedforward(GAINS.kS(), GAINS.kV(), GAINS.kA());

    intakeRollersSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(1),
                PHYSICAL_CONSTANTS.momentOfInertia(),
                INTAKE_ROLLER_CONFIG.reduction()),
            DCMotor.getKrakenX60Foc(1));

    // Enable physics simulation for Phoenix
    var simState = talon.getSimState();
    simState.Orientation =
        INTAKE_ROLLER_CONFIG.inverted()
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  public void runVelocity(double velocity) {
    velocitySetpointRPS = velocity;
    super.runVelocity(velocity);
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    double currentVelocityRPS = intakeRollersSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

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
    intakeRollersSim.setInputVoltage(appliedVoltage);
    intakeRollersSim.update(0.02);

    // Update position tracking
    currentVelocityRPS = intakeRollersSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    rotorPositionRotations += currentVelocityRPS * 0.02;

    inputs.connected = true;
    inputs.positionRads = rotorPositionRotations * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = intakeRollersSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = Math.abs(intakeRollersSim.getCurrentDrawAmps());
  }
}
