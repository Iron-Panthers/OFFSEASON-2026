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

    frc.robot.utility.SimBattery.getInstance()
        .register(() -> lastSupplyCurrentAmps, CURRENT_LIMIT_AMPS * 1.0);
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
    double currentVelocityRPS = intakeRollersSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);

    // Set TalonFX sim state
    talon.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    talon.getSimState().setRawRotorPosition(rotorPositionRotations);
    talon.getSimState().setRotorVelocity(currentVelocityRPS * INTAKE_ROLLER_CONFIG.reduction());

    // Regulate ROTOR velocity, matching Phoenix VelocityVoltage on the real robot.
    // GenericRollersIOTalonFX never sets SensorToMechanismRatio, so the setpoint the subsystem
    // passes down is in rotor rot/s; comparing it against MECHANISM rot/s made the sim settle at
    // a different speed for the same command, which is why the SIM reduction constants had been
    // fudged to the reciprocal of the real ones to compensate.
    double rotorVelocityRPS = currentVelocityRPS * INTAKE_ROLLER_CONFIG.reduction();
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
      // mechanism has 1.
      appliedVoltage =
          frc.robot.utility.SimCurrentLimit.clampToSupplyLimit(
              appliedVoltage,
              intakeRollersSim.getAngularVelocityRadPerSec(),
              INTAKE_ROLLER_CONFIG.reduction(),
              edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1),
              availableVolts,
              CURRENT_LIMIT_AMPS * 1.0);
    }

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
    // getCurrentDrawAmps() is stator current. Supply current is lower by roughly
    // the duty cycle, since the motor controller is a buck converter. Reporting
    // stator as supply overstates pack draw and would make the battery model sag
    // far harder than the real robot does.
    // Signed, not abs(): a negative draw is the mechanism back-driving and returning
    // energy. abs() booked every deceleration as consumption -- 47% of the omniwheel's
    // total error, and the real robot logs supply current down to -69.94 A.
    double statorAmps = intakeRollersSim.getCurrentDrawAmps();
    double dutyCycle = availableVolts > 0.0 ? Math.abs(appliedVoltage) / availableVolts : 0.0;
    inputs.statorCurrentAmps = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    lastSupplyCurrentAmps = inputs.supplyCurrentAmps;
    reportedSupplyCurrentAmps = inputs.supplyCurrentAmps;
  }
}
