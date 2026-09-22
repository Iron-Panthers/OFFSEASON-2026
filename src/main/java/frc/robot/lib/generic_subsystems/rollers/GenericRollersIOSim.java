package frc.robot.lib.generic_subsystems.rollers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.MotorOutputManager;
import frc.robot.utility.SimBattery;
import frc.robot.utility.SimCurrentLimit;

/**
 * Flywheel-plant simulation shared by every roller mechanism.
 *
 * <p>Applies the supply current limit to the voltage, a measured drag load, and reports current per
 * motor like {@link GenericRollersIOTalonFX} does. See docs/sim-fidelity-results.md.
 */
public abstract class GenericRollersIOSim implements GenericRollersIO {
  private static final double DT = 0.02;

  /**
   * Plant description for a roller sim.
   *
   * @param plantMotors motors modelled in the plant
   * @param packMotors motors drawing from the battery; can exceed plantMotors when a follower is
   *     not modelled
   * @param momentOfInertia mechanism MOI, kg m^2
   * @param dragAmpsPerRadPerSec drag current per motor per mechanism rad/s, measured from match
   *     logs
   */
  public record RollerSim(
      int plantMotors, int packMotors, double momentOfInertia, double dragAmpsPerRadPerSec) {}

  protected final TalonFX talon;
  protected boolean coasting = false;

  private final NeutralOut neutralOutput = new NeutralOut();
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);
  private final double reduction;
  private final int currentLimitAmps;
  private final RollerSim sim;
  private final DCMotor motors;
  private final FlywheelSim plant;

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);
  private double kP = 0.0;
  private double velocitySetpointRPS = 0.0;
  private double rotorPositionRotations = 0.0;
  private double packSupplyCurrentAmps = 0.0;

  public GenericRollersIOSim(
      int id,
      int currentLimitAmps,
      boolean inverted,
      boolean brake,
      double reduction,
      RollerSim sim) {
    talon = new TalonFX(id);
    this.reduction = reduction;
    this.currentLimitAmps = currentLimitAmps;
    this.sim = sim;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);
    talon.optimizeBusUtilization();
    talon.getSimState().Orientation =
        inverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;

    motors = DCMotor.getKrakenX60Foc(sim.plantMotors());
    plant =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(motors, sim.momentOfInertia(), reduction), motors);

    // The real IO registers with both; without these sim pack current omitted every mechanism.
    MotorOutputManager.getInstance().registerMotorOutputs(() -> packSupplyCurrentAmps);
    SimBattery.getInstance()
        .register(() -> packSupplyCurrentAmps, currentLimitAmps * sim.packMotors());
  }

  /**
   * Whether the applied voltage comes from the Talon's own closed loop instead of a sim-side
   * velocity loop. The sim-side loop mirrors Phoenix {@code VelocityVoltage} on the rotor.
   */
  protected boolean followsTalonClosedLoop() {
    return false;
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    double busVolts = RobotController.getBatteryVoltage();
    talon.getSimState().setSupplyVoltage(busVolts);
    double mechanismRadPerSec = plant.getAngularVelocityRadPerSec();
    double limitAmps = currentLimitAmps * sim.plantMotors();
    boolean talonLoop = followsTalonClosedLoop();

    double volts;
    if (talonLoop) {
      volts = talon.getSimState().getMotorVoltage();
    } else if (coasting) {
      volts = 0.0;
    } else {
      double rotorRPS = mechanismRadPerSec / (2.0 * Math.PI) * reduction;
      volts = feedforward.calculate(velocitySetpointRPS) + kP * (velocitySetpointRPS - rotorRPS);
    }
    if (talonLoop || !coasting) {
      volts =
          SimCurrentLimit.clampToSupplyLimit(
              volts, mechanismRadPerSec, reduction, motors, busVolts, limitAmps);
    }

    double dragVolts =
        SimCurrentLimit.dragVolts(
            mechanismRadPerSec, motors, sim.dragAmpsPerRadPerSec() * sim.plantMotors());
    plant.setInputVoltage(volts - dragVolts);
    plant.update(DT);

    mechanismRadPerSec = plant.getAngularVelocityRadPerSec();
    double rotorRPS = mechanismRadPerSec / (2.0 * Math.PI) * reduction;
    rotorPositionRotations += rotorRPS * DT;
    talon.getSimState().setRawRotorPosition(rotorPositionRotations);
    talon.getSimState().setRotorVelocity(rotorRPS);

    double statorAmps =
        (!talonLoop && coasting)
            ? 0.0
            : SimCurrentLimit.statorAmps(volts, mechanismRadPerSec, reduction, motors)
                / sim.plantMotors();
    double dutyCycle = busVolts > 0.0 ? Math.abs(volts) / busVolts : 0.0;

    inputs.connected = true;
    inputs.positionRads = rotorPositionRotations / reduction * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = mechanismRadPerSec;
    inputs.appliedVolts = volts;
    inputs.statorCurrentAmps = statorAmps;
    inputs.supplyCurrentAmps = statorAmps * dutyCycle;
    packSupplyCurrentAmps = inputs.supplyCurrentAmps * sim.packMotors();
  }

  @Override
  public void runVelocity(double velocity) {
    coasting = false;
    velocitySetpointRPS = velocity;
    talon.setControl(velocityControl.withVelocity(velocity));
  }

  @Override
  public void stop() {
    coasting = true;
    talon.setControl(neutralOutput);
  }

  @Override
  public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    this.kP = kP;
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    Slot0Configs gainsConfig = new Slot0Configs();
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;
    talon.getConfigurator().apply(gainsConfig);
  }
}
