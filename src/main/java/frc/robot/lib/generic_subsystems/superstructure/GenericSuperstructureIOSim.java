package frc.robot.lib.generic_subsystems.superstructure;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.MotorOutputManager;

public abstract class GenericSuperstructureIOSim implements GenericSuperstructureIO {

  protected final TalonFX talon;

  protected final TalonFXConfiguration config;
  protected Slot0Configs gainsConfig = new Slot0Configs();

  protected final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);

  protected final NeutralOut neutralOutput = new NeutralOut();

  protected final DynamicMotionMagicVoltage positionControl =
      new DynamicMotionMagicVoltage(0, 0, 0).withUpdateFreqHz(0);

  /** Sensor-to-mechanism reduction, so subclasses can convert consistently. */
  protected final double mechanismReduction;

  /** Last supply current reported by the subclass, in amps, for aggregate power bookkeeping. */
  protected double reportedSupplyCurrentAmps = 0.0;

  public GenericSuperstructureIOSim(int id) {
    this(id, 1.0);
  }

  /**
   * @param id CAN id
   * @param reduction sensor-to-mechanism reduction; rotor rotations = mechanism rotations x this
   */
  public GenericSuperstructureIOSim(int id, double reduction) {

    talon = new TalonFX(id);
    talon.setNeutralMode(NeutralModeValue.Brake);
    mechanismReduction = reduction;
    config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            // GenericSuperstructureIOTalonFX sets this and the sim did not, and the sim also never
            // applied its config at all -- only the gains and motion-magic configs were applied.
            // The sim's position loop therefore ran in ROTOR units while the real robot ran in
            // MECHANISM units, a factor of `reduction` (8/pi = 2.55 for the intake rack) in travel
            // distance, kP stiffness and cruise velocity. That is why the simulated rack deployed
            // ~1.7x faster than the real one: it was moving 2.55x less actual distance.
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(reduction));
    talon.getConfigurator().apply(config);

    // See GenericRollersIOSim: the TalonFX class registers with MotorOutputManager and the sim
    // class did not, so simulated TotalAmps omitted every mechanism.
    MotorOutputManager.getInstance().registerMotorOutputs(() -> reportedSupplyCurrentAmps);
  }

  @Override
  public void runPosition(double rotations) {
    talon.setControl(positionControl.withPosition(rotations));
  }

  @Override
  public abstract void updateInputs(GenericSuperstructureIOInputs inputs);

  @Override
  public void runCharacterization() {
    talon.setControl(voltageOutput.withOutput(-1));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }

  @Override
  public void setSlot0(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      double motionMagicAcceleration,
      double motionMagicCruiseVelocity,
      double motionMagicJerk,
      GravityTypeValue gravityTypeValue) {
    gainsConfig = new Slot0Configs();
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;
    gainsConfig.kG = kG;
    gainsConfig.GravityType = gravityTypeValue;

    MotionMagicConfigs motionMagicConfig = new MotionMagicConfigs();
    motionMagicConfig.MotionMagicAcceleration = motionMagicAcceleration;
    motionMagicConfig.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    motionMagicConfig.MotionMagicJerk = motionMagicJerk;

    positionControl.withAcceleration(motionMagicAcceleration);
    positionControl.withVelocity(motionMagicCruiseVelocity);
    positionControl.withJerk(motionMagicJerk);

    talon.getConfigurator().apply(gainsConfig);
    talon.getConfigurator().apply(motionMagicConfig);
  }

  @Override
  public void setMaxCruiseVelocity(double cruiseVelocity) {
    positionControl.withVelocity(cruiseVelocity);
  }
}
