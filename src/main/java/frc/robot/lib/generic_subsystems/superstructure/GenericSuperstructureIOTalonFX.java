package frc.robot.lib.generic_subsystems.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.MotorOutputManager;
import java.util.ArrayList;

public abstract class GenericSuperstructureIOTalonFX implements GenericSuperstructureIO {
  // Talon FX Motor
  protected final TalonFX talon;

  // Follower motors
  protected final ArrayList<TalonFX> followerMotors;
  protected final ArrayList<StatusSignal<Current>> followerMotorSupplyCurrents;

  // Motor config
  protected final TalonFXConfiguration config = new TalonFXConfiguration();

  // status signals
  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRPS;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;

  protected Slot0Configs gainsConfig;

  // zeroing stuff
  private final double zeroingVolts;
  protected final double zeroingOffset;

  protected final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);
  private final NeutralOut neutralOutput = new NeutralOut();
  private final DynamicMotionMagicVoltage positionControl =
      new DynamicMotionMagicVoltage(0, 0, 0).withUpdateFreqHz(0);

  /** Constructs a new GenericSuperstructureIOTalonFX. */
  public GenericSuperstructureIOTalonFX(GenericSuperstructureConfiguration superstructureConfig) {
    /* set the zeroing values such that when the robot zeros it will apply the
     * zeroing volts and when it reaches a resistance from part of the mechanism, it
     * sets the position to the zeroing offset */
    this.zeroingVolts = superstructureConfig.zeroingVolts;
    this.zeroingOffset = superstructureConfig.zeroingOffset;

    // VOLTAGE, LIMITS AND RATIO CONFIG
    config.MotorOutput.Inverted = superstructureConfig.motorDirection;
    config.CurrentLimits.SupplyCurrentLimit = superstructureConfig.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = superstructureConfig.statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Voltage.withPeakForwardVoltage(superstructureConfig.upperVoltLimit);
    config.Voltage.withPeakReverseVoltage(superstructureConfig.lowerVoltLimit);
    config.Feedback.withSensorToMechanismRatio(superstructureConfig.reduction);

    if (superstructureConfig.lowerExtensionLimitEnabled) {
      config.SoftwareLimitSwitch.withReverseSoftLimitEnable(
          superstructureConfig.lowerExtensionLimitEnabled);
      config.SoftwareLimitSwitch.withReverseSoftLimitThreshold(
          superstructureConfig.lowerExtensionLimit);
    }
    if (superstructureConfig.upperExtensionLimitEnabled) {
      config.SoftwareLimitSwitch.withForwardSoftLimitEnable(
          superstructureConfig.upperExtensionLimitEnabled);
      config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
          superstructureConfig.upperExtensionLimit);
    }
    talon = new TalonFX(superstructureConfig.id);

    if (superstructureConfig.canCoderID != -1) { // TODO: Make default -1 or use Optional
      CANcoder canCoder = new CANcoder(superstructureConfig.canCoderID);
      canCoder
          .getConfigurator()
          .apply(
              new CANcoderConfiguration()
                  .withMagnetSensor(
                      new MagnetSensorConfigs()
                          .withAbsoluteSensorDiscontinuityPoint(
                              superstructureConfig.sensorDiscontinuityPoint)
                          .withSensorDirection(superstructureConfig.canCoderDirection)
                          .withMagnetOffset(superstructureConfig.canCoderOffset)));
      config.Feedback.withRemoteCANcoder(canCoder);
    }
    // TODO: Do we need to try until OK?
    talon.getConfigurator().apply(config);
    setOffset();
    talon.setNeutralMode(NeutralModeValue.Brake);

    // Initialize follower motors
    followerMotors = new ArrayList<>();
    followerMotorSupplyCurrents = new ArrayList<>();
    for (GenericSuperstructureConfiguration.FollowerMotorConfig followerConfig :
        superstructureConfig.followerMotors) {
      TalonFX followerTalon = new TalonFX(followerConfig.id());
      followerTalon.setControl(
          new Follower(superstructureConfig.id, followerConfig.motorAlignmentValue()));
      followerTalon.setNeutralMode(NeutralModeValue.Brake);
      followerTalon.getConfigurator().apply(config);
      followerTalon.optimizeBusUtilization();
      followerMotors.add(followerTalon);

      MotorOutputManager.getInstance()
          .registerMotorOutputs(() -> followerTalon.getSupplyCurrent().getValueAsDouble());
    }

    // STATUS SIGNALS
    velocityRPS = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    statorCurrent = talon.getStatorCurrent();
    positionRotations = talon.getPosition();

    MotorOutputManager.getInstance().registerMotorOutputs(() -> supplyCurrent.getValueAsDouble());

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionRotations, velocityRPS, appliedVolts, supplyCurrent, statorCurrent);
    if (followerMotors.size() == 0) {
      talon.optimizeBusUtilization();
    }

    MotorOutputManager.getInstance().registerMotorOutputs(() -> supplyCurrent.getValueAsDouble());

    for (StatusSignal<Current> motorCurrent : followerMotorSupplyCurrents) {
      MotorOutputManager.getInstance().registerMotorOutputs(() -> motorCurrent.getValueAsDouble());
    }
  }

  @Override
  public void updateInputs(GenericSuperstructureIOInputs inputs) {
    inputs.isConnected =
        BaseStatusSignal.refreshAll(
                positionRotations, velocityRPS, appliedVolts, supplyCurrent, statorCurrent)
            .isOK();
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.velocityRotPerSec = velocityRPS.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
  }

  @Override
  public void runPosition(double rotations) {
    talon.setControl(positionControl.withPosition(rotations));
  }

  @Override
  public void runCharacterization() {
    talon.setControl(voltageOutput.withOutput(zeroingVolts));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
  }

  @Override
  public void setOffset() {
    talon.getConfigurator().setPosition(zeroingOffset);
  }

  /**
   * Sets all of the PID and motion magic gains.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   * @param kG Gravity gain
   * @param motionMagicAcceleration Motion magic acceleration (rotations per second squared)
   * @param motionMagicCruiseVelocity Motion magic cruise velocity (rotations per second)
   * @param motionMagicJerk Motion magic jerk (rotations per second cubed)
   * @param gravityTypeValue Gravity compensation type
   */
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
  public void setSupplyCurrentLimit(double amps) {
    if (Math.abs(config.CurrentLimits.SupplyCurrentLimit - amps) > 0.01) {
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = amps;
      config.withSlot0(gainsConfig);
      talon.getConfigurator().apply(config);
    }
  }

  @Override
  public void setMaxCruiseVelocity(double cruiseVelocity) {
    positionControl.withVelocity(cruiseVelocity);
  }
}
