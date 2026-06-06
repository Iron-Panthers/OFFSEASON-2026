package frc.robot.lib.generic_subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.MotorOutputManager;
import java.util.ArrayList;

public abstract class GenericRollersIOTalonFX implements GenericRollersIO {
  protected final TalonFX talon;
  protected final ArrayList<TalonFX> followerMotors;
  protected final ArrayList<StatusSignal<Current>> followerMotorSupplyCurrents;
  protected final TalonFXConfiguration config;
  protected Slot0Configs gainsConfig;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;

  protected final NeutralOut neutralOutput = new NeutralOut();
  protected final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);

  private final double mechanismReduction;

  public GenericRollersIOTalonFX(GenericRollersConfiguration rollersConfig) {
    talon = new TalonFX(rollersConfig.id);

    mechanismReduction = rollersConfig.reduction;

    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = rollersConfig.motorDirection;
    config.MotorOutput.NeutralMode = rollersConfig.neutralMode;
    config.CurrentLimits.SupplyCurrentLimit = rollersConfig.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = rollersConfig.statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    // Initialize follower motors
    followerMotors = new ArrayList<>();
    followerMotorSupplyCurrents = new ArrayList<>();
    for (GenericRollersConfiguration.FollowerMotorConfig followerConfig :
        rollersConfig.followerMotors) {
      TalonFX followerTalon = new TalonFX(followerConfig.id());
      followerTalon.setControl(
          new Follower(rollersConfig.id, followerConfig.motorAlignmentValue()));
      followerTalon.setNeutralMode(rollersConfig.neutralMode);
      followerTalon.getConfigurator().apply(config);
      followerTalon.optimizeBusUtilization();
      followerMotors.add(followerTalon);

      MotorOutputManager.getInstance()
          .registerMotorOutputs(() -> followerTalon.getSupplyCurrent().getValueAsDouble());
    }

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVolts = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    statorCurrent = talon.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, appliedVolts, supplyCurrent);

    if (followerMotors.size() == 0) {
      talon.optimizeBusUtilization();
    }

    MotorOutputManager.getInstance().registerMotorOutputs(() -> supplyCurrent.getValueAsDouble());
  }

  @Override
  public void updateInputs(GenericRollersIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent, statorCurrent)
            .isOK();
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(velocity.getValueAsDouble()) / mechanismReduction;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.positionRads =
        Units.rotationsToRadians(position.getValueAsDouble()) / mechanismReduction;
  }

  @Override
  public void runVelocity(double velocity) {
    talon.setControl(velocityControl.withVelocity(velocity));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOutput);
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
   */
  @Override
  public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
    gainsConfig = new Slot0Configs();
    gainsConfig.kP = kP;
    gainsConfig.kI = kI;
    gainsConfig.kD = kD;
    gainsConfig.kS = kS;
    gainsConfig.kV = kV;
    gainsConfig.kA = kA;

    talon.getConfigurator().apply(gainsConfig);
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
  public void setStatorCurrentLimit(double amps) {
    if (Math.abs(config.CurrentLimits.StatorCurrentLimit - amps) > 0.01) {
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = amps;
      config.withSlot0(gainsConfig);
      talon.getConfigurator().apply(config);
    }
  }
}
