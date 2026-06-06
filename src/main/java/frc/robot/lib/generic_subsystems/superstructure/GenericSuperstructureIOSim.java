package frc.robot.lib.generic_subsystems.superstructure;

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

public abstract class GenericSuperstructureIOSim implements GenericSuperstructureIO {

  protected final TalonFX talon;

  protected final TalonFXConfiguration config;
  protected Slot0Configs gainsConfig = new Slot0Configs();

  protected final VoltageOut voltageOutput = new VoltageOut(0).withUpdateFreqHz(0);

  protected final NeutralOut neutralOutput = new NeutralOut();

  protected final DynamicMotionMagicVoltage positionControl =
      new DynamicMotionMagicVoltage(0, 0, 0).withUpdateFreqHz(0);

  public GenericSuperstructureIOSim(int id) {

    talon = new TalonFX(id);
    talon.setNeutralMode(NeutralModeValue.Brake);
    config =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
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
