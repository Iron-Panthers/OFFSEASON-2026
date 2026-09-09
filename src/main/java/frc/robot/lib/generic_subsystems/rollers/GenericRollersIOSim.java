package frc.robot.lib.generic_subsystems.rollers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.MotorOutputManager;

public abstract class GenericRollersIOSim implements GenericRollersIO {
  protected final TalonFX talon;

  private final NeutralOut neutralOutput = new NeutralOut();

  /**
   * True while the subsystem has commanded a stop.
   *
   * <p>{@link #stop()} only put the Talon into NeutralOut, but every concrete IOSim drives its
   * physics from its own PID and never reads the Talon, so neutral never reached the simulation:
   * the accelerator kept spinning at 322 rad/s for 130.5 s of a 165 s match while commanded to
   * stop. Subclasses check this and apply 0 V.
   */
  protected boolean coasting = false;

  /**
   * Last supply current reported by the subclass, in amps.
   *
   * <p>Subclasses assign this at the end of {@code updateInputs} so the aggregate power bookkeeping
   * below sees the same number the log does.
   */
  protected double reportedSupplyCurrentAmps = 0.0;

  private final double mechanismReduction;
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);

  public GenericRollersIOSim(
      int id, int currentLimitAmps, boolean inverted, boolean brake, double reduction) {
    talon = new TalonFX(id);

    mechanismReduction = reduction;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    talon.getConfigurator().apply(config);

    talon.optimizeBusUtilization();

    // GenericRollersIOTalonFX registers every motor with MotorOutputManager; the sim class did
    // not, and it does not extend the TalonFX class, so in simulation TotalAmps counted the
    // swerve modules ONLY (those inherit registration through ModuleIOTalonFX). All seven
    // mechanisms were missing, which is why sim mean pack draw sat at 89.6 A against a real
    // 158.3 A and why the aggregate fit category never improved.
    MotorOutputManager.getInstance().registerMotorOutputs(() -> reportedSupplyCurrentAmps);
  }

  @Override
  public abstract void updateInputs(GenericRollersIOInputs inputs);

  @Override
  public void runVelocity(double velocity) {
    coasting = false;
    talon.setControl(velocityControl.withVelocity(velocity));
  }

  @Override
  public void stop() {
    coasting = true;
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
   * @param gravityTypeValue Gravity compensation type
   */
  @Override
  public void setSlot0(double kP, double kI, double kD, double kS, double kV, double kA) {
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
