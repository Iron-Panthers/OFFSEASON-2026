package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.TargetShootingState;
import frc.robot.lib.generic_subsystems.rollers.GenericRollers.ControlMode;
import frc.robot.lib.generic_subsystems.superstructure.*;
import frc.robot.subsystems.shooter.serializer.Serializer;
import frc.robot.subsystems.shooter.serializer.Serializer.SerializerTarget;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator;
import frc.robot.subsystems.shooter.shooter_accelerator.ShooterAccelerator.ShooterAcceleratorTarget;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel.ShooterFlywheelTarget;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood.ShooterHoodTarget;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheel;
import frc.robot.subsystems.shooter.shooter_omniwheel.ShooterOmniwheel.ShooterOmniwheelTarget;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterController extends SubsystemBase {
  public enum ShooterState {
    // TO-DO: update states
    /** idle: no spin */
    IDLE(
        ShooterHoodTarget.STOW,
        ShooterFlywheelTarget.IDLE,
        ShooterAcceleratorTarget.IDLE,
        ShooterOmniwheelTarget.IDLE,
        SerializerTarget.IDLE),
    REVERSE(
        ShooterHoodTarget.STOW,
        ShooterFlywheelTarget.INTAKE,
        ShooterAcceleratorTarget.IDLE,
        ShooterOmniwheelTarget.IDLE,
        SerializerTarget.REVERSE),
    /** spin just flywheels */
    FLYWHEEL_SPIN_UP(
        ShooterHoodTarget.STOW,
        ShooterFlywheelTarget.INTAKE,
        ShooterAcceleratorTarget.IDLE,
        ShooterOmniwheelTarget.SLOW_REVERSE,
        SerializerTarget.IDLE),
    /** hold: hold the balls in the hopper */
    HOLD(
        ShooterHoodTarget.STOW,
        ShooterFlywheelTarget.IDLE,
        ShooterAcceleratorTarget.IDLE,
        ShooterOmniwheelTarget.IDLE,
        SerializerTarget.HOLD),
    /** idle: no spin */
    INTAKE(
        ShooterHoodTarget.STOW,
        ShooterFlywheelTarget.INTAKE,
        ShooterAcceleratorTarget.IDLE,
        ShooterOmniwheelTarget.IDLE,
        SerializerTarget.SLOW),
    /** shoot: spinning to shoot */
    SHOOT(
        ShooterHoodTarget.SHOOT_TEMP,
        ShooterFlywheelTarget.SHOOT,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelTarget.SHOOT,
        SerializerTarget.SHOOT),
    /** default_shoot: default shooting position */
    DEFAULT_SHOOT(
        ShooterHoodTarget.DEFAULT_SHOOT,
        ShooterFlywheelTarget.SHOOT,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelTarget.SHOOT,
        SerializerTarget.SHOOT),
    TRENCH_SHOOT(
        ShooterHoodTarget.DEFAULT_SHOOT,
        ShooterFlywheelTarget.SHOOT,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelTarget.SHOOT,
        SerializerTarget.SHOOT),
    TOTAL_SPIN_UP(
        ShooterHoodTarget.SHOOT_TEMP,
        ShooterFlywheelTarget.SHOOT,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelTarget.IDLE,
        SerializerTarget.SPIN_UP),
    COMPACT_SPIN_UP(
        ShooterHoodTarget.STOW,
        ShooterFlywheelTarget.SHOOT,
        ShooterAcceleratorTarget.SHOOT,
        ShooterOmniwheelTarget.IDLE,
        SerializerTarget.SPIN_UP),
    ZEROING(
        ShooterHoodTarget.STOW,
        ShooterFlywheelTarget.IDLE,
        ShooterAcceleratorTarget.IDLE,
        ShooterOmniwheelTarget.IDLE,
        SerializerTarget.IDLE),
    PASS(
        ShooterHoodTarget.PASS,
        ShooterFlywheelTarget.PASS,
        ShooterAcceleratorTarget.PASS,
        ShooterOmniwheelTarget.SHOOT,
        SerializerTarget.SHOOT),
    PASS_SPIN_UP(
        ShooterHoodTarget.PASS,
        ShooterFlywheelTarget.PASS,
        ShooterAcceleratorTarget.PASS,
        ShooterOmniwheelTarget.IDLE,
        SerializerTarget.IDLE);

    public final ShooterHoodTarget hoodTarget;
    public final ShooterFlywheelTarget flywheelTarget;
    public final ShooterAcceleratorTarget acceleratorTarget;
    public final ShooterOmniwheelTarget omniwheelTarget;
    public final SerializerTarget serializerTarget;

    private ShooterState(
        ShooterHoodTarget hoodTarget,
        ShooterFlywheelTarget flywheelTarget,
        ShooterAcceleratorTarget acceleratorTarget,
        ShooterOmniwheelTarget omniwheelTarget,
        SerializerTarget serializerTarget) {
      this.hoodTarget = hoodTarget;
      this.flywheelTarget = flywheelTarget;
      this.acceleratorTarget = acceleratorTarget;
      this.omniwheelTarget = omniwheelTarget;
      this.serializerTarget = serializerTarget;
    }

    public SerializerTarget getSerializerTarget() {
      return serializerTarget;
    }
  }

  private ShooterState targetState = ShooterState.IDLE;
  private boolean stopped = false;
  private boolean autoAim = false;

  // might need sensors defined here and in constructor
  private final ShooterFlywheel shooterFlywheel;
  private final ShooterHood shooterHood;
  private final ShooterOmniwheel shooterOmniwheel;
  private final ShooterAccelerator shooterAccelerator;
  private final Serializer serializer;

  public LoggedNetworkNumber shooterTemp = new LoggedNetworkNumber("Tuning/ShooterStateTemp", 11);

  public ShooterController(
      ShooterFlywheel shooterFlywheel,
      ShooterHood shooterHood,
      ShooterOmniwheel shooterOmniwheel,
      ShooterAccelerator shooterAccelerator,
      Serializer serializer) {
    this.shooterFlywheel = shooterFlywheel;
    this.shooterHood = shooterHood;
    this.shooterOmniwheel = shooterOmniwheel;
    this.shooterAccelerator = shooterAccelerator;
    this.serializer = serializer;
  }

  @Override
  public void periodic() {
    if (stopped) {
      shooterHood.setControlMode(GenericSuperstructure.ControlMode.STOP);
      shooterFlywheel.setControlMode(ControlMode.STOP);
      shooterOmniwheel.setControlMode(ControlMode.STOP);
      shooterAccelerator.setControlMode(ControlMode.STOP);
      serializer.setControlMode(ControlMode.STOP);
    } else if (shooterHood.getControlMode() == GenericSuperstructure.ControlMode.ZEROING) {
      shooterFlywheel.setVelocityTarget(targetState.flywheelTarget);
      shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
      shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
      serializer.setVelocityTarget(targetState.serializerTarget);
      // TODO:should we set the state of serializer to target?
    } else if ((targetState == ShooterState.SHOOT
        || targetState == ShooterState.TOTAL_SPIN_UP
        || targetState == ShooterState.DEFAULT_SHOOT
        || targetState == ShooterState.TRENCH_SHOOT)) {

      TargetShootingState shotState = RobotState.getInstance().calculateTargetShootingState();

      // If shooting, update the hood target based on the calculated shooter angle
      // Hood
      if (targetState == ShooterState.DEFAULT_SHOOT) {
        shooterHood.setPositionTarget(targetState.hoodTarget);
      } else if (targetState == ShooterState.TRENCH_SHOOT) {
        shooterHood.setPositionTargetManual(
            Units.Degrees.of(
                    90 - RobotState.getInstance().getStationaryHoodParams(3.4).shooterAngle())
                .in(Units.Rotation),
            targetState.hoodTarget.getSupplyCurrentLimit());
      } else {
        shooterHood.setPositionTargetManual(
            Units.Rotations.of(.25).minus(shotState.shooterAngle()).in(Units.Rotations),
            targetState.hoodTarget.getSupplyCurrentLimit());
      }

      // Flywheels
      if (targetState == ShooterState.DEFAULT_SHOOT) {
        shooterFlywheel.setVelocityTarget(ShooterFlywheelTarget.SHOOT);
      } else {
        shooterFlywheel.setVelocityManual(
            shotState.shooterSpeed(), targetState.flywheelTarget.getSupplyCurrentLimit());
      }

      // Omniwheels
      if (targetState == ShooterState.SHOOT) {
        if (shooterFlywheel.reachedVelocityTarget() && shooterHood.reachedTarget()) {
          shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
        } else {
          shooterOmniwheel.setVelocityTarget(ShooterOmniwheelTarget.IDLE);
        }
      } else {
        shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
      }

      // Acclerator
      if (shooterOmniwheel.getCurrentVelocity().in(Units.RadiansPerSecond) < 350) {
        shooterAccelerator.setVelocityTarget(ShooterAcceleratorTarget.WARMUP_ACCELERATOR);
      } else {
        shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
      }

      // Serializer
      if (targetState == ShooterState.SHOOT) {
        if (shooterFlywheel.reachedVelocityTarget()) {
          serializer.setVelocityTarget(targetState.serializerTarget);
        } else {
          serializer.setVelocityTarget(SerializerTarget.IDLE);
        }
      } else {
        serializer.setVelocityTarget(targetState.serializerTarget);
      }
    } else {
      if (targetState.flywheelTarget == ShooterFlywheelTarget.INTAKE) {
        TargetShootingState shotState = RobotState.getInstance().calculateTargetShootingState();
        shooterFlywheel.setVelocityManual(
            Units.MetersPerSecond.of(
                Math.min(shotState.shooterSpeed().in(Units.MetersPerSecond), 9.5)),
            targetState.flywheelTarget.getSupplyCurrentLimit());
      } else {
        shooterFlywheel.setVelocityTarget(targetState.flywheelTarget);
      }
      shooterHood.setPositionTarget(targetState.hoodTarget);
      shooterOmniwheel.setVelocityTarget(targetState.omniwheelTarget);
      shooterAccelerator.setVelocityTarget(targetState.acceleratorTarget);
      serializer.setVelocityTarget(targetState.serializerTarget);
    }
    shooterFlywheel.periodic();
    shooterHood.periodic();
    shooterOmniwheel.periodic();
    shooterAccelerator.periodic();
    serializer.periodic();

    Logger.recordOutput("Shooter/Target State", targetState);
    Logger.recordOutput("Shooter/Is Stopped", stopped);
    Logger.recordOutput("Shooter/Auto Aim", autoAim);
  }

  public ShooterState getTargetState() {
    return targetState;
  }

  public void setTargetState(ShooterState targetState) {
    setStopped(false);
    this.targetState = targetState;
  }

  public Command setTargetStateCommand(ShooterState target) {
    return new InstantCommand(() -> setTargetState(target), this);
  }

  public void setStopped(boolean stopped) {
    this.stopped = stopped;
  }

  public Command setStoppedCommand(boolean stopped) {
    return new InstantCommand(() -> setStopped(stopped));
  }

  public Command zeroCommand() {
    return new InstantCommand(
            () -> shooterHood.setControlMode(GenericSuperstructure.ControlMode.ZEROING))
        .alongWith(setTargetStateCommand(ShooterState.ZEROING).alongWith(setStoppedCommand(false)));
  }

  public LinearVelocity getCurrentVelocity() {
    return shooterFlywheel.getCurrentVelocity();
  }

  public void setAutoAim(boolean autoAim) {
    this.autoAim = autoAim;
  }

  public Command setAutoAimCommand(boolean autoAim) {
    return new InstantCommand(() -> setAutoAim(autoAim));
  }

  public Command stopZeroingCommand() {
    return new InstantCommand(() -> shooterHood.endZeroing());
  }

  @AutoLogOutput(key = "Shooter/Flywheels Up To Speed")
  public boolean flywheelsUpToSpeed() {
    return shooterFlywheel.reachedVelocityTarget();
  }
}
