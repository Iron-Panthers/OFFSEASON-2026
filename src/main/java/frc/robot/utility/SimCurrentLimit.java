package frc.robot.utility;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Applies a motor controller's supply current limit inside a WPILib physics sim.
 *
 * <p>WPILib's {@code FlywheelSim} / {@code ElevatorSim} have no notion of a current limit -- {@code
 * getCurrentDrawAmps()} is unbounded -- and the roller {@code *IOSim} classes drive those sims from
 * their own PID without ever consulting the TalonFX, so the {@code SupplyCurrentLimit} configured
 * on the Talon never reaches the physics. Unclamped, the simulated shooter omniwheel reported 952 A
 * against a real robot that peaks near 60 A.
 *
 * <p>Limiting the reported current alone would be bookkeeping. A real current-limited motor also
 * makes less torque, so the limit is applied to the applied voltage, which then feeds through to
 * acceleration, velocity and current together.
 */
public final class SimCurrentLimit {

  private SimCurrentLimit() {}

  /**
   * Largest voltage magnitude that keeps supply current within {@code limitAmps}.
   *
   * <p>For a brushed-DC model with the motor controller acting as a buck converter:
   *
   * <pre>
   *   I_stator = (V - backEmf) / R
   *   I_supply = I_stator * dutyCycle,  dutyCycle = V / V_bus
   *   =&gt; I_supply = V * (V - backEmf) / (R * V_bus)
   * </pre>
   *
   * Setting {@code I_supply = limit} and solving the quadratic for V gives the result below. This
   * is exact rather than iterative, so it costs nothing per loop.
   *
   * @param backEmfVolts back-EMF at the current rotor speed, always passed as a magnitude
   * @param motor the motor model supplying winding resistance
   * @param busVolts present bus voltage
   * @param limitAmps configured supply current limit; non-positive means unlimited
   * @return the voltage magnitude ceiling, or {@code busVolts} when unlimited
   */
  public static double maxVoltageForSupplyLimit(
      double backEmfVolts, DCMotor motor, double busVolts, double limitAmps) {
    if (limitAmps <= 0.0 || busVolts <= 0.0) {
      return busVolts;
    }
    double emf = Math.abs(backEmfVolts);
    double discriminant = emf * emf + 4.0 * limitAmps * motor.rOhms * busVolts;
    double v = 0.5 * (emf + Math.sqrt(Math.max(0.0, discriminant)));
    return Math.min(busVolts, v);
  }

  /**
   * Clamp {@code appliedVolts} so the resulting supply current respects {@code limitAmps}.
   *
   * @param appliedVolts the voltage the controller wants to apply
   * @param mechanismRadPerSec present mechanism velocity
   * @param gearing reduction from mechanism to rotor (rotor = mechanism * gearing)
   * @param motor the motor model
   * @param busVolts present bus voltage
   * @param limitAmps configured supply current limit; non-positive means unlimited
   */
  public static double clampToSupplyLimit(
      double appliedVolts,
      double mechanismRadPerSec,
      double gearing,
      DCMotor motor,
      double busVolts,
      double limitAmps) {
    double backEmf = (mechanismRadPerSec * gearing) / motor.KvRadPerSecPerVolt;
    double ceiling = maxVoltageForSupplyLimit(backEmf, motor, busVolts, limitAmps);
    return Math.max(-ceiling, Math.min(ceiling, appliedVolts));
  }
}
