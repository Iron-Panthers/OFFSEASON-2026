package frc.robot.utility;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Current limiting and load for WPILib physics sims, which have neither. The limit is applied to
 * the voltage so it reduces torque, not just the reported current.
 */
public final class SimCurrentLimit {

  /**
   * Stator current allowed relative to the supply limit.
   *
   * <p>Deliberately below the intake rollers' measured 5.46-6.02x: the honest 6.5 fit worse on two
   * of three matches, because the roller sims still slew too hard and this clips them. See
   * docs/sim-fidelity-results.md.
   */
  public static final double STATOR_TO_SUPPLY_RATIO = 4.0;

  private SimCurrentLimit() {}

  /**
   * Voltage lost to drag at the current speed; subtract it from the plant input.
   *
   * <p>{@code FlywheelSim} reports zero current at any steady state, so pair this with {@link
   * #statorAmps} evaluated on the commanded voltage. Together the steady-state current is {@code
   * dragAmpsPerRadPerSec * omega}.
   *
   * @param mechanismRadPerSec present mechanism velocity, signed
   * @param motor the plant's motor model, however many motors it represents
   * @param dragAmpsPerRadPerSec drag stator current per unit mechanism speed, referred to the same
   *     motor count as {@code motor}; zero disables
   * @return the voltage to subtract, with the same sign as the motion it opposes
   */
  public static double dragVolts(
      double mechanismRadPerSec, DCMotor motor, double dragAmpsPerRadPerSec) {
    if (dragAmpsPerRadPerSec <= 0.0 || mechanismRadPerSec == 0.0) {
      return 0.0;
    }
    double dragAmps = Math.abs(mechanismRadPerSec) * dragAmpsPerRadPerSec;
    return Math.signum(mechanismRadPerSec) * dragAmps * motor.rOhms;
  }

  /**
   * Applies a constant directional load, for a mechanism that holds position against one.
   *
   * @param appliedVolts the voltage the controller wants to apply
   * @param loadVolts voltage the load costs, signed in the direction the load pulls
   * @return the voltage the plant should actually see
   */
  public static double applyConstantLoad(double appliedVolts, double loadVolts) {
    return appliedVolts - loadVolts;
  }

  /**
   * Stator current, {@code (V - backEmf) / R}.
   *
   * <p>Use instead of {@code FlywheelSim.getCurrentDrawAmps()}, which is zero at steady state and
   * multiplies by {@code signum(V)}, turning braking current positive. Pass the commanded voltage,
   * not the drag-reduced one.
   *
   * @param appliedVolts commanded motor voltage
   * @param mechanismRadPerSec present mechanism velocity, signed
   * @param gearing reduction from mechanism to rotor (rotor = mechanism * gearing)
   * @param motor the plant's motor model; the result is that whole group's current
   */
  public static double statorAmps(
      double appliedVolts, double mechanismRadPerSec, double gearing, DCMotor motor) {
    double backEmf = (mechanismRadPerSec * gearing) / motor.KvRadPerSecPerVolt;
    return (appliedVolts - backEmf) / motor.rOhms;
  }

  /**
   * Largest voltage magnitude that keeps supply current within {@code limitAmps}.
   *
   * <p>With the controller as a buck converter, {@code I_supply = V * (V - backEmf) / (R * V_bus)};
   * this solves that quadratic for V.
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
    if (limitAmps <= 0.0 || busVolts <= 0.0) {
      return Math.max(-busVolts, Math.min(busVolts, appliedVolts));
    }

    // The stator window is centred on the SIGNED back-EMF. Centring it on zero lets a fast wheel
    // commanded into reverse draw hundreds of amps.
    double backEmf = (mechanismRadPerSec * gearing) / motor.KvRadPerSecPerVolt;
    double statorCeiling = limitAmps * STATOR_TO_SUPPLY_RATIO * motor.rOhms;
    double low = backEmf - statorCeiling;
    double high = backEmf + statorCeiling;

    double supplyCeiling = maxVoltageForSupplyLimit(backEmf, motor, busVolts, limitAmps);
    low = Math.max(low, -supplyCeiling);
    high = Math.min(high, supplyCeiling);

    low = Math.max(low, -busVolts);
    high = Math.min(high, busVolts);
    if (low > high) {
      // Back-EMF alone exceeds the bus (over-speed); the best we can do is the nearer rail.
      return Math.max(-busVolts, Math.min(busVolts, backEmf));
    }
    return Math.max(low, Math.min(high, appliedVolts));
  }
}
