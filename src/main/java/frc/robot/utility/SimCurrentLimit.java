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

  /**
   * Stator current a motor may draw relative to its configured supply limit.
   *
   * <p>Supply limits do not bound stator current directly -- the controller is a buck converter, so
   * stator exceeds supply by roughly 1/dutyCycle. Measured on the real q54 log: the intake rack
   * holds 85-102 A stator against a 27 A supply limit (~3.8x) and the omniwheel reaches 160 A
   * stator against 60 A supply (~2.7x). The largest measured ratio is the rack's 130.5 A / 27 A =
   * 4.83x, so this sits just above it: the clamp exists to stop unphysical blow-ups, not to
   * truncate real peaks. An earlier value of 4.0 capped the rack at 108 A and cut into its real
   * 130.5 A peaks, which made the currents fit score worse rather than better.
   */
  public static final double STATOR_TO_SUPPLY_RATIO = 5.0;

  private SimCurrentLimit() {}

  /**
   * Clamp a reported stator current to what the motor controller would allow.
   *
   * <p>Needed in addition to the voltage clamp because the WPILib sims evaluate current draw
   * <em>after</em> integrating, so a velocity discontinuity leaks into the reported current: when
   * the intake rack reaches its hard stop, {@code ElevatorSim} zeroes velocity inside {@code
   * update()} and then computes {@code (V - 0)/R} with a voltage that was legal for a moving motor.
   * That reported 206 A against a 27 A limit, purely as a discretization artifact.
   *
   * @param statorAmps the sim's reported stator current, signed
   * @param limitAmps configured supply limit; non-positive means unlimited
   */
  public static double clampStatorCurrent(double statorAmps, double limitAmps) {
    if (limitAmps <= 0.0 || !Double.isFinite(statorAmps)) {
      return statorAmps;
    }
    double cap = limitAmps * STATOR_TO_SUPPLY_RATIO;
    return Math.max(-cap, Math.min(cap, statorAmps));
  }

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
    if (limitAmps <= 0.0 || busVolts <= 0.0) {
      return Math.max(-busVolts, Math.min(busVolts, appliedVolts));
    }

    // Back-EMF is SIGNED. Clamping symmetrically about zero is wrong during braking: a wheel
    // spinning at +553 rad/s commanded to reverse has V negative while back-EMF is positive, so
    // |V - backEmf| is enormous and a symmetric clamp happily permits it. That is exactly how
    // the simulated omniwheel reported 747 A on a spin-down.
    //
    // The physical constraint is a window CENTRED ON THE BACK-EMF: stator current is
    // (V - backEmf)/R, so |V - backEmf| <= I_stator_max * R.
    double backEmf = (mechanismRadPerSec * gearing) / motor.KvRadPerSecPerVolt;
    double statorCeiling = limitAmps * STATOR_TO_SUPPLY_RATIO * motor.rOhms;
    double low = backEmf - statorCeiling;
    double high = backEmf + statorCeiling;

    // Motoring is additionally bounded by the supply limit, which is the tighter constraint
    // once the motor is up to speed.
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
