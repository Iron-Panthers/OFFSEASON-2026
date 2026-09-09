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
   * stator exceeds supply by roughly 1/dutyCycle. Peak stator current divided by the configured
   * supply limit, measured across three real matches:
   *
   * <table>
   * <tr><th>mechanism</th><th>q54</th><th>q93</th><th>q14</th></tr>
   * <tr><td>swerve drive</td><td>3.75</td><td>3.75</td><td>3.74</td></tr>
   * <tr><td>shooter omniwheel</td><td>2.66</td><td>3.01</td><td>2.77</td></tr>
   * <tr><td>intake rack</td><td>4.83</td><td>4.59</td><td>4.15</td></tr>
   * <tr><td>intake rollers</td><td>5.46</td><td>6.02</td><td>5.68</td></tr>
   * </table>
   *
   * <p>Set above the largest of those. This is a guard against unphysical blow-ups, not a
   * behavioural limit -- it must never bind during normal operation. An earlier value of 4.0 was
   * derived from q54 alone and sat BELOW the rollers' real ratio in all three matches, clipping
   * genuine torque. The drive ratio is 3.75 in every match to two decimals; the rollers are not
   * constant, so a per-mechanism ceiling would be the principled fix if this ever needs to bind.
   */
  public static final double STATOR_TO_SUPPLY_RATIO = 6.5;

  private SimCurrentLimit() {}

  /**
   * Voltage that must be spent overcoming steady-state drag at the current speed.
   *
   * <p>WPILib's {@code FlywheelSim} is frictionless, so a mechanism holding its setpoint draws
   * essentially no current, while the real robot keeps pulling 7-9 A against bearing, belt and
   * game-piece drag. That is why the simulation under-draws on average (114 A vs a real 148.8 A)
   * even when its peaks are right, and why the flywheel's filtered current shows a negative mean
   * shift alongside a positive peak shift.
   *
   * <p><b>Currently unused, and the approach is wrong as written.</b> Subtracting a voltage does
   * not create a load: {@code FlywheelSim} has no opposing torque, so the plant simply settles at a
   * slightly lower speed with its current still near zero. Measured against q54 it moved mean pack
   * current only 113.84 -> 113.50 A (real: 148.8 A) while making the currents fit score worse
   * (0.3018 -> 0.3159).
   *
   * <p>Closing the steady-state gap needs a change to the PLANT, not to the command: either an
   * explicit load torque, or replacing {@code LinearSystemId.createFlywheelSystem} with {@code
   * identifyVelocitySystem(kV, kA)} characterised from the real logs, so the terminal speed for a
   * given voltage is right and the motor must genuinely work to hold setpoint. Retained as a
   * starting point for that work.
   *
   * @param mechanismRadPerSec present mechanism velocity, signed
   * @param gearing reduction from mechanism to rotor
   * @param motor the motor model
   * @param dragAmpsPerRadPerSec drag current per unit mechanism speed; zero disables
   * @return the voltage to subtract, with the same sign as the motion it opposes
   */
  public static double dragVolts(
      double mechanismRadPerSec, double gearing, DCMotor motor, double dragAmpsPerRadPerSec) {
    if (dragAmpsPerRadPerSec <= 0.0 || mechanismRadPerSec == 0.0) {
      return 0.0;
    }
    double dragAmps = Math.abs(mechanismRadPerSec) * dragAmpsPerRadPerSec;
    return Math.signum(mechanismRadPerSec) * dragAmps * motor.rOhms;
  }

  /**
   * Clamp a reported stator current to what the motor controller would allow.
   *
   * <p>Needed in addition to the voltage clamp because the WPILib sims evaluate current draw
   * <em>after</em> integrating, so a velocity discontinuity leaks into the reported current: when
   * the intake rack reaches its hard stop, {@code ElevatorSim} zeroes velocity inside {@code
   * update()} and then computes {@code (V - 0)/R} with a voltage that was legal for a moving motor.
   * That reported 206 A against a 27 A limit, purely as a discretization artifact.
   *
   * <p><b>Currently unused.</b> Applying it measured WORSE against the real q54 log than leaving
   * the reported current alone -- currents 0.3143 at a 4.0 ratio and 0.3221 at 5.0, against 0.3018
   * without it. A single global ratio is too blunt: it clips mechanisms whose real stator peaks
   * exceed it while doing little for the ones it was aimed at. Retained because the underlying
   * artifact is real and a per-mechanism ceiling measured from the logs would likely help; do not
   * re-enable it globally without re-measuring.
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
