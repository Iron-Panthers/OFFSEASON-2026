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
   * <p><b>Deliberately set BELOW the rollers' measured ratio, as a compensating approximation.</b>
   * Physically this should sit above every value in the table so it never binds.
   *
   * <p>Retested at 6.5 after the mechanism load model landed, because the earlier note said to
   * revisit once the plant had a real load. The load model shrank the penalty by roughly an order
   * of magnitude but did not remove it -- currents scored q54 0.1966 -> 0.1951 (better), q93 0.1836
   * -> 0.2027 (worse), q14 0.1561 -> 0.1603 (worse). Two of three worse, so 4.0 stands. For
   * reference, before the load model the same experiment measured q54 0.2387 -> 0.2536 and q93
   * 0.2102 -> 0.2319.
   *
   * <p>What remains is that the roller sims still overshoot on transients -- the load model
   * corrects their steady-state draw, not their slew. The clearest case is the omniwheel, which
   * produces 110 A mean stator on its own spin-ups against a real 41 A. A tighter-than-physical
   * ceiling clips that and happens to fit better. It is still a compensating error.
   *
   * <p><b>Revisit again once the mechanisms slew correctly</b>, not merely once they have a load. A
   * per-mechanism ceiling would be better than any single number -- the drive ratio is 3.75 in
   * every match to two decimals, while the rollers swing 5.46-6.02.
   */
  public static final double STATOR_TO_SUPPLY_RATIO = 4.0;

  private SimCurrentLimit() {}

  /**
   * Voltage that must be spent overcoming steady-state drag at the current speed.
   *
   * <p>WPILib's {@code FlywheelSim} is frictionless, and worse than that, its reported current is
   * <em>structurally</em> zero at steady state. {@code FlywheelSim} derives its gearing back out of
   * the plant matrices ({@code G = -Kv*A/B}) and then reports {@code (V - omega*G/Kv)/R}. Any
   * linear plant it is handed settles where {@code A*omega + B*V = 0}, which is exactly where
   * {@code omega*G = Kv*V}, so the two terms cancel. That holds for {@code createFlywheelSystem}
   * and equally for {@code identifyVelocitySystem(kV, kA)} -- characterising the plant from real
   * logs cannot fix it, because the current is not derived from the plant's physics but from an
   * inverse of the plant itself.
   *
   * <p>So the real robot pulling 3-4 A per motor to hold the shooter flywheel at speed, or 23 A to
   * hold the serializer, has no representation at all: measured over q54, mean supply current was
   * 0.30 A simulated against 9.62 A real for the flywheel, 0.00 vs 9.47 for the serializer, 0.08 vs
   * 8.02 for the intake rollers. Summed over every mechanism and every motor that is roughly 74 A,
   * which is essentially the whole of the pack-current gap.
   *
   * <p>The fix has two halves and only works with both:
   *
   * <ol>
   *   <li>Subtract this voltage from the plant input, so the mechanism must genuinely work to hold
   *       its setpoint and coasts down at the right rate.
   *   <li>Report stator current from {@link #statorAmps}, evaluated against the <em>commanded</em>
   *       voltage rather than the reduced one, instead of from {@code getCurrentDrawAmps()}.
   * </ol>
   *
   * <p>An earlier attempt did only the first half and moved mean pack current 113.84 -> 113.50 A,
   * because the sim kept reporting its structural zero. The two together make the steady-state
   * current come out at exactly {@code dragAmpsPerRadPerSec * omega}, which is what was measured.
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
   * Applies a constant load: the part of the command that is spent holding, not moving.
   *
   * <p>The viscous model in {@link #dragVolts} is useless for a mechanism that spends most of the
   * match stationary. The intake rack is deployed and stopped for 66-75% of every match logged, and
   * while stopped the real robot holds it with a consistently POSITIVE 0.28-0.78 V against zero
   * back-EMF -- 9-28 A of stator current doing no work at all. {@code ElevatorSim} is frictionless,
   * so the simulated rack reached its target, needed nothing to stay there, and drew 0.14 A mean
   * against a real 5.67 A.
   *
   * <p>Modelled as a constant force rather than as friction because the sign says so. Friction
   * opposes whichever way the mechanism is pushed, so a controller holding against it settles into
   * a symmetric dither and its mean current is zero -- measured at -0.01 A when that was tried. The
   * real holding voltage never changes sign, which is a load pulling the rack back toward stow, and
   * the motor fighting it.
   *
   * @param appliedVolts the voltage the controller wants to apply
   * @param loadVolts voltage the load costs, signed in the direction the load pulls
   * @return the voltage the plant should actually see
   */
  public static double applyConstantLoad(double appliedVolts, double loadVolts) {
    return appliedVolts - loadVolts;
  }

  /**
   * Stator current a motor draws applying {@code appliedVolts} while spinning at the given speed.
   *
   * <p>{@code I = (V - backEmf) / R}, the textbook relation, evaluated directly instead of through
   * {@code FlywheelSim.getCurrentDrawAmps()}. Two reasons to bypass the sim:
   *
   * <ul>
   *   <li>Its answer is zero at every steady state -- see {@link #dragVolts}.
   *   <li>It multiplies by {@code signum(u)}, which flips the sign of a braking current instead of
   *       reporting it as regen. A wheel spinning forwards while commanded backwards has {@code V -
   *       backEmf} large and negative; the sign flip booked that as a large positive draw, which is
   *       how the simulated omniwheel reported 747 A on a spin-down.
   * </ul>
   *
   * <p>Pass the voltage the controller <em>commanded</em>, not the value handed to the plant after
   * {@link #dragVolts} was subtracted. The difference between the two is precisely the drag, and it
   * is what makes a mechanism holding its setpoint draw current rather than nothing.
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
