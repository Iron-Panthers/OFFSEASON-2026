package frc.robot.utility;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulated battery with load-dependent voltage sag.
 *
 * <p>Before this existed the simulation ran at a fixed 12 V, so brownouts were impossible and
 * motors never lost torque under load. Real match logs show the pack dropping to 5.95 V, and in the
 * one match where it stayed low the flywheel measurably failed to reach speed — so the sag has to
 * feed back into motor behaviour, not merely be reported.
 *
 * <p>That feedback comes for free: every {@code *IOSim} already calls {@code
 * setSupplyVoltage(RobotController.getBatteryVoltage())}, and {@link RoboRioSim#setVInVoltage} is
 * what that reads. Publishing the sagged voltage here reaches every motor on the robot.
 *
 * <p>Model: {@code V = nominal - I_total * R_internal}. Nominal voltage and internal resistance are
 * per-match tunable because battery condition varied between matches.
 */
public final class SimBattery {

  /** Absolute floor; a real pack under a dead short still holds some potential. */
  public static final double MIN_VOLTAGE = 4.0;

  /** roboRIO brownout threshold. */
  /** Real q54 logs SystemStats/BrownoutVoltage = 6.75. */
  public static final double BROWNOUT_VOLTAGE = 6.75;

  public static final double DEFAULT_NOMINAL_VOLTS = 12.8;
  public static final double DEFAULT_RESISTANCE_OHMS = 0.02;

  /**
   * How far a source may transiently exceed its configured supply limit.
   *
   * <p>Phoenix supply limits are soft -- {@code SupplyCurrentLowerLimit}/{@code
   * SupplyCurrentLowerTime} default to 40 A / 1.0 s -- so the real robot overshoots too. Measured
   * on q54: real drive motors peak at 1.55x their 40 A limit.
   */
  public static final double LIMIT_OVERSHOOT_FACTOR = 1.5;

  private static SimBattery instance;

  private final List<DoubleSupplier> currentSources = new ArrayList<>();
  private final List<Double> sourceLimitsAmps = new ArrayList<>();
  private boolean pinnedAtFloor = false;
  private double nominalVolts = DEFAULT_NOMINAL_VOLTS;
  private double resistanceOhms = DEFAULT_RESISTANCE_OHMS;
  private double lastVoltage = DEFAULT_NOMINAL_VOLTS;
  private double lastCurrentAmps = 0.0;
  private boolean brownedOut = false;
  private boolean brownoutThresholdPublished = false;

  private SimBattery() {}

  public static synchronized SimBattery getInstance() {
    if (instance == null) {
      instance = new SimBattery();
    }
    return instance;
  }

  /**
   * Register a supply-current source, in amps, with the supply limit its motor controller enforces.
   *
   * <p>The limit is applied here rather than trusted from the physics sims: WPILib's {@code
   * FlywheelSim}/{@code ElevatorSim} {@code getCurrentDrawAmps()} is unbounded, and the roller
   * IOSims drive those sims from their own PID without ever consulting the Talon, so the configured
   * {@code SupplyCurrentLimit} never reaches the physics. Unclamped, the shooter alone reported 952
   * A on a single mechanism.
   *
   * @param supplyCurrentAmps signed supply current; negative means regenerating
   * @param limitAmps the configured supply limit, or a non-positive value for no limit
   */
  public void register(DoubleSupplier supplyCurrentAmps, double limitAmps) {
    currentSources.add(supplyCurrentAmps);
    sourceLimitsAmps.add(limitAmps);
  }

  /** Register an unlimited source. Prefer the two-argument form. */
  public void register(DoubleSupplier supplyCurrentAmps) {
    register(supplyCurrentAmps, 0.0);
  }

  /** Number of registered current sources, for verifying nothing was missed. */
  public int sourceCount() {
    return currentSources.size();
  }

  /** Clear all sources and set new pack parameters. Intended for tests. */
  public void reset(double nominalVolts, double resistanceOhms) {
    currentSources.clear();
    sourceLimitsAmps.clear();
    this.pinnedAtFloor = false;
    this.brownoutThresholdPublished = false;
    this.nominalVolts = nominalVolts;
    this.resistanceOhms = resistanceOhms;
    this.lastVoltage = nominalVolts;
    this.lastCurrentAmps = 0.0;
    this.brownedOut = false;
  }

  /**
   * Apply a {@code "<nominalVolts>:<resistanceOhms>"} configuration string, as supplied by {@code
   * -Preplay.battery}. Malformed input is ignored so a typo degrades to defaults rather than
   * crashing a long simulation run.
   */
  public void configureFromProperty(String property) {
    if (property == null || property.isBlank()) {
      return;
    }
    String[] parts = property.split(":");
    if (parts.length != 2) {
      System.err.println("[SimBattery] Ignoring malformed battery config: " + property);
      return;
    }
    try {
      nominalVolts = Double.parseDouble(parts[0]);
      resistanceOhms = Double.parseDouble(parts[1]);
      lastVoltage = nominalVolts;
      System.out.println(
          "[SimBattery] nominal=" + nominalVolts + "V, R=" + resistanceOhms + " ohm");
    } catch (NumberFormatException e) {
      System.err.println("[SimBattery] Ignoring malformed battery config: " + property);
    }
  }

  public double lastVoltage() {
    return lastVoltage;
  }

  public boolean isBrownedOut() {
    return brownedOut;
  }

  /** Sum the registered sources and return the resulting pack voltage. */
  public double computeVoltage() {
    double total = 0.0;
    for (int i = 0; i < currentSources.size(); i++) {
      double amps = currentSources.get(i).getAsDouble();
      if (!Double.isFinite(amps)) {
        // A diverging physics sim can emit NaN; letting it through would poison the pack
        // voltage for every motor and silently wreck the whole run.
        continue;
      }
      double limit = sourceLimitsAmps.get(i);
      if (limit > 0.0) {
        double cap = limit * LIMIT_OVERSHOOT_FACTOR;
        amps = Math.max(-cap, Math.min(cap, amps));
      }
      // Sum SIGNED current, matching the real robot's MotorOutputManager/TotalAmps, which
      // reaches -101.7 A under regenerative braking. Discarding negatives per source made the
      // two series different measurements and biased the sim mean by +7%.
      total += amps;
    }
    lastCurrentAmps = total;

    // Clamp the VOLTAGE rather than each current. Regen on the real robot never charges the
    // pack -- at its most negative (-101.7 A) the real pack still read 9.78 V -- so the model
    // must not rise above open-circuit voltage even though the signed sum goes negative.
    double raw = nominalVolts - total * resistanceOhms;
    pinnedAtFloor = raw < MIN_VOLTAGE;
    lastVoltage = Math.max(MIN_VOLTAGE, Math.min(nominalVolts, raw));
    brownedOut = lastVoltage < BROWNOUT_VOLTAGE;
    return lastVoltage;
  }

  /**
   * Compute the pack voltage, publish it to the simulated roboRIO so every motor sees it, and log
   * it. Call once per simulation loop.
   */
  public void update() {
    double voltage = computeVoltage();

    // Published once rather than every loop, and from here rather than getInstance() so that
    // computeVoltage()/register() stay callable without an initialised HAL -- unit tests
    // exercise this class directly.
    if (!brownoutThresholdPublished) {
      RoboRioSim.setBrownoutVoltage(BROWNOUT_VOLTAGE);
      brownoutThresholdPublished = true;
    }

    RoboRioSim.setVInVoltage(voltage);
    Logger.recordOutput("SimBattery/Voltage", voltage);
    // Surfaced so a broken current model is visible instead of being reported as a plausible
    // mean: the floor silently pinned 87 samples in the baseline run.
    Logger.recordOutput("SimBattery/VoltagePinnedAtFloor", pinnedAtFloor);
    Logger.recordOutput("SimBattery/TotalCurrentAmps", lastCurrentAmps);
    Logger.recordOutput("SimBattery/BrownedOut", brownedOut);
    Logger.recordOutput("SimBattery/SourceCount", currentSources.size());
  }
}
