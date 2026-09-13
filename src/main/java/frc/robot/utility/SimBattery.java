package frc.robot.utility;

import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulated battery: {@code V = OCV(t) - I_total * R}, published to {@link RoboRioSim} so every
 * motor sim sees the sag. Parameters are fitted per match; see docs/sim-fidelity-results.md.
 */
public final class SimBattery {

  /** Absolute floor; a real pack under a dead short still holds some potential. */
  public static final double MIN_VOLTAGE = 4.0;

  /** roboRIO brownout threshold, as logged by the real robot. */
  public static final double BROWNOUT_VOLTAGE = 6.75;

  /** Open-circuit voltage at match start; middle of the fitted per-match range. */
  public static final double DEFAULT_NOMINAL_VOLTS = 12.24;

  /** Pack internal resistance; consistent at 10.5-12.0 mOhm across fitted matches. */
  public static final double DEFAULT_RESISTANCE_OHMS = 0.0112;

  /** Open-circuit voltage lost per enabled minute; varies with the battery, 0.35-1.02 fitted. */
  public static final double DEFAULT_DROOP_VOLTS_PER_MINUTE = 0.6;

  /** Phoenix supply limits are soft; real drive motors peak at 1.55x their limit. */
  public static final double LIMIT_OVERSHOOT_FACTOR = 1.5;

  private static SimBattery instance;

  private final List<DoubleSupplier> currentSources = new ArrayList<>();
  private final List<Double> sourceLimitsAmps = new ArrayList<>();
  private boolean pinnedAtFloor = false;
  private double nominalVolts = DEFAULT_NOMINAL_VOLTS;
  private double resistanceOhms = DEFAULT_RESISTANCE_OHMS;
  private double droopVoltsPerMinute = DEFAULT_DROOP_VOLTS_PER_MINUTE;
  private double enabledSeconds = 0.0;
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
   * Register a supply-current source with the supply limit its motor controller enforces.
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

  /** Open-circuit voltage right now, after discharge droop. */
  public double openCircuitVolts() {
    return nominalVolts - droopVoltsPerMinute * (enabledSeconds / 60.0);
  }

  /** Seconds the robot has been enabled, which is what drives the droop. */
  public double enabledSeconds() {
    return enabledSeconds;
  }

  /** Advance the discharge clock. */
  public void addEnabledTime(double seconds) {
    enabledSeconds += seconds;
  }

  /** Clear all sources and set new pack parameters. Intended for tests. */
  public void reset(double nominalVolts, double resistanceOhms) {
    reset(nominalVolts, resistanceOhms, DEFAULT_DROOP_VOLTS_PER_MINUTE);
  }

  /** Clear all sources and set new pack parameters including droop. Intended for tests. */
  public void reset(double nominalVolts, double resistanceOhms, double droopVoltsPerMinute) {
    this.droopVoltsPerMinute = droopVoltsPerMinute;
    this.enabledSeconds = 0.0;
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
   * Apply {@code -Preplay.battery}, {@code "<volts>:<ohms>[:<droopVoltsPerMinute>]"}. Malformed
   * input is ignored rather than aborting a long run.
   */
  public void configureFromProperty(String property) {
    if (property == null || property.isBlank()) {
      return;
    }
    String[] parts = property.split(":");
    if (parts.length < 2 || parts.length > 3) {
      System.err.println("[SimBattery] Ignoring malformed battery config: " + property);
      return;
    }
    try {
      nominalVolts = Double.parseDouble(parts[0]);
      resistanceOhms = Double.parseDouble(parts[1]);
      if (parts.length == 3) {
        droopVoltsPerMinute = Double.parseDouble(parts[2]);
      }
      lastVoltage = nominalVolts;
      System.out.println(
          "[SimBattery] nominal="
              + nominalVolts
              + "V, R="
              + resistanceOhms
              + " ohm, droop="
              + droopVoltsPerMinute
              + " V/min");
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
        continue;
      }
      double limit = sourceLimitsAmps.get(i);
      if (limit > 0.0) {
        double cap = limit * LIMIT_OVERSHOOT_FACTOR;
        amps = Math.max(-cap, Math.min(cap, amps));
      }
      // Signed, like the real TotalAmps, which goes negative under regen.
      total += amps;
    }
    lastCurrentAmps = total;

    double ocv = openCircuitVolts();

    // Regen never charges the real pack above open-circuit voltage.
    double raw = ocv - total * resistanceOhms;
    pinnedAtFloor = raw < MIN_VOLTAGE;
    lastVoltage = Math.max(MIN_VOLTAGE, Math.min(ocv, raw));
    brownedOut = lastVoltage < BROWNOUT_VOLTAGE;
    return lastVoltage;
  }

  /** Compute, publish and log the pack voltage. Call once per loop after the arena ticks. */
  public void update() {
    if (edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
      addEnabledTime(frc.robot.Constants.PERIODIC_LOOP_SEC);
    }

    double voltage = computeVoltage();

    // Set here rather than in getInstance() so tests can use this class without the HAL.
    if (!brownoutThresholdPublished) {
      RoboRioSim.setBrownoutVoltage(BROWNOUT_VOLTAGE);
      brownoutThresholdPublished = true;
    }

    RoboRioSim.setVInVoltage(voltage);
    Logger.recordOutput("SimBattery/Voltage", voltage);
    Logger.recordOutput("SimBattery/VoltagePinnedAtFloor", pinnedAtFloor);
    Logger.recordOutput("SimBattery/TotalCurrentAmps", lastCurrentAmps);
    Logger.recordOutput("SimBattery/BrownedOut", brownedOut);
    Logger.recordOutput("SimBattery/SourceCount", currentSources.size());
    Logger.recordOutput("SimBattery/OpenCircuitVolts", openCircuitVolts());
    Logger.recordOutput("SimBattery/EnabledSeconds", enabledSeconds);
  }
}
