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
  public static final double BROWNOUT_VOLTAGE = 6.8;

  public static final double DEFAULT_NOMINAL_VOLTS = 12.8;
  public static final double DEFAULT_RESISTANCE_OHMS = 0.02;

  private static SimBattery instance;

  private final List<DoubleSupplier> currentSources = new ArrayList<>();
  private double nominalVolts = DEFAULT_NOMINAL_VOLTS;
  private double resistanceOhms = DEFAULT_RESISTANCE_OHMS;
  private double lastVoltage = DEFAULT_NOMINAL_VOLTS;
  private double lastCurrentAmps = 0.0;
  private boolean brownedOut = false;

  private SimBattery() {}

  public static synchronized SimBattery getInstance() {
    if (instance == null) {
      instance = new SimBattery();
    }
    return instance;
  }

  /** Register a supply-current source, in amps. Called once per simulated motor at construction. */
  public void register(DoubleSupplier supplyCurrentAmps) {
    currentSources.add(supplyCurrentAmps);
  }

  /** Number of registered current sources, for verifying nothing was missed. */
  public int sourceCount() {
    return currentSources.size();
  }

  /** Clear all sources and set new pack parameters. Intended for tests. */
  public void reset(double nominalVolts, double resistanceOhms) {
    currentSources.clear();
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
    for (DoubleSupplier source : currentSources) {
      double amps = source.getAsDouble();
      // Braking motors can report negative draw. Treating that as a recharge
      // would push the pack above nominal, which is not a behaviour we want.
      if (amps > 0.0 && Double.isFinite(amps)) {
        total += amps;
      }
    }
    lastCurrentAmps = total;
    lastVoltage = Math.max(MIN_VOLTAGE, nominalVolts - total * resistanceOhms);
    brownedOut = lastVoltage < BROWNOUT_VOLTAGE;
    return lastVoltage;
  }

  /**
   * Compute the pack voltage, publish it to the simulated roboRIO so every motor sees it, and log
   * it. Call once per simulation loop.
   */
  public void update() {
    double voltage = computeVoltage();
    RoboRioSim.setVInVoltage(voltage);
    RoboRioSim.setBrownoutVoltage(BROWNOUT_VOLTAGE);
    Logger.recordOutput("SimBattery/Voltage", voltage);
    Logger.recordOutput("SimBattery/TotalCurrentAmps", lastCurrentAmps);
    Logger.recordOutput("SimBattery/BrownedOut", brownedOut);
    Logger.recordOutput("SimBattery/SourceCount", currentSources.size());
  }
}
