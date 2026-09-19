package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SimBatteryTest {

  @BeforeEach
  void reset() {
    SimBattery.getInstance().reset(12.8, 0.02, 0.0);
  }

  @Test
  void openCircuitVoltageDroopsAsThePackDischarges() {
    // Real packs end a match measurably flatter than they start: fitted droop across four
    // matches was 0.345 to 1.019 V/min. Without this the sim pack is as fresh at 165 s as at 0.
    SimBattery.getInstance().reset(12.4, 0.0112, 0.6);
    assertEquals(12.4, SimBattery.getInstance().openCircuitVolts(), 1e-9);
    SimBattery.getInstance().addEnabledTime(60.0);
    assertEquals(11.8, SimBattery.getInstance().openCircuitVolts(), 1e-9);
    SimBattery.getInstance().addEnabledTime(105.0);
    // 165 s at 0.6 V/min = 1.65 V of droop, matching the real q54 span of ~1.2 V.
    assertEquals(12.4 - 1.65, SimBattery.getInstance().openCircuitVolts(), 1e-9);
  }

  @Test
  void droopLowersTheLoadedVoltageToo() {
    SimBattery.getInstance().reset(12.4, 0.0112, 0.6);
    SimBattery.getInstance().register(() -> 100.0);
    double fresh = SimBattery.getInstance().computeVoltage();
    SimBattery.getInstance().addEnabledTime(60.0);
    double tired = SimBattery.getInstance().computeVoltage();
    assertEquals(0.6, fresh - tired, 1e-9);
  }

  @Test
  void voltageCeilingFollowsTheDroopedOpenCircuitVoltage() {
    // Regen must not push the pack back up to its START-of-match voltage.
    SimBattery.getInstance().reset(12.4, 0.0112, 0.6);
    SimBattery.getInstance().addEnabledTime(60.0);
    SimBattery.getInstance().register(() -> -500.0);
    assertEquals(11.8, SimBattery.getInstance().computeVoltage(), 1e-9);
  }

  @Test
  void configStringAcceptsAnOptionalDroopField() {
    SimBattery.getInstance().configureFromProperty("12.451:0.01196:0.439");
    SimBattery.getInstance().addEnabledTime(60.0);
    assertEquals(12.451 - 0.439, SimBattery.getInstance().openCircuitVolts(), 1e-9);
  }

  @Test
  void configStringWithoutDroopKeepsTheDefault() {
    SimBattery.getInstance().reset(12.4, 0.0112, 0.6);
    SimBattery.getInstance().configureFromProperty("12.1:0.0135");
    SimBattery.getInstance().addEnabledTime(60.0);
    assertEquals(12.1 - 0.6, SimBattery.getInstance().openCircuitVolts(), 1e-9);
  }

  @Test
  void configStringWithFourFieldsIsRejected() {
    SimBattery.getInstance().reset(12.4, 0.0112, 0.6);
    SimBattery.getInstance().configureFromProperty("1:2:3:4");
    assertEquals(12.4, SimBattery.getInstance().openCircuitVolts(), 1e-9);
  }

  @Test
  void noLoadSitsAtNominalVoltage() {
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void loadSagsProportionalToCurrentAndResistance() {
    SimBattery.getInstance().register(() -> 100.0);
    // 12.8 V - (100 A * 0.02 ohm) = 10.8 V
    assertEquals(10.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void multipleSourcesSum() {
    SimBattery.getInstance().register(() -> 50.0);
    SimBattery.getInstance().register(() -> 50.0);
    assertEquals(10.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void voltageNeverGoesBelowFloor() {
    SimBattery.getInstance().register(() -> 10_000.0);
    assertTrue(SimBattery.getInstance().computeVoltage() >= SimBattery.MIN_VOLTAGE);
  }

  @Test
  void regenNeverInflatesVoltageAboveNominal() {
    // Regen is now summed signed rather than discarded, so the guard is on the voltage:
    // a braking motor must not push the pack above its open-circuit voltage.
    SimBattery.getInstance().register(() -> -100.0);
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void nonFiniteCurrentIsIgnored() {
    // A diverging physics sim can produce NaN. Letting it through would poison
    // the pack voltage for every motor and silently wreck a whole run.
    SimBattery.getInstance().register(() -> Double.NaN);
    SimBattery.getInstance().register(() -> 100.0);
    assertEquals(10.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void brownoutFlagsBelowThreshold() {
    SimBattery.getInstance().register(() -> 400.0); // 12.8 - 8.0 = 4.8 V
    SimBattery.getInstance().computeVoltage();
    assertTrue(SimBattery.getInstance().isBrownedOut());
  }

  @Test
  void noBrownoutUnderLightLoad() {
    SimBattery.getInstance().register(() -> 10.0);
    SimBattery.getInstance().computeVoltage();
    assertFalse(SimBattery.getInstance().isBrownedOut());
  }

  @Test
  void resetClearsRegisteredSources() {
    SimBattery.getInstance().register(() -> 100.0);
    SimBattery.getInstance().reset(12.8, 0.02);
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
    assertEquals(0, SimBattery.getInstance().sourceCount());
  }

  @Test
  void parsesConfigString() {
    SimBattery.getInstance().configureFromProperty("12.5:0.015");
    SimBattery.getInstance().register(() -> 100.0);
    assertEquals(11.0, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void ignoresMalformedConfigString() {
    SimBattery.getInstance().configureFromProperty("nonsense");
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void perSourceLimitClampsRunawayCurrent() {
    // An unbounded FlywheelSim reported 952 A on one mechanism in the baseline run.
    SimBattery.getInstance().register(() -> 952.0, 60.0);
    // Clamped to 60 * 1.5 = 90 A, so 12.8 - 90*0.02 = 11.0 V.
    assertEquals(11.0, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void unlimitedSourceIsNotClamped() {
    // 300 A keeps the result above MIN_VOLTAGE so this tests the clamp, not the floor.
    SimBattery.getInstance().register(() -> 300.0, 0.0);
    assertEquals(12.8 - 300.0 * 0.02, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void regenIsSummedSignedButNeverRaisesVoltageAboveNominal() {
    // Real TotalAmps reaches -101.7 A, and the real pack still read 9.78 V there -- regen
    // never charges it. The signed sum must be kept, but the voltage clamped at nominal.
    SimBattery.getInstance().register(() -> -101.7, 0.0);
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void regenOffsetsDrawInTheSignedSum() {
    SimBattery.getInstance().register(() -> 200.0, 0.0);
    SimBattery.getInstance().register(() -> -100.0, 0.0);
    // Net +100 A, not 300 A as the old positives-only sum would have given.
    assertEquals(12.8 - 100.0 * 0.02, SimBattery.getInstance().computeVoltage(), 1e-6);
  }

  @Test
  void brownoutThresholdMatchesTheRealRoboRio() {
    assertEquals(6.75, SimBattery.BROWNOUT_VOLTAGE, 1e-9);
  }

  @Test
  void ignoresNullConfigString() {
    SimBattery.getInstance().configureFromProperty(null);
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }
}
