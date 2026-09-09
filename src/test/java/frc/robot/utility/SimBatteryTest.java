package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SimBatteryTest {

  @BeforeEach
  void reset() {
    SimBattery.getInstance().reset(12.8, 0.02);
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
  void negativeCurrentFromRegenIsIgnored() {
    // A motor braking can report negative draw; it must not inflate pack voltage
    // above nominal, which would be unphysical for our purposes.
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
  void ignoresNullConfigString() {
    SimBattery.getInstance().configureFromProperty(null);
    assertEquals(12.8, SimBattery.getInstance().computeVoltage(), 1e-6);
  }
}
