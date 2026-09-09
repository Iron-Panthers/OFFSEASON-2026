package frc.robot.utility;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.Test;

class SimCurrentLimitTest {

  private static final DCMotor KRAKEN = DCMotor.getKrakenX60Foc(1);
  private static final double BUS = 12.0;

  /** Supply current implied by an applied voltage, per the model the clamp inverts. */
  private static double supplyAmps(double volts, double backEmf) {
    return volts * (volts - backEmf) / (KRAKEN.rOhms * BUS);
  }

  @Test
  void unlimitedReturnsBusVoltage() {
    assertEquals(BUS, SimCurrentLimit.maxVoltageForSupplyLimit(0.0, KRAKEN, BUS, 0.0), 1e-9);
    assertEquals(BUS, SimCurrentLimit.maxVoltageForSupplyLimit(0.0, KRAKEN, BUS, -5.0), 1e-9);
  }

  @Test
  void ceilingProducesExactlyTheLimitAtStall() {
    // At zero speed there is no back-EMF, so the ceiling should yield exactly the limit.
    double ceiling = SimCurrentLimit.maxVoltageForSupplyLimit(0.0, KRAKEN, BUS, 60.0);
    assertEquals(60.0, supplyAmps(ceiling, 0.0), 1e-6);
  }

  @Test
  void ceilingProducesExactlyTheLimitWhenSpinning() {
    double backEmf = 4.0;
    double ceiling = SimCurrentLimit.maxVoltageForSupplyLimit(backEmf, KRAKEN, BUS, 30.0);
    assertEquals(30.0, supplyAmps(ceiling, backEmf), 1e-6);
  }

  @Test
  void ceilingNeverExceedsTheBus() {
    // A huge limit must still not permit more than the bus can supply.
    assertEquals(BUS, SimCurrentLimit.maxVoltageForSupplyLimit(0.0, KRAKEN, BUS, 10_000.0), 1e-9);
  }

  @Test
  void higherBackEmfAllowsHigherVoltage() {
    // A spinning motor can take more voltage for the same current, because back-EMF opposes it.
    double slow = SimCurrentLimit.maxVoltageForSupplyLimit(0.0, KRAKEN, BUS, 40.0);
    double fast = SimCurrentLimit.maxVoltageForSupplyLimit(6.0, KRAKEN, BUS, 40.0);
    assertTrue(fast > slow, "expected " + fast + " > " + slow);
  }

  @Test
  void clampLeavesModestVoltageUntouched() {
    // 1 V into a stalled Kraken is well under a 60 A supply limit; nothing should change.
    double out = SimCurrentLimit.clampToSupplyLimit(1.0, 0.0, 1.0, KRAKEN, BUS, 60.0);
    assertEquals(1.0, out, 1e-9);
  }

  @Test
  void clampCutsRunawayCommand() {
    double limit = 20.0;
    double out = SimCurrentLimit.clampToSupplyLimit(12.0, 0.0, 1.0, KRAKEN, BUS, limit);
    assertTrue(out < 12.0, "expected the clamp to bite, got " + out);

    // Two constraints apply, and at stall the stator window is the tighter one: supply current
    // is stator * dutyCycle, and dutyCycle is small here, so the supply limit alone would permit
    // an unphysical stator draw.
    double stator = Math.abs(out) / KRAKEN.rOhms;
    assertEquals(limit * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO, stator, 1e-6);
    assertTrue(supplyAmps(out, 0.0) <= limit + 1e-6, "supply current exceeded its limit");
  }

  @Test
  void gearingIsAppliedToBackEmf() {
    // Rotor speed is mechanism speed times the reduction, so a geared mechanism at the same
    // mechanism speed has more back-EMF and therefore a higher voltage ceiling.
    double direct = SimCurrentLimit.clampToSupplyLimit(12.0, 100.0, 1.0, KRAKEN, BUS, 20.0);
    double geared = SimCurrentLimit.clampToSupplyLimit(12.0, 100.0, 4.0, KRAKEN, BUS, 20.0);
    assertTrue(geared > direct, "expected " + geared + " > " + direct);
  }

  @Test
  void brakingAFastWheelIsBoundedByTheBackEmfWindow() {
    // Regression: a wheel spinning fast and commanded hard into reverse. Clamping symmetrically
    // about zero permitted |V - backEmf| ~= 25 V here, which is how the simulated omniwheel
    // reported 747 A on a spin-down.
    double mechRadPerSec = 553.0;
    double out = SimCurrentLimit.clampToSupplyLimit(-12.0, mechRadPerSec, 1.0, KRAKEN, BUS, 60.0);
    double backEmf = mechRadPerSec / KRAKEN.KvRadPerSecPerVolt;
    double statorAmps = Math.abs(out - backEmf) / KRAKEN.rOhms;
    assertTrue(
        statorAmps <= 60.0 * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO + 1e-6,
        "stator current " + statorAmps + " A exceeded the window");
  }

  @Test
  void brakingClampIsNotSymmetricAboutZero() {
    // The permitted window is centred on back-EMF, so it must NOT be symmetric when spinning.
    double forward = SimCurrentLimit.clampToSupplyLimit(12.0, 400.0, 1.0, KRAKEN, BUS, 30.0);
    double reverse = SimCurrentLimit.clampToSupplyLimit(-12.0, 400.0, 1.0, KRAKEN, BUS, 30.0);
    assertTrue(forward != -reverse, "expected an asymmetric window while spinning");
  }

  @Test
  void stationaryClampStaysSymmetric() {
    // With no back-EMF the window IS centred on zero, so symmetry must hold.
    double forward = SimCurrentLimit.clampToSupplyLimit(12.0, 0.0, 1.0, KRAKEN, BUS, 20.0);
    double reverse = SimCurrentLimit.clampToSupplyLimit(-12.0, 0.0, 1.0, KRAKEN, BUS, 20.0);
    assertEquals(forward, -reverse, 1e-9);
  }

  @Test
  void statorClampBoundsPostIntegrationSpikes() {
    // The rack hitting its hard stop reported 206 A against a 27 A limit, because ElevatorSim
    // zeroes velocity inside update() and then evaluates (V - 0)/R.
    assertEquals(
        27.0 * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO,
        SimCurrentLimit.clampStatorCurrent(206.0, 27.0),
        1e-9);
  }

  @Test
  void statorClampPreservesSignAndSmallValues() {
    assertEquals(-108.0, SimCurrentLimit.clampStatorCurrent(-206.0, 27.0), 1e-9);
    assertEquals(5.0, SimCurrentLimit.clampStatorCurrent(5.0, 27.0), 1e-9);
  }

  @Test
  void statorClampIsInertWhenUnlimited() {
    assertEquals(999.0, SimCurrentLimit.clampStatorCurrent(999.0, 0.0), 1e-9);
  }

  @Test
  void zeroBusVoltageYieldsZeroCeiling() {
    assertEquals(0.0, SimCurrentLimit.maxVoltageForSupplyLimit(0.0, KRAKEN, 0.0, 40.0), 1e-9);
  }
}
