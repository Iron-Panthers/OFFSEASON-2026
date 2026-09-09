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

    // Two constraints apply -- the supply-limit quadratic and the stator window -- and which
    // one binds depends on speed and on STATOR_TO_SUPPLY_RATIO. Assert the contract rather
    // than which constraint wins, so this does not break when the ratio is retuned.
    double stator = Math.abs(out) / KRAKEN.rOhms;
    double supply = supplyAmps(out, 0.0);
    assertTrue(supply <= limit + 1e-6, "supply " + supply + " A exceeded its limit");
    assertTrue(
        stator <= limit * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO + 1e-6,
        "stator " + stator + " A exceeded its window");
    assertTrue(
        supply >= limit - 1e-6 || stator >= limit * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO - 1e-6,
        "neither constraint was binding, so the clamp is looser than intended");
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
  void statorCeilingSitsBelowTheRacksRealPeak() {
    // Documents a known limitation rather than asserting an ideal. The real intake rack reaches
    // 130.5 A stator against a 27 A supply limit (4.83x), above this ceiling -- so applying
    // clampStatorCurrent globally would truncate real behaviour. Raising the ratio to 5.0 to
    // cover it measured worse overall (currents 0.3221 vs 0.3018), so the ratio stays at 4.0 and
    // the reported-current clamp stays unused. A per-mechanism ceiling would be the real fix.
    double rackCeiling = 27.0 * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO;
    assertTrue(rackCeiling < 130.5, "if this now passes, revisit enabling clampStatorCurrent");
  }

  @Test
  void statorClampPreservesSignAndSmallValues() {
    assertEquals(
        -27.0 * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO,
        SimCurrentLimit.clampStatorCurrent(-1000.0, 27.0),
        1e-9);
    assertEquals(5.0, SimCurrentLimit.clampStatorCurrent(5.0, 27.0), 1e-9);
  }

  @Test
  void statorClampIsInertWhenUnlimited() {
    assertEquals(999.0, SimCurrentLimit.clampStatorCurrent(999.0, 0.0), 1e-9);
  }

  @Test
  void dragIsZeroWhenDisabledOrStopped() {
    assertEquals(0.0, SimCurrentLimit.dragVolts(400.0, 1.0, KRAKEN, 0.0), 1e-12);
    assertEquals(0.0, SimCurrentLimit.dragVolts(0.0, 1.0, KRAKEN, 0.05), 1e-12);
  }

  @Test
  void dragOpposesMotionInBothDirections() {
    double forward = SimCurrentLimit.dragVolts(400.0, 1.0, KRAKEN, 0.02);
    double reverse = SimCurrentLimit.dragVolts(-400.0, 1.0, KRAKEN, 0.02);
    assertTrue(forward > 0.0, "drag should oppose forward motion");
    assertEquals(forward, -reverse, 1e-12);
  }

  @Test
  void dragScalesWithSpeed() {
    double slow = SimCurrentLimit.dragVolts(100.0, 1.0, KRAKEN, 0.02);
    double fast = SimCurrentLimit.dragVolts(400.0, 1.0, KRAKEN, 0.02);
    assertEquals(4.0, fast / slow, 1e-9);
  }

  @Test
  void dragVoltageMatchesTheIntendedDragCurrent() {
    // 8 A of drag at 240 rad/s is the flywheel's real steady-state draw.
    double coefficient = 8.0 / 240.0;
    double volts = SimCurrentLimit.dragVolts(240.0, 1.0, KRAKEN, coefficient);
    assertEquals(8.0, volts / KRAKEN.rOhms, 1e-6);
  }

  @Test
  void zeroBusVoltageYieldsZeroCeiling() {
    assertEquals(0.0, SimCurrentLimit.maxVoltageForSupplyLimit(0.0, KRAKEN, 0.0, 40.0), 1e-9);
  }
}
