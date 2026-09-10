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
  void statorCeilingIsDeliberatelyBelowTheRollersRealRatio() {
    // Documents a known compensating approximation, so it is not "fixed" by accident.
    // Real peak stator / supply limit: drive 3.75x, omniwheel 2.66-3.01x, rack 4.15-4.83x,
    // rollers 5.46-6.02x. The physically honest ceiling clears all of them.
    //
    // Retested at 6.5 AFTER the mechanism load model landed, which was the condition the previous
    // note set for revisiting. The penalty shrank by roughly an order of magnitude but survived:
    // currents q54 0.1966 -> 0.1951, q93 0.1836 -> 0.2027, q14 0.1561 -> 0.1603. Two of three
    // worse, so 4.0 stands. The mechanisms now have the right steady-state load but still slew too
    // hard, and the tight ceiling clips that. Revisit once they slew correctly.
    assertTrue(30.0 * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO < 180.5, "rollers, 6.02x");
    // It must still clear the mechanisms it is not compensating for.
    assertTrue(40.0 * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO > 150.0, "swerve drive, 3.75x");
    assertTrue(60.0 * SimCurrentLimit.STATOR_TO_SUPPLY_RATIO > 180.7, "omniwheel, 3.01x");
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
    assertEquals(0.0, SimCurrentLimit.dragVolts(400.0, KRAKEN, 0.0), 1e-12);
    assertEquals(0.0, SimCurrentLimit.dragVolts(0.0, KRAKEN, 0.05), 1e-12);
  }

  @Test
  void dragOpposesMotionInBothDirections() {
    double forward = SimCurrentLimit.dragVolts(400.0, KRAKEN, 0.02);
    double reverse = SimCurrentLimit.dragVolts(-400.0, KRAKEN, 0.02);
    assertTrue(forward > 0.0, "drag should oppose forward motion");
    assertEquals(forward, -reverse, 1e-12);
  }

  @Test
  void dragScalesWithSpeed() {
    double slow = SimCurrentLimit.dragVolts(100.0, KRAKEN, 0.02);
    double fast = SimCurrentLimit.dragVolts(400.0, KRAKEN, 0.02);
    assertEquals(4.0, fast / slow, 1e-9);
  }

  @Test
  void dragVoltageMatchesTheIntendedDragCurrent() {
    // 8 A of drag at 240 rad/s is the flywheel's real steady-state draw.
    double coefficient = 8.0 / 240.0;
    double volts = SimCurrentLimit.dragVolts(240.0, KRAKEN, coefficient);
    assertEquals(8.0, volts / KRAKEN.rOhms, 1e-6);
  }

  @Test
  void statorAmpsIsZeroWhenVoltageExactlyBalancesBackEmf() {
    double mechRadPerSec = 240.0;
    double gearing = 1.411;
    double backEmf = mechRadPerSec * gearing / KRAKEN.KvRadPerSecPerVolt;
    assertEquals(0.0, SimCurrentLimit.statorAmps(backEmf, mechRadPerSec, gearing, KRAKEN), 1e-9);
  }

  @Test
  void statorAmpsIsNegativeWhenBackDriven() {
    // A wheel spinning faster than its applied voltage supports is returning energy. WPILib's
    // FlywheelSim.getCurrentDrawAmps() multiplies by signum(V) and books this as a large POSITIVE
    // draw, which is how the simulated omniwheel reported 747 A on a spin-down.
    double amps = SimCurrentLimit.statorAmps(-12.0, 553.0, 1.0, KRAKEN);
    assertTrue(amps < 0.0, "expected regen, got " + amps + " A");
  }

  @Test
  void dragAndStatorAmpsAgreeAtSteadyState() {
    // This is the whole point of the pair: the plant is fed (V - drag) so it settles where back-EMF
    // equals that reduced voltage, and the current is then evaluated against the FULL command. The
    // leftover is exactly the drag current -- which is what a frictionless FlywheelSim cannot
    // produce on its own, since it reports zero at every steady state.
    double coefficient = 0.0145 * 4;
    double mechRadPerSec = 239.0;
    double gearing = 1.411;
    double drag = SimCurrentLimit.dragVolts(mechRadPerSec, KRAKEN, coefficient);
    // At steady state the plant has settled so that back-EMF equals its input voltage.
    double commanded = mechRadPerSec * gearing / KRAKEN.KvRadPerSecPerVolt + drag;
    assertEquals(
        coefficient * mechRadPerSec,
        SimCurrentLimit.statorAmps(commanded, mechRadPerSec, gearing, KRAKEN),
        1e-6);
  }

  @Test
  void constantLoadIsInertWhenZero() {
    assertEquals(5.0, SimCurrentLimit.applyConstantLoad(5.0, 0.0), 1e-12);
  }

  @Test
  void constantLoadKeepsItsSignWhicheverWayTheCommandPoints() {
    // Not friction: the load pulls the same way whether the mechanism is being driven out or
    // back. Modelling it as friction let the controller dither symmetrically about the target,
    // and the rack's mean supply current came out at -0.01 A against a real 5.67 A.
    assertEquals(11.39, SimCurrentLimit.applyConstantLoad(12.0, 0.61), 1e-12);
    assertEquals(-12.61, SimCurrentLimit.applyConstantLoad(-12.0, 0.61), 1e-12);
  }

  @Test
  void constantLoadHoldingCurrentMatchesTheMeasuredRack() {
    // Holding at target, the plant sees zero and the mechanism stays put, but the controller is
    // still commanding LOAD_VOLTS -- and against zero back-EMF that is pure stator current. The
    // real rack logs 25.9 A mean while held.
    double commanded = 0.61;
    assertEquals(0.0, SimCurrentLimit.applyConstantLoad(commanded, 0.61), 1e-12);
    assertEquals(24.6, SimCurrentLimit.statorAmps(commanded, 0.0, 1.0, KRAKEN), 0.5);
  }

  @Test
  void zeroBusVoltageYieldsZeroCeiling() {
    assertEquals(0.0, SimCurrentLimit.maxVoltageForSupplyLimit(0.0, KRAKEN, 0.0, 40.0), 1e-9);
  }
}
