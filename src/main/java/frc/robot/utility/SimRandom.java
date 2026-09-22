package frc.robot.utility;

import java.util.Random;

/**
 * Seeded random streams for simulation, so two runs of different code can share the same noise.
 *
 * <p>The simulation draws randomness in two places — shot spread in {@code RobotSimState.shootFuel}
 * and hub dispersal in {@code FuelSim.Hub} — and both used bare {@code Math.random()}. That makes
 * two identical runs score differently, which in turn makes A/B testing a change impossible: the
 * run-to-run noise swamps the effect being measured.
 *
 * <p>Setting {@code -Dab.sim.seed=<n>} (Gradle: {@code -Psim.seed=<n>}) makes every stream
 * reproducible. Run the baseline and the change at the same seed and they see the same field
 * conditions, so the difference between them is the change and not the dice. This is the standard
 * "common random numbers" variance reduction technique.
 *
 * <p><b>Streams are per-consumer on purpose.</b> A single shared stream would desynchronise the
 * moment one arm fires an extra ball: every subsequent draw would shift by one and the pairing
 * would be lost. Independent streams keep the shooter's noise sequence aligned across arms
 * regardless of what the hub did.
 *
 * <p>PhotonVision's simulated camera noise is seeded alongside these, in {@code
 * VisionIOPhotonvisionSim}, via the vendor's own {@code SimCameraProperties.setRandomSeed}. That
 * one matters more than it looks: a noisy pose estimate moves where auto-aim points, and leaving it
 * unseeded held the shotgrid benchmark's measured noise floor at 0.23 hit-rate. With it seeded the
 * floor is zero — an A/A run reproduces exactly.
 *
 * <p><b>What this does not fix.</b> Pairing holds while the two arms behave similarly. Once a
 * change genuinely diverges — firing at a different time, driving a different path — its draws
 * diverge too. That is correct physics rather than a defect, but it means variance reduction is
 * strongest for small changes. And a seed fixes one field realisation, not all of them: agreement
 * across several seeds is what shows a change helps generally rather than getting lucky once.
 *
 * <p>With no seed property set, every stream is seeded randomly and behaviour matches the old
 * {@code Math.random()} exactly. Nothing changes for normal simulation use.
 */
public final class SimRandom {

  /** Gradle {@code -Psim.seed=<n>} arrives as this system property. */
  private static final String SEED_PROPERTY = "ab.sim.seed";

  private static final Long SEED = Long.getLong(SEED_PROPERTY);

  /** Shot spread applied across the shooter's width when fuel is launched. */
  private static final Random SHOOTER = streamFor(1);

  /** Velocity given to fuel as it is dispersed back out of a hub after scoring. */
  private static final Random HUB = streamFor(2);

  private SimRandom() {}

  /**
   * Builds one stream, offset so the streams do not produce identical sequences.
   *
   * @param streamIndex distinct small integer per consumer
   */
  private static Random streamFor(int streamIndex) {
    // Mixed with a large odd constant rather than added, so that adjacent seeds (1, 2, 3 — exactly
    // what a paired A/B sweep uses) do not produce streams that are near-neighbours of each other.
    return SEED == null
        ? new Random()
        : new Random(SEED * 0x9E3779B97F4A7C15L + streamIndex * 0x7FFFFFFFL);
  }

  /** True when a fixed seed was supplied, so runs are reproducible. */
  public static boolean isSeeded() {
    return SEED != null;
  }

  /** The configured seed, or null when running unseeded. */
  public static Long seed() {
    return SEED;
  }

  /** Stream for shooter launch noise. */
  public static Random shooter() {
    return SHOOTER;
  }

  /** Stream for hub dispersal noise. */
  public static Random hub() {
    return HUB;
  }
}
