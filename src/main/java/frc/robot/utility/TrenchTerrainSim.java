package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Computes robot 3D contact state with the trench/bump terrain in the 2026 FRC field.
 *
 * <p>The bump is a tent-shaped ramp crossing the trench structure:
 *
 * <ul>
 *   <li>Blue bump ramp up: X ∈ [3.96, 4.61], Z rises from 0 to 0.165 m
 *   <li>Blue bump ramp down: X ∈ [4.61, 5.18], Z falls from 0.165 m to 0
 *   <li>Red side is mirrored at FIELD_LENGTH − X
 *   <li>Active Y zones: [1.57, 3.42] and [4.62, 6.47]
 *   <li>Center clear lane Y ∈ [3.42, 4.62] has NO bump
 * </ul>
 *
 * <p>Geometry sourced from {@link FuelSim#FIELD_XZ_LINE_STARTS} / {@link
 * FuelSim#FIELD_XZ_LINE_ENDS} segments 1–8.
 */
public class TrenchTerrainSim {

  // ── Field dimensions (mirror FuelSim constants) ────────────────────────────
  private static final double FIELD_LENGTH = FuelSim.FIELD_LENGTH; // 16.51 m
  private static final double FIELD_WIDTH = FuelSim.FIELD_WIDTH; // 8.04 m

  // ── Bump X boundaries (blue side) ──────────────────────────────────────────
  private static final double BUMP_X_BLUE_START = 3.96;
  private static final double BUMP_X_BLUE_PEAK = 4.61;
  private static final double BUMP_X_BLUE_END = 5.18;
  private static final double BUMP_PEAK_Z = 0.165; // metres

  // Pre-computed slopes (dZ/dX) on each ramp face
  private static final double SLOPE_BLUE_UP =
      BUMP_PEAK_Z / (BUMP_X_BLUE_PEAK - BUMP_X_BLUE_START); // ≈ +0.2538
  private static final double SLOPE_BLUE_DOWN =
      -BUMP_PEAK_Z / (BUMP_X_BLUE_END - BUMP_X_BLUE_PEAK); // ≈ −0.2895

  // ── Bump X boundaries (red side — mirrored) ────────────────────────────────
  private static final double BUMP_X_RED_START =
      FIELD_LENGTH - BUMP_X_BLUE_END; // ≈ 11.33 (approach from red wall)
  private static final double BUMP_X_RED_PEAK = FIELD_LENGTH - BUMP_X_BLUE_PEAK; // ≈ 11.90
  private static final double BUMP_X_RED_END = FIELD_LENGTH - BUMP_X_BLUE_START; // ≈ 12.55

  // Red slopes are the mirror of blue (steeper face is the one closer to the alliance wall)
  private static final double SLOPE_RED_UP =
      BUMP_PEAK_Z / (BUMP_X_RED_PEAK - BUMP_X_RED_START); // ≈ +0.2895
  private static final double SLOPE_RED_DOWN =
      -BUMP_PEAK_Z / (BUMP_X_RED_END - BUMP_X_RED_PEAK); // ≈ −0.2538

  // ── Y zone boundaries ──────────────────────────────────────────────────────
  private static final double BUMP_Y_LEFT_MIN = 1.57;
  private static final double BUMP_Y_LEFT_MAX = FIELD_WIDTH / 2.0 - 0.60; // ≈ 3.42
  private static final double BUMP_Y_RIGHT_MIN = FIELD_WIDTH / 2.0 + 0.60; // ≈ 4.62
  private static final double BUMP_Y_RIGHT_MAX = FIELD_WIDTH - 1.57; // ≈ 6.47

  // ── Public API ─────────────────────────────────────────────────────────────

  /**
   * Returns the terrain height (Z, metres above the floor) at a given field position.
   *
   * @param x field X coordinate (metres)
   * @param y field Y coordinate (metres)
   * @return terrain Z ≥ 0
   */
  public static double getTerrainZAt(double x, double y) {
    if (!isInBumpYZone(y)) return 0.0;

    // Blue side
    if (x >= BUMP_X_BLUE_START && x <= BUMP_X_BLUE_PEAK)
      return SLOPE_BLUE_UP * (x - BUMP_X_BLUE_START);
    if (x > BUMP_X_BLUE_PEAK && x <= BUMP_X_BLUE_END)
      return BUMP_PEAK_Z + SLOPE_BLUE_DOWN * (x - BUMP_X_BLUE_PEAK);

    // Red side
    if (x >= BUMP_X_RED_START && x <= BUMP_X_RED_PEAK) return SLOPE_RED_UP * (x - BUMP_X_RED_START);
    if (x > BUMP_X_RED_PEAK && x <= BUMP_X_RED_END)
      return BUMP_PEAK_Z + SLOPE_RED_DOWN * (x - BUMP_X_RED_PEAK);

    return 0.0;
  }

  /**
   * Returns dZ/dX (field-X terrain gradient) at a given position. dZ/dY is always 0 because the
   * bump extends uniformly along the Y axis within each active zone.
   *
   * @param x field X coordinate (metres)
   * @param y field Y coordinate (metres)
   * @return dZ/dX — positive on uphill approach, negative on downhill departure
   */
  public static double getTerrainGradientX(double x, double y) {
    if (!isInBumpYZone(y)) return 0.0;

    if (x >= BUMP_X_BLUE_START && x <= BUMP_X_BLUE_PEAK) return SLOPE_BLUE_UP;
    if (x > BUMP_X_BLUE_PEAK && x <= BUMP_X_BLUE_END) return SLOPE_BLUE_DOWN;
    if (x >= BUMP_X_RED_START && x <= BUMP_X_RED_PEAK) return SLOPE_RED_UP;
    if (x > BUMP_X_RED_PEAK && x <= BUMP_X_RED_END) return SLOPE_RED_DOWN;

    return 0.0;
  }

  /**
   * Per-loop snapshot of the robot's 3D terrain contact state.
   *
   * <p>Note on naming convention (intentionally non-standard to match log key expectations):
   *
   * <ul>
   *   <li>{@code rollRad} — front/rear tilt (nose-up when climbing the bump); this is the large
   *       signal. Stored in Rotation3d slot2 (about Y-axis) → correct nose-up 3D visual.
   *   <li>{@code pitchRad} — left/right tilt; nearly zero for straight-ahead driving. Stored in
   *       Rotation3d slot1 (about X-axis).
   * </ul>
   *
   * @param robotBodyZ chassis centre height above the floor (metres)
   * @param pitchRad left/right tilt (side roll); nearly zero during straight bump crossing
   * @param rollRad nose-up/down tilt from front/rear wheel height difference; negative = nose up
   * @param avgGradientX average dZ/dX over the four wheel contact points — used for gravity force
   * @param wheelZ individual wheel contact heights [FL, FR, BL, BR] (metres)
   */
  public record TerrainContactState(
      double robotBodyZ, double pitchRad, double rollRad, double avgGradientX, double[] wheelZ) {}

  /**
   * Computes the robot's terrain contact state from its current 2D pose.
   *
   * <p>Module index convention (matches {@code DriveConstants.MODULE_TRANSLATIONS}):
   *
   * <pre>
   *   [0] FL — (+x, +y)   [1] FR — (+x, −y)
   *   [2] BL — (−x, +y)   [3] BR — (−x, −y)
   * </pre>
   *
   * @param robotPose 2D pose of the robot centre on the field
   * @param moduleTranslations robot-frame module positions [FL, FR, BL, BR]
   * @param wheelRadius drive wheel radius (metres)
   * @param wheelbase front-to-rear wheel separation (metres) — used for rollRad (front/rear tilt)
   * @param trackWidth left-to-right wheel separation (metres) — used for pitchRad (left/right tilt)
   * @return terrain contact state for this loop
   */
  public static TerrainContactState compute(
      Pose2d robotPose,
      Translation2d[] moduleTranslations,
      double wheelRadius,
      double wheelbase,
      double trackWidth) {

    double[] wheelZ = new double[4];
    double gradientSum = 0.0;

    for (int i = 0; i < 4; i++) {
      // Rotate module offset from robot frame → field frame
      Translation2d fieldOffset = moduleTranslations[i].rotateBy(robotPose.getRotation());
      double wx = robotPose.getX() + fieldOffset.getX();
      double wy = robotPose.getY() + fieldOffset.getY();
      wheelZ[i] = getTerrainZAt(wx, wy);
      gradientSum += getTerrainGradientX(wx, wy);
    }

    double avgGradientX = gradientSum / 4.0;
    double robotBodyZ = (wheelZ[0] + wheelZ[1] + wheelZ[2] + wheelZ[3]) / 4.0 + wheelRadius;

    // Roll (front/rear tilt, logged as RollDegrees — the large signal during bump crossing).
    // Goes into Rotation3d slot2 (about Y-axis) → nose-up 3D visual.
    // Negative sign: nose-up when front wheels are higher → negative angle about Y.
    double frontAvgZ = (wheelZ[0] + wheelZ[1]) / 2.0;
    double rearAvgZ = (wheelZ[2] + wheelZ[3]) / 2.0;
    double rollRad = -Math.atan2(frontAvgZ - rearAvgZ, wheelbase);

    // Pitch (left/right tilt, logged as PitchDegrees — nearly zero for straight-ahead driving).
    // Goes into Rotation3d slot1 (about X-axis).
    double leftAvgZ = (wheelZ[0] + wheelZ[2]) / 2.0;
    double rightAvgZ = (wheelZ[1] + wheelZ[3]) / 2.0;
    double pitchRad = Math.atan2(leftAvgZ - rightAvgZ, trackWidth);

    return new TerrainContactState(robotBodyZ, pitchRad, rollRad, avgGradientX, wheelZ);
  }

  // ── Internal helpers ───────────────────────────────────────────────────────

  private static boolean isInBumpYZone(double y) {
    return (y >= BUMP_Y_LEFT_MIN && y <= BUMP_Y_LEFT_MAX)
        || (y >= BUMP_Y_RIGHT_MIN && y <= BUMP_Y_RIGHT_MAX);
  }
}
