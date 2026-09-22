package frc.robot.utility.rendering;

/**
 * A 36h11 AprilTag bitmap projected onto the vinyl panel the field model already has in the right
 * place.
 *
 * <p>The export names each panel {@code FE-00083-ID<n>-Vinyl}, so the tag never has to be
 * positioned by hand: the CAD geometry supplies the pose and this only supplies the pattern. The
 * panel is a 10.5 in square; the 10x10 tag image covers a centred 8.125 in of it and the rest stays
 * white vinyl, which is how the real decal is printed.
 *
 * @param id field tag ID, 1 through 32 for the 2026 field
 * @param bitmap {@code size * size} linear reflectances, row 0 at the top of the tag
 * @param size bitmap edge length in cells, 10 for 36h11
 * @param center panel centre in WPILib field coordinates, xyz
 * @param axisU unit world direction that image +u runs along
 * @param axisV unit world direction that image +v runs along, i.e. down the rows
 * @param printedSizeMeters world edge length the bitmap is stretched across
 */
record TagDecal(
    int id,
    float[] bitmap,
    int size,
    double[] center,
    double[] axisU,
    double[] axisV,
    double printedSizeMeters) {

  /**
   * Black ink on white vinyl, as linear reflectance.
   *
   * <p>Not 0 and 1. A real printed tag under arena light lands near 5% and 80%, and feeding a
   * detector perfect black-on-white would make these frames easier than any real one.
   */
  static final float INK_REFLECTANCE = 0.045f;

  static final float VINYL_REFLECTANCE = 0.78f;

  /**
   * Samples the panel at a world-space point.
   *
   * @return linear reflectance at that point, falling back to bare vinyl outside the printed area
   */
  float sample(double x, double y, double z) {
    double dx = x - center[0];
    double dy = y - center[1];
    double dz = z - center[2];
    double u = dx * axisU[0] + dy * axisU[1] + dz * axisU[2];
    double v = dx * axisV[0] + dy * axisV[1] + dz * axisV[2];

    double half = printedSizeMeters * 0.5;
    if (u < -half || u > half || v < -half || v > half) {
      return VINYL_REFLECTANCE;
    }

    int col = (int) ((u + half) / printedSizeMeters * size);
    int row = (int) ((v + half) / printedSizeMeters * size);
    col = Math.min(size - 1, Math.max(0, col));
    row = Math.min(size - 1, Math.max(0, row));
    return bitmap[row * size + col];
  }
}
