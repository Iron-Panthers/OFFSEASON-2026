package frc.robot.utility.rendering;

/**
 * Deterministic value noise, used to give the carpet a surface.
 *
 * <p>The field export models the carpet as one flat slab of a single grey. A camera 45 cm off the
 * ground sees mostly floor, and a perfectly uniform floor is the strongest single cue that a frame
 * is synthetic, so the weave is generated here instead.
 *
 * <p>Hash based rather than table based: no state, no allocation, safe to call from every render
 * thread at once, and identical across runs so two renders of the same scene match exactly.
 */
final class Noise {

  private Noise() {}

  /** Integer hash from the finaliser in MurmurHash3, which is cheap and mixes well enough here. */
  private static int hash(int x, int y) {
    int h = x * 0x27D4EB2D ^ y * 0x165667B1;
    h ^= h >>> 15;
    h *= 0x2C1B3C6D;
    h ^= h >>> 12;
    h *= 0x297A2D39;
    h ^= h >>> 15;
    return h;
  }

  private static float gridValue(int x, int y) {
    return (hash(x, y) >>> 8) * (1f / (1 << 24));
  }

  /** Value noise in [0, 1] with smoothstep interpolation. */
  static float value(float x, float y) {
    int x0 = (int) Math.floor(x);
    int y0 = (int) Math.floor(y);
    float fx = x - x0;
    float fy = y - y0;
    float sx = fx * fx * (3f - 2f * fx);
    float sy = fy * fy * (3f - 2f * fy);

    float v00 = gridValue(x0, y0);
    float v10 = gridValue(x0 + 1, y0);
    float v01 = gridValue(x0, y0 + 1);
    float v11 = gridValue(x0 + 1, y0 + 1);

    float top = v00 + (v10 - v00) * sx;
    float bottom = v01 + (v11 - v01) * sx;
    return top + (bottom - top) * sy;
  }

  /**
   * Fractal noise over a point in space, in [0, 1].
   *
   * <p>Built from two decorrelated 2D lookups rather than true 3D noise. A single {@code (x, y)}
   * lookup is constant along z, which paints vertical streaks down every wall on the field, and
   * that reads worse than no variation at all. Mixing in a second lookup on skewed axes costs one
   * extra evaluation and removes the streaking.
   */
  static float spatial(float x, float y, float z, int octaves) {
    float planar = fractal(x, y, octaves);
    float skewed = fractal(y * 1.7f + z * 2.3f, x * 0.9f - z * 1.3f, octaves);
    return 0.5f * (planar + skewed);
  }

  /**
   * Fractal sum of {@link #value} octaves, in [0, 1].
   *
   * @param octaves how many doublings of frequency to sum; 3 is enough at camera distances
   */
  static float fractal(float x, float y, int octaves) {
    float sum = 0;
    float amplitude = 0.5f;
    float total = 0;
    for (int octave = 0; octave < octaves; octave++) {
      sum += value(x, y) * amplitude;
      total += amplitude;
      x *= 2.07f; // non-integer so octaves do not line their grids up
      y *= 2.03f;
      amplitude *= 0.5f;
    }
    return sum / total;
  }
}
