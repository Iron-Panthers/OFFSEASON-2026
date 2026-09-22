package frc.robot.utility.rendering;

import java.util.stream.IntStream;

/**
 * Edge-avoiding a-trous filter over the indirect lighting.
 *
 * <p>Two samples a pixel is nowhere near enough to resolve soft shadows and bounce light cleanly,
 * and pushing the sample count high enough to do it honestly would cost about twenty times the
 * frame time. Filtering instead is the standard trade, and the edge stopping functions are what
 * keep it from turning into a blur: a neighbour only contributes if it faces the same way and sits
 * at a similar distance.
 *
 * <p>The filter runs on radiance divided by albedo. That separation matters here more than in a
 * typical renderer, because the detail this scene depends on, the carpet weave and the hard edges
 * of an AprilTag, lives entirely in the albedo and has to come through untouched.
 */
final class Denoiser {

  private Denoiser() {}

  /** B3 spline row, the usual a-trous kernel. */
  private static final float[] KERNEL = {1f / 16f, 1f / 4f, 3f / 8f, 1f / 4f, 1f / 16f};

  /** Depth tolerance in metres at the first, finest pass. */
  private static final float DEPTH_TOLERANCE = 0.06f;

  /**
   * Brightness tolerance, as a fraction of local brightness rather than an absolute difference.
   *
   * <p>It has to be relative. Demodulating by albedo means a deep blue wall, whose red and green
   * albedo are near zero, comes out of the division with enormous values in those channels. An
   * absolute tolerance treats every neighbour there as a different surface, drives all the weights
   * to zero, and hands back the unfiltered noise, which is exactly what the alliance walls looked
   * like before this was relative.
   */
  private static final float LUMINANCE_TOLERANCE = 0.75f;

  /** Keeps the relative tolerance from collapsing on dark pixels. */
  private static final float LUMINANCE_FLOOR = 0.22f;

  /**
   * Floor on the albedo used for demodulation. Applied identically on the way in and the way out,
   * so it never changes the result, only how the filter sees near-black surfaces.
   */
  private static final float ALBEDO_FLOOR = 0.05f;

  /**
   * How many standard deviations above its neighbours a pixel may be before it is treated as a
   * sampling accident rather than as a real highlight.
   *
   * <p>Without this the filter actively protects fireflies: an isolated bright pixel disagrees with
   * every neighbour, the luminance edge stop drives all their weights to zero, and the outlier is
   * copied through untouched. Clamping first is what makes the rest of the filter work.
   */
  private static final float FIREFLY_SIGMA = 3.0f;

  /**
   * Filters a frame in place.
   *
   * @param passes how many doublings of the filter footprint to run; 3 covers a 33 pixel radius
   */
  static void apply(Renderer.Frame frame, int passes) {
    int width = frame.width();
    int height = frame.height();
    int pixels = width * height;

    float[] current = new float[pixels * 3];
    float[] next = new float[pixels * 3];

    // Demodulate: take the albedo out so the filter only ever sees lighting.
    for (int i = 0; i < pixels; i++) {
      for (int c = 0; c < 3; c++) {
        int at = i * 3 + c;
        current[at] = frame.color()[at] / Math.max(ALBEDO_FLOOR, frame.albedo()[at]);
      }
    }

    clampFireflies(current, next, width, height);
    float[] swap = current;
    current = next;
    next = swap;

    for (int pass = 0; pass < passes; pass++) {
      int stride = 1 << pass;
      filter(frame, current, next, width, height, stride);
      swap = current;
      current = next;
      next = swap;
    }

    for (int i = 0; i < pixels; i++) {
      for (int c = 0; c < 3; c++) {
        int at = i * 3 + c;
        frame.color()[at] = current[at] * Math.max(ALBEDO_FLOOR, frame.albedo()[at]);
      }
    }
  }

  /** Scales down any pixel far brighter than the eight around it, preserving its hue. */
  private static void clampFireflies(float[] source, float[] target, int width, int height) {
    IntStream.range(0, height)
        .parallel()
        .forEach(y -> clampFireflyRow(source, target, width, height, y));
  }

  private static void clampFireflyRow(
      float[] source, float[] target, int width, int height, int y) {
    {
      for (int x = 0; x < width; x++) {
        int center = y * width + x;
        float sum = 0;
        float sumSquares = 0;
        int counted = 0;

        for (int ky = -1; ky <= 1; ky++) {
          int sy = y + ky;
          if (sy < 0 || sy >= height) {
            continue;
          }
          for (int kx = -1; kx <= 1; kx++) {
            int sx = x + kx;
            if (sx < 0 || sx >= width || (kx == 0 && ky == 0)) {
              continue;
            }
            float luminance = luminance(source, sy * width + sx);
            sum += luminance;
            sumSquares += luminance * luminance;
            counted++;
          }
        }

        float centerLuminance = luminance(source, center);
        float scale = 1f;
        if (counted > 1) {
          float mean = sum / counted;
          float variance = Math.max(0f, sumSquares / counted - mean * mean);
          float limit = mean + FIREFLY_SIGMA * (float) Math.sqrt(variance);
          if (centerLuminance > limit && centerLuminance > 1e-4f) {
            scale = limit / centerLuminance;
          }
        }
        target[center * 3] = source[center * 3] * scale;
        target[center * 3 + 1] = source[center * 3 + 1] * scale;
        target[center * 3 + 2] = source[center * 3 + 2] * scale;
      }
    }
  }

  /**
   * Runs one a-trous pass.
   *
   * <p>Rows are independent, and this filter is a large enough share of the post-processing that
   * leaving it single threaded halves the achievable frame rate on its own.
   */
  private static void filter(
      Renderer.Frame frame, float[] source, float[] target, int width, int height, int stride) {
    IntStream.range(0, height)
        .parallel()
        .forEach(y -> filterRow(frame, source, target, width, height, stride, y));
  }

  private static void filterRow(
      Renderer.Frame frame,
      float[] source,
      float[] target,
      int width,
      int height,
      int stride,
      int y) {

    float[] normals = frame.normal();
    float[] depth = frame.depth();
    float inverseDepthTolerance = 1f / (DEPTH_TOLERANCE * stride);

    {
      for (int x = 0; x < width; x++) {
        int center = y * width + x;

        float centerDepth = depth[center];
        boolean centerIsSky = Float.isInfinite(centerDepth);
        float centerNormalX = normals[center * 3];
        float centerNormalY = normals[center * 3 + 1];
        float centerNormalZ = normals[center * 3 + 2];
        float centerLuminance = luminance(source, center);
        // Relative to local brightness, but with a floor: a purely proportional tolerance goes to
        // zero in the shadows and starts protecting noise again down there.
        float inverseLuminanceTolerance =
            1f / (LUMINANCE_TOLERANCE * (Math.abs(centerLuminance) + LUMINANCE_FLOOR));

        float sumR = 0;
        float sumG = 0;
        float sumB = 0;
        float sumWeight = 0;

        for (int ky = -2; ky <= 2; ky++) {
          int sy = y + ky * stride;
          if (sy < 0 || sy >= height) {
            continue;
          }
          float rowWeight = KERNEL[ky + 2];
          for (int kx = -2; kx <= 2; kx++) {
            int sx = x + kx * stride;
            if (sx < 0 || sx >= width) {
              continue;
            }
            int sample = sy * width + sx;

            // Sky and geometry never share lighting, so they never share a filter footprint.
            if (centerIsSky != Float.isInfinite(depth[sample])) {
              continue;
            }
            float weight = rowWeight * KERNEL[kx + 2];

            if (!centerIsSky) {
              float normalAgreement =
                  centerNormalX * normals[sample * 3]
                      + centerNormalY * normals[sample * 3 + 1]
                      + centerNormalZ * normals[sample * 3 + 2];
              if (normalAgreement <= 0) {
                continue;
              }
              weight *= pow48(normalAgreement);
              weight *= expNegative(Math.abs(centerDepth - depth[sample]) * inverseDepthTolerance);
            }

            weight *=
                expNegative(
                    Math.abs(centerLuminance - luminance(source, sample))
                        * inverseLuminanceTolerance);

            if (weight <= 1e-6f) {
              continue;
            }
            sumR += source[sample * 3] * weight;
            sumG += source[sample * 3 + 1] * weight;
            sumB += source[sample * 3 + 2] * weight;
            sumWeight += weight;
          }
        }

        if (sumWeight > 1e-6f) {
          target[center * 3] = sumR / sumWeight;
          target[center * 3 + 1] = sumG / sumWeight;
          target[center * 3 + 2] = sumB / sumWeight;
        } else {
          target[center * 3] = source[center * 3];
          target[center * 3 + 1] = source[center * 3 + 1];
          target[center * 3 + 2] = source[center * 3 + 2];
        }
      }
    }
  }

  /**
   * The normal agreement term, {@code x} to the 48th.
   *
   * <p>By repeated squaring rather than {@link Math#pow}, which is called twenty-five times per
   * pixel per pass and dominated the filter otherwise.
   */
  private static float pow48(float x) {
    float x2 = x * x;
    float x4 = x2 * x2;
    float x8 = x4 * x4;
    float x16 = x8 * x8;
    float x32 = x16 * x16;
    return x32 * x16;
  }

  /** Table size and range for the decay lookup; beyond the range the weight is negligible. */
  private static final int EXP_SAMPLES = 1024;

  private static final float EXP_RANGE = 12f;
  private static final float[] EXP_TABLE = new float[EXP_SAMPLES + 1];

  static {
    for (int i = 0; i <= EXP_SAMPLES; i++) {
      EXP_TABLE[i] = (float) Math.exp(-EXP_RANGE * i / EXP_SAMPLES);
    }
  }

  /** {@code exp(-t)} for non-negative t, by interpolated lookup. */
  private static float expNegative(float t) {
    if (t >= EXP_RANGE) {
      return 0f;
    }
    float scaled = t * (EXP_SAMPLES / EXP_RANGE);
    int index = (int) scaled;
    float fraction = scaled - index;
    return EXP_TABLE[index] + (EXP_TABLE[index + 1] - EXP_TABLE[index]) * fraction;
  }

  private static float luminance(float[] buffer, int pixel) {
    return 0.2126f * buffer[pixel * 3]
        + 0.7152f * buffer[pixel * 3 + 1]
        + 0.0722f * buffer[pixel * 3 + 2];
  }
}
