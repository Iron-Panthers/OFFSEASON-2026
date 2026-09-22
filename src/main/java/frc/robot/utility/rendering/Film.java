package frc.robot.utility.rendering;

import java.awt.image.BufferedImage;
import java.util.Random;

/**
 * Turns linear radiance into something a camera would actually have produced.
 *
 * <p>This stage is not decoration. A vision pipeline never sees radiance; it sees an
 * auto-exposed, tone mapped, vignetted, noisy, lens-distorted JPEG. Skipping it would hand a
 * detector cleaner data than it will ever get on the field, and a threshold tuned against clean
 * data is a threshold that fails at an event.
 *
 * <p>Exposure is metered and smoothed across frames the way a real auto-exposure loop behaves, so
 * driving from a bright open field into the shade under the trench produces the same lag and
 * overshoot the real camera does.
 */
final class Film {

  /** Target average scene luminance after exposure, the usual middle grey. */
  private static final float MIDDLE_GREY = 0.18f;

  /** How fast the exposure loop chases a change, per frame. */
  private static final float EXPOSURE_ADAPTION = 0.12f;

  /** Exposure is clamped so a frame of nothing but shadow does not blow the gain wide open. */
  private static final float MIN_EXPOSURE = 0.02f;

  private static final float MAX_EXPOSURE = 8.0f;

  /**
   * Electrons per unit of exposed radiance, which sets the photon shot noise floor. Chosen so that
   * a well-lit frame at unit gain lands around the signal to noise of an OV9281 indoors.
   */
  private static final float PHOTON_SCALE = 9000f;

  /** Read noise as a fraction of full scale, present even in the dark. */
  private static final float READ_NOISE = 0.0022f;

  /** Lateral chromatic aberration at the image corner, as a fraction of the radius. */
  private static final float CHROMATIC_ABERRATION = 0.0016f;

  /** Extra corner falloff beyond the natural cosine-fourth term. */
  private static final float VIGNETTE_STRENGTH = 0.32f;

  private final Random noise = new Random(0x5EED);
  private float smoothedExposure = 1f;
  private boolean metered;

  /**
   * Develops a frame.
   *
   * @param gain multiplies sensor noise; 1 is a well-exposed camera
   * @return an 8 bit sRGB image ready to encode
   */
  BufferedImage develop(Renderer.Frame frame, float gain) {
    int width = frame.width();
    int height = frame.height();

    float exposure = meter(frame);
    float[] exposed = applyExposureAndAberration(frame, exposure);

    BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
    float centerX = width * 0.5f;
    float centerY = height * 0.5f;
    float maxRadius = (float) Math.hypot(centerX, centerY);

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int pixel = y * width + x;

        float radius = (float) Math.hypot(x - centerX, y - centerY) / maxRadius;
        float falloff = vignette(radius);

        int rgb = 0;
        for (int c = 0; c < 3; c++) {
          float value = exposed[pixel * 3 + c] * falloff;
          value = addSensorNoise(value, gain);
          value = acesToneMap(value);
          int quantised = Math.round(linearToSrgb(value) * 255f);
          rgb = (rgb << 8) | Math.min(255, Math.max(0, quantised));
        }
        image.setRGB(x, y, rgb);
      }
    }
    return image;
  }

  /** Log-average luminance metering with temporal smoothing, as a real camera does it. */
  private float meter(Renderer.Frame frame) {
    int pixels = frame.width() * frame.height();
    // Every sixteenth pixel is plenty to estimate a scene average and keeps metering off the
    // critical path.
    int step = Math.max(1, pixels / 4096);

    double logSum = 0;
    int counted = 0;
    for (int i = 0; i < pixels; i += step) {
      float luminance =
          0.2126f * frame.color()[i * 3]
              + 0.7152f * frame.color()[i * 3 + 1]
              + 0.0722f * frame.color()[i * 3 + 2];
      logSum += Math.log(Math.max(1e-5f, luminance));
      counted++;
    }
    float average = counted == 0 ? MIDDLE_GREY : (float) Math.exp(logSum / counted);
    float target = Math.min(MAX_EXPOSURE, Math.max(MIN_EXPOSURE, MIDDLE_GREY / average));

    if (!metered) {
      smoothedExposure = target;
      metered = true;
    } else {
      smoothedExposure += (target - smoothedExposure) * EXPOSURE_ADAPTION;
    }
    return smoothedExposure;
  }

  /**
   * Scales by exposure and resamples the channels at slightly different radii.
   *
   * <p>Real lenses focus red and blue at marginally different magnifications, which shows up as
   * coloured fringing on high contrast edges, most visibly on the black and white boundary of an
   * AprilTag. Since that fringing is exactly what a corner detector has to work through, it is
   * worth reproducing rather than assuming away.
   */
  private static float[] applyExposureAndAberration(Renderer.Frame frame, float exposure) {
    int width = frame.width();
    int height = frame.height();
    float[] source = frame.color();
    float[] out = new float[source.length];

    float centerX = width * 0.5f;
    float centerY = height * 0.5f;
    float[] channelScale = {1f + CHROMATIC_ABERRATION, 1f, 1f - CHROMATIC_ABERRATION};

    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int pixel = y * width + x;
        for (int c = 0; c < 3; c++) {
          float sampleX = centerX + (x - centerX) * channelScale[c];
          float sampleY = centerY + (y - centerY) * channelScale[c];
          out[pixel * 3 + c] = bilinear(source, width, height, sampleX, sampleY, c) * exposure;
        }
      }
    }
    return out;
  }

  private static float bilinear(
      float[] buffer, int width, int height, float x, float y, int channel) {
    float clampedX = Math.min(width - 1.001f, Math.max(0f, x));
    float clampedY = Math.min(height - 1.001f, Math.max(0f, y));
    int x0 = (int) clampedX;
    int y0 = (int) clampedY;
    int x1 = Math.min(width - 1, x0 + 1);
    int y1 = Math.min(height - 1, y0 + 1);
    float fx = clampedX - x0;
    float fy = clampedY - y0;

    float top =
        buffer[(y0 * width + x0) * 3 + channel] * (1 - fx)
            + buffer[(y0 * width + x1) * 3 + channel] * fx;
    float bottom =
        buffer[(y1 * width + x0) * 3 + channel] * (1 - fx)
            + buffer[(y1 * width + x1) * 3 + channel] * fx;
    return top * (1 - fy) + bottom * fy;
  }

  /** Natural cosine-fourth falloff plus a little extra, as a wide lens on a small sensor shows. */
  private static float vignette(float normalisedRadius) {
    float cosine = 1f / (1f + normalisedRadius * normalisedRadius);
    float natural = cosine * cosine;
    return 1f - VIGNETTE_STRENGTH * (1f - natural);
  }

  /**
   * Photon shot noise plus read noise.
   *
   * <p>Shot noise scales with the square root of the signal, so bright areas are proportionally
   * cleaner than dark ones. That asymmetry is why a detector that works on a brightly lit tag can
   * still fail on one in shadow, and a fixed-sigma noise model would hide it.
   */
  private float addSensorNoise(float value, float gain) {
    if (gain <= 0f) {
      return value;
    }
    float signal = Math.max(0f, value);
    float shot = (float) Math.sqrt(signal / PHOTON_SCALE) * gain;
    float total = shot + READ_NOISE * gain;
    // nextGaussian is synchronised on the Random instance, but developing a frame is single
    // threaded and takes a fraction of the trace, so contention never arises.
    return Math.max(0f, signal + (float) noise.nextGaussian() * total);
  }

  /** Narkowicz's fit to the ACES filmic curve. */
  private static float acesToneMap(float x) {
    float a = 2.51f;
    float b = 0.03f;
    float c = 2.43f;
    float d = 0.59f;
    float e = 0.14f;
    float result = (x * (a * x + b)) / (x * (c * x + d) + e);
    return Math.min(1f, Math.max(0f, result));
  }

  private static float linearToSrgb(float channel) {
    return channel <= 0.0031308f
        ? channel * 12.92f
        : 1.055f * (float) Math.pow(channel, 1.0 / 2.4) - 0.055f;
  }
}
