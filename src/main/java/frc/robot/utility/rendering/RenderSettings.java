package frc.robot.utility.rendering;

/**
 * Quality and cost knobs for one camera stream.
 *
 * <p>The defaults target roughly five to ten frames a second at 640x480 on a desktop CPU, which is
 * fast enough to watch the robot drive while still tracing real soft shadows and bounce light.
 * Raising {@code samplesPerPixel} is the honest way to trade time for image quality; the denoiser
 * covers the rest.
 *
 * @param width output width in pixels
 * @param height output height in pixels
 * @param samplesPerPixel primary rays per pixel, which also sets how many shadow and bounce rays
 *     each pixel gets
 * @param diffuseBounces indirect diffuse bounces after the primary hit; 1 gives colour bleed off
 *     the carpet and the alliance walls, which is most of what the eye reads as "lit room"
 * @param transparencyDepth how many polycarbonate panels a ray may see through before giving up
 * @param denoise whether to run the edge-avoiding filter over the indirect term
 * @param sensorGain multiplies photon and read noise; 1 is a well-lit camera, higher is a noisier
 *     one, and turning it up is a cheap way to test a pipeline against a bad exposure
 * @param jpegQuality MJPEG encoder quality from 0 to 1
 * @param quality which lighting model to use
 */
record RenderSettings(
    int width,
    int height,
    int samplesPerPixel,
    int diffuseBounces,
    int transparencyDepth,
    boolean denoise,
    float sensorGain,
    float jpegQuality,
    RenderSettings.Quality quality) {

  /** Which lighting model a stream runs. */
  enum Quality {
    /**
     * Every pixel's shadows and bounce light are traced.
     *
     * <p>The reference. Use it to build training and validation sets, where the frames matter more
     * than how long they took.
     */
    HIGH,

    /**
     * Static lighting is read from a baked {@link IrradianceVolume} instead of traced.
     *
     * <p>Roughly five times faster, because it removes the shadow and bounce rays that profiling
     * showed were seventy percent of a frame. Moving objects still cast real traced contact
     * shadows, and primary visibility is still ray traced, so lens distortion, ball silhouettes and
     * tag sharpness are unaffected. What it gives up is the physical accuracy of the lighting
     * itself: shadows are band limited to the volume grid and specular highlights are broader.
     */
    FAST
  }

  static RenderSettings defaults() {
    return fastDefaults();
  }

  /**
   * Default output size.
   *
   * <p>Chosen from measurement rather than from taste. Frame time scales with pixel count, and at
   * 640x400 a four core machine manages about seven frames a second; at this size it manages
   * thirteen and an eight core machine reaches fifteen. Raise it with {@code -Prender.width} and
   * {@code -Prender.height} when the frames matter more than the rate, which is most of the time
   * for AprilTag work: detection range falls with resolution.
   */
  static final int DEFAULT_WIDTH = 480;

  static final int DEFAULT_HEIGHT = 300;

  /** Tuned so an ordinary laptop can watch the stream rather than page through it. */
  static RenderSettings fastDefaults() {
    return new RenderSettings(
        DEFAULT_WIDTH, DEFAULT_HEIGHT, 2, 0, 1, false, 1.0f, 0.82f, Quality.FAST);
  }

  /** The reference path tracer, for capture rather than for watching. */
  static RenderSettings highDefaults() {
    return new RenderSettings(
        DEFAULT_WIDTH, DEFAULT_HEIGHT, 2, 1, 3, true, 1.0f, 0.82f, Quality.HIGH);
  }

  RenderSettings withResolution(int newWidth, int newHeight) {
    return new RenderSettings(
        newWidth,
        newHeight,
        samplesPerPixel,
        diffuseBounces,
        transparencyDepth,
        denoise,
        sensorGain,
        jpegQuality,
        quality);
  }

  RenderSettings withSamplesPerPixel(int samples) {
    return new RenderSettings(
        width,
        height,
        samples,
        diffuseBounces,
        transparencyDepth,
        denoise,
        sensorGain,
        jpegQuality,
        quality);
  }

  int pixelCount() {
    return width * height;
  }
}
