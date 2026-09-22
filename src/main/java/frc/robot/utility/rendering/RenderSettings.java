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
 */
record RenderSettings(
    int width,
    int height,
    int samplesPerPixel,
    int diffuseBounces,
    int transparencyDepth,
    boolean denoise,
    float sensorGain,
    float jpegQuality) {

  static RenderSettings defaults() {
    return new RenderSettings(640, 480, 2, 1, 3, true, 1.0f, 0.82f);
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
        jpegQuality);
  }

  RenderSettings withSamplesPerPixel(int samples) {
    return new RenderSettings(
        width, height, samples, diffuseBounces, transparencyDepth, denoise, sensorGain, jpegQuality);
  }

  int pixelCount() {
    return width * height;
  }
}
