package frc.robot.utility.rendering;

import java.util.ArrayList;
import java.util.List;

/**
 * The light rig an FRC field is played under.
 *
 * <p>Competition fields are lit from a truss above the field perimeter rather than by a single
 * source, which is why real match footage has soft, overlapping shadows and almost no pure black.
 * Modelling the fixtures as rectangular area lights reproduces that directly: the penumbra comes
 * out of the geometry instead of being faked with a shadow bias.
 *
 * <p>Fixture count is a quality decision as much as a physical one. Every fixture is a shadow ray,
 * and six broad ones give the same soft overlapping light as a dozen narrow ones for half the
 * tracing, so the rig is deliberately coarser than a real truss.
 *
 * <p>The surrounding venue is folded into an analytic sky rather than modelled. Nothing in the
 * export extends past the driver stations, so without it every ray that leaves the field would come
 * back black. It is kept dim on purpose: an arena interior is much darker than the lit field, and
 * making the two similar is what makes a render look like fog instead of like a photograph.
 */
final class ArenaLighting {

  /**
   * One rectangular fixture.
   *
   * @param center fixture centre in field coordinates
   * @param axisU in-plane unit axis
   * @param axisV in-plane unit axis, perpendicular to {@code axisU}
   * @param halfU half extent along {@code axisU}, metres
   * @param halfV half extent along {@code axisV}, metres
   * @param normal emission direction, pointing at the field
   * @param radiance linear RGB radiance leaving the fixture
   */
  record AreaLight(
      float[] center,
      float[] axisU,
      float[] axisV,
      float halfU,
      float halfV,
      float[] normal,
      float[] radiance) {

    float area() {
      return 4f * halfU * halfV;
    }
  }

  /** Height of the lighting truss above the carpet. */
  private static final float TRUSS_HEIGHT = 8.2f;

  /** Fixtures sit outboard of the field edge, as they do on a real truss. */
  private static final float TRUSS_OUTBOARD = 1.9f;

  private static final int FIXTURES_PER_SIDE = 3;

  private static final float FIXTURE_HALF_LENGTH = 1.9f;

  private static final float FIXTURE_HALF_WIDTH = 0.75f;

  /**
   * Arena high-bay LEDs run cool white. Slightly blue-shifted from neutral, and bright enough that
   * the auto exposure settles near where a real camera sits indoors.
   */
  private static final float[] FIXTURE_RADIANCE = {62.0f, 63.4f, 67.0f};

  /**
   * Bounce off the house ceiling and the far walls, which keeps shadows from crushing to black.
   *
   * <p>Deliberately much darker than the lit field. What a camera sees over the field wall is an
   * unlit arena interior, and every step this gets brighter is a step toward the frame looking like
   * it was shot in fog.
   */
  private static final float[] ZENITH_RADIANCE = {0.074f, 0.080f, 0.098f};

  private static final float[] HORIZON_RADIANCE = {0.043f, 0.044f, 0.050f};

  /** Below the horizon a ray is looking at the surrounding floor, not at sky. */
  private static final float[] GROUND_RADIANCE = {0.030f, 0.028f, 0.026f};

  private final List<AreaLight> lights;

  private ArenaLighting(List<AreaLight> lights) {
    this.lights = lights;
  }

  /** Builds a symmetric truss rig sized to the field. */
  static ArenaLighting forField(double fieldLength, double fieldWidth) {
    return forField(fieldLength, fieldWidth, FIXTURES_PER_SIDE);
  }

  /** Builds a rig with an explicit fixture count, for measuring what shadow rays actually cost. */
  static ArenaLighting forField(double fieldLength, double fieldWidth, int fixturesPerSide) {
    List<AreaLight> lights = new ArrayList<>();
    float[] down = {0, 0, -1};

    for (int i = 0; i < fixturesPerSide; i++) {
      // Spread the fixtures evenly along the field, inset by half a spacing from each end.
      float x = (float) (fieldLength * (i + 0.5) / fixturesPerSide);
      for (int side = 0; side < 2; side++) {
        float y = side == 0 ? -TRUSS_OUTBOARD : (float) fieldWidth + TRUSS_OUTBOARD;
        lights.add(
            new AreaLight(
                new float[] {x, y, TRUSS_HEIGHT},
                new float[] {1, 0, 0},
                new float[] {0, 1, 0},
                FIXTURE_HALF_LENGTH,
                FIXTURE_HALF_WIDTH,
                down,
                FIXTURE_RADIANCE));
      }
    }
    return new ArenaLighting(lights);
  }

  List<AreaLight> lights() {
    return lights;
  }

  /**
   * Radiance arriving from the venue along a direction that hit nothing.
   *
   * @param out receives linear RGB
   */
  void sky(float dx, float dy, float dz, float[] out) {
    if (dz >= 0) {
      // Smoothstep rather than a straight lerp, so the horizon band does not read as a hard edge.
      float t = dz * dz * (3f - 2f * dz);
      out[0] = HORIZON_RADIANCE[0] + (ZENITH_RADIANCE[0] - HORIZON_RADIANCE[0]) * t;
      out[1] = HORIZON_RADIANCE[1] + (ZENITH_RADIANCE[1] - HORIZON_RADIANCE[1]) * t;
      out[2] = HORIZON_RADIANCE[2] + (ZENITH_RADIANCE[2] - HORIZON_RADIANCE[2]) * t;
    } else {
      float t = Math.min(1f, -dz * 3f);
      out[0] = HORIZON_RADIANCE[0] + (GROUND_RADIANCE[0] - HORIZON_RADIANCE[0]) * t;
      out[1] = HORIZON_RADIANCE[1] + (GROUND_RADIANCE[1] - HORIZON_RADIANCE[1]) * t;
      out[2] = HORIZON_RADIANCE[2] + (GROUND_RADIANCE[2] - HORIZON_RADIANCE[2]) * t;
    }
  }

  /**
   * Radiance for a ray that left the scene, including the fixtures themselves.
   *
   * <p>Only for paths that did not already account for the lights by sampling them. A diffuse
   * bounce uses {@link #sky} instead, because counting a fixture twice, once by sampling it and
   * again by happening to hit it, is what produces the isolated blown-out pixels that no amount of
   * filtering removes cleanly.
   */
  void escapedRadiance(float ox, float oy, float oz, float dx, float dy, float dz, float[] out) {
    for (AreaLight light : lights) {
      float denominator = light.normal()[0] * dx + light.normal()[1] * dy + light.normal()[2] * dz;
      if (denominator > -1e-6f) {
        continue; // travelling away from the emitting face
      }
      float toX = light.center()[0] - ox;
      float toY = light.center()[1] - oy;
      float toZ = light.center()[2] - oz;
      float distance =
          (light.normal()[0] * toX + light.normal()[1] * toY + light.normal()[2] * toZ)
              / denominator;
      if (distance <= 0) {
        continue;
      }
      float hitX = ox + dx * distance - light.center()[0];
      float hitY = oy + dy * distance - light.center()[1];
      float hitZ = oz + dz * distance - light.center()[2];
      float u = hitX * light.axisU()[0] + hitY * light.axisU()[1] + hitZ * light.axisU()[2];
      float v = hitX * light.axisV()[0] + hitY * light.axisV()[1] + hitZ * light.axisV()[2];
      if (Math.abs(u) <= light.halfU() && Math.abs(v) <= light.halfV()) {
        out[0] = light.radiance()[0];
        out[1] = light.radiance()[1];
        out[2] = light.radiance()[2];
        return;
      }
    }
    sky(dx, dy, dz, out);
  }

  /**
   * Average sky radiance over the hemisphere about a normal.
   *
   * <p>Used as the diffuse ambient term for rays that are not worth tracing, and as the base for
   * the specular environment approximation. Two lobe samples, one along the normal and one tilted
   * toward the horizon, are enough for a gradient this smooth.
   *
   * @param scratch caller-owned three-float scratch; this runs on every shaded hit, so it must not
   *     allocate
   */
  void ambient(float nx, float ny, float nz, float[] out, float[] scratch) {
    float[] up = out;
    float[] side = scratch;
    sky(nx, ny, nz, up);

    float hx = nx;
    float hy = ny;
    float hz = nz * 0.15f;
    float length = (float) Math.sqrt(hx * hx + hy * hy + hz * hz);
    if (length < 1e-6f) {
      hx = 1;
      hy = 0;
      hz = 0;
      length = 1;
    }
    sky(hx / length, hy / length, hz / length, side);
    for (int c = 0; c < 3; c++) {
      out[c] = 0.55f * up[c] + 0.45f * side[c];
    }
  }
}
