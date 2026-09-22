package frc.robot.utility.rendering;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;

/**
 * The ray tracer.
 *
 * <p>Primary visibility, shadows, ambient occlusion and one diffuse bounce all come from tracing
 * real rays against the field tree. Nothing here is a screen-space trick, which is the reason the
 * output holds up as training and test data: contact shadows sit where the geometry actually
 * touches, the carpet picks up colour from the alliance walls beside it, and a ball half under the
 * trench is lit the way a ball half under the trench is lit.
 *
 * <p>Output is linear high dynamic range. Exposure, tone mapping and sensor behaviour belong to
 * {@link Film}, so the physically meaningful part of the pipeline never has to know what a display
 * expects.
 */
final class Renderer {

  private static final float PI = (float) Math.PI;

  /** Nothing on a 16 m field is further away than this, so a ray that gets here has escaped. */
  private static final float FAR = 120f;

  /** Lifts secondary rays off the surface they start on, in metres. */
  private static final float SHADOW_BIAS = 2e-4f;

  /**
   * Deep enough for the primary ray, the panels it may see through, and a diffuse bounce, with
   * room left over. Recursion is capped by this rather than by the settings alone.
   */
  private static final int MAX_LEVELS = 8;

  /**
   * How much of the environment a surface still sees when its bounce ray was blocked.
   *
   * <p>A blocked bounce ray says the hemisphere is occluded for diffuse, but a glossy surface
   * reflects a narrow lobe that often escapes anyway, so specular is dimmed rather than cut.
   */
  private static final float OCCLUDED_SPECULAR_FRACTION = 0.35f;

  private final FieldScene scene;
  private final ArenaLighting lighting;
  private final ExecutorService workers;
  private final int threadCount;

  Renderer(FieldScene scene, ArenaLighting lighting, ExecutorService workers, int threadCount) {
    this.scene = scene;
    this.lighting = lighting;
    this.workers = workers;
    this.threadCount = threadCount;
  }

  /**
   * One rendered frame in linear light, plus the buffers the denoiser needs.
   *
   * @param color linear RGB radiance, three floats per pixel
   * @param albedo surface colour under the shading, used to demodulate before filtering
   * @param normal shading normal, used as a denoiser edge guide
   * @param depth distance to the first hit, infinite where the ray escaped
   */
  record Frame(int width, int height, float[] color, float[] albedo, float[] normal, float[] depth) {

    static Frame allocate(int width, int height) {
      int pixels = width * height;
      return new Frame(
          width,
          height,
          new float[pixels * 3],
          new float[pixels * 3],
          new float[pixels * 3],
          new float[pixels]);
    }
  }

  /**
   * Scratch for one recursion level.
   *
   * <p>Shading recurses, so anything a shading call writes has to belong to its own level. Sharing
   * one buffer across levels is the kind of bug that shows up as a faint wrong-coloured haze rather
   * than as a crash, so the storage is split up front.
   */
  private static final class Level {
    final ShadePoint point = new ShadePoint();
    final float[] color = new float[3];
    final float[] albedo = new float[3];
    final float[] direction = new float[3];
    final float[] incoming = new float[3];
    final float[] brdf = new float[3];
    final float[] skySharp = new float[3];
    final float[] skyBroad = new float[3];
    final float[] skyScratch = new float[3];

    /**
     * What a transparent hit sees behind itself.
     *
     * <p>Held here rather than read straight out of the next level, because shading this hit
     * launches a bounce ray that reuses the next level and would overwrite the answer before it
     * could be mixed in.
     */
    final float[] behindColor = new float[3];

    final float[] behindAlbedo = new float[3];
  }

  /** Everything one render thread needs, allocated per band rather than per ray. */
  private static final class Context {
    final Bvh.Scratch bvhScratch = new Bvh.Scratch();
    final DynamicScene.Query query = new DynamicScene.Query();
    final ShadePoint movingHit = new ShadePoint();
    final ShadePoint occlusionScratch = new ShadePoint();
    final Level[] levels = new Level[MAX_LEVELS];
    final float[] rayDirection = new float[3];

    DynamicScene dynamic;
    int random;

    /** Which of the pixel's samples is being traced, used to stratify light sampling. */
    int sampleIndex;

    int sampleCount;

    Context() {
      for (int i = 0; i < levels.length; i++) {
        levels[i] = new Level();
      }
    }
  }

  /**
   * Renders one frame.
   *
   * @param frameIndex mixed into the sampling pattern, so a given frame index always renders
   *     identically and a logged run can be reproduced exactly
   */
  Frame render(
      RenderCamera camera, RenderSettings settings, DynamicScene dynamic, long frameIndex) {

    Frame frame = Frame.allocate(settings.width(), settings.height());
    int bandHeight = Math.max(1, settings.height() / (threadCount * 4));

    List<Future<?>> pending = new ArrayList<>();
    for (int startRow = 0; startRow < settings.height(); startRow += bandHeight) {
      int from = startRow;
      int to = Math.min(settings.height(), startRow + bandHeight);
      pending.add(
          workers.submit(
              () -> {
                Context context = new Context();
                context.dynamic = dynamic;
                renderBand(context, camera, settings, frame, from, to, frameIndex);
              }));
    }
    for (Future<?> task : pending) {
      try {
        task.get();
      } catch (InterruptedException interrupted) {
        Thread.currentThread().interrupt();
        throw new IllegalStateException("Render interrupted", interrupted);
      } catch (Exception failed) {
        throw new IllegalStateException("Render worker failed", failed);
      }
    }
    return frame;
  }

  private void renderBand(
      Context context,
      RenderCamera camera,
      RenderSettings settings,
      Frame frame,
      int fromRow,
      int toRow,
      long frameIndex) {

    int width = settings.width();
    int samples = settings.samplesPerPixel();
    float inverseSamples = 1f / samples;

    float[] accumulated = new float[3];
    float[] albedoSum = new float[3];
    float[] normalSum = new float[3];

    for (int y = fromRow; y < toRow; y++) {
      for (int x = 0; x < width; x++) {
        int pixel = y * width + x;
        accumulated[0] = accumulated[1] = accumulated[2] = 0;
        albedoSum[0] = albedoSum[1] = albedoSum[2] = 0;
        normalSum[0] = normalSum[1] = normalSum[2] = 0;
        float depthSum = 0;
        int depthSamples = 0;

        for (int sample = 0; sample < samples; sample++) {
          context.random = seed(pixel, sample, frameIndex);
          context.sampleIndex = sample;
          context.sampleCount = samples;

          // Jitter inside the pixel for anti-aliasing. Tag edges alias badly without it, and a
          // detector fed aliased tag corners behaves differently from one fed real ones.
          double jitterX = x + (samples == 1 ? 0.5 : nextFloat(context));
          double jitterY = y + (samples == 1 ? 0.5 : nextFloat(context));
          camera.ray(jitterX, jitterY, context.rayDirection);

          Level level = context.levels[0];
          boolean hit =
              radiance(
                  context,
                  (float) camera.originX(),
                  (float) camera.originY(),
                  (float) camera.originZ(),
                  context.rayDirection[0],
                  context.rayDirection[1],
                  context.rayDirection[2],
                  settings.transparencyDepth(),
                  settings.diffuseBounces(),
                  0,
                  true);

          accumulated[0] += level.color[0];
          accumulated[1] += level.color[1];
          accumulated[2] += level.color[2];
          if (hit) {
            albedoSum[0] += level.albedo[0];
            albedoSum[1] += level.albedo[1];
            albedoSum[2] += level.albedo[2];
            normalSum[0] += level.point.normalX;
            normalSum[1] += level.point.normalY;
            normalSum[2] += level.point.normalZ;
            depthSum += level.point.distance;
            depthSamples++;
          } else {
            // Sky is its own albedo, so demodulation leaves it untouched.
            albedoSum[0] += 1f;
            albedoSum[1] += 1f;
            albedoSum[2] += 1f;
          }
        }

        int base = pixel * 3;
        frame.color()[base] = accumulated[0] * inverseSamples;
        frame.color()[base + 1] = accumulated[1] * inverseSamples;
        frame.color()[base + 2] = accumulated[2] * inverseSamples;
        frame.albedo()[base] = albedoSum[0] * inverseSamples;
        frame.albedo()[base + 1] = albedoSum[1] * inverseSamples;
        frame.albedo()[base + 2] = albedoSum[2] * inverseSamples;

        float normalLength =
            (float)
                Math.sqrt(
                    normalSum[0] * normalSum[0]
                        + normalSum[1] * normalSum[1]
                        + normalSum[2] * normalSum[2]);
        if (normalLength > 1e-6f) {
          frame.normal()[base] = normalSum[0] / normalLength;
          frame.normal()[base + 1] = normalSum[1] / normalLength;
          frame.normal()[base + 2] = normalSum[2] / normalLength;
        }
        frame.depth()[pixel] =
            depthSamples == 0 ? Float.POSITIVE_INFINITY : depthSum / depthSamples;
      }
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Tracing
  // ---------------------------------------------------------------------------------------------

  /**
   * Radiance arriving along a ray, written into {@code context.levels[level].color}.
   *
   * @param transparencyLeft how many more transparent panels this ray may pass through
   * @param bouncesLeft how many more diffuse bounces are allowed
   * @param level recursion level, which selects the scratch storage
   * @param countEmitters whether a ray that leaves the scene may see the light fixtures. False on
   *     paths that already accounted for the lights by sampling them, since counting a fixture
   *     both ways produces blown-out single pixels that no filter removes cleanly.
   * @return true if the ray hit geometry rather than escaping to sky
   */
  private boolean radiance(
      Context context,
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      int transparencyLeft,
      int bouncesLeft,
      int level,
      boolean countEmitters) {

    Level current = context.levels[level];
    float[] out = current.color;
    ShadePoint point = current.point;

    if (!trace(context, ox, oy, oz, dx, dy, dz, FAR, point)) {
      if (countEmitters) {
        lighting.escapedRadiance(ox, oy, oz, dx, dy, dz, out);
      } else {
        lighting.sky(dx, dy, dz, out);
      }
      // Sky is its own albedo, so demodulating by it leaves the radiance untouched.
      current.albedo[0] = 1f;
      current.albedo[1] = 1f;
      current.albedo[2] = 1f;
      return false;
    }
    point.faceForward(dx, dy, dz);

    float[] albedo = current.albedo;
    surfaceAlbedo(point, albedo);

    Surface surface = point.surface;
    boolean canRecurse = level + 1 < MAX_LEVELS;

    if (surface.isTransparent() && transparencyLeft > 0 && canRecurse) {
      // Step past the panel and keep going, then mix by opacity. The field walls are mostly clear,
      // so most of what the camera sees through them has to survive.
      radiance(
          context,
          point.x + dx * SHADOW_BIAS * 4f,
          point.y + dy * SHADOW_BIAS * 4f,
          point.z + dz * SHADOW_BIAS * 4f,
          dx,
          dy,
          dz,
          transparencyLeft - 1,
          bouncesLeft,
          level + 1,
          countEmitters);
      System.arraycopy(context.levels[level + 1].color, 0, current.behindColor, 0, 3);
      System.arraycopy(context.levels[level + 1].albedo, 0, current.behindAlbedo, 0, 3);

      shadeOpaque(context, level, -dx, -dy, -dz, bouncesLeft);

      float opacity = surface.alpha();
      for (int c = 0; c < 3; c++) {
        out[c] = out[c] * opacity + current.behindColor[c] * (1f - opacity);
        // The denoiser divides by this, so it has to describe what the pixel actually shows.
        // Leaving it as the panel's own colour makes everything seen through the field walls
        // demodulate against the wrong surface and keep its noise.
        albedo[c] = albedo[c] * opacity + current.behindAlbedo[c] * (1f - opacity);
      }
      return true;
    }

    shadeOpaque(context, level, -dx, -dy, -dz, bouncesLeft);
    return true;
  }

  /** Nearest hit across the static field and everything that moves. */
  private boolean trace(
      Context context,
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      float maxDistance,
      ShadePoint out) {

    boolean hitField =
        scene.bvh.intersect(scene.soup, ox, oy, oz, dx, dy, dz, maxDistance, context.bvhScratch);
    float nearest = hitField ? context.bvhScratch.hit.distance : maxDistance;

    ShadePoint moving = context.movingHit;
    float movingDistance =
        context.dynamic.intersect(ox, oy, oz, dx, dy, dz, nearest, context.query, moving);

    if (movingDistance < nearest) {
      out.x = moving.x;
      out.y = moving.y;
      out.z = moving.z;
      out.normalX = moving.normalX;
      out.normalY = moving.normalY;
      out.normalZ = moving.normalZ;
      out.surface = moving.surface;
      out.distance = movingDistance;
      return true;
    }
    if (!hitField) {
      return false;
    }

    Bvh.Hit hit = context.bvhScratch.hit;
    out.distance = hit.distance;
    out.x = ox + dx * hit.distance;
    out.y = oy + dy * hit.distance;
    out.z = oz + dz * hit.distance;
    out.surface = scene.soup.surfaces.get(scene.soup.surfaceIds[hit.triangle]);
    interpolateNormal(hit, out);
    return true;
  }

  private void interpolateNormal(Bvh.Hit hit, ShadePoint out) {
    float[] normals = scene.soup.normals;
    int base = hit.triangle * 3;
    int a = scene.soup.indices[base] * 3;
    int b = scene.soup.indices[base + 1] * 3;
    int c = scene.soup.indices[base + 2] * 3;
    float w = 1f - hit.u - hit.v;

    float nx = normals[a] * w + normals[b] * hit.u + normals[c] * hit.v;
    float ny = normals[a + 1] * w + normals[b + 1] * hit.u + normals[c + 1] * hit.v;
    float nz = normals[a + 2] * w + normals[b + 2] * hit.u + normals[c + 2] * hit.v;
    float length = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (length < 1e-9f) {
      length = 1f;
    }
    out.normalX = nx / length;
    out.normalY = ny / length;
    out.normalZ = nz / length;
  }

  // ---------------------------------------------------------------------------------------------
  // Shading
  // ---------------------------------------------------------------------------------------------

  /** Base colour at a hit, including whichever procedural pattern the surface carries. */
  private void surfaceAlbedo(ShadePoint point, float[] out) {
    Surface surface = point.surface;
    out[0] = surface.red();
    out[1] = surface.green();
    out[2] = surface.blue();

    switch (surface.pattern()) {
      case CARPET -> {
        // Two scales: the loop pile itself, and the broad wear and lighting mottle that makes a
        // real floor read as a floor rather than as a painted plane.
        float pile = Noise.fractal(point.x * 320f, point.y * 320f, 2);
        float mottle = Noise.fractal(point.x * 6.5f, point.y * 6.5f, 3);
        float tint = 1f + 0.42f * (pile - 0.5f) + 0.26f * (mottle - 0.5f);
        out[0] *= tint;
        out[1] *= tint;
        out[2] *= tint;

        // Perturb the normal along the pile gradient, so grazing light catches the weave.
        float step = 0.0035f;
        float gradientX = Noise.fractal((point.x + step) * 320f, point.y * 320f, 2) - pile;
        float gradientY = Noise.fractal(point.x * 320f, (point.y + step) * 320f, 2) - pile;
        float bump = 0.55f;
        float nx = point.normalX - gradientX * bump;
        float ny = point.normalY - gradientY * bump;
        float nz = point.normalZ;
        float length = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
        point.normalX = nx / length;
        point.normalY = ny / length;
        point.normalZ = nz / length;
      }
      case APRIL_TAG -> {
        float reflectance = surface.decal().sample(point.x, point.y, point.z);
        out[0] = reflectance;
        out[1] = reflectance;
        out[2] = reflectance;
      }
      case NONE -> {
        // Flat base colour.
      }
    }
  }

  /**
   * Full shading for an opaque hit: emission, direct light from the truss, one diffuse bounce, and
   * a specular environment term.
   */
  private void shadeOpaque(
      Context context, int level, float viewX, float viewY, float viewZ, int bouncesLeft) {

    Level current = context.levels[level];
    ShadePoint point = current.point;
    float[] albedo = current.albedo;
    float[] out = current.color;

    Surface surface = point.surface;
    float roughness = Materials.clampRoughness(surface.roughness());
    float metallic = surface.metallic();

    out[0] = albedo[0] * surface.emissive();
    out[1] = albedo[1] * surface.emissive();
    out[2] = albedo[2] * surface.emissive();

    // The primary hit is what the eye and the detector actually look at, so it gets a shadow ray
    // to every fixture. A bounce is about to be filtered anyway, so one fixture will do.
    directLight(context, level, viewX, viewY, viewZ, roughness, metallic, level == 0);

    // One cosine-weighted bounce serves as both the ambient term and the occlusion test: if it
    // escapes it brings back sky, and if it does not it brings back whatever it landed on.
    float[] incoming = current.incoming;
    boolean escaped = true;
    if (bouncesLeft > 0 && level + 1 < MAX_LEVELS) {
      cosineHemisphere(context, point.normalX, point.normalY, point.normalZ, current.direction);
      boolean bounceHit =
          radiance(
              context,
              point.x + point.normalX * SHADOW_BIAS,
              point.y + point.normalY * SHADOW_BIAS,
              point.z + point.normalZ * SHADOW_BIAS,
              current.direction[0],
              current.direction[1],
              current.direction[2],
              0,
              bouncesLeft - 1,
              level + 1,
              false);
      escaped = !bounceHit;
      float[] bounce = context.levels[level + 1].color;
      incoming[0] = bounce[0];
      incoming[1] = bounce[1];
      incoming[2] = bounce[2];
    } else {
      lighting.ambient(
          point.normalX, point.normalY, point.normalZ, incoming, current.skyScratch);
    }

    // Cosine sampling cancels the cosine and the pi, leaving just the albedo.
    float diffuseWeight = 1f - metallic;
    out[0] += albedo[0] * diffuseWeight * incoming[0];
    out[1] += albedo[1] * diffuseWeight * incoming[1];
    out[2] += albedo[2] * diffuseWeight * incoming[2];

    environmentSpecular(context, level, viewX, viewY, viewZ, roughness, metallic, escaped);
  }

  /**
   * Accumulates direct light from the truss.
   *
   * @param allFixtures sample every fixture, rather than picking one and weighting it back up.
   *     Picking one is an eight-fold variance amplifier, which shows up as salt-and-pepper noise
   *     that survives filtering, so the primary hit pays for the full set.
   */
  private void directLight(
      Context context,
      int level,
      float viewX,
      float viewY,
      float viewZ,
      float roughness,
      float metallic,
      boolean allFixtures) {

    List<ArenaLighting.AreaLight> lights = lighting.lights();
    if (lights.isEmpty()) {
      return;
    }
    if (allFixtures) {
      for (int i = 0; i < lights.size(); i++) {
        sampleFixture(context, level, viewX, viewY, viewZ, roughness, metallic, lights.get(i), 1f);
      }
    } else {
      int index = (nextRandom(context) >>> 1) % lights.size();
      sampleFixture(
          context,
          level,
          viewX,
          viewY,
          viewZ,
          roughness,
          metallic,
          lights.get(index),
          lights.size());
    }
  }

  /**
   * Traces one shadow ray to a point on one fixture.
   *
   * @param weight scales the result back up when only a subset of fixtures was sampled
   */
  private void sampleFixture(
      Context context,
      int level,
      float viewX,
      float viewY,
      float viewZ,
      float roughness,
      float metallic,
      ArenaLighting.AreaLight light,
      float weight) {

    Level current = context.levels[level];
    ShadePoint point = current.point;
    float[] albedo = current.albedo;
    float[] out = current.color;

    // Stratify across the pixel's samples so two samples of the same fixture cannot land in the
    // same corner of it.
    int grid = Math.max(1, (int) Math.ceil(Math.sqrt(context.sampleCount)));
    int cell = context.sampleIndex % (grid * grid);
    float u = ((cell % grid) + nextFloat(context)) / grid * 2f - 1f;
    float v = ((cell / grid) + nextFloat(context)) / grid * 2f - 1f;

    float sampleX =
        light.center()[0]
            + light.axisU()[0] * u * light.halfU()
            + light.axisV()[0] * v * light.halfV();
    float sampleY =
        light.center()[1]
            + light.axisU()[1] * u * light.halfU()
            + light.axisV()[1] * v * light.halfV();
    float sampleZ =
        light.center()[2]
            + light.axisU()[2] * u * light.halfU()
            + light.axisV()[2] * v * light.halfV();

    float toX = sampleX - point.x;
    float toY = sampleY - point.y;
    float toZ = sampleZ - point.z;
    float distanceSquared = toX * toX + toY * toY + toZ * toZ;
    float distance = (float) Math.sqrt(distanceSquared);
    if (distance < 1e-4f) {
      return;
    }
    float lightX = toX / distance;
    float lightY = toY / distance;
    float lightZ = toZ / distance;

    float normalDotLight = point.normalX * lightX + point.normalY * lightY + point.normalZ * lightZ;
    if (normalDotLight <= 0) {
      return;
    }
    float cosLight =
        -(light.normal()[0] * lightX + light.normal()[1] * lightY + light.normal()[2] * lightZ);
    if (cosLight <= 0) {
      return;
    }

    float originX = point.x + point.normalX * SHADOW_BIAS;
    float originY = point.y + point.normalY * SHADOW_BIAS;
    float originZ = point.z + point.normalZ * SHADOW_BIAS;
    float reach = distance - SHADOW_BIAS * 4f;

    if (context.dynamic.occluded(
        originX,
        originY,
        originZ,
        lightX,
        lightY,
        lightZ,
        reach,
        context.query,
        context.occlusionScratch)) {
      return;
    }
    float visibility =
        scene.bvh.transmittance(
            scene.soup,
            originX,
            originY,
            originZ,
            lightX,
            lightY,
            lightZ,
            reach,
            context.bvhScratch);
    if (visibility <= 0f) {
      return;
    }

    // Area measure to solid angle, times the fraction of fixtures actually sampled.
    float geometry = cosLight * light.area() / distanceSquared * weight * visibility;

    float[] brdf = current.brdf;
    evaluateBrdf(
        albedo,
        metallic,
        roughness,
        point,
        viewX,
        viewY,
        viewZ,
        lightX,
        lightY,
        lightZ,
        normalDotLight,
        brdf);

    out[0] += light.radiance()[0] * brdf[0] * normalDotLight * geometry;
    out[1] += light.radiance()[1] * brdf[1] * normalDotLight * geometry;
    out[2] += light.radiance()[2] * brdf[2] * normalDotLight * geometry;
  }

  /** Cook-Torrance GGX plus a Lambert diffuse lobe, returned as a single RGB weight. */
  private static void evaluateBrdf(
      float[] albedo,
      float metallic,
      float roughness,
      ShadePoint point,
      float viewX,
      float viewY,
      float viewZ,
      float lightX,
      float lightY,
      float lightZ,
      float normalDotLight,
      float[] out) {

    float halfX = viewX + lightX;
    float halfY = viewY + lightY;
    float halfZ = viewZ + lightZ;
    float halfLength = (float) Math.sqrt(halfX * halfX + halfY * halfY + halfZ * halfZ);
    if (halfLength < 1e-8f) {
      out[0] = out[1] = out[2] = 0;
      return;
    }
    halfX /= halfLength;
    halfY /= halfLength;
    halfZ /= halfLength;

    float normalDotView =
        Math.max(1e-4f, point.normalX * viewX + point.normalY * viewY + point.normalZ * viewZ);
    float normalDotHalf =
        Math.max(0f, point.normalX * halfX + point.normalY * halfY + point.normalZ * halfZ);
    float viewDotHalf = Math.max(0f, viewX * halfX + viewY * halfY + viewZ * halfZ);

    float alpha = roughness * roughness;
    float alphaSquared = alpha * alpha;
    float denominator = normalDotHalf * normalDotHalf * (alphaSquared - 1f) + 1f;
    float distribution = alphaSquared / (PI * denominator * denominator);

    // Height correlated Smith visibility, which already carries the 1 / (4 NdotL NdotV).
    float lambdaView =
        normalDotLight
            * (float) Math.sqrt(normalDotView * normalDotView * (1f - alphaSquared) + alphaSquared);
    float lambdaLight =
        normalDotView
            * (float)
                Math.sqrt(normalDotLight * normalDotLight * (1f - alphaSquared) + alphaSquared);
    float visibility = 0.5f / Math.max(1e-6f, lambdaView + lambdaLight);
    float specularStrength = distribution * visibility;

    float fresnelScale = (float) Math.pow(1.0 - viewDotHalf, 5.0);
    for (int c = 0; c < 3; c++) {
      // Dielectrics reflect about four percent head on; metals reflect their own colour.
      float f0 = 0.04f + (albedo[c] - 0.04f) * metallic;
      float fresnel = f0 + (1f - f0) * fresnelScale;
      float diffuse = albedo[c] * (1f - metallic) * (1f - fresnel) / PI;
      out[c] = diffuse + specularStrength * fresnel;
    }
  }

  /**
   * Specular reflection of the environment, using the split sum approximation rather than tracing
   * glossy rays.
   *
   * <p>Nothing on the field is a mirror, so a narrow analytic lobe is indistinguishable from a
   * traced one at a fraction of the cost, and it is noise free. The environment BRDF term is
   * Karis's mobile fit to the split sum integral.
   */
  private void environmentSpecular(
      Context context,
      int level,
      float viewX,
      float viewY,
      float viewZ,
      float roughness,
      float metallic,
      boolean unoccluded) {

    Level current = context.levels[level];
    ShadePoint point = current.point;
    float[] albedo = current.albedo;
    float[] out = current.color;

    float normalDotView =
        Math.max(1e-4f, point.normalX * viewX + point.normalY * viewY + point.normalZ * viewZ);
    float reflectX = 2f * normalDotView * point.normalX - viewX;
    float reflectY = 2f * normalDotView * point.normalY - viewY;
    float reflectZ = 2f * normalDotView * point.normalZ - viewZ;

    float[] sharp = current.skySharp;
    float[] broad = current.skyBroad;
    lighting.sky(reflectX, reflectY, reflectZ, sharp);
    lighting.ambient(point.normalX, point.normalY, point.normalZ, broad, current.skyScratch);

    float rx = 1f - roughness;
    float ry = 0.0425f - 0.0275f * roughness;
    float rz = 1.04f - 0.572f * roughness;
    float rw = 0.022f * roughness - 0.04f;
    float a004 =
        Math.min(rx * rx, (float) Math.pow(2.0, -9.28 * normalDotView)) * rx + ry;
    float scale = -1.04f * a004 + rz;
    float bias = 1.04f * a004 + rw;

    float occlusion = unoccluded ? 1f : OCCLUDED_SPECULAR_FRACTION;
    for (int c = 0; c < 3; c++) {
      float prefiltered = sharp[c] * (1f - roughness) + broad[c] * roughness;
      float f0 = 0.04f + (albedo[c] - 0.04f) * metallic;
      out[c] += prefiltered * (f0 * scale + bias) * occlusion;
    }
  }

  // ---------------------------------------------------------------------------------------------
  // Sampling
  // ---------------------------------------------------------------------------------------------

  /** Cosine-weighted direction in the hemisphere about a normal. */
  private static void cosineHemisphere(
      Context context, float nx, float ny, float nz, float[] out) {
    float r1 = nextFloat(context);
    float r2 = nextFloat(context);
    float radius = (float) Math.sqrt(r1);
    float phi = 2f * PI * r2;
    float localX = radius * (float) Math.cos(phi);
    float localY = radius * (float) Math.sin(phi);
    float localZ = (float) Math.sqrt(Math.max(0f, 1f - r1));

    // Branchless orthonormal basis, Duff et al.
    float sign = nz >= 0f ? 1f : -1f;
    float a = -1f / (sign + nz);
    float b = nx * ny * a;
    float tangentX = 1f + sign * nx * nx * a;
    float tangentY = sign * b;
    float tangentZ = -sign * nx;
    float bitangentX = b;
    float bitangentY = sign + ny * ny * a;
    float bitangentZ = -ny;

    out[0] = tangentX * localX + bitangentX * localY + nx * localZ;
    out[1] = tangentY * localX + bitangentY * localY + ny * localZ;
    out[2] = tangentZ * localX + bitangentZ * localY + nz * localZ;
  }

  /**
   * Seeds a pixel's sample sequence.
   *
   * <p>Derived from the pixel, the sample and the frame rather than from a shared counter, so
   * threads never interact and re-rendering a frame index reproduces it bit for bit.
   */
  private static int seed(int pixel, int sample, long frameIndex) {
    int h = pixel * 0x9E3779B9 + sample * 0x85EBCA6B + (int) frameIndex * 0xC2B2AE35;
    h ^= h >>> 16;
    h *= 0x7FEB352D;
    h ^= h >>> 15;
    return h == 0 ? 1 : h; // xorshift cannot escape zero
  }

  private static int nextRandom(Context context) {
    int x = context.random;
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    context.random = x;
    return x;
  }

  private static float nextFloat(Context context) {
    return (nextRandom(context) >>> 8) * (1f / (1 << 24));
  }
}
