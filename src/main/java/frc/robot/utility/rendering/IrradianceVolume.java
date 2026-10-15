package frc.robot.utility.rendering;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.List;
import java.util.stream.IntStream;

/**
 * Static lighting, baked into a grid once and then read instead of traced.
 *
 * <p>This is what makes the fast renderer fast. Profiling the path traced renderer put 45 percent
 * of a frame in shadow rays and another 25 percent in the diffuse bounce, and every one of those
 * rays recomputes something that cannot change: the field does not move and neither does the light
 * truss, so the light reaching any point in the building is fixed before the match starts. Baking
 * it turns roughly eight rays per sample into one array lookup.
 *
 * <p>Stored as an ambient cube: six incoming radiance values per cell, one per axis direction.
 * Cheap to evaluate, cheap to interpolate, and enough to carry both the direction of the truss
 * above and the darkening under the trench.
 *
 * <p>The 20 cm cell size looks coarse and is not, for this scene specifically. A 3.8 m fixture 8 m
 * up casts a penumbra around half a metre wide, so the real shadows on this field are softer than
 * the grid. What the grid cannot represent is a shadow sharper than its own cells, and there are
 * none here. Contact shadows under fuel are sharp, but those come from live geometry and are still
 * traced.
 */
final class IrradianceVolume {

  private static final int MAGIC = 0x49525244; // "IRRD"
  private static final int FORMAT_VERSION = 1;

  /** Six axis directions, in the order the cube stores them. */
  private static final float[][] AXES = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
  };

  private static final int VALUES_PER_CELL = AXES.length * 3;

  /**
   * Shadow rays per fixture per cell. Visibility is direction independent, so this is all of it.
   */
  private static final int FIXTURE_SAMPLES = 3;

  /** Hemisphere rays per axis, for sky and for the bounce pass. */
  private static final int HEMISPHERE_SAMPLES = 14;

  private static final float PI = (float) Math.PI;

  private final float originX;
  private final float originY;
  private final float originZ;
  private final float cellSize;
  private final float inverseCellSize;
  private final int countX;
  private final int countY;
  private final int countZ;

  /** {@code countX * countY * countZ * 18} radiance values. */
  private final float[] cube;

  private IrradianceVolume(
      float originX,
      float originY,
      float originZ,
      float cellSize,
      int countX,
      int countY,
      int countZ,
      float[] cube) {
    this.originX = originX;
    this.originY = originY;
    this.originZ = originZ;
    this.cellSize = cellSize;
    this.inverseCellSize = 1f / cellSize;
    this.countX = countX;
    this.countY = countY;
    this.countZ = countZ;
    this.cube = cube;
  }

  int cellCount() {
    return countX * countY * countZ;
  }

  float cellSize() {
    return cellSize;
  }

  // ---------------------------------------------------------------------------------------------
  // Lookup
  // ---------------------------------------------------------------------------------------------

  /**
   * Irradiance arriving at a point on a surface facing a given direction.
   *
   * <p>Callers should offset the sample point along the normal by about a cell before calling.
   * Cells that fall inside solid geometry see no light, and a point sitting exactly on a wall sits
   * exactly between a lit cell and a dark one.
   *
   * @param out receives linear RGB
   */
  void sample(float x, float y, float z, float nx, float ny, float nz, float[] out) {
    float gridX = (x - originX) * inverseCellSize - 0.5f;
    float gridY = (y - originY) * inverseCellSize - 0.5f;
    float gridZ = (z - originZ) * inverseCellSize - 0.5f;

    int x0 = clamp((int) Math.floor(gridX), countX);
    int y0 = clamp((int) Math.floor(gridY), countY);
    int z0 = clamp((int) Math.floor(gridZ), countZ);
    int x1 = clamp(x0 + 1, countX);
    int y1 = clamp(y0 + 1, countY);
    int z1 = clamp(z0 + 1, countZ);

    float fx = Math.min(1f, Math.max(0f, gridX - x0));
    float fy = Math.min(1f, Math.max(0f, gridY - y0));
    float fz = Math.min(1f, Math.max(0f, gridZ - z0));

    // Squared normal components select and weight the three facing axes of the cube.
    int axisX = nx >= 0 ? 0 : 1;
    int axisY = ny >= 0 ? 2 : 3;
    int axisZ = nz >= 0 ? 4 : 5;
    float weightX = nx * nx;
    float weightY = ny * ny;
    float weightZ = nz * nz;

    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    for (int corner = 0; corner < 8; corner++) {
      int cx = (corner & 1) == 0 ? x0 : x1;
      int cy = (corner & 2) == 0 ? y0 : y1;
      int cz = (corner & 4) == 0 ? z0 : z1;
      float weight =
          ((corner & 1) == 0 ? 1 - fx : fx)
              * ((corner & 2) == 0 ? 1 - fy : fy)
              * ((corner & 4) == 0 ? 1 - fz : fz);
      if (weight <= 0f) {
        continue;
      }
      int base = ((cz * countY + cy) * countX + cx) * VALUES_PER_CELL;
      for (int c = 0; c < 3; c++) {
        out[c] +=
            weight
                * (cube[base + axisX * 3 + c] * weightX
                    + cube[base + axisY * 3 + c] * weightY
                    + cube[base + axisZ * 3 + c] * weightZ);
      }
    }
  }

  private static int clamp(int value, int count) {
    return value < 0 ? 0 : Math.min(value, count - 1);
  }

  // ---------------------------------------------------------------------------------------------
  // Bake
  // ---------------------------------------------------------------------------------------------

  /**
   * Bakes the volume, or loads it if an identical one is already on disk.
   *
   * @param cacheDirectory where to keep the result, or null to always rebake
   * @param key identifies the scene and lighting this was baked for
   */
  static IrradianceVolume bakeOrLoad(
      FieldScene scene, ArenaLighting lighting, float cellSize, Path cacheDirectory, String key) {

    if (cacheDirectory != null && key != null) {
      IrradianceVolume cached = read(cacheDirectory.resolve("light-" + key + ".volume"));
      if (cached != null) {
        return cached;
      }
    }
    IrradianceVolume baked = bake(scene, lighting, cellSize);
    if (cacheDirectory != null && key != null) {
      baked.write(cacheDirectory, "light-" + key + ".volume");
    }
    return baked;
  }

  private static IrradianceVolume bake(FieldScene scene, ArenaLighting lighting, float cellSize) {

    float[] bounds = sceneBounds(scene);
    // One cell of margin, so surfaces on the outside of the field still interpolate against a
    // lit cell rather than against the edge of the grid.
    float originX = bounds[0] - cellSize;
    float originY = bounds[1] - cellSize;
    float originZ = bounds[2] - cellSize;
    int countX = (int) Math.ceil((bounds[3] - bounds[0]) / cellSize) + 3;
    int countY = (int) Math.ceil((bounds[4] - bounds[1]) / cellSize) + 3;
    int countZ = (int) Math.ceil((bounds[5] - bounds[2]) / cellSize) + 3;

    float[] direct = new float[countX * countY * countZ * VALUES_PER_CELL];
    IrradianceVolume volume =
        new IrradianceVolume(originX, originY, originZ, cellSize, countX, countY, countZ, direct);

    // Pass one: the truss and the sky, with static shadowing.
    volume.fill(scene, lighting, null);

    // Pass two: one bounce, lit by the result of pass one. Written into a copy so that every cell
    // bounces off the same first-pass lighting rather than off whatever its neighbour got to
    // first.
    IrradianceVolume first =
        new IrradianceVolume(
            originX, originY, originZ, cellSize, countX, countY, countZ, direct.clone());
    volume.fill(scene, lighting, first);

    volume.blur();
    return volume;
  }

  /**
   * Fills every cell.
   *
   * @param bounceSource when null, gathers the truss and sky directly; otherwise adds one bounce,
   *     lighting each hit surface from an already-baked volume
   */
  private void fill(FieldScene scene, ArenaLighting lighting, IrradianceVolume bounceSource) {
    List<ArenaLighting.AreaLight> fixtures = lighting.lights();
    IntStream.range(0, countZ)
        .parallel()
        .forEach(z -> bakeSlice(scene, lighting, fixtures, bounceSource, z));
  }

  private void bakeSlice(
      FieldScene scene,
      ArenaLighting lighting,
      List<ArenaLighting.AreaLight> fixtures,
      IrradianceVolume bounceSource,
      int z) {

    Bvh.Scratch scratch = new Bvh.Scratch();
    float[] sky = new float[3];
    float[] gathered = new float[3];
    float[] axisSum = new float[VALUES_PER_CELL];
    int random = 0x9E3779B9 ^ (z * 0x85EBCA6B);

    for (int y = 0; y < countY; y++) {
      for (int x = 0; x < countX; x++) {
        float px = originX + (x + 0.5f) * cellSize;
        float py = originY + (y + 0.5f) * cellSize;
        float pz = originZ + (z + 0.5f) * cellSize;
        java.util.Arrays.fill(axisSum, 0f);

        if (bounceSource == null) {
          random = gatherFixtures(scene, fixtures, px, py, pz, axisSum, scratch, random);
          random = gatherHemispheres(scene, lighting, px, py, pz, axisSum, scratch, sky, random);
        } else {
          random =
              gatherBounce(scene, px, py, pz, axisSum, scratch, gathered, bounceSource, random);
        }

        int base = ((z * countY + y) * countX + x) * VALUES_PER_CELL;
        for (int i = 0; i < VALUES_PER_CELL; i++) {
          cube[base + i] += axisSum[i];
        }
      }
    }
  }

  /** Direct light from every fixture. Visibility does not depend on the axis, so it is shared. */
  private int gatherFixtures(
      FieldScene scene,
      List<ArenaLighting.AreaLight> fixtures,
      float px,
      float py,
      float pz,
      float[] axisSum,
      Bvh.Scratch scratch,
      int random) {

    for (ArenaLighting.AreaLight light : fixtures) {
      for (int sample = 0; sample < FIXTURE_SAMPLES; sample++) {
        random = nextRandom(random);
        float u = toFloat(random) * 2f - 1f;
        random = nextRandom(random);
        float v = toFloat(random) * 2f - 1f;

        float sx = light.center()[0] + light.axisU()[0] * u * light.halfU();
        float sy = light.center()[1] + light.axisU()[1] * u * light.halfU();
        float sz = light.center()[2] + light.axisU()[2] * u * light.halfU();
        sx += light.axisV()[0] * v * light.halfV();
        sy += light.axisV()[1] * v * light.halfV();
        sz += light.axisV()[2] * v * light.halfV();

        float toX = sx - px;
        float toY = sy - py;
        float toZ = sz - pz;
        float distanceSquared = toX * toX + toY * toY + toZ * toZ;
        float distance = (float) Math.sqrt(distanceSquared);
        if (distance < 1e-3f) {
          continue;
        }
        float wx = toX / distance;
        float wy = toY / distance;
        float wz = toZ / distance;

        float cosLight =
            -(light.normal()[0] * wx + light.normal()[1] * wy + light.normal()[2] * wz);
        if (cosLight <= 0) {
          continue;
        }
        float visibility =
            scene.bvh.transmittance(scene.soup, px, py, pz, wx, wy, wz, distance - 1e-3f, scratch);
        if (visibility <= 0) {
          continue;
        }

        float geometry = cosLight * light.area() / distanceSquared * visibility / FIXTURE_SAMPLES;
        for (int axis = 0; axis < AXES.length; axis++) {
          float cosSurface = AXES[axis][0] * wx + AXES[axis][1] * wy + AXES[axis][2] * wz;
          if (cosSurface <= 0) {
            continue;
          }
          float weight = geometry * cosSurface;
          axisSum[axis * 3] += light.radiance()[0] * weight;
          axisSum[axis * 3 + 1] += light.radiance()[1] * weight;
          axisSum[axis * 3 + 2] += light.radiance()[2] * weight;
        }
      }
    }
    return random;
  }

  /** Sky seen from this cell, per axis. Rays that hit geometry contribute nothing in pass one. */
  private int gatherHemispheres(
      FieldScene scene,
      ArenaLighting lighting,
      float px,
      float py,
      float pz,
      float[] axisSum,
      Bvh.Scratch scratch,
      float[] sky,
      int random) {

    float[] direction = new float[3];
    for (int axis = 0; axis < AXES.length; axis++) {
      float sumR = 0;
      float sumG = 0;
      float sumB = 0;
      for (int sample = 0; sample < HEMISPHERE_SAMPLES; sample++) {
        random = cosineDirection(AXES[axis], direction, random);
        if (scene.bvh.intersect(
            scene.soup, px, py, pz, direction[0], direction[1], direction[2], 200f, scratch)) {
          continue;
        }
        lighting.sky(direction[0], direction[1], direction[2], sky);
        sumR += sky[0];
        sumG += sky[1];
        sumB += sky[2];
      }
      // Cosine weighted sampling, so the estimator for irradiance is pi times the mean radiance.
      float scale = PI / HEMISPHERE_SAMPLES;
      axisSum[axis * 3] += sumR * scale;
      axisSum[axis * 3 + 1] += sumG * scale;
      axisSum[axis * 3 + 2] += sumB * scale;
    }
    return random;
  }

  /** One bounce: what the surfaces around this cell throw back at it. */
  private int gatherBounce(
      FieldScene scene,
      float px,
      float py,
      float pz,
      float[] axisSum,
      Bvh.Scratch scratch,
      float[] gathered,
      IrradianceVolume source,
      int random) {

    float[] direction = new float[3];
    for (int axis = 0; axis < AXES.length; axis++) {
      float sumR = 0;
      float sumG = 0;
      float sumB = 0;
      for (int sample = 0; sample < HEMISPHERE_SAMPLES; sample++) {
        random = cosineDirection(AXES[axis], direction, random);
        if (!scene.bvh.intersect(
            scene.soup, px, py, pz, direction[0], direction[1], direction[2], 200f, scratch)) {
          continue;
        }
        Bvh.Hit hit = scratch.hit;
        Surface surface = scene.soup.surfaces.get(scene.soup.surfaceIds[hit.triangle]);
        if (surface.isTransparent()) {
          continue; // seeing through a panel is not a bounce off it
        }

        float hx = px + direction[0] * hit.distance;
        float hy = py + direction[1] * hit.distance;
        float hz = pz + direction[2] * hit.distance;
        // The surface faces back toward this cell, so light it from that side.
        source.sample(
            hx - direction[0] * cellSize,
            hy - direction[1] * cellSize,
            hz - direction[2] * cellSize,
            -direction[0],
            -direction[1],
            -direction[2],
            gathered);

        // Lambert: radiance leaving is albedo times irradiance over pi.
        sumR += surface.red() * gathered[0] / PI + surface.red() * surface.emissive();
        sumG += surface.green() * gathered[1] / PI + surface.green() * surface.emissive();
        sumB += surface.blue() * gathered[2] / PI + surface.blue() * surface.emissive();
      }
      float scale = PI / HEMISPHERE_SAMPLES;
      axisSum[axis * 3] += sumR * scale;
      axisSum[axis * 3 + 1] += sumG * scale;
      axisSum[axis * 3 + 2] += sumB * scale;
    }
    return random;
  }

  /** Smooths out the sampling noise the bake leaves behind. */
  private void blur() {
    float[] source = cube.clone();
    IntStream.range(0, countZ)
        .parallel()
        .forEach(
            z -> {
              for (int y = 0; y < countY; y++) {
                for (int x = 0; x < countX; x++) {
                  int target = ((z * countY + y) * countX + x) * VALUES_PER_CELL;
                  for (int i = 0; i < VALUES_PER_CELL; i++) {
                    float sum = 0;
                    int counted = 0;
                    for (int dz = -1; dz <= 1; dz++) {
                      int sz = z + dz;
                      if (sz < 0 || sz >= countZ) {
                        continue;
                      }
                      for (int dy = -1; dy <= 1; dy++) {
                        int sy = y + dy;
                        if (sy < 0 || sy >= countY) {
                          continue;
                        }
                        for (int dx = -1; dx <= 1; dx++) {
                          int sx = x + dx;
                          if (sx < 0 || sx >= countX) {
                            continue;
                          }
                          sum += source[((sz * countY + sy) * countX + sx) * VALUES_PER_CELL + i];
                          counted++;
                        }
                      }
                    }
                    cube[target + i] = sum / counted;
                  }
                }
              }
            });
  }

  private static float[] sceneBounds(FieldScene scene) {
    float[] bounds = {
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY,
      Float.NEGATIVE_INFINITY,
      Float.NEGATIVE_INFINITY,
      Float.NEGATIVE_INFINITY
    };
    float[] positions = scene.soup.positions;
    for (int v = 0; v < positions.length; v += 3) {
      for (int axis = 0; axis < 3; axis++) {
        bounds[axis] = Math.min(bounds[axis], positions[v + axis]);
        bounds[axis + 3] = Math.max(bounds[axis + 3], positions[v + axis]);
      }
    }
    return bounds;
  }

  private static int cosineDirection(float[] axis, float[] out, int random) {
    random = nextRandom(random);
    float r1 = toFloat(random);
    random = nextRandom(random);
    float r2 = toFloat(random);

    float radius = (float) Math.sqrt(r1);
    float phi = 2f * PI * r2;
    float localX = radius * (float) Math.cos(phi);
    float localY = radius * (float) Math.sin(phi);
    float localZ = (float) Math.sqrt(Math.max(0f, 1f - r1));

    float nx = axis[0];
    float ny = axis[1];
    float nz = axis[2];
    float sign = nz >= 0f ? 1f : -1f;
    float a = -1f / (sign + nz);
    float b = nx * ny * a;
    out[0] = (1f + sign * nx * nx * a) * localX + b * localY + nx * localZ;
    out[1] = (sign * b) * localX + (sign + ny * ny * a) * localY + ny * localZ;
    out[2] = (-sign * nx) * localX + (-ny) * localY + nz * localZ;
    return random;
  }

  private static int nextRandom(int state) {
    int x = state == 0 ? 1 : state;
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    return x;
  }

  private static float toFloat(int random) {
    return (random >>> 8) * (1f / (1 << 24));
  }

  // ---------------------------------------------------------------------------------------------
  // Cache
  // ---------------------------------------------------------------------------------------------

  private static IrradianceVolume read(Path file) {
    if (!Files.isRegularFile(file)) {
      return null;
    }
    try (FileChannel channel = FileChannel.open(file, StandardOpenOption.READ)) {
      ByteBuffer header = ByteBuffer.allocate(40).order(ByteOrder.LITTLE_ENDIAN);
      while (header.hasRemaining()) {
        if (channel.read(header) < 0) {
          return null;
        }
      }
      header.flip();
      if (header.getInt() != MAGIC || header.getInt() != FORMAT_VERSION) {
        return null;
      }
      float originX = header.getFloat();
      float originY = header.getFloat();
      float originZ = header.getFloat();
      float cellSize = header.getFloat();
      int countX = header.getInt();
      int countY = header.getInt();
      int countZ = header.getInt();
      int length = header.getInt();
      if (length != countX * countY * countZ * VALUES_PER_CELL) {
        return null;
      }

      float[] cube = new float[length];
      ByteBuffer buffer = ByteBuffer.allocateDirect(1 << 20).order(ByteOrder.LITTLE_ENDIAN);
      int chunk = (1 << 20) / 4;
      for (int offset = 0; offset < length; offset += chunk) {
        int span = Math.min(chunk, length - offset);
        buffer.clear().limit(span * 4);
        while (buffer.hasRemaining()) {
          if (channel.read(buffer) < 0) {
            return null;
          }
        }
        buffer.flip().asFloatBuffer().get(cube, offset, span);
      }
      return new IrradianceVolume(
          originX, originY, originZ, cellSize, countX, countY, countZ, cube);
    } catch (IOException | RuntimeException unreadable) {
      return null;
    }
  }

  private void write(Path directory, String name) {
    Path temporary = directory.resolve(name + ".partial");
    try {
      Files.createDirectories(directory);
      try (FileChannel channel =
          FileChannel.open(
              temporary,
              StandardOpenOption.CREATE,
              StandardOpenOption.TRUNCATE_EXISTING,
              StandardOpenOption.WRITE)) {
        ByteBuffer header = ByteBuffer.allocate(40).order(ByteOrder.LITTLE_ENDIAN);
        header
            .putInt(MAGIC)
            .putInt(FORMAT_VERSION)
            .putFloat(originX)
            .putFloat(originY)
            .putFloat(originZ)
            .putFloat(cellSize)
            .putInt(countX)
            .putInt(countY)
            .putInt(countZ)
            .putInt(cube.length)
            .flip();
        channel.write(header);

        ByteBuffer buffer = ByteBuffer.allocateDirect(1 << 20).order(ByteOrder.LITTLE_ENDIAN);
        int chunk = (1 << 20) / 4;
        for (int offset = 0; offset < cube.length; offset += chunk) {
          int span = Math.min(chunk, cube.length - offset);
          buffer.clear();
          buffer.asFloatBuffer().put(cube, offset, span);
          buffer.limit(span * 4);
          while (buffer.hasRemaining()) {
            channel.write(buffer);
          }
        }
      }
      Files.move(temporary, directory.resolve(name), StandardCopyOption.REPLACE_EXISTING);
    } catch (IOException unwritable) {
      try {
        Files.deleteIfExists(temporary);
      } catch (IOException ignored) {
        // Nothing useful to do about a leftover partial file.
      }
    }
  }
}
