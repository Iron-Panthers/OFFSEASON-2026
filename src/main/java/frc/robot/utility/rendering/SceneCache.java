package frc.robot.utility.rendering;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.FileChannel;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.List;

/**
 * On-disk cache of the built field geometry.
 *
 * <p>Parsing eighteen megabytes of glTF and running a surface area heuristic over four million
 * triangles takes long enough that doing it on every simulator start would make the renderer
 * unusable to iterate on. The result is deterministic, so it is written once and mapped back in
 * afterwards.
 *
 * <p>The cache key covers both the asset files and the compiled classes that decide how those
 * assets become geometry. Editing {@link Materials} therefore invalidates the cache automatically,
 * which is the failure mode a hand-maintained version number would actually hit.
 */
final class SceneCache {

  private static final int MAGIC = 0x52454E44; // "REND"
  private static final int FORMAT_VERSION = 1;

  /** Bulk array transfers move through one reusable buffer rather than element by element. */
  private static final int TRANSFER_BYTES = 8 << 20;

  private final Path directory;

  SceneCache(Path directory) {
    this.directory = directory;
  }

  record Entry(TriangleSoup soup, Bvh bvh, double tagAlignmentError) {}

  /** Derives a key from the asset files plus the classes that interpret them. */
  static String keyFor(Path modelFile, Path configFile, Path tagDirectory) {
    try {
      MessageDigest digest = MessageDigest.getInstance("SHA-256");
      digest.update(ByteBuffer.allocate(4).putInt(FORMAT_VERSION).array());

      for (Path file : List.of(modelFile, configFile)) {
        digest.update(file.getFileName().toString().getBytes(StandardCharsets.UTF_8));
        digest.update(ByteBuffer.allocate(8).putLong(Files.size(file)).array());
        digest.update(
            ByteBuffer.allocate(8).putLong(Files.getLastModifiedTime(file).toMillis()).array());
      }
      if (Files.isDirectory(tagDirectory)) {
        try (var entries = Files.list(tagDirectory)) {
          long total = 0;
          for (Path file : entries.sorted().toList()) {
            total += Files.size(file) * 31 + file.getFileName().toString().hashCode();
          }
          digest.update(ByteBuffer.allocate(8).putLong(total).array());
        }
      }
      for (Class<?> type : List.of(Materials.class, FieldScene.class, Surface.class, Bvh.class)) {
        digest.update(classBytes(type));
      }
      return HexFormat.of().formatHex(digest.digest()).substring(0, 24);
    } catch (NoSuchAlgorithmException | IOException unusable) {
      return null; // no key means no caching, which is a slow start rather than a wrong one
    }
  }

  private static byte[] classBytes(Class<?> type) throws IOException {
    String resource = type.getName().replace('.', '/') + ".class";
    try (InputStream stream = type.getClassLoader().getResourceAsStream(resource)) {
      return stream == null ? type.getName().getBytes(StandardCharsets.UTF_8) : stream.readAllBytes();
    }
  }

  /**
   * @return the cached scene, or null if there is no usable entry for this key
   */
  Entry read(String key) {
    if (key == null) {
      return null;
    }
    Path file = directory.resolve("field-" + key + ".scene");
    if (!Files.isRegularFile(file)) {
      return null;
    }
    try (FileChannel channel = FileChannel.open(file, StandardOpenOption.READ)) {
      ByteBuffer header = ByteBuffer.allocate(16).order(ByteOrder.LITTLE_ENDIAN);
      channel.read(header);
      header.flip();
      if (header.getInt() != MAGIC || header.getInt() != FORMAT_VERSION) {
        return null;
      }
      double tagAlignmentError = header.getDouble();

      float[] positions = readFloats(channel);
      float[] normals = readFloats(channel);
      int[] indices = readInts(channel);
      int[] surfaceIds = readInts(channel);
      List<Surface> surfaces = readSurfaces(channel);

      int nodesUsed = readInt(channel);
      float[] nodeBounds = readFloats(channel);
      int[] nodeLeftFirst = readInts(channel);
      int[] nodeCount = readInts(channel);
      int[] order = readInts(channel);

      TriangleSoup soup =
          TriangleSoup.fromArrays(positions, normals, indices, surfaceIds, surfaces);
      Bvh bvh = Bvh.fromArrays(nodeBounds, nodeLeftFirst, nodeCount, order, nodesUsed);
      return new Entry(soup, bvh, tagAlignmentError);
    } catch (IOException | RuntimeException unreadable) {
      // A truncated or stale cache should cost a rebuild, never a crash.
      return null;
    }
  }

  void write(String key, TriangleSoup soup, Bvh bvh, double tagAlignmentError) {
    if (key == null) {
      return;
    }
    Path file = directory.resolve("field-" + key + ".scene");
    Path temporary = directory.resolve("field-" + key + ".scene.partial");
    try {
      Files.createDirectories(directory);
      try (FileChannel channel =
          FileChannel.open(
              temporary,
              StandardOpenOption.CREATE,
              StandardOpenOption.TRUNCATE_EXISTING,
              StandardOpenOption.WRITE)) {
        ByteBuffer header = ByteBuffer.allocate(16).order(ByteOrder.LITTLE_ENDIAN);
        header.putInt(MAGIC).putInt(FORMAT_VERSION).putDouble(tagAlignmentError).flip();
        channel.write(header);

        writeFloats(channel, soup.positions);
        writeFloats(channel, soup.normals);
        writeInts(channel, soup.indices);
        writeInts(channel, soup.surfaceIds);
        writeSurfaces(channel, soup.surfaces);

        writeInt(channel, bvh.nodesUsed);
        writeFloats(channel, bvh.nodeBounds);
        writeInts(channel, bvh.nodeLeftFirst);
        writeInts(channel, bvh.nodeCount);
        writeInts(channel, bvh.order);
      }
      Files.move(temporary, file, java.nio.file.StandardCopyOption.REPLACE_EXISTING);
    } catch (IOException unwritable) {
      // Caching is an optimisation; losing it is not worth failing the run over.
      try {
        Files.deleteIfExists(temporary);
      } catch (IOException ignored) {
        // Nothing useful to do about a leftover partial file.
      }
    }
  }

  // -------------------------------------------------------------------------------------------
  // Array and record transfer
  // -------------------------------------------------------------------------------------------

  private static void writeInt(FileChannel channel, int value) throws IOException {
    ByteBuffer buffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
    buffer.putInt(value).flip();
    channel.write(buffer);
  }

  private static int readInt(FileChannel channel) throws IOException {
    ByteBuffer buffer = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN);
    readFully(channel, buffer);
    return buffer.flip().getInt();
  }

  private static void writeFloats(FileChannel channel, float[] values) throws IOException {
    writeInt(channel, values.length);
    ByteBuffer buffer = ByteBuffer.allocateDirect(TRANSFER_BYTES).order(ByteOrder.LITTLE_ENDIAN);
    int chunk = TRANSFER_BYTES / 4;
    for (int offset = 0; offset < values.length; offset += chunk) {
      int length = Math.min(chunk, values.length - offset);
      buffer.clear();
      buffer.asFloatBuffer().put(values, offset, length);
      buffer.limit(length * 4);
      while (buffer.hasRemaining()) {
        channel.write(buffer);
      }
    }
  }

  private static float[] readFloats(FileChannel channel) throws IOException {
    int count = readInt(channel);
    float[] values = new float[count];
    ByteBuffer buffer = ByteBuffer.allocateDirect(TRANSFER_BYTES).order(ByteOrder.LITTLE_ENDIAN);
    int chunk = TRANSFER_BYTES / 4;
    for (int offset = 0; offset < count; offset += chunk) {
      int length = Math.min(chunk, count - offset);
      buffer.clear().limit(length * 4);
      readFully(channel, buffer);
      buffer.flip().asFloatBuffer().get(values, offset, length);
    }
    return values;
  }

  private static void writeInts(FileChannel channel, int[] values) throws IOException {
    writeInt(channel, values.length);
    ByteBuffer buffer = ByteBuffer.allocateDirect(TRANSFER_BYTES).order(ByteOrder.LITTLE_ENDIAN);
    int chunk = TRANSFER_BYTES / 4;
    for (int offset = 0; offset < values.length; offset += chunk) {
      int length = Math.min(chunk, values.length - offset);
      buffer.clear();
      buffer.asIntBuffer().put(values, offset, length);
      buffer.limit(length * 4);
      while (buffer.hasRemaining()) {
        channel.write(buffer);
      }
    }
  }

  private static int[] readInts(FileChannel channel) throws IOException {
    int count = readInt(channel);
    int[] values = new int[count];
    ByteBuffer buffer = ByteBuffer.allocateDirect(TRANSFER_BYTES).order(ByteOrder.LITTLE_ENDIAN);
    int chunk = TRANSFER_BYTES / 4;
    for (int offset = 0; offset < count; offset += chunk) {
      int length = Math.min(chunk, count - offset);
      buffer.clear().limit(length * 4);
      readFully(channel, buffer);
      buffer.flip().asIntBuffer().get(values, offset, length);
    }
    return values;
  }

  private static void readFully(FileChannel channel, ByteBuffer buffer) throws IOException {
    while (buffer.hasRemaining()) {
      if (channel.read(buffer) < 0) {
        throw new IOException("Scene cache ended early");
      }
    }
  }

  /** Surfaces are few and irregular, so they go through a plain stream rather than bulk buffers. */
  private static void writeSurfaces(FileChannel channel, List<Surface> surfaces)
      throws IOException {
    var bytes = new java.io.ByteArrayOutputStream();
    DataOutputStream out = new DataOutputStream(bytes);
    out.writeInt(surfaces.size());
    for (Surface surface : surfaces) {
      out.writeFloat(surface.red());
      out.writeFloat(surface.green());
      out.writeFloat(surface.blue());
      out.writeFloat(surface.metallic());
      out.writeFloat(surface.roughness());
      out.writeFloat(surface.alpha());
      out.writeFloat(surface.emissive());
      out.writeInt(surface.pattern().ordinal());
      TagDecal decal = surface.decal();
      out.writeBoolean(decal != null);
      if (decal != null) {
        out.writeInt(decal.id());
        out.writeInt(decal.size());
        out.writeDouble(decal.printedSizeMeters());
        for (float cell : decal.bitmap()) {
          out.writeFloat(cell);
        }
        for (double[] vector : List.of(decal.center(), decal.axisU(), decal.axisV())) {
          for (double component : vector) {
            out.writeDouble(component);
          }
        }
      }
    }
    out.flush();
    byte[] payload = bytes.toByteArray();
    writeInt(channel, payload.length);
    channel.write(ByteBuffer.wrap(payload));
  }

  private static List<Surface> readSurfaces(FileChannel channel) throws IOException {
    int length = readInt(channel);
    ByteBuffer buffer = ByteBuffer.allocate(length);
    readFully(channel, buffer);
    DataInputStream in = new DataInputStream(new java.io.ByteArrayInputStream(buffer.array()));

    int count = in.readInt();
    List<Surface> surfaces = new ArrayList<>(count);
    Surface.Pattern[] patterns = Surface.Pattern.values();
    for (int i = 0; i < count; i++) {
      float red = in.readFloat();
      float green = in.readFloat();
      float blue = in.readFloat();
      float metallic = in.readFloat();
      float roughness = in.readFloat();
      float alpha = in.readFloat();
      float emissive = in.readFloat();
      Surface.Pattern pattern = patterns[in.readInt()];
      TagDecal decal = null;
      if (in.readBoolean()) {
        int id = in.readInt();
        int size = in.readInt();
        double printedSize = in.readDouble();
        float[] bitmap = new float[size * size];
        for (int cell = 0; cell < bitmap.length; cell++) {
          bitmap[cell] = in.readFloat();
        }
        double[] center = readVector(in);
        double[] axisU = readVector(in);
        double[] axisV = readVector(in);
        decal = new TagDecal(id, bitmap, size, center, axisU, axisV, printedSize);
      }
      surfaces.add(
          new Surface(red, green, blue, metallic, roughness, alpha, emissive, pattern, decal));
    }
    return surfaces;
  }

  private static double[] readVector(DataInputStream in) throws IOException {
    return new double[] {in.readDouble(), in.readDouble(), in.readDouble()};
  }
}
