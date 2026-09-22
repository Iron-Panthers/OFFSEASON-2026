package frc.robot.utility.rendering;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Loader for binary glTF 2.0 (.glb), scoped to what the AdvantageScope field and robot exports
 * actually contain: a single buffer, triangle primitives, POSITION and NORMAL attributes, and
 * metallic-roughness materials with no textures.
 *
 * <p>Deliberately unsupported, because no AdvantageScope asset uses them: Draco or meshopt
 * compression, sparse accessors, morph targets, skins, and external .bin buffers. Each is rejected
 * loudly rather than silently producing broken geometry.
 *
 * <p>Node transforms are baked into the returned vertex data, so callers get flat model-space
 * geometry and never have to walk the hierarchy themselves.
 */
final class Glb {

  private static final int MAGIC = 0x46546C67; // "glTF"
  private static final int CHUNK_JSON = 0x4E4F534A; // "JSON"
  private static final int CHUNK_BIN = 0x004E4942; // "BIN\0"
  private static final int MODE_TRIANGLES = 4;

  private static final int COMPONENT_UNSIGNED_BYTE = 5121;
  private static final int COMPONENT_UNSIGNED_SHORT = 5123;
  private static final int COMPONENT_UNSIGNED_INT = 5125;
  private static final int COMPONENT_FLOAT = 5126;

  /** A metallic-roughness base colour. Alpha below 1 marks the field polycarbonate panels. */
  record Material(String name, float red, float green, float blue, float alpha) {}

  /**
   * One triangle list in model space.
   *
   * @param positions xyz per vertex, node transform already applied
   * @param normals unit xyz per vertex, matching {@code positions}
   * @param indices three vertex indices per triangle
   * @param material index into {@link Model#materials()}, or -1 if the primitive declared none
   * @param nodeName nearest named ancestor node. This is the CAD part name, and the only clue
   *     available for working out what a surface is actually made of, because the export carries no
   *     textures and stamps every material with the same metallic and roughness values.
   */
  record Mesh(float[] positions, float[] normals, int[] indices, int material, String nodeName) {}

  record Model(List<Mesh> meshes, List<Material> materials) {}

  private final JsonNode gltf;
  private final ByteBuffer bin;
  private final List<Mesh> meshes = new ArrayList<>();

  private Glb(JsonNode gltf, ByteBuffer bin) {
    this.gltf = gltf;
    this.bin = bin;
  }

  static Model load(Path file) throws IOException {
    ByteBuffer buf = ByteBuffer.wrap(Files.readAllBytes(file)).order(ByteOrder.LITTLE_ENDIAN);

    if (buf.remaining() < 12 || buf.getInt() != MAGIC) {
      throw new IOException(file + " is not a binary glTF file");
    }
    int version = buf.getInt();
    if (version != 2) {
      throw new IOException(file + " is glTF version " + version + ", only 2 is supported");
    }
    buf.getInt(); // declared total length; the buffer already knows how long it is

    JsonNode gltf = null;
    ByteBuffer bin = null;
    while (buf.remaining() >= 8) {
      int chunkLength = buf.getInt();
      int chunkType = buf.getInt();
      if (chunkType == CHUNK_JSON) {
        byte[] json = new byte[chunkLength];
        buf.slice().get(json);
        gltf = new ObjectMapper().readTree(new String(json, StandardCharsets.UTF_8));
      } else if (chunkType == CHUNK_BIN) {
        bin = buf.slice().order(ByteOrder.LITTLE_ENDIAN).limit(chunkLength);
      }
      buf.position(buf.position() + chunkLength);
    }
    if (gltf == null) {
      throw new IOException(file + " has no JSON chunk");
    }

    for (JsonNode required : gltf.path("extensionsRequired")) {
      throw new IOException(
          file + " requires glTF extension " + required.asText() + ", which is not supported");
    }

    Glb loader = new Glb(gltf, bin);
    loader.walkScenes();
    return new Model(loader.meshes, loader.readMaterials());
  }

  private List<Material> readMaterials() {
    List<Material> out = new ArrayList<>();
    for (JsonNode material : gltf.path("materials")) {
      JsonNode factor = material.path("pbrMetallicRoughness").path("baseColorFactor");
      out.add(
          new Material(
              material.path("name").asText(""),
              factor.has(0) ? (float) factor.get(0).asDouble() : 1f,
              factor.has(1) ? (float) factor.get(1).asDouble() : 1f,
              factor.has(2) ? (float) factor.get(2).asDouble() : 1f,
              factor.has(3) ? (float) factor.get(3).asDouble() : 1f));
    }
    return out;
  }

  private void walkScenes() throws IOException {
    int sceneIndex = gltf.path("scene").asInt(0);
    JsonNode scenes = gltf.path("scenes");
    JsonNode roots =
        scenes.has(sceneIndex)
            ? scenes.get(sceneIndex).path("nodes")
            : scenes.path(0).path("nodes");
    for (JsonNode root : roots) {
      walkNode(root.asInt(), Mat4.identity(), "");
    }
  }

  private void walkNode(int nodeIndex, Mat4 parent, String inheritedName) throws IOException {
    JsonNode node = gltf.path("nodes").get(nodeIndex);
    Mat4 world = parent.times(localTransform(node));

    // Leaf primitives are usually unnamed, so carry the nearest named ancestor down the tree.
    String name = node.path("name").asText("");
    if (name.isEmpty()) {
      name = inheritedName;
    }

    if (node.has("mesh")) {
      JsonNode primitives = gltf.path("meshes").get(node.get("mesh").asInt()).path("primitives");
      for (JsonNode primitive : primitives) {
        Mesh mesh = readPrimitive(primitive, world, name);
        if (mesh != null) {
          meshes.add(mesh);
        }
      }
    }
    for (JsonNode child : node.path("children")) {
      walkNode(child.asInt(), world, name);
    }
  }

  private static Mat4 localTransform(JsonNode node) {
    if (node.has("matrix")) {
      double[] m = new double[16];
      for (int i = 0; i < 16; i++) {
        m[i] = node.get("matrix").get(i).asDouble();
      }
      return Mat4.fromGltfMatrix(m);
    }
    return Mat4.fromTrs(
        readDoubles(node, "translation", 3),
        readDoubles(node, "rotation", 4),
        readDoubles(node, "scale", 3));
  }

  private static double[] readDoubles(JsonNode node, String field, int count) {
    if (!node.has(field)) {
      return null;
    }
    double[] out = new double[count];
    for (int i = 0; i < count; i++) {
      out[i] = node.get(field).get(i).asDouble();
    }
    return out;
  }

  private Mesh readPrimitive(JsonNode primitive, Mat4 world, String nodeName) throws IOException {
    if (primitive.path("mode").asInt(MODE_TRIANGLES) != MODE_TRIANGLES) {
      return null; // points and lines contribute nothing to a rendered camera view
    }
    JsonNode attributes = primitive.path("attributes");
    if (!attributes.has("POSITION")) {
      return null;
    }

    float[] positions = readFloatAccessor(attributes.get("POSITION").asInt(), 3);
    int vertexCount = positions.length / 3;
    int[] indices =
        primitive.has("indices")
            ? readIndices(primitive.get("indices").asInt())
            : sequentialIndices(vertexCount);

    float[] normals =
        attributes.has("NORMAL")
            ? readFloatAccessor(attributes.get("NORMAL").asInt(), 3)
            : faceNormals(positions, indices);

    double[] out = new double[3];
    for (int v = 0; v < vertexCount; v++) {
      world.applyToPoint(positions[v * 3], positions[v * 3 + 1], positions[v * 3 + 2], out);
      positions[v * 3] = (float) out[0];
      positions[v * 3 + 1] = (float) out[1];
      positions[v * 3 + 2] = (float) out[2];

      world.applyToDirection(normals[v * 3], normals[v * 3 + 1], normals[v * 3 + 2], out);
      double length = Math.sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
      if (length > 1e-12) {
        normals[v * 3] = (float) (out[0] / length);
        normals[v * 3 + 1] = (float) (out[1] / length);
        normals[v * 3 + 2] = (float) (out[2] / length);
      }
    }

    return new Mesh(positions, normals, indices, primitive.path("material").asInt(-1), nodeName);
  }

  private static int[] sequentialIndices(int vertexCount) {
    int[] indices = new int[vertexCount];
    for (int i = 0; i < vertexCount; i++) {
      indices[i] = i;
    }
    return indices;
  }

  /** Derives flat per-vertex normals for a primitive that shipped without them. */
  private static float[] faceNormals(float[] positions, int[] indices) {
    float[] normals = new float[positions.length];
    for (int t = 0; t + 2 < indices.length; t += 3) {
      int a = indices[t] * 3;
      int b = indices[t + 1] * 3;
      int c = indices[t + 2] * 3;
      float ux = positions[b] - positions[a];
      float uy = positions[b + 1] - positions[a + 1];
      float uz = positions[b + 2] - positions[a + 2];
      float vx = positions[c] - positions[a];
      float vy = positions[c + 1] - positions[a + 1];
      float vz = positions[c + 2] - positions[a + 2];
      float nx = uy * vz - uz * vy;
      float ny = uz * vx - ux * vz;
      float nz = ux * vy - uy * vx;
      float length = (float) Math.sqrt(nx * nx + ny * ny + nz * nz);
      if (length > 1e-20f) {
        nx /= length;
        ny /= length;
        nz /= length;
      }
      normals[a] = nx;
      normals[a + 1] = ny;
      normals[a + 2] = nz;
      normals[b] = nx;
      normals[b + 1] = ny;
      normals[b + 2] = nz;
      normals[c] = nx;
      normals[c + 1] = ny;
      normals[c + 2] = nz;
    }
    return normals;
  }

  private int[] readIndices(int accessorIndex) throws IOException {
    JsonNode accessor = gltf.path("accessors").get(accessorIndex);
    int count = accessor.path("count").asInt();
    int componentType = accessor.path("componentType").asInt();
    ByteBuffer view = accessorView(accessor);
    int stride = elementStride(accessor, componentSize(componentType));

    int[] out = new int[count];
    for (int i = 0; i < count; i++) {
      int at = i * stride;
      out[i] =
          switch (componentType) {
            case COMPONENT_UNSIGNED_BYTE -> view.get(at) & 0xFF;
            case COMPONENT_UNSIGNED_SHORT -> view.getShort(at) & 0xFFFF;
            case COMPONENT_UNSIGNED_INT -> view.getInt(at);
            default -> throw new IOException("Unsupported index component type " + componentType);
          };
    }
    return out;
  }

  private float[] readFloatAccessor(int accessorIndex, int components) throws IOException {
    JsonNode accessor = gltf.path("accessors").get(accessorIndex);
    if (accessor.has("sparse")) {
      throw new IOException("Sparse accessors are not supported");
    }
    int componentType = accessor.path("componentType").asInt();
    if (componentType != COMPONENT_FLOAT) {
      throw new IOException("Expected float attribute data, got component type " + componentType);
    }
    int count = accessor.path("count").asInt();
    ByteBuffer view = accessorView(accessor);
    int stride = elementStride(accessor, components * 4);

    float[] out = new float[count * components];
    for (int i = 0; i < count; i++) {
      int at = i * stride;
      for (int c = 0; c < components; c++) {
        out[i * components + c] = view.getFloat(at + c * 4);
      }
    }
    return out;
  }

  /** Slices the BIN chunk down to the start of one accessor, leaving stride to the caller. */
  private ByteBuffer accessorView(JsonNode accessor) throws IOException {
    if (bin == null) {
      throw new IOException("Accessor data requested but the file has no BIN chunk");
    }
    JsonNode bufferView = gltf.path("bufferViews").get(accessor.path("bufferView").asInt());
    int offset = bufferView.path("byteOffset").asInt(0) + accessor.path("byteOffset").asInt(0);
    return bin.slice(offset, bin.limit() - offset).order(ByteOrder.LITTLE_ENDIAN);
  }

  /** Interleaved buffer views declare a stride; tightly packed ones omit it. */
  private int elementStride(JsonNode accessor, int tightlyPackedSize) {
    JsonNode bufferView = gltf.path("bufferViews").get(accessor.path("bufferView").asInt());
    int declared = bufferView.path("byteStride").asInt(0);
    return declared > 0 ? declared : tightlyPackedSize;
  }

  private static int componentSize(int componentType) {
    return switch (componentType) {
      case COMPONENT_UNSIGNED_BYTE -> 1;
      case COMPONENT_UNSIGNED_SHORT -> 2;
      default -> 4;
    };
  }
}
