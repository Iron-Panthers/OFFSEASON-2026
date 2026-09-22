package frc.robot.utility.rendering;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * The static scene as flat, indexed, world-space triangles.
 *
 * <p>Struct-of-arrays rather than objects on purpose. The 2026 field is roughly four million
 * triangles once the staged fuel is removed, and one Java object per triangle would cost more in
 * headers and pointer chasing than the geometry itself. Positions and indices are the only arrays
 * touched during traversal; normals and surface IDs are read once per hit.
 */
final class TriangleSoup {

  /** xyz per vertex. */
  final float[] positions;

  /** Unit xyz per vertex, parallel to {@link #positions}. */
  final float[] normals;

  /** Three vertex indices per triangle. */
  final int[] indices;

  /** One index into {@link #surfaces} per triangle. */
  final int[] surfaceIds;

  final List<Surface> surfaces;

  /**
   * Per-surface opacity flags and alphas, flattened out of {@link #surfaces}.
   *
   * <p>Shadow rays test these for every triangle they touch, which is the single hottest loop in
   * the renderer. Reaching through an {@code ArrayList} and a record to ask the same question costs
   * more there than the ray-triangle intersection that preceded it.
   */
  final boolean[] surfaceOpaque;

  final float[] surfaceAlpha;

  final int triangleCount;

  private TriangleSoup(
      float[] positions, float[] normals, int[] indices, int[] surfaceIds, List<Surface> surfaces) {
    this.positions = positions;
    this.normals = normals;
    this.indices = indices;
    this.surfaceIds = surfaceIds;
    this.surfaces = surfaces;
    this.surfaceOpaque = new boolean[surfaces.size()];
    this.surfaceAlpha = new float[surfaces.size()];
    for (int i = 0; i < surfaces.size(); i++) {
      surfaceOpaque[i] = !surfaces.get(i).isTransparent();
      surfaceAlpha[i] = surfaces.get(i).alpha();
    }
    this.triangleCount = indices.length / 3;
  }

  /** Rebuilds an instance from cached arrays, skipping the glTF parse entirely. */
  static TriangleSoup fromArrays(
      float[] positions, float[] normals, int[] indices, int[] surfaceIds, List<Surface> surfaces) {
    return new TriangleSoup(positions, normals, indices, surfaceIds, surfaces);
  }

  /** Accumulates meshes and compacts them into one soup. */
  static final class Builder {
    private final List<float[]> meshPositions = new ArrayList<>();
    private final List<float[]> meshNormals = new ArrayList<>();
    private final List<int[]> meshIndices = new ArrayList<>();
    private final List<Surface> meshSurfaces = new ArrayList<>();

    private final List<Surface> surfaces = new ArrayList<>();
    private final Map<Surface, Integer> surfaceIds = new HashMap<>();

    private int totalVertices;
    private int totalIndices;

    /**
     * Adds one triangle list, all of which shares a single surface.
     *
     * <p>AprilTag panels each get their own surface because each carries its own decal, so the
     * dedupe map is keyed on the whole record rather than on the material index.
     */
    void add(float[] positions, float[] normals, int[] indices, Surface surface) {
      if (indices.length < 3) {
        return;
      }
      meshPositions.add(positions);
      meshNormals.add(normals);
      meshIndices.add(indices);
      meshSurfaces.add(surface);
      totalVertices += positions.length / 3;
      totalIndices += indices.length;
    }

    TriangleSoup build() {
      float[] positions = new float[totalVertices * 3];
      float[] normals = new float[totalVertices * 3];
      int[] indices = new int[totalIndices];
      int[] triangleSurfaces = new int[totalIndices / 3];

      int vertexCursor = 0;
      int indexCursor = 0;
      for (int mesh = 0; mesh < meshPositions.size(); mesh++) {
        float[] meshPos = meshPositions.get(mesh);
        float[] meshNrm = meshNormals.get(mesh);
        int[] meshIdx = meshIndices.get(mesh);
        int meshVertices = meshPos.length / 3;

        System.arraycopy(meshPos, 0, positions, vertexCursor * 3, meshPos.length);
        System.arraycopy(meshNrm, 0, normals, vertexCursor * 3, meshNrm.length);

        int surfaceId =
            surfaceIds.computeIfAbsent(
                meshSurfaces.get(mesh),
                surface -> {
                  surfaces.add(surface);
                  return surfaces.size() - 1;
                });

        for (int i = 0; i < meshIdx.length; i++) {
          indices[indexCursor + i] = meshIdx[i] + vertexCursor;
        }
        for (int t = 0; t < meshIdx.length / 3; t++) {
          triangleSurfaces[indexCursor / 3 + t] = surfaceId;
        }

        vertexCursor += meshVertices;
        indexCursor += meshIdx.length;

        // Release each source mesh as it is consumed; both copies alive at once is 300 MB.
        meshPositions.set(mesh, null);
        meshNormals.set(mesh, null);
        meshIndices.set(mesh, null);
      }

      return new TriangleSoup(positions, normals, indices, triangleSurfaces, surfaces);
    }
  }

  /** Writes the axis-aligned bounds of one triangle into {@code out} as min xyz then max xyz. */
  void triangleBounds(int triangle, float[] out) {
    int base = triangle * 3;
    int a = indices[base] * 3;
    int b = indices[base + 1] * 3;
    int c = indices[base + 2] * 3;
    for (int axis = 0; axis < 3; axis++) {
      float va = positions[a + axis];
      float vb = positions[b + axis];
      float vc = positions[c + axis];
      out[axis] = Math.min(va, Math.min(vb, vc));
      out[axis + 3] = Math.max(va, Math.max(vb, vc));
    }
  }
}
