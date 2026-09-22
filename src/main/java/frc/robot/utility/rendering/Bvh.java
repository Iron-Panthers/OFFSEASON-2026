package frc.robot.utility.rendering;

import java.util.Arrays;

/**
 * Bounding volume hierarchy over the static field triangles, built with a binned surface area
 * heuristic.
 *
 * <p>Concrete rather than generic. This is the hot path for every primary, shadow, bounce and
 * occlusion ray, and keeping the triangle intersection inlined here rather than behind an interface
 * is worth the lost reuse. Moving fuel is indexed separately by {@link SphereIndex}.
 *
 * <p>Nodes live in flat arrays. Children are allocated in pairs so the right child is always at
 * {@code left + 1}, which halves the per-node bookkeeping. Every query takes a caller-owned {@link
 * Scratch}: at a few million rays per frame, allocating a traversal stack per ray costs more than
 * the traversal.
 */
final class Bvh {

  private static final int BIN_COUNT = 16;

  /** Below this a split cannot pay for itself, so stop without consulting the heuristic. */
  private static final int MIN_SPLIT_SIZE = 4;

  /** Leaves wider than this hurt traversal more than the extra nodes cost in memory. */
  private static final int MAX_LEAF_SIZE = 32;

  /** Rough cost of visiting a node relative to testing one triangle, for the SAH comparison. */
  private static final float TRAVERSAL_COST = 1.5f;

  /** Deep enough for any tree this build can produce over the field. */
  private static final int TRAVERSAL_STACK_DEPTH = 64;

  /** Six floats per node: min xyz then max xyz. */
  float[] nodeBounds;

  /** Interior nodes hold their left child index; leaves hold their first slot in {@link #order}. */
  int[] nodeLeftFirst;

  /** Triangle count for leaves, zero for interior nodes. */
  int[] nodeCount;

  /** Triangle indices, permuted so every leaf owns a contiguous run. */
  int[] order;

  int nodesUsed;

  private Bvh() {}

  /** Result of a closest-hit query. */
  static final class Hit {
    int triangle;
    float distance;

    /** Barycentric coordinates of the hit, for interpolating vertex normals. */
    float u;

    float v;

    void reset(float maxDistance) {
      triangle = -1;
      distance = maxDistance;
    }
  }

  /** Reusable per-thread traversal state. One instance per render worker. */
  static final class Scratch {
    final int[] stack = new int[TRAVERSAL_STACK_DEPTH];
    final Hit hit = new Hit();
    private final Hit shadowHit = new Hit();
  }

  // ---------------------------------------------------------------------------------------------
  // Build
  // ---------------------------------------------------------------------------------------------

  /** Centroid xyz per triangle, kept only for the duration of the build. */
  private float[] centroids;

  private final float[] binMin = new float[BIN_COUNT * 3];
  private final float[] binMax = new float[BIN_COUNT * 3];
  private final int[] binCount = new int[BIN_COUNT];
  private final float[] leftArea = new float[BIN_COUNT - 1];
  private final int[] leftCount = new int[BIN_COUNT - 1];
  private final float[] runningBox = new float[6];
  private final float[] triBounds = new float[6];
  private final float[] nodeBox = new float[6];

  static Bvh build(TriangleSoup soup) {
    int count = soup.triangleCount;
    Bvh bvh = new Bvh();
    bvh.order = new int[count];
    for (int i = 0; i < count; i++) {
      bvh.order[i] = i;
    }

    // A SAH tree over N triangles settles well under N nodes at these leaf sizes; grow if wrong.
    int capacity = Math.max(64, count / 2);
    bvh.nodeBounds = new float[capacity * 6];
    bvh.nodeLeftFirst = new int[capacity];
    bvh.nodeCount = new int[capacity];

    bvh.centroids = new float[count * 3];
    float[] scratch = new float[6];
    for (int t = 0; t < count; t++) {
      soup.triangleBounds(t, scratch);
      bvh.centroids[t * 3] = (scratch[0] + scratch[3]) * 0.5f;
      bvh.centroids[t * 3 + 1] = (scratch[1] + scratch[4]) * 0.5f;
      bvh.centroids[t * 3 + 2] = (scratch[2] + scratch[5]) * 0.5f;
    }

    bvh.nodesUsed = 1;
    bvh.buildTree(soup, count);
    bvh.centroids = null;
    bvh.trim();
    bvh.flattenOrder(soup);
    return bvh;
  }

  /**
   * Permutes the triangles into traversal order so {@link #order} becomes the identity.
   *
   * <p>The inner loop used to read {@code order[i]} to get a triangle, then that triangle's three
   * vertex indices, then those vertices. The first of those is a random access into an eleven
   * megabyte array on every triangle test, and it exists only because the build shuffles triangles
   * rather than moving them. Moving them once at the end removes the lookup outright and leaves
   * each leaf's indices contiguous in memory.
   */
  private void flattenOrder(TriangleSoup soup) {
    soup.permute(order);
    for (int i = 0; i < order.length; i++) {
      order[i] = i;
    }
  }

  /**
   * Builds every node from the root down.
   *
   * <p>Iterative with an explicit stack. Four million triangles is deep enough that recursion risks
   * the JVM stack on the unlucky splits.
   */
  private void buildTree(TriangleSoup soup, int triangleCount) {
    int[] stack = new int[192];
    int stackSize = 0;
    stack[stackSize++] = 0;
    stack[stackSize++] = 0;
    stack[stackSize++] = triangleCount;

    while (stackSize > 0) {
      int rangeEnd = stack[--stackSize];
      int rangeStart = stack[--stackSize];
      int node = stack[--stackSize];
      int count = rangeEnd - rangeStart;

      computeBounds(soup, rangeStart, rangeEnd, nodeBox);
      System.arraycopy(nodeBox, 0, nodeBounds, node * 6, 6);

      int split = count < MIN_SPLIT_SIZE ? -1 : findSplit(soup, rangeStart, rangeEnd);
      if (split < 0) {
        nodeLeftFirst[node] = rangeStart;
        nodeCount[node] = count;
        continue;
      }

      int left = allocatePair();
      nodeLeftFirst[node] = left;
      nodeCount[node] = 0;

      if (stackSize + 6 > stack.length) {
        stack = Arrays.copyOf(stack, stack.length * 2);
      }
      stack[stackSize++] = left;
      stack[stackSize++] = rangeStart;
      stack[stackSize++] = split;
      stack[stackSize++] = left + 1;
      stack[stackSize++] = split;
      stack[stackSize++] = rangeEnd;
    }
  }

  /**
   * Picks a split with a 16-bin surface area heuristic across all three axes and partitions {@link
   * #order} in place.
   *
   * @return the index {@code order} was partitioned about, or -1 if a leaf is cheaper
   */
  private int findSplit(TriangleSoup soup, int start, int end) {
    int count = end - start;
    float parentArea = surfaceArea(nodeBox);
    float bestCost = Float.POSITIVE_INFINITY;
    int bestAxis = -1;
    float bestPosition = 0;

    for (int axis = 0; axis < 3; axis++) {
      float centroidMin = Float.POSITIVE_INFINITY;
      float centroidMax = Float.NEGATIVE_INFINITY;
      for (int i = start; i < end; i++) {
        float c = centroids[order[i] * 3 + axis];
        centroidMin = Math.min(centroidMin, c);
        centroidMax = Math.max(centroidMax, c);
      }
      if (centroidMax - centroidMin < 1e-7f) {
        continue; // every centroid on this axis coincides, nothing to separate
      }

      Arrays.fill(binCount, 0);
      Arrays.fill(binMin, Float.POSITIVE_INFINITY);
      Arrays.fill(binMax, Float.NEGATIVE_INFINITY);
      float scale = BIN_COUNT / (centroidMax - centroidMin);

      for (int i = start; i < end; i++) {
        int triangle = order[i];
        int bin =
            Math.min(BIN_COUNT - 1, (int) ((centroids[triangle * 3 + axis] - centroidMin) * scale));
        binCount[bin]++;
        soup.triangleBounds(triangle, triBounds);
        for (int a = 0; a < 3; a++) {
          binMin[bin * 3 + a] = Math.min(binMin[bin * 3 + a], triBounds[a]);
          binMax[bin * 3 + a] = Math.max(binMax[bin * 3 + a], triBounds[a + 3]);
        }
      }

      // Sweep the bin boundaries, accumulating bounds from the left then back from the right.
      resetBox(runningBox);
      int runningCount = 0;
      for (int bin = 0; bin < BIN_COUNT - 1; bin++) {
        runningCount += binCount[bin];
        mergeBin(runningBox, bin);
        leftCount[bin] = runningCount;
        leftArea[bin] = surfaceArea(runningBox);
      }

      resetBox(runningBox);
      runningCount = 0;
      for (int bin = BIN_COUNT - 1; bin > 0; bin--) {
        runningCount += binCount[bin];
        mergeBin(runningBox, bin);
        int boundary = bin - 1;
        if (leftCount[boundary] == 0 || runningCount == 0) {
          continue;
        }
        float cost =
            TRAVERSAL_COST
                + (leftArea[boundary] * leftCount[boundary]
                        + surfaceArea(runningBox) * runningCount)
                    / parentArea;
        if (cost < bestCost) {
          bestCost = cost;
          bestAxis = axis;
          bestPosition = centroidMin + (boundary + 1) / scale;
        }
      }
    }

    if (bestAxis < 0 || (bestCost >= count && count <= MAX_LEAF_SIZE)) {
      return -1;
    }

    int split = partition(start, end, bestAxis, bestPosition);
    if (split == start || split == end) {
      // The heuristic chose a plane every centroid falls on one side of. Halve instead, so the
      // build always terminates even on degenerate geometry.
      if (count <= MAX_LEAF_SIZE) {
        return -1;
      }
      split = (start + end) >>> 1;
      medianSelect(start, end, split, bestAxis);
    }
    return split;
  }

  /** Hoare-style partition of {@link #order} about a centroid plane. */
  private int partition(int start, int end, int axis, float position) {
    int low = start;
    int high = end - 1;
    while (low <= high) {
      if (centroids[order[low] * 3 + axis] < position) {
        low++;
      } else {
        int swap = order[low];
        order[low] = order[high];
        order[high] = swap;
        high--;
      }
    }
    return low;
  }

  /** Partial sort so that {@code order[pivot]} lands where a full sort by centroid would put it. */
  private void medianSelect(int start, int end, int pivot, int axis) {
    int low = start;
    int high = end - 1;
    while (low < high) {
      float value = centroids[order[(low + high) >>> 1] * 3 + axis];
      int i = low;
      int j = high;
      while (i <= j) {
        while (centroids[order[i] * 3 + axis] < value) {
          i++;
        }
        while (centroids[order[j] * 3 + axis] > value) {
          j--;
        }
        if (i <= j) {
          int swap = order[i];
          order[i] = order[j];
          order[j] = swap;
          i++;
          j--;
        }
      }
      if (pivot <= j) {
        high = j;
      } else if (pivot >= i) {
        low = i;
      } else {
        return;
      }
    }
  }

  private void computeBounds(TriangleSoup soup, int start, int end, float[] out) {
    resetBox(out);
    for (int i = start; i < end; i++) {
      soup.triangleBounds(order[i], triBounds);
      for (int axis = 0; axis < 3; axis++) {
        out[axis] = Math.min(out[axis], triBounds[axis]);
        out[axis + 3] = Math.max(out[axis + 3], triBounds[axis + 3]);
      }
    }
  }

  private static void resetBox(float[] box) {
    box[0] = box[1] = box[2] = Float.POSITIVE_INFINITY;
    box[3] = box[4] = box[5] = Float.NEGATIVE_INFINITY;
  }

  private void mergeBin(float[] box, int bin) {
    for (int axis = 0; axis < 3; axis++) {
      box[axis] = Math.min(box[axis], binMin[bin * 3 + axis]);
      box[axis + 3] = Math.max(box[axis + 3], binMax[bin * 3 + axis]);
    }
  }

  private static float surfaceArea(float[] box) {
    float dx = box[3] - box[0];
    float dy = box[4] - box[1];
    float dz = box[5] - box[2];
    if (dx < 0 || dy < 0 || dz < 0) {
      return 0;
    }
    return 2f * (dx * dy + dy * dz + dz * dx);
  }

  private int allocatePair() {
    if (nodesUsed + 2 > nodeCount.length) {
      int grown = Math.max(nodesUsed + 2, nodeCount.length * 2);
      nodeBounds = Arrays.copyOf(nodeBounds, grown * 6);
      nodeLeftFirst = Arrays.copyOf(nodeLeftFirst, grown);
      nodeCount = Arrays.copyOf(nodeCount, grown);
    }
    int left = nodesUsed;
    nodesUsed += 2;
    return left;
  }

  private void trim() {
    if (nodesUsed < nodeCount.length) {
      nodeBounds = Arrays.copyOf(nodeBounds, nodesUsed * 6);
      nodeLeftFirst = Arrays.copyOf(nodeLeftFirst, nodesUsed);
      nodeCount = Arrays.copyOf(nodeCount, nodesUsed);
    }
  }

  /** Rebuilds an instance from cached arrays, skipping the SAH sweep entirely. */
  static Bvh fromArrays(float[] bounds, int[] leftFirst, int[] counts, int[] order, int nodesUsed) {
    Bvh bvh = new Bvh();
    bvh.nodeBounds = bounds;
    bvh.nodeLeftFirst = leftFirst;
    bvh.nodeCount = counts;
    bvh.order = order;
    bvh.nodesUsed = nodesUsed;
    return bvh;
  }

  // ---------------------------------------------------------------------------------------------
  // Traversal
  // ---------------------------------------------------------------------------------------------

  /**
   * Finds the nearest triangle along a ray.
   *
   * @return true if anything was hit closer than {@code maxDistance}, with the result in {@code
   *     scratch.hit}
   */
  boolean intersect(
      TriangleSoup soup,
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      float maxDistance,
      Scratch scratch) {
    Hit hit = scratch.hit;
    hit.reset(maxDistance);

    float invX = 1f / dx;
    float invY = 1f / dy;
    float invZ = 1f / dz;

    int[] stack = scratch.stack;
    int stackSize = 0;
    int node = 0;

    while (true) {
      int count = nodeCount[node];
      if (count > 0) {
        int first = nodeLeftFirst[node];
        for (int i = 0; i < count; i++) {
          intersectTriangle(soup, first + i, ox, oy, oz, dx, dy, dz, hit);
        }
      } else {
        int left = nodeLeftFirst[node];
        float tNear = slab(left, ox, oy, oz, invX, invY, invZ, hit.distance);
        float tFar = slab(left + 1, ox, oy, oz, invX, invY, invZ, hit.distance);

        // Visit the nearer child first, so the far one is more likely culled by the time we get
        // to it.
        int near = left;
        int far = left + 1;
        if (tFar < tNear) {
          near = left + 1;
          far = left;
          float swap = tNear;
          tNear = tFar;
          tFar = swap;
        }
        if (tNear < Float.POSITIVE_INFINITY) {
          if (tFar < Float.POSITIVE_INFINITY && stackSize < TRAVERSAL_STACK_DEPTH) {
            stack[stackSize++] = far;
          }
          node = near;
          continue;
        }
      }
      if (stackSize == 0) {
        break;
      }
      node = stack[--stackSize];
    }
    return hit.triangle >= 0;
  }

  /**
   * Measures how much light survives the path between two points.
   *
   * <p>Returns a fraction rather than a boolean so the polycarbonate field walls and the hub net
   * cast the partial shadows they really do instead of reading as opaque sheets.
   *
   * @return 1 for a clear path, 0 for fully blocked
   */
  float transmittance(
      TriangleSoup soup,
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      float maxDistance,
      Scratch scratch) {
    float invX = 1f / dx;
    float invY = 1f / dy;
    float invZ = 1f / dz;

    int[] stack = scratch.stack;
    Hit hit = scratch.shadowHit;
    int stackSize = 0;
    int node = 0;
    float transmission = 1f;

    while (true) {
      int count = nodeCount[node];
      if (count > 0) {
        int first = nodeLeftFirst[node];
        for (int i = 0; i < count; i++) {
          int triangle = first + i;
          hit.reset(maxDistance);
          intersectTriangle(soup, triangle, ox, oy, oz, dx, dy, dz, hit);
          if (hit.triangle < 0) {
            continue;
          }
          int surface = soup.surfaceIds[triangle];
          if (soup.surfaceOpaque[surface]) {
            return 0f;
          }
          transmission *= 1f - soup.surfaceAlpha[surface];
          if (transmission < 0.01f) {
            return 0f;
          }
        }
      } else {
        int left = nodeLeftFirst[node];
        boolean hitLeft =
            slab(left, ox, oy, oz, invX, invY, invZ, maxDistance) < Float.POSITIVE_INFINITY;
        boolean hitRight =
            slab(left + 1, ox, oy, oz, invX, invY, invZ, maxDistance) < Float.POSITIVE_INFINITY;
        if (hitLeft) {
          if (hitRight && stackSize < TRAVERSAL_STACK_DEPTH) {
            stack[stackSize++] = left + 1;
          }
          node = left;
          continue;
        }
        if (hitRight) {
          node = left + 1;
          continue;
        }
      }
      if (stackSize == 0) {
        break;
      }
      node = stack[--stackSize];
    }
    return transmission;
  }

  /** Slab test. Returns the near intersection distance, or infinity when the ray misses. */
  private float slab(
      int node,
      float ox,
      float oy,
      float oz,
      float invX,
      float invY,
      float invZ,
      float maxDistance) {
    int base = node * 6;
    float tx1 = (nodeBounds[base] - ox) * invX;
    float tx2 = (nodeBounds[base + 3] - ox) * invX;
    float tMin = Math.min(tx1, tx2);
    float tMax = Math.max(tx1, tx2);

    float ty1 = (nodeBounds[base + 1] - oy) * invY;
    float ty2 = (nodeBounds[base + 4] - oy) * invY;
    tMin = Math.max(tMin, Math.min(ty1, ty2));
    tMax = Math.min(tMax, Math.max(ty1, ty2));

    float tz1 = (nodeBounds[base + 2] - oz) * invZ;
    float tz2 = (nodeBounds[base + 5] - oz) * invZ;
    tMin = Math.max(tMin, Math.min(tz1, tz2));
    tMax = Math.min(tMax, Math.max(tz1, tz2));

    float entry = Math.max(tMin, 0f);
    return tMax >= entry && tMin < maxDistance ? entry : Float.POSITIVE_INFINITY;
  }

  /** Moller-Trumbore, double sided because the field export marks every material double sided. */
  private static void intersectTriangle(
      TriangleSoup soup,
      int triangle,
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      Hit hit) {
    int base = triangle * 3;
    int ia = soup.indices[base] * 3;
    int ib = soup.indices[base + 1] * 3;
    int ic = soup.indices[base + 2] * 3;

    float ax = soup.positions[ia];
    float ay = soup.positions[ia + 1];
    float az = soup.positions[ia + 2];

    float e1x = soup.positions[ib] - ax;
    float e1y = soup.positions[ib + 1] - ay;
    float e1z = soup.positions[ib + 2] - az;
    float e2x = soup.positions[ic] - ax;
    float e2y = soup.positions[ic + 1] - ay;
    float e2z = soup.positions[ic + 2] - az;

    float px = dy * e2z - dz * e2y;
    float py = dz * e2x - dx * e2z;
    float pz = dx * e2y - dy * e2x;
    float determinant = e1x * px + e1y * py + e1z * pz;
    if (determinant > -1e-9f && determinant < 1e-9f) {
      return;
    }

    float inverse = 1f / determinant;
    float tx = ox - ax;
    float ty = oy - ay;
    float tz = oz - az;
    float u = (tx * px + ty * py + tz * pz) * inverse;
    if (u < 0f || u > 1f) {
      return;
    }

    float qx = ty * e1z - tz * e1y;
    float qy = tz * e1x - tx * e1z;
    float qz = tx * e1y - ty * e1x;
    float v = (dx * qx + dy * qy + dz * qz) * inverse;
    if (v < 0f || u + v > 1f) {
      return;
    }

    float distance = (e2x * qx + e2y * qy + e2z * qz) * inverse;
    if (distance > 1e-4f && distance < hit.distance) {
      hit.distance = distance;
      hit.triangle = triangle;
      hit.u = u;
      hit.v = v;
    }
  }
}
