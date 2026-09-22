package frc.robot.utility.rendering;

import java.util.Arrays;

/**
 * Spatial index over the fuel currently on the field.
 *
 * <p>Fuel is rendered as true spheres rather than as the 1244-triangle ball in the export. A sphere
 * has an exact silhouette at any distance and never facets, which matters because these frames are
 * meant to be fed to ball detectors: a subtly polygonal ball would be a cue that does not exist in
 * a real camera frame.
 *
 * <p>Rebuilt every frame with a median split rather than a surface area heuristic. There are only a
 * few hundred balls and they move, so build time matters more than traversal quality.
 */
final class SphereIndex {

  private static final int LEAF_SIZE = 4;

  /** All fuel is the same size; {@code FuelSim} models a 150 mm ball. */
  static final float FUEL_RADIUS = 0.075f;

  private final float[] centers;
  private final int count;
  private final float radius;

  private final float[] nodeBounds;
  private final int[] nodeLeftFirst;
  private final int[] nodeCount;
  private final int[] order;
  private int nodesUsed;

  /** Result of a closest-hit query against the fuel. */
  static final class Hit {
    int sphere;
    float distance;
    float normalX;
    float normalY;
    float normalZ;
  }

  /**
   * @param centers xyz per sphere in WPILib field coordinates
   * @param count number of spheres in {@code centers}
   */
  SphereIndex(float[] centers, int count, float radius) {
    this.centers = centers;
    this.count = count;
    this.radius = radius;

    int capacity = Math.max(2, count * 2);
    this.nodeBounds = new float[capacity * 6];
    this.nodeLeftFirst = new int[capacity];
    this.nodeCount = new int[capacity];
    this.order = new int[count];
    for (int i = 0; i < count; i++) {
      order[i] = i;
    }
    this.nodesUsed = 1;
    if (count > 0) {
      build();
    } else {
      // An empty index still has to answer queries; give the root a box nothing can hit.
      Arrays.fill(nodeBounds, 0, 6, Float.NaN);
      nodeCount[0] = 0;
      nodeLeftFirst[0] = 0;
    }
  }

  private void build() {
    int[] stack = new int[128];
    int stackSize = 0;
    stack[stackSize++] = 0;
    stack[stackSize++] = 0;
    stack[stackSize++] = count;

    while (stackSize > 0) {
      int end = stack[--stackSize];
      int start = stack[--stackSize];
      int node = stack[--stackSize];
      int size = end - start;

      float minX = Float.POSITIVE_INFINITY;
      float minY = Float.POSITIVE_INFINITY;
      float minZ = Float.POSITIVE_INFINITY;
      float maxX = Float.NEGATIVE_INFINITY;
      float maxY = Float.NEGATIVE_INFINITY;
      float maxZ = Float.NEGATIVE_INFINITY;
      for (int i = start; i < end; i++) {
        int s = order[i] * 3;
        minX = Math.min(minX, centers[s] - radius);
        minY = Math.min(minY, centers[s + 1] - radius);
        minZ = Math.min(minZ, centers[s + 2] - radius);
        maxX = Math.max(maxX, centers[s] + radius);
        maxY = Math.max(maxY, centers[s + 1] + radius);
        maxZ = Math.max(maxZ, centers[s + 2] + radius);
      }
      int base = node * 6;
      nodeBounds[base] = minX;
      nodeBounds[base + 1] = minY;
      nodeBounds[base + 2] = minZ;
      nodeBounds[base + 3] = maxX;
      nodeBounds[base + 4] = maxY;
      nodeBounds[base + 5] = maxZ;

      if (size <= LEAF_SIZE) {
        nodeLeftFirst[node] = start;
        nodeCount[node] = size;
        continue;
      }

      int axis = 0;
      float spanX = maxX - minX;
      float spanY = maxY - minY;
      float spanZ = maxZ - minZ;
      if (spanY > spanX && spanY >= spanZ) {
        axis = 1;
      } else if (spanZ > spanX && spanZ > spanY) {
        axis = 2;
      }

      int middle = (start + end) >>> 1;
      sortRangeByAxis(start, end, axis);

      int left = nodesUsed;
      nodesUsed += 2;
      nodeLeftFirst[node] = left;
      nodeCount[node] = 0;

      if (stackSize + 6 > stack.length) {
        stack = Arrays.copyOf(stack, stack.length * 2);
      }
      stack[stackSize++] = left;
      stack[stackSize++] = start;
      stack[stackSize++] = middle;
      stack[stackSize++] = left + 1;
      stack[stackSize++] = middle;
      stack[stackSize++] = end;
    }
  }

  /** Insertion sort; ranges here are small and already partly ordered by the parent split. */
  private void sortRangeByAxis(int start, int end, int axis) {
    for (int i = start + 1; i < end; i++) {
      int value = order[i];
      float key = centers[value * 3 + axis];
      int j = i - 1;
      while (j >= start && centers[order[j] * 3 + axis] > key) {
        order[j + 1] = order[j];
        j--;
      }
      order[j + 1] = value;
    }
  }

  int count() {
    return count;
  }

  /**
   * Finds the nearest fuel along a ray.
   *
   * @param hit receives the result; only meaningful when this returns true
   * @return true if a ball was hit closer than {@code maxDistance}
   */
  boolean intersect(
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      float maxDistance,
      Hit hit,
      int[] stack) {
    if (count == 0) {
      return false;
    }
    hit.sphere = -1;
    hit.distance = maxDistance;

    float invX = 1f / dx;
    float invY = 1f / dy;
    float invZ = 1f / dz;
    int stackSize = 0;
    int node = 0;

    while (true) {
      int leafSize = nodeCount[node];
      if (leafSize > 0) {
        int first = nodeLeftFirst[node];
        for (int i = 0; i < leafSize; i++) {
          intersectSphere(order[first + i], ox, oy, oz, dx, dy, dz, hit);
        }
      } else {
        // A zero count only ever means an interior node: leaf ranges are never empty.
        int left = nodeLeftFirst[node];
        boolean hitLeft = slab(left, ox, oy, oz, invX, invY, invZ, hit.distance);
        boolean hitRight = slab(left + 1, ox, oy, oz, invX, invY, invZ, hit.distance);
        if (hitLeft) {
          if (hitRight && stackSize < stack.length) {
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

    if (hit.sphere >= 0) {
      float hx = ox + dx * hit.distance;
      float hy = oy + dy * hit.distance;
      float hz = oz + dz * hit.distance;
      int s = hit.sphere * 3;
      hit.normalX = (hx - centers[s]) / radius;
      hit.normalY = (hy - centers[s + 1]) / radius;
      hit.normalZ = (hz - centers[s + 2]) / radius;
      return true;
    }
    return false;
  }

  /** Fuel is opaque, so occlusion is a plain boolean. */
  boolean occluded(
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      float maxDistance,
      Hit scratch,
      int[] stack) {
    return intersect(ox, oy, oz, dx, dy, dz, maxDistance, scratch, stack);
  }

  private boolean slab(
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

    return tMax >= Math.max(tMin, 0f) && tMin < maxDistance;
  }

  private void intersectSphere(
      int sphere, float ox, float oy, float oz, float dx, float dy, float dz, Hit hit) {
    int s = sphere * 3;
    float cx = ox - centers[s];
    float cy = oy - centers[s + 1];
    float cz = oz - centers[s + 2];

    // Ray directions are unit length, so the quadratic leading coefficient is 1.
    float half = cx * dx + cy * dy + cz * dz;
    float c = cx * cx + cy * cy + cz * cz - radius * radius;
    float discriminant = half * half - c;
    if (discriminant < 0) {
      return;
    }

    float root = (float) Math.sqrt(discriminant);
    float distance = -half - root;
    if (distance < 1e-4f) {
      distance = -half + root; // origin is inside the ball
    }
    if (distance > 1e-4f && distance < hit.distance) {
      hit.distance = distance;
      hit.sphere = sphere;
    }
  }
}
