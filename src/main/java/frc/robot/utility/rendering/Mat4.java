package frc.robot.utility.rendering;

/**
 * Minimal 4x4 affine transform, enough to flatten a glTF node hierarchy into world space.
 *
 * <p>Stored row-major. glTF hands out column-major matrices, so {@link #fromGltfMatrix} transposes
 * on the way in. Doubles are used because the transforms are composed once at load time and the
 * field model spans 22 m, where accumulated float error in a deep node chain is visible.
 */
final class Mat4 {
  /** Row-major 4x4; the bottom row is always (0, 0, 0, 1) for the affine transforms glTF uses. */
  private final double[] m;

  private Mat4(double[] rowMajor) {
    this.m = rowMajor;
  }

  static Mat4 identity() {
    return new Mat4(new double[] {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1});
  }

  /** Builds from a glTF {@code node.matrix}, which is 16 doubles in column-major order. */
  static Mat4 fromGltfMatrix(double[] columnMajor) {
    double[] r = new double[16];
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 4; col++) {
        r[row * 4 + col] = columnMajor[col * 4 + row];
      }
    }
    return new Mat4(r);
  }

  /**
   * Builds from glTF {@code node.translation} / {@code rotation} / {@code scale}. The spec composes
   * these as T * R * S.
   *
   * @param t translation, or null for none
   * @param q rotation quaternion as (x, y, z, w), or null for none
   * @param s scale, or null for none
   */
  static Mat4 fromTrs(double[] t, double[] q, double[] s) {
    double[] r = new double[16];
    // Rotation block from the quaternion, identity when absent.
    double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0, wx = 0, wy = 0, wz = 0;
    if (q != null) {
      double x = q[0], y = q[1], z = q[2], w = q[3];
      xx = x * x;
      xy = x * y;
      xz = x * z;
      yy = y * y;
      yz = y * z;
      zz = z * z;
      wx = w * x;
      wy = w * y;
      wz = w * z;
    }
    double[][] rot =
        q == null
            ? new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
            : new double[][] {
              {1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)},
              {2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)},
              {2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)}
            };
    double sx = s == null ? 1 : s[0];
    double sy = s == null ? 1 : s[1];
    double sz = s == null ? 1 : s[2];
    double[] scale = {sx, sy, sz};
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        r[row * 4 + col] = rot[row][col] * scale[col];
      }
      r[row * 4 + 3] = t == null ? 0 : t[row];
    }
    r[15] = 1;
    return new Mat4(r);
  }

  /** Returns {@code this * other}, i.e. apply {@code other} first then {@code this}. */
  Mat4 times(Mat4 other) {
    double[] r = new double[16];
    for (int row = 0; row < 4; row++) {
      for (int col = 0; col < 4; col++) {
        double sum = 0;
        for (int k = 0; k < 4; k++) {
          sum += m[row * 4 + k] * other.m[k * 4 + col];
        }
        r[row * 4 + col] = sum;
      }
    }
    return new Mat4(r);
  }

  /** Transforms a point, applying translation. Writes x, y, z into {@code out}. */
  void applyToPoint(double x, double y, double z, double[] out) {
    out[0] = m[0] * x + m[1] * y + m[2] * z + m[3];
    out[1] = m[4] * x + m[5] * y + m[6] * z + m[7];
    out[2] = m[8] * x + m[9] * y + m[10] * z + m[11];
  }

  /**
   * Transforms a direction, ignoring translation.
   *
   * <p>This uses the matrix itself rather than its inverse-transpose, so it is only correct for
   * normals under rotation and uniform scale. Every transform in the AdvantageScope field export is
   * a rigid placement of a CAD part, so that holds; {@link Glb} renormalises afterwards regardless.
   */
  void applyToDirection(double x, double y, double z, double[] out) {
    out[0] = m[0] * x + m[1] * y + m[2] * z;
    out[1] = m[4] * x + m[5] * y + m[6] * z;
    out[2] = m[8] * x + m[9] * y + m[10] * z;
  }
}
