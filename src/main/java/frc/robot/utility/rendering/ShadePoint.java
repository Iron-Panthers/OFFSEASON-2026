package frc.robot.utility.rendering;

/**
 * A resolved ray hit, reused rather than allocated.
 *
 * <p>Mutable and thread-confined on purpose: a frame resolves several million hits, and returning a
 * record from each one is the difference between the renderer being allocation-free in steady state
 * and it spending most of its time in the collector.
 */
final class ShadePoint {

  float x;
  float y;
  float z;

  /** Unit shading normal, already flipped to face the incoming ray. */
  float normalX;

  float normalY;
  float normalZ;

  float distance;

  Surface surface;

  void faceForward(float dx, float dy, float dz) {
    if (normalX * dx + normalY * dy + normalZ * dz > 0) {
      normalX = -normalX;
      normalY = -normalY;
      normalZ = -normalZ;
    }
  }
}
