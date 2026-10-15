package frc.robot.utility.rendering;

import java.util.ArrayList;
import java.util.List;

/**
 * The part of the scene that moves: fuel on the carpet and the robots pushing it around.
 *
 * <p>Rebuilt from a {@link SceneSnapshot} once per frame and then read-only, so every ray in a
 * frame sees the same instant. The static field lives in {@link FieldScene} and is never rebuilt.
 *
 * <p>Robots are instanced rather than baked: the ray is pulled into each robot's local frame and
 * tested against that model's own tree. There are only ever a handful of robots, so a flat loop
 * beats maintaining a top level tree over them.
 */
final class DynamicScene {

  /** Linear albedo of a fuel ball, converted from the orange in the export. */
  static final Surface FUEL_SURFACE =
      Surface.opaque(
          Materials.srgbToLinear(0.9559733f),
          Materials.srgbToLinear(0.5972018f),
          Materials.srgbToLinear(0.0561285f),
          0f,
          0.52f);

  private final SphereIndex fuel;
  private final List<RobotModel.Instance> robots;

  private DynamicScene(SphereIndex fuel, List<RobotModel.Instance> robots) {
    this.fuel = fuel;
    this.robots = robots;
  }

  /**
   * Builds the frame's dynamic geometry.
   *
   * @param egoModel the robot the cameras are mounted on, or null to leave it out
   * @param opponentModel model used for the other robots on the field, or null to leave them out
   */
  static DynamicScene from(SceneSnapshot snapshot, RobotModel egoModel, RobotModel opponentModel) {
    SphereIndex fuel =
        new SphereIndex(snapshot.fuelCenters(), snapshot.fuelCount(), SphereIndex.FUEL_RADIUS);

    List<RobotModel.Instance> robots = new ArrayList<>();
    if (egoModel != null) {
      robots.add(egoModel.at(snapshot.robotPose()));
    }
    if (opponentModel != null) {
      for (var pose : snapshot.opponents()) {
        robots.add(opponentModel.at(pose));
      }
    }
    return new DynamicScene(fuel, robots);
  }

  int fuelCount() {
    return fuel.count();
  }

  /** Scratch a render thread needs to query this scene. */
  static final class Query {
    final SphereIndex.Hit sphereHit = new SphereIndex.Hit();
    final int[] sphereStack = new int[64];
    final Bvh.Scratch robotScratch = new Bvh.Scratch();
    final float[] localOrigin = new float[3];
    final float[] localDirection = new float[3];
    final float[] localNormal = new float[3];
  }

  /**
   * Finds the nearest moving surface along a ray.
   *
   * @param out receives position, normal and surface when something is hit
   * @return distance to the hit, or {@code maxDistance} if nothing was closer
   */
  float intersect(
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      float maxDistance,
      Query query,
      ShadePoint out) {

    float closest = maxDistance;
    boolean found = false;

    if (fuel.intersect(ox, oy, oz, dx, dy, dz, closest, query.sphereHit, query.sphereStack)) {
      closest = query.sphereHit.distance;
      out.normalX = query.sphereHit.normalX;
      out.normalY = query.sphereHit.normalY;
      out.normalZ = query.sphereHit.normalZ;
      out.surface = FUEL_SURFACE;
      found = true;
    }

    for (RobotModel.Instance robot : robots) {
      if (robot.intersect(ox, oy, oz, dx, dy, dz, closest, query, out)) {
        closest = out.distance;
        found = true;
      }
    }

    if (found) {
      out.distance = closest;
      out.x = ox + dx * closest;
      out.y = oy + dy * closest;
      out.z = oz + dz * closest;
    }
    return found ? closest : maxDistance;
  }

  /** True if anything moving blocks the path. Fuel and robots are both opaque. */
  boolean occluded(
      float ox,
      float oy,
      float oz,
      float dx,
      float dy,
      float dz,
      float maxDistance,
      Query query,
      ShadePoint scratch) {
    if (fuel.occluded(ox, oy, oz, dx, dy, dz, maxDistance, query.sphereHit, query.sphereStack)) {
      return true;
    }
    for (RobotModel.Instance robot : robots) {
      if (robot.intersect(ox, oy, oz, dx, dy, dz, maxDistance, query, scratch)) {
        return true;
      }
    }
    return false;
  }
}
