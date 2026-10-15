package frc.robot.utility.rendering;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.nio.file.Path;

/**
 * A robot chassis, loaded once in its own frame and then instanced wherever a robot stands.
 *
 * <p>The ego robot matters more than it looks like it should. A camera bolted to the robot sees its
 * own bumpers and intake at the edges of every frame, and those edges occlude the field. Leaving
 * the robot out would produce frames with a clean view in every direction, which is exactly the
 * kind of easy data that makes a detector look better in simulation than it is on the field.
 *
 * <p>Only the chassis mesh is rendered. The AdvantageScope export also ships articulated components
 * as separate files, but those need live mechanism positions to place correctly, so moving parts
 * are out of frame for now.
 */
final class RobotModel {

  private final TriangleSoup soup;
  private final Bvh bvh;

  private RobotModel(TriangleSoup soup, Bvh bvh) {
    this.soup = soup;
    this.bvh = bvh;
  }

  int triangleCount() {
    return soup.triangleCount;
  }

  /**
   * Loads a robot chassis from an AdvantageScope robot asset folder.
   *
   * <p>Those assets are authored Y-up and facing along the model's own Z, and the folder's {@code
   * config.json} carries the rotations that stand them up. For every 2026 robot asset that works
   * out to X+90 then Z+90, which maps glTF {@code (x, y, z)} onto robot {@code (z, x, y)}: robot +X
   * forward, +Y left, +Z up, matching WPILib.
   *
   * @param modelFile the {@code model.glb} inside the robot asset folder
   */
  static RobotModel load(Path modelFile) throws IOException {
    Glb.Model model = Glb.load(modelFile);
    TriangleSoup.Builder builder = new TriangleSoup.Builder();

    for (Glb.Mesh mesh : model.meshes()) {
      float[] positions = mesh.positions();
      float[] normals = mesh.normals();
      for (int v = 0; v < positions.length; v += 3) {
        float x = positions[v];
        float y = positions[v + 1];
        float z = positions[v + 2];
        positions[v] = z;
        positions[v + 1] = x;
        positions[v + 2] = y;

        float nx = normals[v];
        float ny = normals[v + 1];
        float nz = normals[v + 2];
        normals[v] = nz;
        normals[v + 1] = nx;
        normals[v + 2] = ny;
      }

      Glb.Material material =
          mesh.material() >= 0 && mesh.material() < model.materials().size()
              ? model.materials().get(mesh.material())
              : new Glb.Material("", 0.4f, 0.4f, 0.4f, 1f);
      builder.add(positions, normals, mesh.indices(), Materials.resolve(material, mesh.nodeName()));
    }

    TriangleSoup soup = builder.build();
    return new RobotModel(soup, Bvh.build(soup));
  }

  /** Places this model at a pose for one frame. */
  Instance at(Pose3d pose) {
    return new Instance(pose);
  }

  /**
   * One placement of the model.
   *
   * <p>Rays are pulled into the model's frame rather than the geometry being pushed into the world,
   * so the tree is built once and reused no matter how many robots there are or how they move. The
   * transform is a rigid motion, so distances survive it untouched and hit distances compare
   * directly against everything else in the scene.
   */
  final class Instance {
    /** Rotation rows, which double as the columns of its inverse. */
    private final double[] rotation = new double[9];

    private final double translationX;
    private final double translationY;
    private final double translationZ;

    private Instance(Pose3d pose) {
      Rotation3d r = pose.getRotation();
      Translation3d column0 = new Translation3d(1, 0, 0).rotateBy(r);
      Translation3d column1 = new Translation3d(0, 1, 0).rotateBy(r);
      Translation3d column2 = new Translation3d(0, 0, 1).rotateBy(r);
      rotation[0] = column0.getX();
      rotation[1] = column1.getX();
      rotation[2] = column2.getX();
      rotation[3] = column0.getY();
      rotation[4] = column1.getY();
      rotation[5] = column2.getY();
      rotation[6] = column0.getZ();
      rotation[7] = column1.getZ();
      rotation[8] = column2.getZ();

      translationX = pose.getX();
      translationY = pose.getY();
      translationZ = pose.getZ();
    }

    /**
     * @return true if this robot was hit closer than {@code maxDistance}, filling {@code out}
     */
    boolean intersect(
        float ox,
        float oy,
        float oz,
        float dx,
        float dy,
        float dz,
        float maxDistance,
        DynamicScene.Query query,
        ShadePoint out) {

      double px = ox - translationX;
      double py = oy - translationY;
      double pz = oz - translationZ;
      // Multiply by the transpose to invert the rotation.
      float localOriginX = (float) (rotation[0] * px + rotation[3] * py + rotation[6] * pz);
      float localOriginY = (float) (rotation[1] * px + rotation[4] * py + rotation[7] * pz);
      float localOriginZ = (float) (rotation[2] * px + rotation[5] * py + rotation[8] * pz);
      float localDirX = (float) (rotation[0] * dx + rotation[3] * dy + rotation[6] * dz);
      float localDirY = (float) (rotation[1] * dx + rotation[4] * dy + rotation[7] * dz);
      float localDirZ = (float) (rotation[2] * dx + rotation[5] * dy + rotation[8] * dz);

      if (!bvh.intersect(
          soup,
          localOriginX,
          localOriginY,
          localOriginZ,
          localDirX,
          localDirY,
          localDirZ,
          maxDistance,
          query.robotScratch)) {
        return false;
      }

      Bvh.Hit hit = query.robotScratch.hit;
      interpolateNormal(hit, query.localNormal);
      out.distance = hit.distance;
      out.normalX =
          (float)
              (rotation[0] * query.localNormal[0]
                  + rotation[1] * query.localNormal[1]
                  + rotation[2] * query.localNormal[2]);
      out.normalY =
          (float)
              (rotation[3] * query.localNormal[0]
                  + rotation[4] * query.localNormal[1]
                  + rotation[5] * query.localNormal[2]);
      out.normalZ =
          (float)
              (rotation[6] * query.localNormal[0]
                  + rotation[7] * query.localNormal[1]
                  + rotation[8] * query.localNormal[2]);
      out.surface = soup.surfaces.get(soup.surfaceIds[hit.triangle]);
      return true;
    }

    private void interpolateNormal(Bvh.Hit hit, float[] out) {
      int base = hit.triangle * 3;
      int a = soup.indices[base] * 3;
      int b = soup.indices[base + 1] * 3;
      int c = soup.indices[base + 2] * 3;
      float w = 1f - hit.u - hit.v;
      for (int axis = 0; axis < 3; axis++) {
        out[axis] =
            soup.normals[a + axis] * w
                + soup.normals[b + axis] * hit.u
                + soup.normals[c + axis] * hit.v;
      }
      float length = (float) Math.sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
      if (length > 1e-9f) {
        out[0] /= length;
        out[1] /= length;
        out[2] /= length;
      }
    }
  }
}
