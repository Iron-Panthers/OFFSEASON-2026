package frc.robot.utility.rendering;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Assembles the static half of the scene from the vendored AdvantageScope field export.
 *
 * <p>Three things happen here that are easy to get wrong and expensive to debug later, so each is
 * checked rather than assumed: the coordinate change into WPILib field space, the removal of the
 * 455 staged fuel balls that the export bakes into the field mesh, and the placement of the real
 * 36h11 bitmaps onto the tag panels the CAD model already positions correctly.
 */
final class FieldScene {

  /** Node names for the staged fuel that ships inside the field mesh. */
  private static final String STAGED_FUEL_NODE = "Fuel";

  /** Names the tag panels as {@code FE-00083-ID13-Vinyl: AprilTag Vinyl}. */
  private static final Pattern TAG_PANEL = Pattern.compile("-ID(\\d+)-Vinyl");

  /**
   * World size of the 10x10 tag image.
   *
   * <p>A 36h11 tag is 10 cells across, of which the 8 inner cells are the 6.5 in black-bordered
   * square that defines the tag. The printed image is therefore 6.5 * 10 / 8 inches.
   */
  private static final double TAG_IMAGE_SIZE_METERS = Units.inchesToMeters(6.5 * 10.0 / 8.0);

  final TriangleSoup soup;
  final Bvh bvh;
  final double fieldLength;
  final double fieldWidth;

  /** Largest gap between a rendered tag panel and where the field layout says that tag is. */
  final double tagAlignmentErrorMeters;

  private FieldScene(
      TriangleSoup soup,
      Bvh bvh,
      double fieldLength,
      double fieldWidth,
      double tagAlignmentErrorMeters) {
    this.soup = soup;
    this.bvh = bvh;
    this.fieldLength = fieldLength;
    this.fieldWidth = fieldWidth;
    this.tagAlignmentErrorMeters = tagAlignmentErrorMeters;
  }

  /**
   * Loads the field, optionally reusing a cached triangle soup and BVH.
   *
   * @param fieldDirectory vendored {@code advantage_scope_files/Field3d_2026FRCFieldV2}
   * @param tagDirectory vendored {@code advantage_scope_files/AprilTag_36h11}
   * @param layout the same field layout the vision code uses, which defines world origin and size
   * @param cacheDirectory where to keep the built BVH, or null to rebuild every time
   */
  static FieldScene load(
      Path fieldDirectory, Path tagDirectory, AprilTagFieldLayout layout, Path cacheDirectory)
      throws IOException {

    Path modelFile = fieldDirectory.resolve("model.glb");
    Path configFile = fieldDirectory.resolve("config.json");
    double fieldLength = layout.getFieldLength();
    double fieldWidth = layout.getFieldWidth();

    SceneCache cache = cacheDirectory == null ? null : new SceneCache(cacheDirectory);
    String key = cache == null ? null : SceneCache.keyFor(modelFile, configFile, tagDirectory);

    if (cache != null) {
      SceneCache.Entry cached = cache.read(key);
      if (cached != null) {
        return new FieldScene(
            cached.soup(), cached.bvh(), fieldLength, fieldWidth, cached.tagAlignmentError());
      }
    }

    Glb.Model model = Glb.load(modelFile);
    TagAtlas tags = TagAtlas.load(tagDirectory);
    Map<Integer, Double> tagFacing = readTagFacing(configFile);

    TriangleSoup.Builder builder = new TriangleSoup.Builder();
    double worstTagError = 0;

    for (Glb.Mesh mesh : model.meshes()) {
      if (mesh.nodeName().contains(STAGED_FUEL_NODE)) {
        // Fuel is drawn from the live simulation as spheres, so the staged balls would be both
        // wrong and 295 k triangles of duplicate work.
        continue;
      }

      toFieldCoordinates(mesh.positions(), mesh.normals(), fieldLength, fieldWidth);

      Glb.Material material =
          mesh.material() >= 0 && mesh.material() < model.materials().size()
              ? model.materials().get(mesh.material())
              : new Glb.Material("", 0.5f, 0.5f, 0.5f, 1f);
      Surface surface = Materials.resolve(material, mesh.nodeName());

      Matcher tagMatch = TAG_PANEL.matcher(mesh.nodeName());
      if (tagMatch.find()) {
        int id = Integer.parseInt(tagMatch.group(1));
        float[] bitmap = tags.bitmap(id);
        Double facingDegrees = tagFacing.get(id);
        if (bitmap != null && facingDegrees != null) {
          TagDecal decal = buildDecal(id, bitmap, tags.size(), mesh.positions(), facingDegrees);
          surface = surface.withDecal(decal);
          worstTagError = Math.max(worstTagError, tagPlacementError(layout, id, decal));
        }
      }

      builder.add(mesh.positions(), mesh.normals(), mesh.indices(), surface);
    }

    TriangleSoup soup = builder.build();
    Bvh bvh = Bvh.build(soup);

    if (cache != null) {
      cache.write(key, soup, bvh, worstTagError);
    }
    return new FieldScene(soup, bvh, fieldLength, fieldWidth, worstTagError);
  }

  /**
   * Rewrites vertices from the export frame into WPILib field coordinates, in place.
   *
   * <p>The export is glTF Y-up and centred on the field, and AdvantageScope presents it
   * {@code wall-blue}, which puts the blue alliance wall at negative X with the field running away
   * to the right. WPILib instead puts the origin at the blue wall corner with X running toward the
   * red alliance, so both horizontal axes reverse:
   *
   * <pre>
   *   X =  -gltfX + fieldLength / 2
   *   Y =   gltfZ + fieldWidth / 2
   *   Z =   gltfY
   * </pre>
   *
   * <p>That linear part is a proper rotation, so winding and handedness survive. The mapping was
   * not assumed: it is the only one of the four sign choices that lines all 32 tag panels up with
   * {@code AprilTagFieldLayout}, and it does so to 0.8 mm. {@link FieldSceneTest} re-checks it.
   */
  private static void toFieldCoordinates(
      float[] positions, float[] normals, double fieldLength, double fieldWidth) {
    float halfLength = (float) (fieldLength / 2.0);
    float halfWidth = (float) (fieldWidth / 2.0);

    for (int v = 0; v < positions.length; v += 3) {
      float x = positions[v];
      float y = positions[v + 1];
      float z = positions[v + 2];
      positions[v] = -x + halfLength;
      positions[v + 1] = z + halfWidth;
      positions[v + 2] = y;

      float nx = normals[v];
      float ny = normals[v + 1];
      float nz = normals[v + 2];
      normals[v] = -nx;
      normals[v + 1] = nz;
      normals[v + 2] = ny;
    }
  }

  /**
   * Reads each tag's facing from the AdvantageScope field config.
   *
   * <p>The config gives every tag a single rotation about Z. Comparing the values against the tag
   * positions shows the convention is that an unrotated tag faces +X, so the outward normal is
   * {@code (cos, sin, 0)}. Taking the facing from the config rather than from the panel mesh
   * matters because a flat panel does not say which of its two faces is the printed one.
   *
   * @return tag ID to rotation about Z in degrees
   */
  private static Map<Integer, Double> readTagFacing(Path configFile) throws IOException {
    Map<Integer, Double> facing = new HashMap<>();
    JsonNode config = new ObjectMapper().readTree(Files.readAllBytes(configFile));
    for (JsonNode tag : config.path("aprilTags")) {
      double degrees = 0;
      for (JsonNode rotation : tag.path("rotations")) {
        if ("z".equals(rotation.path("axis").asText())) {
          degrees = rotation.path("degrees").asDouble();
        }
      }
      facing.put(tag.path("id").asInt(), degrees);
    }
    return facing;
  }

  /**
   * Builds the decal basis for one tag panel from the panel geometry plus its facing.
   *
   * <p>Image rows run downward and columns run to the viewer's right, which for a viewer looking at
   * a tag whose outward normal is {@code n} means {@code axisU = up x n} and {@code axisV = -up}.
   */
  private static TagDecal buildDecal(
      int id, float[] bitmap, int size, float[] positions, double facingDegrees) {
    double minX = Double.POSITIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double minZ = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    double maxZ = Double.NEGATIVE_INFINITY;
    for (int v = 0; v < positions.length; v += 3) {
      minX = Math.min(minX, positions[v]);
      maxX = Math.max(maxX, positions[v]);
      minY = Math.min(minY, positions[v + 1]);
      maxY = Math.max(maxY, positions[v + 1]);
      minZ = Math.min(minZ, positions[v + 2]);
      maxZ = Math.max(maxZ, positions[v + 2]);
    }

    // The config rotation is stated in the export frame, so the outward normal picks up the same
    // reversal of both horizontal axes that the vertices do in toFieldCoordinates.
    double facing = Math.toRadians(facingDegrees);
    double[] normal = {-Math.cos(facing), -Math.sin(facing), 0};
    // up x n, with up = +Z.
    double[] axisU = {-normal[1], normal[0], 0};
    double length = Math.hypot(axisU[0], axisU[1]);
    if (length < 1e-9) {
      axisU = new double[] {0, 1, 0};
    } else {
      axisU[0] /= length;
      axisU[1] /= length;
    }
    double[] axisV = {0, 0, -1};

    double[] center = {(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2};
    return new TagDecal(id, bitmap, size, center, axisU, axisV, TAG_IMAGE_SIZE_METERS);
  }

  /**
   * Distance between a rendered tag panel and the pose the vision code believes that tag has.
   *
   * <p>A large value means the field model and the field layout disagree, which would make every
   * pose estimate derived from these frames quietly wrong. Cheap to compute, so it is always
   * computed and reported.
   */
  private static double tagPlacementError(AprilTagFieldLayout layout, int id, TagDecal decal) {
    Optional<Pose3d> pose = layout.getTagPose(id);
    if (pose.isEmpty()) {
      return 0;
    }
    return pose
        .get()
        .getTranslation()
        .getDistance(
            new edu.wpi.first.math.geometry.Translation3d(
                decal.center()[0], decal.center()[1], decal.center()[2]));
  }
}
