// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.utility;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.File;
import java.io.IOException;

/**
 * Utility class to mirror PathPlanner paths across the field centerline. FRC field width is 8.21 m,
 * so centerline is at y = 4.105 m.
 *
 * <p>Uses Jackson's JsonNode / ObjectNode (tree model) to read and write the path JSON.
 */
public class MirrorPathUtil {
  private static final double FIELD_WIDTH_METERS = 8.07;
  private static final ObjectMapper MAPPER = new ObjectMapper();

  /** Mirrors a Y coordinate across the field centerline. */
  private static double mirrorY(double y) {
    return FIELD_WIDTH_METERS - y;
  }

  /** Normalizes an angle in degrees to [-180, 180). */
  private static double normalizeAngleDeg(double deg) {
    return ((deg + 180) % 360 + 360) % 360 - 180;
  }

  /** Mirrors a rotation angle across the horizontal centerline. */
  private static double mirrorRotationDeg(double deg) {
    return normalizeAngleDeg(-deg);
  }

  /** Mirrors a point node (object with "x" and "y") in place. */
  private static void mirrorPointInPlace(ObjectNode point) {
    if (point == null || !point.has("y")) return;
    point.put("y", mirrorY(point.get("y").asDouble()));
  }

  /**
   * Converts a linked waypoint name from the Right path to the Left path form: e.g. "Shooting Pose
   * Right" -> "Shooting Pose Left"
   */
  private static String mirrorLinkedName(String linkedName) {
    if (linkedName == null) return null;
    if (linkedName.endsWith(" Right")) {
      return linkedName.substring(0, linkedName.length() - 6) + " Left";
    }
    return linkedName;
  }

  /**
   * Mirrors a PathPlanner path (modifies the given root in place). When add180 is true (path name
   * ends with "I Right"): rotation targets, point-towards zones, and ideal starting state are
   * mirrored and rotated 180°; goal end state is only mirrored. When add180 is false: all rotations
   * are only mirrored (no +180°). Linked waypoints become "___ Left".
   */
  private static void mirrorPathInPlace(ObjectNode root, boolean add180) {
    // Mirror waypoints
    JsonNode waypoints = root.get("waypoints");
    if (waypoints != null && waypoints.isArray()) {
      ArrayNode arr = (ArrayNode) waypoints;
      for (JsonNode wp : arr) {
        if (wp.isObject()) {
          ObjectNode wpObj = (ObjectNode) wp;
          if (wpObj.has("anchor")) mirrorPointInPlace((ObjectNode) wpObj.get("anchor"));
          if (wpObj.has("prevControl") && !wpObj.get("prevControl").isNull()) {
            mirrorPointInPlace((ObjectNode) wpObj.get("prevControl"));
          }
          if (wpObj.has("nextControl") && !wpObj.get("nextControl").isNull()) {
            mirrorPointInPlace((ObjectNode) wpObj.get("nextControl"));
          }
          // Linked waypoints: use "___ Left" instead of referencing the other side
          if (wpObj.has("linkedName") && !wpObj.get("linkedName").isNull()) {
            String name = wpObj.get("linkedName").asText();
            wpObj.put("linkedName", mirrorLinkedName(name));
          }
        }
      }
    }

    double rotationOffset = add180 ? 180 : 0;

    // Mirror rotation targets (+180° only when path ends with "I Right")
    JsonNode rotationTargets = root.get("rotationTargets");
    if (rotationTargets != null && rotationTargets.isArray()) {
      for (JsonNode rt : rotationTargets) {
        if (rt.isObject() && rt.has("rotationDegrees")) {
          ((ObjectNode) rt)
              .put(
                  "rotationDegrees",
                  mirrorRotationDeg(rt.get("rotationDegrees").asDouble()) + rotationOffset);
        }
      }
    }

    // Point towards zones: mirror fieldPosition and rotation (+180° only when path ends with "I
    // Right")
    JsonNode pointTowardsZones = root.get("pointTowardsZones");
    if (pointTowardsZones != null && pointTowardsZones.isArray()) {
      for (JsonNode zone : pointTowardsZones) {
        if (zone.isObject()) {
          ObjectNode zoneObj = (ObjectNode) zone;
          if (zoneObj.has("fieldPosition") && zoneObj.get("fieldPosition").isObject()) {
            mirrorPointInPlace((ObjectNode) zoneObj.get("fieldPosition"));
          }
          if (zoneObj.has("rotationOffset")) {
            zoneObj.put(
                "rotationOffset",
                mirrorRotationDeg(zoneObj.get("rotationOffset").asDouble()) + rotationOffset);
          }
        }
      }
    }

    // Goal end state: only mirror rotation (no +180°)
    if (root.has("goalEndState") && root.get("goalEndState").isObject()) {
      ObjectNode ges = (ObjectNode) root.get("goalEndState");
      if (ges.has("rotation"))
        ges.put("rotation", mirrorRotationDeg(ges.get("rotation").asDouble()));
    }

    // Ideal starting state (+180° only when path ends with "I Right")
    if (root.has("idealStartingState") && root.get("idealStartingState").isObject()) {
      ObjectNode iss = (ObjectNode) root.get("idealStartingState");
      if (iss.has("rotation"))
        iss.put("rotation", mirrorRotationDeg(iss.get("rotation").asDouble()) + rotationOffset);
    }

    root.put("folder", "Left Paths (auto generated)");
  }

  /**
   * Mirrors a single path file: reads the .path file, mirrors it, writes "* Left.path". Only
   * processes paths that end with " Right". Paths ending with "I Right" get +180° applied to
   * rotation targets, point-towards zones, and ideal starting state; other Right paths do not.
   */
  public static void mirrorPathFile(String inputPath) throws IOException {
    File inputFile = new File(inputPath);
    if (!inputFile.exists()) {
      throw new IOException("Input file does not exist: " + inputPath);
    }

    String name = inputFile.getName();
    int dot = name.lastIndexOf('.');
    String baseName = dot > 0 ? name.substring(0, dot) : name;

    // Only mirror paths that end with " Right"
    if (!baseName.endsWith(" Right")) {
      return;
    }

    // +180° only when path name ends with "I Right"
    boolean add180 = baseName.endsWith("I Right");

    JsonNode root = MAPPER.readTree(inputFile);
    if (!root.isObject()) {
      throw new IOException("Path file is not a JSON object: " + inputPath);
    }

    // Deep copy so we don't modify the original tree while writing
    ObjectNode copy = (ObjectNode) MAPPER.readTree(MAPPER.writeValueAsString(root));
    mirrorPathInPlace(copy, add180);

    String parent = inputFile.getParent();
    String ext = dot > 0 ? name.substring(dot) : "";
    // Strip " Right" suffix before appending " Left"
    String baseWithoutRight = baseName.substring(0, baseName.length() - 6);
    File outputFile = new File(parent, baseWithoutRight + " Left" + ext);

    MAPPER.writerWithDefaultPrettyPrinter().writeValue(outputFile, copy);
  }

  /** Main for Gradle JavaExec: each argument is a path to a .path file. */
  public static void main(String[] args) {
    if (args.length == 0) {
      System.err.println("Usage: MirrorPathUtil <path-to-.path-file> [ ... ]");
      System.exit(1);
    }
    for (String path : args) {
      try {
        mirrorPathFile(path);
      } catch (IOException e) {
        System.err.println("Error processing " + path + ": " + e.getMessage());
        e.printStackTrace();
        System.exit(1);
      }
    }
  }
}
