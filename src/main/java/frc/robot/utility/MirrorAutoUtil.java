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
import com.fasterxml.jackson.databind.node.ObjectNode;
import java.io.File;
import java.io.IOException;

/**
 * Utility class to mirror PathPlanner auto files to use left-side (mirrored) paths.
 *
 * <p>Recursively finds all "path" type commands and updates their pathName to reference the left
 * version (appends " Left" to the path name).
 */
public class MirrorAutoUtil {
  private static final ObjectMapper MAPPER = new ObjectMapper();

  /** Recursively finds and updates all path references in a command structure. */
  private static void mirrorPathsInCommand(JsonNode command) {
    if (command == null || !command.isObject()) return;

    ObjectNode cmdObj = (ObjectNode) command;

    // If this is a path command, update the pathName
    if (cmdObj.has("type") && "path".equals(cmdObj.get("type").asText())) {
      JsonNode data = cmdObj.get("data");
      if (data != null && data.isObject() && data.has("pathName")) {
        ObjectNode dataObj = (ObjectNode) data;
        String pathName = dataObj.get("pathName").asText();
        // Only add " Left" if not already a left variant
        if (!pathName.endsWith(" Left")) {
          // Strip " Right" suffix if present before appending " Left"
          if (pathName.endsWith(" Right")) {
            pathName = pathName.substring(0, pathName.length() - 6);
          }
          dataObj.put("pathName", pathName + " Left");
        }
      }
    }

    // If this command has nested commands (like sequential, parallel, etc.), recurse
    if (cmdObj.has("data")) {
      JsonNode data = cmdObj.get("data");
      if (data != null && data.isObject() && data.has("commands")) {
        JsonNode commands = data.get("commands");
        if (commands != null && commands.isArray()) {
          for (JsonNode nestedCmd : commands) {
            mirrorPathsInCommand(nestedCmd);
          }
        }
      }
    }
  }

  /**
   * Mirrors a single auto file: reads the .auto file, updates path references, writes "*
   * Left.auto".
   */
  public static void mirrorAutoFile(String inputPath) throws IOException {
    File inputFile = new File(inputPath);
    if (!inputFile.exists()) {
      throw new IOException("Input file does not exist: " + inputPath);
    }

    JsonNode root = MAPPER.readTree(inputFile);
    if (!root.isObject()) {
      throw new IOException("Auto file is not a JSON object: " + inputPath);
    }

    // Deep copy so we don't modify the original tree while writing
    ObjectNode copy = (ObjectNode) MAPPER.readTree(MAPPER.writeValueAsString(root));

    // Recursively update all path references
    if (copy.has("command")) {
      mirrorPathsInCommand(copy.get("command"));
    }

    // Set folder to "Left Autos (auto generated)"
    copy.put("folder", "Left Autos (auto generated)");

    // Generate output filename
    String parent = inputFile.getParent();
    String name = inputFile.getName();
    int dot = name.lastIndexOf('.');
    String baseName = dot > 0 ? name.substring(0, dot) : name;
    String ext = dot > 0 ? name.substring(dot) : "";
    // Strip " Right" suffix if present before appending " Left"
    if (baseName.endsWith(" Right")) {
      baseName = baseName.substring(0, baseName.length() - 6);
    }
    File outputFile = new File(parent, baseName + " Left" + ext);

    MAPPER.writerWithDefaultPrettyPrinter().writeValue(outputFile, copy);
  }

  /** Main for Gradle JavaExec: each argument is a path to a .auto file. */
  public static void main(String[] args) {
    if (args.length == 0) {
      System.err.println("Usage: MirrorAutoUtil <path-to-.auto-file> [ ... ]");
      System.exit(1);
    }
    for (String path : args) {
      try {
        mirrorAutoFile(path);
      } catch (IOException e) {
        System.err.println("Error processing " + path + ": " + e.getMessage());
        e.printStackTrace();
        System.exit(1);
      }
    }
  }
}
