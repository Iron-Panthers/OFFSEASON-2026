package frc.robot.utility.rendering;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import javax.imageio.ImageIO;

/**
 * Loads the 36h11 tag bitmaps that were pulled out of the AdvantageScope application bundle.
 *
 * <p>These are the genuine tag patterns, 10x10 pixels each: a one-cell white quiet zone, a one-cell
 * black border, and the 6x6 payload. Using the real bitmaps rather than a generated approximation
 * is what makes the rendered frames decodable by an actual AprilTag pipeline, which is the whole
 * point of putting tags in the scene.
 */
final class TagAtlas {

  private final Map<Integer, float[]> bitmaps = new HashMap<>();
  private int size = 10;

  /**
   * Reads every {@code NNN.png} in the given directory.
   *
   * @param directory the vendored {@code advantage_scope_files/AprilTag_36h11} folder
   */
  static TagAtlas load(Path directory) throws IOException {
    TagAtlas atlas = new TagAtlas();
    if (!Files.isDirectory(directory)) {
      throw new IOException("AprilTag bitmap directory not found: " + directory);
    }
    try (var entries = Files.list(directory)) {
      for (Path file : entries.toList()) {
        String name = file.getFileName().toString();
        if (!name.endsWith(".png")) {
          continue;
        }
        int id;
        try {
          id = Integer.parseInt(name.substring(0, name.length() - 4));
        } catch (NumberFormatException notATagBitmap) {
          continue;
        }
        atlas.read(id, file);
      }
    }
    if (atlas.bitmaps.isEmpty()) {
      throw new IOException("No AprilTag bitmaps found in " + directory);
    }
    return atlas;
  }

  private void read(int id, Path file) throws IOException {
    BufferedImage image = ImageIO.read(file.toFile());
    if (image == null) {
      throw new IOException("Could not decode tag bitmap " + file);
    }
    if (image.getWidth() != image.getHeight()) {
      throw new IOException("Tag bitmap " + file + " is not square");
    }
    size = image.getWidth();

    float[] cells = new float[size * size];
    for (int row = 0; row < size; row++) {
      for (int col = 0; col < size; col++) {
        // The bitmaps are pure black or pure white, so one channel decides the cell.
        boolean white = (image.getRGB(col, row) & 0xFF) > 127;
        cells[row * size + col] =
            white ? TagDecal.VINYL_REFLECTANCE : TagDecal.INK_REFLECTANCE;
      }
    }
    bitmaps.put(id, cells);
  }

  /**
   * @return the bitmap for a field tag ID, or null if that ID was not vendored
   */
  float[] bitmap(int id) {
    return bitmaps.get(id);
  }

  int size() {
    return size;
  }

  int count() {
    return bitmaps.size();
  }
}
