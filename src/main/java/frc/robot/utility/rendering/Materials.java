package frc.robot.utility.rendering;

/**
 * Turns AdvantageScope field materials into surfaces that survive being lit.
 *
 * <p>The exporter stamps every single material in the 2026 field model with {@code
 * metallicFactor = 1.0} and {@code roughnessFactor = 0.212}, and ships no textures at all. Rendered
 * literally that makes the carpet, the vinyl banners and the polycarbonate walls all behave like
 * polished chrome, which is the difference between a frame that looks like a photo and one that
 * looks like a screensaver. So the only usable signal is the base colour plus the CAD part name on
 * the owning node, and every rule below is keyed on one of those two.
 *
 * <p>The mapping was read off the actual model rather than guessed: each case names the part
 * families that carry that material and how many triangles they account for.
 */
final class Materials {

  private Materials() {}

  /** Nothing in the export is a perfect mirror, and near-zero roughness aliases badly. */
  private static final float MIN_ROUGHNESS = 0.06f;

  /**
   * The hub carries {@code GE-26314: Hub Light Mount} behind its diffuser panels, so they are lit
   * rather than merely white. Kept modest so the panels read as illuminated without clipping the
   * auto-exposure meter on the rest of the frame.
   */
  private static final float DIFFUSER_EMISSIVE = 2.2f;

  /**
   * Maps one glTF material plus the CAD part name it was used on to a renderable surface.
   *
   * @param material base colour from the export; its metallic and roughness are discarded
   * @param nodeName nearest named ancestor node, e.g. {@code "GE-26213: Trench Side Skirt"}
   */
  static Surface resolve(Glb.Material material, String nodeName) {
    float red = srgbToLinear(material.red());
    float green = srgbToLinear(material.green());
    float blue = srgbToLinear(material.blue());
    float alpha = material.alpha();
    String name = nodeName == null ? "" : nodeName;

    // --- Surfaces identified by part name, which beats colour where the two disagree. ---

    // FE-2026-01: Playing Field Carpet. One 22.5 m x 9.1 m slab, mat_4 at 0.22 grey.
    if (name.contains("Playing Field Carpet")) {
      return Surface.opaque(red, green, blue, 0f, 0.95f).withPattern(Surface.Pattern.CARPET);
    }

    // FE-00083-ID<n>-Vinyl. Printed vinyl; the caller attaches the tag bitmap and basis.
    if (name.contains("AprilTag Vinyl")) {
      return Surface.opaque(0.75f, 0.75f, 0.74f, 0f, 0.55f);
    }

    // GE-263xx Hub diffusers and the standalone Diffuser part; backlit acrylic.
    if (name.contains("Diffuser")) {
      return Surface.opaque(red, green, blue, 0f, 0.42f).withEmissive(DIFFUSER_EMISSIVE);
    }

    // Gaffer, hook, loop and carpet seaming tape. Cloth backing, no specular to speak of.
    if (name.contains("Tape")) {
      return Surface.opaque(red, green, blue, 0f, 0.88f);
    }

    // GE-26318: Hub Net. Modelled as a solid sheet, so the alpha carries the open weave.
    if (name.contains("Net")) {
      return Surface.transparent(red, green, blue, 0.7f, alpha);
    }

    // Zinc-plated fasteners: PEM inserts (679 k tris), hex bolts, thread-forming screws.
    if (name.startsWith("PEM ")
        || name.contains("Hex bolt")
        || name.contains("Screw_")
        || name.contains("Rivet")) {
      return Surface.opaque(red, green, blue, 1f, 0.34f);
    }

    // REV bearings and unthreaded spacers; the bronze-toned mat_27 keeps its own colour.
    if (name.contains("Bearing") || name.contains("Spacer")) {
      return Surface.opaque(red, green, blue, 1f, 0.38f);
    }

    // Wheel tread and the gas springs, both matte rubber.
    if (name.contains("Tread") || name.contains("Gas Spring") || name.contains("Cable Tie")) {
      return Surface.opaque(red, green, blue, 0f, 0.72f);
    }

    // Printed vinyl wraps over the alliance-coloured structures, plus sponsor panels.
    if (name.contains("Vinyl") || name.contains("Sponsor")) {
      return Surface.opaque(red, green, blue, 0f, 0.48f);
    }

    // --- Anything left over falls through to colour-based rules. ---

    // Polycarbonate field walls, driver station acrylic, AprilTag backing poly.
    if (alpha < 0.999f) {
      return Surface.transparent(red, green, blue, 0.08f, alpha);
    }

    // mat_9, the blue-grey of plated steel hardware that escaped the name rules.
    if (isNear(material, 0.32f, 0.38f, 0.43f)) {
      return Surface.opaque(red, green, blue, 1f, 0.32f);
    }

    // mat_8 and mat_18, mill-finish and lightly anodised aluminium extrusion and plate.
    if (isNear(material, 0.79f, 0.79f, 0.79f) || isNear(material, 0.82f, 0.82f, 0.82f)) {
      return Surface.opaque(red, green, blue, 1f, 0.36f);
    }

    // mat_15 and mat_21, machined aluminium that reads a little duller than extrusion.
    if (isNear(material, 0.45f, 0.45f, 0.45f) || isNear(material, 0.60f, 0.60f, 0.60f)) {
      return Surface.opaque(red, green, blue, 1f, 0.44f);
    }

    // The near-blacks, mat_6 / mat_14 / mat_17 / mat_20: moulded plastic and rubber, never metal.
    if (red < 0.02f && green < 0.02f && blue < 0.02f) {
      return Surface.opaque(red, green, blue, 0f, 0.62f);
    }

    // Saturated alliance red and blue, and the FIRST orange: painted or wrapped plastic.
    if (isSaturated(red, green, blue)) {
      return pigment(red, green, blue, 0.45f);
    }

    // Remaining neutral greys are structural metal.
    return Surface.opaque(red, green, blue, 1f, 0.40f);
  }

  /**
   * Peak reflectance of a saturated pigment.
   *
   * <p>The export writes alliance red as pure {@code (1, 0, 0)} and alliance blue as pure
   * {@code (0, 0, 1)}. No real pigment behaves like that: a bright vinyl wrap returns roughly 40
   * percent of the light in its own band, and never returns zero in the others. Rendered
   * literally the walls clip to pure primaries, which both looks wrong and destroys two of the
   * three colour channels for anything trying to white balance or threshold against them.
   */
  private static final float PIGMENT_PEAK_REFLECTANCE = 0.42f;

  /** Even the deepest pigment scatters a little broadband light. */
  private static final float PIGMENT_FLOOR = 0.022f;

  private static Surface pigment(float red, float green, float blue, float roughness) {
    float peak = Math.max(red, Math.max(green, blue));
    if (peak <= 1e-4f) {
      return Surface.opaque(red, green, blue, 0f, roughness);
    }
    float scale = PIGMENT_PEAK_REFLECTANCE / peak;
    return Surface.opaque(
        PIGMENT_FLOOR + red * scale,
        PIGMENT_FLOOR + green * scale,
        PIGMENT_FLOOR + blue * scale,
        0f,
        roughness);
  }

  /** Clamps a resolved roughness into the range the GGX evaluation stays well behaved over. */
  static float clampRoughness(float roughness) {
    return Math.min(1f, Math.max(MIN_ROUGHNESS, roughness));
  }

  /** Compares against an exporter base colour, which is authored in sRGB. */
  private static boolean isNear(Glb.Material material, float red, float green, float blue) {
    return Math.abs(material.red() - red) < 0.02f
        && Math.abs(material.green() - green) < 0.02f
        && Math.abs(material.blue() - blue) < 0.02f;
  }

  private static boolean isSaturated(float red, float green, float blue) {
    float max = Math.max(red, Math.max(green, blue));
    float min = Math.min(red, Math.min(green, blue));
    return max > 0.05f && (max - min) > 0.25f * max;
  }

  /**
   * glTF base colour factors are authored in sRGB by this exporter even though the spec calls for
   * linear, which is why the raw 0.22 grey carpet looks washed out if used directly.
   */
  static float srgbToLinear(float channel) {
    return channel <= 0.04045f
        ? channel / 12.92f
        : (float) Math.pow((channel + 0.055f) / 1.055f, 2.4);
  }
}
