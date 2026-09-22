package frc.robot.utility.rendering;

/**
 * Physically based description of one field surface.
 *
 * <p>Colours are linear, not sRGB. The renderer works in linear light throughout and only encodes
 * to sRGB in {@link Film}, so any value written here must already be linear.
 *
 * @param red linear base colour
 * @param green linear base colour
 * @param blue linear base colour
 * @param metallic 0 for dielectrics, 1 for bare metal; values between are only meaningful where a
 *     single material index covers both, which the field export does in a few places
 * @param roughness GGX roughness, clamped away from 0 so perfectly sharp highlights never alias
 * @param alpha below 1 for the polycarbonate panels and the hub net
 * @param emissive linear radiance added on top, used for the lit hub diffusers
 * @param pattern surface detail applied procedurally, since the export carries no textures or UVs
 * @param decal AprilTag placement, non-null only when {@code pattern} is {@link Pattern#APRIL_TAG}
 */
record Surface(
    float red,
    float green,
    float blue,
    float metallic,
    float roughness,
    float alpha,
    float emissive,
    Surface.Pattern pattern,
    TagDecal decal) {

  /** Procedural detail layered on top of the flat base colour. */
  enum Pattern {
    /** Flat base colour. */
    NONE,
    /**
     * Woven carpet. The export models the carpet as a single featureless slab, and a featureless
     * floor is the most obvious tell in a camera mounted 45 cm off the ground, so the fibre detail
     * has to be generated.
     */
    CARPET,
    /** A 36h11 tag bitmap projected onto the vinyl panel it is printed on. */
    APRIL_TAG
  }

  static Surface opaque(float red, float green, float blue, float metallic, float roughness) {
    return new Surface(red, green, blue, metallic, roughness, 1f, 0f, Pattern.NONE, null);
  }

  static Surface transparent(
      float red, float green, float blue, float roughness, float alpha) {
    return new Surface(red, green, blue, 0f, roughness, alpha, 0f, Pattern.NONE, null);
  }

  Surface withPattern(Pattern newPattern) {
    return new Surface(
        red, green, blue, metallic, roughness, alpha, emissive, newPattern, decal);
  }

  Surface withDecal(TagDecal newDecal) {
    return new Surface(
        red, green, blue, metallic, roughness, alpha, emissive, Pattern.APRIL_TAG, newDecal);
  }

  Surface withEmissive(float newEmissive) {
    return new Surface(
        red, green, blue, metallic, roughness, alpha, newEmissive, pattern, decal);
  }

  boolean isTransparent() {
    return alpha < 0.999f;
  }
}
