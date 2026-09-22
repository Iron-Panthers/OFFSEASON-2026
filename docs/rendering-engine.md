# Rendering Engine

Renders what the robot's cameras would see during a match and serves each one as an MJPEG stream on
localhost, so vision and object-detection work can be developed against simulated footage.

Simulation only. The engine refuses to start outside `Mode.SIM`, so none of it is ever loaded on a
roboRIO.

## Running it

```bash
./gradlew simulateJava -Prender
```

There are two modes. **Fast** is the default and is what you want for watching the robot drive or
collecting a lot of frames. **High** is the reference path tracer, five to nine times slower, for
when the frames matter more than the rate.

```bash
./gradlew simulateJava -Prender                    # fast
./gradlew simulateJava -Prender -Prender.mode=high # reference path tracer
```

Each camera in `VisionConstants.CAMERA_TRANSFORM` gets its own port starting at **1191**:

| URL | What it is |
| --- | --- |
| `http://localhost:1191/` | A page with the stream embedded. Open this first. |
| `http://localhost:1191/stream.mjpg` | `multipart/x-mixed-replace` MJPEG, same shape PhotonVision serves. |
| `http://localhost:1191/snapshot.jpg` | The most recent frame, once. |
| `http://localhost:1191/info.json` | Resolution, frames rendered, viewers connected. |

Camera 1 is on 1192, camera 2 on 1193, and so on. **Not 1181**: the PhotonVision simulation running
in the same process already binds 1181 upward for its own streams. If a port is taken the engine
walks upward until it finds a free one and prints where each camera actually landed.

A camera is only rendered while something is asking for it — a connected stream client, or a
snapshot request within the last five seconds. With three cameras registered and one browser tab
open, you get roughly three times the frame rate you otherwise would.

### Options

All are `-P` flags on `simulateJava`.

| Flag | Default | Notes |
| --- | --- | --- |
| `-Prender.mode` | `fast` | `fast` or `high`. |
| `-Prender.width` / `-Prender.height` | 480 x 300 | Frame time scales with pixel count, so this is the biggest dial. |
| `-Prender.spp` | 2 | Samples per pixel. In fast mode this buys anti-aliasing only, since the lighting carries no noise. |
| `-Prender.bounces` | 0 fast / 1 high | Traced indirect bounces. Fast mode reads indirect light from the bake instead. |
| `-Prender.transparency` | 1 fast / 3 high | How many polycarbonate panels a ray may see through. |
| `-Prender.denoise` | off fast / on high | Fast mode has no Monte Carlo noise to remove, and filtering would only soften tags. |
| `-Prender.gain` | 1.0 | Sensor noise multiplier. Raise it to test a pipeline against a bad exposure. |
| `-Prender.jpeg` | 0.82 | JPEG quality. |
| `-Prender.port` | 1191 | First port to try. |

**Resolution sets tag detection range**, so raise it for AprilTag work. At the simulated camera's
own 1280x800 tags decode out to 5 m; at the 480x300 default they are good to roughly 2.5-3 m.

```bash
# The frames matter more than the frame rate
./gradlew simulateJava -Prender -Prender.mode=high -Prender.width=1280 -Prender.height=800 -Prender.spp=4
```

## Assets

Vendored under `advantage_scope_files/`, pulled from a local AdvantageScope install:

| Path | Contents |
| --- | --- |
| `Field3d_2026FRCFieldV2/model.glb` | The 2026 field. 18 MB, 4.28 M triangles, 28 materials, no textures. |
| `Field3d_2026FRCFieldV2/model_0.glb` | The Fuel ball. Kept for reference; the renderer draws fuel analytically. |
| `Field3d_2026FRCFieldV2/config.json` | Tag poses, field size, game piece definitions. |
| `AprilTag_36h11/0NN.png` | The real 36h11 bitmaps, 10x10 px, extracted from the AdvantageScope bundle and SHA256-verified against its own index. |

## The two modes

|  | fast (default) | high |
| --- | --- | --- |
| Static shadows and indirect light | read from a baked `IrradianceVolume` | traced per pixel |
| Contact shadows from balls and robots | traced, against moving geometry only | traced |
| Primary visibility | ray traced | ray traced |
| Lens distortion, ball silhouettes, tag sharpness | identical | identical |

Fast mode bakes the static lighting once into a grid of 108k cells, about 1.6 s, cached to disk
next to the geometry. Each cell holds an ambient cube: six incoming radiance values, one per axis.
Shading then reads and interpolates that instead of tracing eight rays.

The 20 cm cells sound coarse and are not, for this scene specifically. A 3.8 m fixture 8 m up casts
a penumbra around half a metre wide, so the real shadows on this field are already softer than the
grid. What a grid cannot represent is a shadow sharper than its own cells, and the only sharp
shadows here are the ones under fuel, which are still traced live.

What fast mode gives up is the physical accuracy of the lighting: specular highlights are broader,
because the reflection direction reads the same band-limited volume, and light leaks slightly
across thin geometry where a cell straddles both sides of it. What it does not touch is anything
geometric, which is why the tag decode tests pass in both modes.

## How it works

```
FuelSim / RobotSimState  ->  SceneSnapshot  ->  Renderer  ->  Denoiser  ->  Film  ->  MjpegServer
       (robot thread)        (atomic ref)      (worker pool, off the robot thread)
```

The robot loop only publishes an immutable snapshot into an `AtomicReference`, costing it
microseconds. Rendering runs on its own lower-priority threads at whatever rate it manages. The
20 ms loop is never blocked on a frame, and a frame is never torn across two simulation instants.

Primary visibility, shadows, ambient occlusion and one diffuse bounce are all traced against a
surface-area-heuristic BVH over the field. Nothing is a screen-space approximation: contact shadows
sit where geometry actually touches, and the carpet picks up colour from the alliance walls next to
it.

Fuel is rendered as **analytic spheres**, not as the 1244-triangle ball in the export. A sphere has
an exact silhouette at any distance and never facets, which matters if these frames are going to
train a ball detector.

### Things worth knowing

**AdvantageScope stamps every field material `metallic = 1.0, roughness = 0.212`, and ships no
textures.** Rendered literally, the carpet is polished chrome. `Materials` remaps every material
using its base colour and the CAD part name on the owning node; the mapping was read off the model
rather than guessed, and each rule names the part families it covers. It also rewrites the pure
`(1,0,0)` and `(0,0,1)` alliance colours into plausible pigment reflectances, since no real paint
returns zero in two channels.

**The coordinate change is verified, not assumed.** The export is glTF Y-up and centred on the
field, presented `wall-blue`. The mapping to WPILib coordinates is

```
X = -gltfX + fieldLength / 2
Y =  gltfZ + fieldWidth / 2
Z =  gltfY
```

which is the only one of the four sign choices that lines all 32 tag panels up with
`AprilTagFieldLayout` — and it does so to 0.8 mm. `FieldSceneTest` re-checks it on every build.

**The field model bakes 455 staged fuel balls into its mesh.** Those nodes are stripped at load;
fuel comes from the live `FuelSim` instead.

**The first start builds a BVH over ~4 M triangles and then caches it** to `build/render-cache/`.
Later starts load in well under a second. The cache key covers the asset files *and* the compiled
classes that interpret them, so editing `Materials` invalidates it automatically.

### Camera realism

`Film` is not decoration. A vision pipeline never sees radiance; it sees an auto-exposed, tone
mapped, vignetted, noisy, lens-distorted JPEG, and a threshold tuned against clean data is a
threshold that fails at an event. The pipeline models:

- Brown-Conrady radial distortion, applied by bending the ray rather than warping the image
- Metered log-average auto exposure with the same lag a real AE loop has
- ACES tone mapping
- Photon shot noise plus read noise, so bright areas are proportionally cleaner than dark ones
- Lateral chromatic aberration and cosine-fourth vignetting
- JPEG compression at a controlled quality

## Tests

| Test | What it protects |
| --- | --- |
| `FieldSceneTest` | Geometry lands in the right place, staged fuel is gone, all 32 tags are decalled and within 20 mm of the layout, the carpet is not metal. |
| `TagDecodeTest` | **WPILib's own AprilTag detector reads the rendered tags back** and gets the ID the field layout says should be there, out to 5 m, at the real camera resolution. This is the test that proves the frames are usable rather than merely convincing. |
| `RenderPreviewTest` | Three fixed viewpoints render, are sensibly exposed and have real contrast. Also writes them to `build/render-preview/` so a person can look. Pinned to high mode, since it exists to judge the best the renderer can do. |

Fast mode decodes tags too, and slightly more strongly than the path tracer does: margin 137
against 128 at the same range, because there is no sampling noise in the lighting to fight.

Tag detection currently holds to **5 m** at 1280x800 through a 70 degree lens, with decision
margins above 100. That is an optical limit rather than a rendering one: a 6.5 inch tag subtends
about 38 px at that range, and neither more samples nor more resolution changes the margin.

## Performance

Every number below is a whole frame at 640x400, including denoise, tone mapping and JPEG encode,
measured by pinning the renderer to a thread count on one machine. Treat the thread counts as a
stand-in for machine size rather than as exact laptop figures.

| Threads | high (path traced) | fast, 2 spp | fast, 1 spp |
| --- | --- | --- | --- |
| 4 | 0.6 FPS | 3.7 | **7.2** |
| 8 | 1.1 FPS | 4.7 | **8.7** |
| 24 | 1.7 FPS | 8.7 | **15.0** |

At the 480x300 default:

| Threads | fast, 2 spp | fast, 1 spp |
| --- | --- | --- |
| 4 | 5.8 FPS | ~11 |
| 24 | 16.1 FPS | ~28 |

**Those are standalone figures. Running alongside the simulation costs roughly half.** Measured
live through the MJPEG stream on a 24 core machine with the physics running: 8.1 FPS at 480x300.
The renderer deliberately leaves a core free and runs below the simulation's priority, and
maple-sim plus the fuel simulation are not cheap themselves. So a four core laptop should expect
something closer to 3-4 FPS live than to 5.8.

Two things that are *not* the cause, both checked: the instanced ego robot model is free (181 ms
against 173 ms, inside the noise) and thread scaling is healthy (11.3x on 24 threads).

### Where the time goes, and what did not help

Profiling the path tracer at 640x400 on four threads put the frame at 1124 ms:

| Stage | Cost | Share |
| --- | --- | --- |
| Shadow rays, six fixtures | ~515 ms | 45% |
| Diffuse bounce and its shadow ray | ~285 ms | 25% |
| Per-hit shading maths | ~175 ms | 16% |
| Primary rays | ~150 ms | 13% |

Fast mode exists because the first two of those recompute something that cannot change. It removes
them and is five to nine times faster as a result.

These were tried and **rejected on measurement**, and are recorded so nobody spends the afternoon
again:

- Shrinking BVH node memory from 93 MB to 9 MB by growing the leaves: **no change at all**. The
  traversal is not bound by node memory.
- Removing the `order[]` indirection from the triangle loop, so leaves read contiguously: **no
  change**.
- Culling the 32% of triangles that are rivets and PEM nuts: 1.29x on eight threads, *slower* on
  four, inside the run-to-run noise. Available as a flag, not enabled.
- Replacing `Math.pow` in the shading hot path: 2.5%. Kept, but it is not a lever.
- The procedural surface noise: 3%. The sphere index at 300 balls: 14%.

What remains is primary ray traversal at roughly 0.85 M rays per second per thread, which is about
what an unvectorised Java BVH costs. Going meaningfully past it means either fewer pixels or
rasterising primary visibility, and rasterising would cost the exact lens distortion and analytic
ball silhouettes that make these frames worth rendering in the first place.

## If it still is not fast enough

In order of effort:

1. `-Prender.spp=1`. Fast mode carries no lighting noise, so the second sample buys anti-aliasing
   and nothing else. Roughly doubles the rate, at the cost of harder edges on tags.
2. Drop the resolution further. Frame time is linear in pixel count.
3. Rasterise primary visibility instead of tracing it. This is the only remaining large win and it
   has not been done, because a rasteriser produces a pinhole image: the lens distortion would have
   to be applied afterwards by resampling, and the fuel would have to be tessellated or drawn as
   impostors. Those two properties, exact distortion and exact ball silhouettes, are most of why
   these frames are worth more than a screenshot of AdvantageScope.

## Known limits

- **Articulated robot parts are not drawn.** The ego robot's chassis and bumpers are rendered from
  `Robot_2026FRC/model.glb`, but the intake rack and hood ship as separate component files that
  need live mechanism positions to place. Bumpers are what actually occlude the cameras, so this
  matters less than it sounds.
- **No motion blur.** A robot crossing the field at 4 m/s would smear a real rolling-shutter frame;
  these frames are instantaneous.
- **The venue beyond the field is analytic**, a dim gradient rather than modelled seating. Nothing
  in the export extends past the driver stations.
- **Hub diffusers are assumed lit.** The model carries a `Hub Light Mount` behind them, so they are
  given a modest emissive value. If the real field leaves them dark, drop `DIFFUSER_EMISSIVE`.
