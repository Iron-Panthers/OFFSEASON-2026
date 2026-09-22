# Rendering Engine

Renders what the robot's cameras would see during a match and serves each one as an MJPEG stream on
localhost, so vision and object-detection work can be developed against simulated footage.

Simulation only. The engine refuses to start outside `Mode.SIM`, so none of it is ever loaded on a
roboRIO.

## Running it

```bash
./gradlew simulateJava -Prender
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
| `-Prender.width` / `-Prender.height` | 640 x 400 | Half the simulated camera resolution. Cost scales with pixel count. |
| `-Prender.spp` | 2 | Samples per pixel. The main quality/time dial. |
| `-Prender.bounces` | 1 | Indirect diffuse bounces. 0 flattens the lighting noticeably. |
| `-Prender.transparency` | 3 | How many polycarbonate panels a ray may see through. |
| `-Prender.denoise` | true | `false` shows the raw Monte Carlo noise. |
| `-Prender.gain` | 1.0 | Sensor noise multiplier. Raise it to test a pipeline against a bad exposure. |
| `-Prender.quality` | 0.82 | JPEG quality. |
| `-Prender.port` | 1191 | First port to try. |

Example, matching the real camera resolution at higher quality:

```bash
./gradlew simulateJava -Prender -Prender.width=1280 -Prender.height=800 -Prender.spp=4
```

## Assets

Vendored under `advantage_scope_files/`, pulled from a local AdvantageScope install:

| Path | Contents |
| --- | --- |
| `Field3d_2026FRCFieldV2/model.glb` | The 2026 field. 18 MB, 4.28 M triangles, 28 materials, no textures. |
| `Field3d_2026FRCFieldV2/model_0.glb` | The Fuel ball. Kept for reference; the renderer draws fuel analytically. |
| `Field3d_2026FRCFieldV2/config.json` | Tag poses, field size, game piece definitions. |
| `AprilTag_36h11/0NN.png` | The real 36h11 bitmaps, 10x10 px, extracted from the AdvantageScope bundle and SHA256-verified against its own index. |

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
| `RenderPreviewTest` | Three fixed viewpoints render, are sensibly exposed and have real contrast. Also writes them to `build/render-preview/` so a person can look. |

Tag detection currently holds to **5 m** at 1280x800 through a 70 degree lens, with decision
margins above 100. That is an optical limit rather than a rendering one: a 6.5 inch tag subtends
about 38 px at that range, and neither more samples nor more resolution changes the margin.

## Performance

Measured on this machine, headless simulation running alongside:

| Setting | Rate |
| --- | --- |
| 640 x 400, 2 spp, one camera being watched | **~1.5 FPS** |
| 480 x 360, 4 spp, standalone (no simulation) | ~0.45 s trace + 0.02 s post per frame |

The cost is dominated by BVH traversal over roughly four million triangles, not by shading or by
post-processing, so the levers that actually matter are resolution and `-Prender.spp`. Flattening
the shadow-ray inner loop and parallelising the denoiser and the film stage together moved
post-processing from about 200 ms to about 20 ms a frame and barely changed the total.

This is fast enough to watch the robot drive and more than fast enough to capture a varied test set
(~90 frames a minute), but it is not real time and is not meant to be. Only cameras being watched
are rendered, so closing a stream gives the remaining ones their time back.

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
