# Coprocessors

Runs coprocessor code beside the simulation, as its own process, reading a rendered camera instead
of a real one.

The program being run is the real coprocessor program. The Python in `coprocessor/` is what ships
to the Rubik Pi; simulation only changes where its frames come from and which NetworkTables server
it talks to. Nothing on the Python side knows the difference, so what you debug here is what runs
at an event.

Simulation only. `CoprocessorManager` refuses to start outside `Mode.SIM`.

## Running it

```bash
./gradlew simulateJava -Prender -Pcoproc=objdetect --no-daemon
```

`-Prender` is required: without it there is no camera to read. You will see

```
[Rendering] Camera 3 (object detection) -> http://localhost:1194/  (480x300, 2 spp, FAST)
[Coprocessor] objdetect -> http://localhost:1194/stream.mjpg, annotated on http://localhost:1294/
[objdetect] rebuilt-fuel-v1.pt, keeping all classes, radius 0.075 m, ranging by ground plane from 0.600 m, +20.0 deg
```

### Which camera it watches

**Its own, not one of the vision cameras.** The detector reads
`ObjectDetectionConstants.ROBOT_TO_CAMERA` — mounted low, tilted 20 degrees down and facing the
intake — which the renderer serves immediately after the vision cameras in `CAMERA_TRANSFORM`.

This is not a detail. The vision cameras are pitched *up* to put AprilTags on walls in frame
(negative pitch is nose up in WPILib's `Rotation3d`). Camera 2 sits 0.485 m up tilted 11 degrees
up, which puts its horizon below the whole lower half of the sensor: it cannot see floor nearer
than about 2.5 m, and fuel lands as a five-pixel strip crushed against the bottom edge. Pointed
there, **both the trained model and stock COCO weights detect zero balls in every frame**. Aimed
at the floor instead, the same model finds around fifty. No change of weights, confidence floor or
input size recovers a camera that is not looking at the thing.

| URL | What it is |
| --- | --- |
| `http://localhost:1293/` | The annotated stream, boxes drawn on. Open this. |
| `http://localhost:1293/stream.mjpg` | MJPEG, same shape the renderer serves. |
| `http://localhost:1293/info.json` | Frames produced, viewers connected. |

Output ports are `1291 + cameraIndex`, clear of the renderer at 1191+ and PhotonVision at 1181+.

### Options

`-P` flags on `simulateJava`. Anything of the form `-Pcoproc.<x>` is passed through to the module,
so a new module's options need no Gradle changes.

| Flag | Default | Notes |
| --- | --- | --- |
| `-Pcoproc` | — | Comma-separated modules. Currently only `objdetect`. |
| `-Pcoproc.python` | auto | Interpreter. Finds `coprocessor/.venv` on its own. |
| `-Pcoproc.objdetect.camera` | object-detection camera | Camera index. Defaults to the detector's own camera, after the vision cameras. |
| `-Pcoproc.objdetect.source` | renderer | Read somewhere else entirely — a real camera, a recording. |
| `-Pcoproc.objdetect.model` | `rebuilt-fuel-v1.pt` | Swap in other weights without touching code. |
| `-Pcoproc.objdetect.conf` | `0.25` | Confidence floor. |
| `-Pcoproc.objdetect.classes` | `auto` | Class ids to keep, `all`, or `auto` for a single-class model. |
| `-Pcoproc.objdetect.radius` | `0.075` | Ball radius, metres. Sets the floor plane and scales size-based range. |
| `-Pcoproc.objdetect.imgsz` | `640` | Model input size. Smaller is faster and worse on distant balls. |
| `-Pcoproc.objdetect.disagreement` | `0.20` | Gap between the two ranges that flags a detection suspect. |
| `-Pcoproc.objdetect.device` | auto | Torch device, e.g. `cpu`. |

### Install

```bash
python -m venv coprocessor/.venv
coprocessor/.venv/Scripts/pip install -r coprocessor/requirements.txt
```

Without it the simulation still runs; the module prints why it did not start and the sim carries on.

## What it publishes

Table `/coprocessor/objdetect/cam<N>`, parallel arrays of equal length, one entry per ball, nearest
first:

| Key | Type | Meaning |
| --- | --- | --- |
| `count` | int | Detections this frame |
| `yaw` / `pitch` | double[] | Degrees. Yaw counter-clockwise positive, pitch up positive |
| `distance` | double[] | Metres to the ball centre |
| `tx` / `ty` / `tz` | double[] | Camera frame: X forward, Y left, Z up |
| `confidence` | double[] | 0–1 |
| `edge` | boolean[] | Box touches the frame border — its size understates the ball |
| `sizeDistance` | double[] | Range from apparent size alone |
| `groundDistance` | double[] | Range from the ground plane alone; NaN when the ray never meets it |
| `suspect` | boolean[] | The two ranges disagree — **the box is probably wrong** |
| `captureTimestamp` | double | Seconds, when the frame arrived |
| `latencyMs` | double | Capture to publish |
| `fps` | double | Smoothed |
| `connected` | boolean | Whether the source stream is up |

Positions are camera-relative, so the coprocessor holds no robot state. Robot code applies
`VisionConstants.CAMERA_TRANSFORM[N]` to get robot- or field-relative positions.

Everything for a frame is published and then flushed once, which puts it in a single NT4 packet in
practice. NT gives no cross-topic atomicity, so a consumer should check the arrays are the same
length before zipping them. `count` is written last, so it never exceeds what is actually there.

## How range works

Two independent estimates, every detection.

**From apparent size.** A sphere of radius *r* whose silhouette subtends half-angle *θ* is at
`d = r / sin(θ)`. Exact for a sphere viewed from any angle, which is why the renderer drawing fuel
as analytic spheres rather than as the 1244-triangle mesh in the field export matters: the
silhouette it produces is the one this inverts.

*θ* is the mean of the box's horizontal and vertical angular half-extents, computed from real ray
directions rather than from `pixels / focal length`. The cheap version is a small-angle
approximation that is fine in the centre of the image and wrong at the corners, which is where a
wide-angle lens puts most of its field. Measured against an exact projection of the silhouette, the
remaining error stays **under 1% out to the edge of the lens**.

That is the error when the box is right. When it is wrong this estimate fails hard, and the failure
is *range-independent*: it reads the box **extent**, so a box merged across two neighbouring balls
halves the reported range at 1 m and at 5 m alike, and a clipped one doubles it.

**From the ground plane.** Fuel resting on the floor has its centre at exactly its own radius, so
that plane is known without measuring anything. Intersecting the bearing through the box centre
with it gives a range that reads the box **centre** instead of its extent — and the centre is what
the failures above barely move. The error is bounded and grows only as the ball approaches the
horizon: about 2% at 1 m, 4% at 2 m, 8% at 4 m.

This is the reported `distance`, because a ball on the floor is the case this module exists for.
It needs the camera's height and downward tilt, which `CoprocessorManager` passes from
`ObjectDetectionConstants.ROBOT_TO_CAMERA`. Without `--camera-height` and `--camera-pitch` the
module falls back to apparent size alone and says so at start-up. Heading is deliberately not
passed: rotating a camera about the vertical cannot change where its rays meet the floor.

**The cross-check.** The two fail in uncorrelated ways, so them disagreeing is evidence the box is
wrong, and the gap is diagnostic. A box wrong along one axis — merged across a neighbour, clipped
down one side by a robot in front — moves the size-based range by exactly 3/2 or 2/3, because the
angular radius averages the two half-extents and only one changed. That is a 0.33 gap at *every*
range. The `suspect` threshold sits at 0.20: below the signal, above the honest error.

This is what catches **occlusion away from the frame border**, which the `edge` flag cannot see and
which no single estimate can. It also fires for a ball in flight, which is not on the floor and has
no honest ground-plane range — there the disagreement is telling the truth about a real assumption
being violated. `suspect` boxes are drawn red, `edge` boxes amber.

The focal length is derived from the frame size and the camera's own diagonal FOV, which for the
object-detection camera is computed from `ObjectDetectionConstants.HORIZONTAL_FOV_RAD` and
`VERTICAL_FOV_RAD` rather than written down a third time. If the stream arrives at a different
resolution than declared, the intrinsics are rescaled and a line is printed — using the declared
focal length against a different frame size biases every range, silently.

## Things worth knowing

**Stock COCO weights do not work at all.** Class 32 is `sports ball`, trained on soccer and
tennis balls. Measured against rendered frames of the centre pile it fires on Fuel **zero times**,
at every confidence floor from 0.10 down. An earlier version of this document claimed 0.3-0.6
confidence hits; that does not reproduce. The default is now `rebuilt-fuel-v1.pt`, a single-class
YOLO26n trained on Fuel for 300 epochs, which finds around **fifty balls per frame** on the same
images. `-Pcoproc.objdetect.model=yolov8n.pt` still selects the old weights.

**Localisation in a dense pile is the remaining weakness.** The trained model fires readily on the
staging array, but its boxes in the packed interior are loose and frequently merged across
neighbours — which is precisely why the cross-check exists rather than being decoration. No public
dataset contains a hundred touching balls viewed from 0.5 m. The fix is fine-tuning on renderer
output: the simulation knows where every ball actually is, and `geometry.sphere_bbox` is the
forward model that turns a known position into an exactly labelled box, so the training data is
free.

**A clipped ball reads as further away than it is.** Its box is smaller than its true silhouette.
Detections touching the frame border carry the `edge` flag and are drawn amber. Occlusion *away*
from the border carries no `edge` flag, but is now caught by the range cross-check above and
flagged `suspect`.

**Same code, different weights on the Pi.** The Python that runs here is the Python that ships, but
the model artifact is not: simulation runs `.pt` through CPU Torch, and the Rubik Pi's NPU needs
`.rknn`. Different numerics, and a conversion step in between. Simulation accuracy is therefore not
a prediction of field accuracy, only of pipeline correctness.

**The renderer is the bottleneck, not the detector — by a wide margin.** Benchmarked on CPU Torch
at 480x300, `rebuilt-fuel-v1.pt` runs at **41 FPS** at `imgsz=640` and 78 FPS at 320; `yolov8n.pt`
is within a few percent of it. The renderer serves the object-detection camera at roughly 8-12 FPS
depending on scene complexity, so the detector spends most of its time waiting for frames and
lowering `imgsz` buys nothing but worse recall on distant balls. Cameras nobody is watching are not
rendered at all, so the object-detection camera is free when unused. The reader therefore blocks rather than polls and keeps only the newest frame
rather than queueing — otherwise the detector would be working on images that are already seconds
old. On the Pi against a 30 FPS camera the same code hits the opposite regime and drops frames
correctly. The renderer also re-sends its current frame every two seconds to keep browsers
connected, so byte-identical frames are suppressed rather than re-detected.

**The launcher, not Gradle, spawns the process.** The renderer walks upward from 1191 when a port
is taken, so the port a camera actually landed on is only knowable inside the simulation JVM.
Anything that hardcodes a port number is one busy port away from silently reading nothing.

## Tests

| Test | What it protects |
| --- | --- |
| `tests/test_geometry.py` | Round trip against an exact silhouette projection; WPILib sign conventions; the off-axis error bound; rescaled intrinsics recovering the same distance. Ground-plane ranging recovering a ball on the floor; a merged box and a mid-frame occlusion both flagged `suspect`; a ball above the horizon getting no ground range; yaw not affecting it. |
| `tests/test_mjpeg.py` | Frames split across reads, leading garbage, duplicate suppression, newest-frame-wins, buffer growth on a desynchronised stream. |
| `tests/test_runtime_end_to_end.py` | The whole loop against a live local MJPEG server and a stub module: frames reach `process`, detections come back, the annotated stream serves decodable JPEG, annotation is skipped when nobody is watching, intrinsics rescale, a dead source does not kill the loop. |
| `CoprocessorModuleTest` | Registry lookup, typos naming the valid options, output ports staying clear of the render range. That the detector defaults to its own camera and never a vision camera, that that camera is pitched *down*, and that its rendered FOV matches the calibrated one. |

```bash
cd coprocessor && python -m pytest tests/ -q
./gradlew test --no-daemon
```

Neither covers process spawning or the model itself: those need a Python environment with
Torch in it and a live renderer, which belongs in a manual run. Everything between the stream and
NetworkTables is covered.

## Adding a module

Three steps, in `coprocessor/README.md`. The framework supplies frames, reconnects, NetworkTables,
the annotated stream and timing; a module supplies `process(frame) -> detections`.
