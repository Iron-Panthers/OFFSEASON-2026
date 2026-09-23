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
[Rendering] Camera 2 -> http://localhost:1193/  (480x300, 2 spp, FAST)
[Coprocessor] objdetect -> http://localhost:1193/stream.mjpg, annotated on http://localhost:1293/
[objdetect] yolov8n.pt, keeping 32 (sports ball), radius 0.075 m
```

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
| `-Pcoproc.objdetect.camera` | `2` | Camera index. 2 is served on 1193. |
| `-Pcoproc.objdetect.source` | renderer | Read somewhere else entirely — a real camera, a recording. |
| `-Pcoproc.objdetect.model` | `yolov8n.pt` | Swap in trained weights without touching code. |
| `-Pcoproc.objdetect.conf` | `0.25` | Confidence floor. |
| `-Pcoproc.objdetect.classes` | `32` | COCO class ids to keep, or `all`. |
| `-Pcoproc.objdetect.radius` | `0.075` | Ball radius, metres. A pure scale factor on every range. |
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
| `edge` | boolean[] | Box touches the frame border — **range is not trustworthy** |
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

A sphere of radius *r* whose silhouette subtends half-angle *θ* is at `d = r / sin(θ)`. Exact for a
sphere viewed from any angle, which is why the renderer drawing fuel as analytic spheres rather
than as the 1244-triangle mesh in the field export matters: the silhouette it produces is the one
this inverts.

*θ* is the mean of the box's horizontal and vertical angular half-extents, computed from real ray
directions rather than from `pixels / focal length`. The cheap version is a small-angle
approximation that is fine in the centre of the image and wrong at the corners, which is where a
wide-angle lens puts most of its field. Measured against an exact projection of the silhouette, the
remaining error stays **under 1% out to the edge of a 70° lens** — far below the box jitter of any
detector. `test_off_axis_range_error_stays_bounded` pins that number.

The focal length is derived from the frame size and the diagonal FOV, matching
`VisionConstants.SIM_CAMERA_FOV_DIAGONAL_DEGREES`. If the stream arrives at a different resolution
than declared, the intrinsics are rescaled and a line is printed — using the declared focal length
against a different frame size biases every range, silently.

## Things worth knowing

**Stock COCO weights are a stand-in, but they do work.** Class 32 is `sports ball`, trained on
soccer and tennis balls rather than on Fuel under arena lighting. Measured against rendered frames
it does fire on Fuel, at confidences around **0.3 to 0.6** -- useful, and well short of what
trained weights would give. Expect misses on distant or partly occluded balls. Swapping in real
weights is a `-Pcoproc.objdetect.model` change and nothing else. The simulation
knows where every ball actually is, so auto-labelled training data is cheap to generate later —
`geometry.sphere_bbox` is the forward model that turns a known position into a labelled box.

**A clipped ball reads as further away than it is.** Its box is smaller than its true silhouette,
and range comes from size. Detections touching the frame border carry the `edge` flag and are drawn
in amber. **Occlusion away from the border is undetectable** — a ball half hidden behind another
robot mid-frame reads as too far and carries no warning. `test_occlusion_away_from_the_border_is_not_flagged`
records that limit so the flag is not mistaken for more than it is.

**The renderer is the bottleneck, not the detector.** Measured end to end at 480x300 with one
camera watched, the whole pipeline runs at about **16 FPS**, and that ceiling belongs to the
renderer rather than to YOLOv8n; at 640x400 the renderer alone manages ~1.5 FPS. The detector
spends most of its time waiting for frames. The reader therefore blocks rather than polls and keeps only the newest frame
rather than queueing — otherwise the detector would be working on images that are already seconds
old. On the Pi against a 30 FPS camera the same code hits the opposite regime and drops frames
correctly. The renderer also re-sends its current frame every two seconds to keep browsers
connected, so byte-identical frames are suppressed rather than re-detected.

**The launcher, not Gradle, spawns the process.** The renderer walks upward from 1191 when a port
is taken, so the port a camera actually landed on is only knowable inside the simulation JVM.
Anything that hardcodes 1193 is one busy port away from silently reading nothing.

## Tests

| Test | What it protects |
| --- | --- |
| `tests/test_geometry.py` | Round trip against an exact silhouette projection; WPILib sign conventions; the off-axis error bound; rescaled intrinsics recovering the same distance. |
| `tests/test_mjpeg.py` | Frames split across reads, leading garbage, duplicate suppression, newest-frame-wins, buffer growth on a desynchronised stream. |
| `tests/test_runtime_end_to_end.py` | The whole loop against a live local MJPEG server and a stub module: frames reach `process`, detections come back, the annotated stream serves decodable JPEG, annotation is skipped when nobody is watching, intrinsics rescale, a dead source does not kill the loop. |
| `CoprocessorModuleTest` | Registry lookup, typos naming the valid options, output ports staying clear of the render range. |

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
