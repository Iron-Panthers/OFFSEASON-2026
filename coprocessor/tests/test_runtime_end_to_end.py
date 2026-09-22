"""The runtime loop, driven for real.

A local MJPEG server feeds synthetic frames to a stub module through the actual
:func:`coproc.runtime.runner.run`, and the annotated output is read back over HTTP. No model and
no NetworkTables server are involved, so this runs anywhere the tests run -- but every other piece
is the real one, which is the part worth checking: the units are covered individually and it is
the wiring between them that silently does nothing.
"""

import argparse
import io
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from coproc.modules.object_detection import geometry
from coproc.runtime.module import CoprocessorModule

cv2 = pytest.importorskip("cv2", reason="the runtime loop decodes JPEG with OpenCV")
np = pytest.importorskip("numpy")

RADIUS = 0.075
WIDTH, HEIGHT = 320, 200


def synthetic_frame(seed: int) -> bytes:
    """A frame with a pale disc on a dark field, different every call."""
    image = np.full((HEIGHT, WIDTH, 3), 30, dtype=np.uint8)
    centre = (80 + (seed * 7) % 120, 100)
    cv2.circle(image, centre, 18, (40, 200, 240), -1)
    ok, buffer = cv2.imencode(".jpg", image)
    assert ok
    return buffer.tobytes()


class FakeCamera:
    """The smallest thing that looks like the renderer's MJPEG endpoint."""

    def __init__(self):
        self.server = None
        self.port = 0
        self._stop = threading.Event()

    def start(self) -> "FakeCamera":
        owner = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *_args):  # noqa: N802 - stdlib signature
                pass

            def do_GET(self):  # noqa: N802 - stdlib signature
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frameboundary"
                )
                self.end_headers()
                seed = 0
                try:
                    while not owner._stop.is_set():
                        payload = synthetic_frame(seed)
                        seed += 1
                        self.wfile.write(b"--frameboundary\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(payload)}\r\n\r\n".encode())
                        self.wfile.write(payload)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        time.sleep(0.02)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        return self

    @property
    def url(self) -> str:
        return f"http://127.0.0.1:{self.port}/stream.mjpg"

    def stop(self) -> None:
        self._stop.set()
        if self.server is not None:
            self.server.shutdown()


class StubModule(CoprocessorModule):
    """Finds the disc by thresholding. No model, but a real detection from a real frame."""

    name = "stub"

    def __init__(self):
        self.frames = 0
        self.annotated = 0
        self.sizes: list[tuple[int, int]] = []

    def add_arguments(self, parser):
        pass

    def setup(self, context):
        pass

    def process(self, image, context):
        self.frames += 1
        self.sizes.append(image.shape[1::-1])

        grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(grey, 100, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w < 4 or h < 4:
                continue
            found.append(
                geometry.locate_sphere(
                    (float(x), float(y), float(x + w), float(y + h)),
                    context.intrinsics,
                    RADIUS,
                    "disc",
                    1.0,
                )
            )
        return found

    def annotate(self, image, detections):
        self.annotated += 1
        canvas = image.copy()
        for detection in detections:
            x0, y0, x1, y1 = (int(v) for v in detection.bbox)
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 255, 0), 2)
        return canvas


def make_args(source: str, output_port: int) -> argparse.Namespace:
    return argparse.Namespace(
        module="stub",
        source=source,
        camera=0,
        nt=None,  # no NetworkTables server in a unit test
        nt_port=5810,
        width=WIDTH,
        height=HEIGHT,
        fov=70.0,
        output_port=output_port,
        jpeg_quality=70,
    )


@pytest.fixture
def camera():
    fake = FakeCamera().start()
    yield fake
    fake.stop()


def run_until(module, args, predicate, timeout=15.0):
    """Run the loop on a background thread until ``predicate`` holds, then stop it."""
    from coproc.runtime import runner

    stop = threading.Event()
    thread = threading.Thread(target=runner.run, args=(module, args, stop), daemon=True)
    thread.start()

    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return False
    finally:
        stop.set()
        thread.join(timeout=10.0)


def test_frames_reach_the_module(camera):
    module = StubModule()
    assert run_until(module, make_args(camera.url, 0), lambda: module.frames >= 3), (
        f"only {module.frames} frames reached the module"
    )


def test_detections_come_out_of_real_frames(camera):
    """End to end: a drawn disc is found, and its range is physically sensible."""
    module = StubModule()
    seen: list = []

    original = module.process

    def capture(image, context):
        found = original(image, context)
        if found:
            seen.append(found[0])
        return found

    module.process = capture  # type: ignore[method-assign]

    assert run_until(module, make_args(camera.url, 0), lambda: len(seen) >= 2)

    detection = seen[0]
    assert detection.label == "disc"
    # A 36 px disc in a 320 px frame at 70 degrees diagonal is a little over a metre away.
    assert 0.5 < detection.distance_meters < 3.0
    assert abs(detection.yaw_degrees) < 45.0


def test_annotated_stream_serves_what_the_module_drew(camera):
    """The output stream is only fed while somebody is watching, so watch it."""
    module = StubModule()
    args = make_args(camera.url, 0)

    # Port 0 means "no stream", so pick a real free one.
    import socket

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        args.output_port = probe.getsockname()[1]

    from coproc.runtime import runner

    stop = threading.Event()
    thread = threading.Thread(target=runner.run, args=(module, args, stop), daemon=True)
    thread.start()

    try:
        url = f"http://127.0.0.1:{args.output_port}/stream.mjpg"
        deadline = time.monotonic() + 20.0
        received = b""
        while time.monotonic() < deadline and b"\xff\xd9" not in received:
            try:
                with urllib.request.urlopen(url, timeout=5) as response:
                    received = response.read(200_000)
            except Exception:  # noqa: BLE001 - the server may not be up on the first try
                time.sleep(0.2)

        assert b"\xff\xd8" in received and b"\xff\xd9" in received, "no JPEG on the output stream"
        assert module.annotated > 0, "annotate() must be called while a client is connected"

        start = received.index(b"\xff\xd8")
        end = received.index(b"\xff\xd9", start) + 2
        decoded = cv2.imdecode(
            np.frombuffer(received[start:end], dtype=np.uint8), cv2.IMREAD_COLOR
        )
        assert decoded is not None, "the served frame is not a decodable JPEG"
        assert decoded.shape[:2] == (HEIGHT, WIDTH)
    finally:
        stop.set()
        thread.join(timeout=10.0)


def test_nobody_watching_means_no_annotation(camera):
    """The annotate/encode cost must not be paid when no client is connected."""
    module = StubModule()
    assert run_until(module, make_args(camera.url, 0), lambda: module.frames >= 3)
    assert module.annotated == 0


def test_intrinsics_are_rescaled_when_the_stream_is_not_the_declared_size(camera):
    """Declared 640x400, actually 320x200: the focal length must follow the real frame.

    Using the declared focal length against a different frame size biases every range, and
    nothing about the output would look wrong.
    """
    module = StubModule()
    args = make_args(camera.url, 0)
    args.width, args.height = 640, 400  # deliberately not what the fake camera serves

    ranges: list[float] = []
    original = module.process

    def capture(image, context):
        found = original(image, context)
        if found:
            ranges.append(found[0].distance_meters)
        return found

    module.process = capture  # type: ignore[method-assign]
    assert run_until(module, args, lambda: len(ranges) >= 2)

    # Had the intrinsics not been rescaled, the focal length would be twice too long and every
    # range twice too far.
    assert 0.5 < ranges[0] < 3.0, f"range {ranges[0]:.2f} m suggests intrinsics were not rescaled"
    assert module.sizes[0] == (WIDTH, HEIGHT)


def test_a_dead_source_does_not_crash_the_loop():
    """Nothing is listening. The module should wait, not die."""
    module = StubModule()
    import socket

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        dead_port = probe.getsockname()[1]

    args = make_args(f"http://127.0.0.1:{dead_port}/stream.mjpg", 0)

    from coproc.runtime import runner

    stop = threading.Event()
    result: list = []
    thread = threading.Thread(
        target=lambda: result.append(runner.run(module, args, stop)), daemon=True
    )
    thread.start()
    time.sleep(2.0)

    assert thread.is_alive(), "the loop gave up on a source that is merely not up yet"
    assert module.frames == 0

    stop.set()
    thread.join(timeout=10.0)
    assert result == [0], "a clean stop should exit zero"
