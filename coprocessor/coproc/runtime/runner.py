"""The loop that runs a module: frames in, detections out.

This is the whole framework from a module author's point of view. Nothing here knows what is
being detected.
"""

from __future__ import annotations

import signal
import sys
import threading
import time
from typing import Optional

from .intrinsics import Intrinsics
from .mjpeg import MjpegReader
from .module import CoprocessorModule, ModuleContext
from .publisher import NtPublisher, Rate
from .server import AnnotatedStream

# How long to wait for a frame before reporting the source is quiet. Generous: the simulated
# renderer needs several seconds to load the field before it serves anything at all.
_FRAME_TIMEOUT_SECONDS = 5.0


def run(
    module: CoprocessorModule, args, stop: Optional[threading.Event] = None
) -> int:
    """Run ``module`` until ``stop`` is set or the process is interrupted.

    ``stop`` exists so the loop can be driven from a test or embedded in something larger; the
    signal handlers set the same event.

    :return: a process exit code
    """
    import cv2  # deferred so --list and the geometry tests need no OpenCV

    intrinsics = Intrinsics.from_diagonal_fov(args.width, args.height, args.fov)
    context = ModuleContext(intrinsics=intrinsics, args=args)

    print(f"[{module.name}] loading...", flush=True)
    try:
        module.setup(context)
    except Exception as failed:  # noqa: BLE001 - report and exit rather than a bare traceback
        print(f"[{module.name}] setup failed: {failed}", file=sys.stderr, flush=True)
        return 1

    reader = MjpegReader(args.source).start()
    stream: Optional[AnnotatedStream] = None
    if args.output_port > 0:
        stream = AnnotatedStream(f"{module.name} (annotated)", args.output_port).start()
        print(f"[{module.name}] annotated stream -> http://localhost:{args.output_port}/", flush=True)

    publisher: Optional[NtPublisher] = None
    if args.nt:
        publisher = NtPublisher(module.name, f"cam{args.camera}", args.nt, args.nt_port)
        try:
            publisher.start()
            print(
                f"[{module.name}] publishing to {args.nt}:{args.nt_port} "
                f"{publisher.table_path}",
                flush=True,
            )
        except Exception as failed:  # noqa: BLE001
            print(f"[{module.name}] NetworkTables disabled: {failed}", file=sys.stderr, flush=True)
            publisher = None

    print(f"[{module.name}] reading {args.source}", flush=True)

    stop = stop if stop is not None else threading.Event()

    def handle_signal(_signum, _frame):
        stop.set()

    try:
        signal.signal(signal.SIGINT, handle_signal)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, handle_signal)
    except ValueError:
        # Signal handlers can only be installed from the main thread. Running off it is a
        # legitimate thing to do; the caller owns the stop event in that case.
        pass

    rate = Rate()
    sequence = -1
    waiting_reported = False
    declared = (args.width, args.height)

    try:
        while not stop.is_set():
            newest = reader.next_frame(sequence, timeout=_FRAME_TIMEOUT_SECONDS)
            if newest is None:
                if not waiting_reported:
                    detail = f" ({reader.last_error})" if reader.last_error else ""
                    print(f"[{module.name}] waiting for {args.source}{detail}", flush=True)
                    waiting_reported = True
                if publisher is not None:
                    publisher.publish([], 0.0, 0.0, 0.0, source_connected=False)
                continue
            waiting_reported = False

            frame, sequence = newest
            import numpy as np

            image = cv2.imdecode(np.frombuffer(frame.jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                continue

            height, width = image.shape[:2]
            if (width, height) != declared:
                # The stream is not the resolution we were told to expect. Rescaling the
                # intrinsics matters: using the declared focal length against a different frame
                # size biases every single range estimate, silently.
                print(
                    f"[{module.name}] stream is {width}x{height}, not {declared[0]}x{declared[1]}; "
                    "rescaling intrinsics",
                    flush=True,
                )
                declared = (width, height)
                context.intrinsics = intrinsics.scaled_to(width, height)

            detections = module.process(image, context)
            fps = rate.tick()
            latency_ms = (time.time() - frame.received_at) * 1000.0

            if publisher is not None:
                publisher.publish(
                    detections,
                    capture_timestamp=frame.received_at,
                    latency_ms=latency_ms,
                    fps=fps,
                    source_connected=reader.connected,
                )

            if stream is not None and stream.wanted:
                annotated = module.annotate(image, detections)
                encoded, buffer = cv2.imencode(
                    ".jpg",
                    annotated if annotated is not None else image,
                    [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality],
                )
                if encoded:
                    stream.publish(buffer.tobytes())
    finally:
        print(f"[{module.name}] stopping", flush=True)
        reader.stop()
        if stream is not None:
            stream.stop()
        if publisher is not None:
            publisher.stop()
    return 0
