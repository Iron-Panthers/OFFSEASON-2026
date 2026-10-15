"""Reading an MJPEG stream, and serving one back out.

The input side has to survive a source that is slower than the consumer, faster than the
consumer, restarted underneath it, or not up yet. All four happen: the simulated renderer manages
about 1.5 FPS, a real camera does 30, and in simulation the coprocessor is usually launched before
the renderer has finished loading the field.
"""

from __future__ import annotations

import hashlib
import re
import socket
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Optional

_SOI = b"\xff\xd8"  # JPEG start of image
_EOI = b"\xff\xd9"  # JPEG end of image

# Cap on how much unparseable data we will hold before giving up on the connection. A stream that
# has desynchronised never recovers on its own, and buffering it forever is how this turns into a
# memory leak rather than a reconnect.
_MAX_BUFFER_BYTES = 16 * 1024 * 1024

# How much of a stream with no image in it is kept for the next read. Enough to hold one part's
# headers, which may arrive in a different read from the image they describe; nothing like enough to
# grow without bound on a stream that carries no JPEG at all.
_HEADER_TAIL_BYTES = 512

# The capture time of the scene in the image, in the source's own clock. The simulated renderer
# stamps the robot time its snapshot was taken at; mjpg-streamer uses the same header. Only a
# complete line counts, so a value cut off at the end of a read is not parsed half-written.
_TIMESTAMP_HEADER = re.compile(rb"X-Timestamp:\s*([-+0-9.eE]+)\r?\n", re.IGNORECASE)


@dataclass(frozen=True)
class Frame:
    """One JPEG, with the wall-clock instant it finished arriving."""

    jpeg: bytes
    received_at: float

    source_timestamp: Optional[float] = None
    """When the source says the scene was captured, in the source's clock, if it said at all.

    For the simulated renderer this is robot time, which is what lets the robot place a detection
    using where it was when the frame was taken rather than when the frame arrived.
    """


class MjpegReader:
    """Reads an MJPEG stream on a background thread, keeping only the newest frame.

    Newest-only is the important part. Queueing frames from a source faster than the detector
    means detecting on images that are already stale, which for a moving robot is worse than
    dropping them. Conversely the simulated renderer re-sends its current frame every two seconds
    to keep browsers from timing out, so identical frames are suppressed rather than counted.
    """

    def __init__(self, url: str, reconnect_delay: float = 1.0, timeout: float = 10.0):
        self.url = url
        self.reconnect_delay = reconnect_delay
        self.timeout = timeout

        self._lock = threading.Condition()
        self._frame: Optional[Frame] = None
        self._sequence = 0
        self._last_digest: Optional[bytes] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._connected = False
        self._last_error: Optional[str] = None
        # A timestamp header that has arrived ahead of the image it belongs to.
        self._pending_timestamp: Optional[float] = None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> "MjpegReader":
        self._running = True
        self._thread = threading.Thread(target=self._run, name="MjpegReader", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._running = False
        with self._lock:
            self._lock.notify_all()

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_error(self) -> Optional[str]:
        return self._last_error

    # -- consumption -------------------------------------------------------

    def next_frame(self, after_sequence: int, timeout: float = 5.0) -> Optional[tuple[Frame, int]]:
        """Block until a frame newer than ``after_sequence`` exists.

        Returns ``(frame, sequence)``, or None if the timeout passed with nothing new. Pass the
        returned sequence back in on the next call.
        """
        deadline = time.monotonic() + timeout
        with self._lock:
            # Also wait while nothing has arrived at all. The runner starts from -1, which the
            # sequence (0 before the first frame) already exceeds, so without this the first call
            # returned at once and the runner spun publishing "waiting" as fast as it could:
            # tens of thousands of NetworkTables updates a second, and a CPU core, until the
            # source came up.
            while self._running and (self._frame is None or self._sequence <= after_sequence):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._lock.wait(remaining)
            if self._frame is None:
                return None
            return (self._frame, self._sequence)

    # -- internals ---------------------------------------------------------

    def _publish(self, jpeg: bytes, source_timestamp: Optional[float] = None) -> None:
        # The renderer resends its current frame on a timer so browsers do not drop the
        # connection. Re-running a detector over bytes we have already seen would waste the
        # little CPU headroom there is and report an FPS the pipeline is not really achieving.
        digest = hashlib.blake2b(jpeg, digest_size=16).digest()
        if digest == self._last_digest:
            return
        self._last_digest = digest
        with self._lock:
            self._frame = Frame(
                jpeg=jpeg, received_at=time.time(), source_timestamp=source_timestamp
            )
            self._sequence += 1
            self._lock.notify_all()

    def _run(self) -> None:
        while self._running:
            try:
                self._read_stream()
            except (urllib.error.URLError, OSError, socket.timeout) as failed:
                self._last_error = str(failed)
            finally:
                self._connected = False
            if self._running:
                # The source is commonly just not up yet; in simulation the renderer spends
                # several seconds loading the field before it binds a port.
                time.sleep(self.reconnect_delay)

    def _read_stream(self) -> None:
        request = urllib.request.Request(self.url, headers={"Accept": "multipart/x-mixed-replace"})
        with urllib.request.urlopen(request, timeout=self.timeout) as response:
            self._connected = True
            self._last_error = None
            buffer = bytearray()
            while self._running:
                chunk = response.read(65536)
                if not chunk:
                    return  # source closed; the caller reconnects
                buffer.extend(chunk)
                self._drain(buffer)
                if len(buffer) > _MAX_BUFFER_BYTES:
                    raise OSError("MJPEG stream desynchronised; reconnecting")

    def _drain(self, buffer: bytearray) -> None:
        """Pull every complete JPEG out of the buffer, leaving any partial tail behind.

        Scanning for JPEG markers rather than parsing multipart boundaries is deliberate: it
        costs nothing in robustness here and works against the several cameras that get the
        boundary syntax subtly wrong.
        """
        while True:
            start = buffer.find(_SOI)
            if start < 0:
                # No image has begun. Note any header that has, then keep only a short tail: enough
                # for SOI or a header line straddling two chunks, never an unbounded backlog.
                self._note_headers(buffer)
                if len(buffer) > _HEADER_TAIL_BYTES:
                    del buffer[: len(buffer) - _HEADER_TAIL_BYTES]
                return
            if start > 0:
                # Part headers precede the image. Read them before they are discarded.
                self._note_headers(buffer[:start])
                del buffer[:start]
                start = 0
            end = buffer.find(_EOI, start + 2)
            if end < 0:
                return
            jpeg = bytes(buffer[start : end + 2])
            del buffer[: end + 2]
            timestamp, self._pending_timestamp = self._pending_timestamp, None
            self._publish(jpeg, timestamp)

    def _note_headers(self, region: bytearray | bytes) -> None:
        """Remember the capture timestamp in a run of part headers, for the image that follows."""
        match = None
        for match in _TIMESTAMP_HEADER.finditer(region):
            pass  # the last one wins; there is only ever one per part
        if match is None:
            return
        try:
            self._pending_timestamp = float(match.group(1))
        except ValueError:
            self._pending_timestamp = None
