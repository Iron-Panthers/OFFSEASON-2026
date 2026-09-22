"""Reading an MJPEG stream, and serving one back out.

The input side has to survive a source that is slower than the consumer, faster than the
consumer, restarted underneath it, or not up yet. All four happen: the simulated renderer manages
about 1.5 FPS, a real camera does 30, and in simulation the coprocessor is usually launched before
the renderer has finished loading the field.
"""

from __future__ import annotations

import hashlib
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


@dataclass(frozen=True)
class Frame:
    """One JPEG, with the wall-clock instant it finished arriving."""

    jpeg: bytes
    received_at: float


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
            while self._running and self._sequence <= after_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._lock.wait(remaining)
            if self._frame is None:
                return None
            return (self._frame, self._sequence)

    # -- internals ---------------------------------------------------------

    def _publish(self, jpeg: bytes) -> None:
        # The renderer resends its current frame on a timer so browsers do not drop the
        # connection. Re-running a detector over bytes we have already seen would waste the
        # little CPU headroom there is and report an FPS the pipeline is not really achieving.
        digest = hashlib.blake2b(jpeg, digest_size=16).digest()
        if digest == self._last_digest:
            return
        self._last_digest = digest
        with self._lock:
            self._frame = Frame(jpeg=jpeg, received_at=time.time())
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
                # No image has begun. Keep only a byte in case SOI straddles two chunks.
                if len(buffer) > 1:
                    del buffer[: len(buffer) - 1]
                return
            end = buffer.find(_EOI, start + 2)
            if end < 0:
                if start > 0:
                    del buffer[:start]  # discard part headers preceding the image
                return
            jpeg = bytes(buffer[start : end + 2])
            del buffer[: end + 2]
            self._publish(jpeg)
