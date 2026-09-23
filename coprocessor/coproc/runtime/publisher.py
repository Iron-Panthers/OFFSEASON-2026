"""Publishing detections to NetworkTables.

Parallel arrays rather than a struct: they are readable in Glass and AdvantageScope with no
schema, and trivial to consume from Java. The cost is that NT gives no cross-topic atomicity, so
everything for a frame is published and then flushed once, which puts it in a single NT4 packet in
practice. A consumer should still check that the arrays are the same length before zipping them.
"""

from __future__ import annotations

import time
from typing import Optional, Sequence

from .module import Detection


class NtPublisher:
    """Publishes one module's detections under ``/coprocessor/<module>/<camera>``.

    Import of ``ntcore`` is deferred to :meth:`start` so the geometry tests, and ``--list``, run
    without robotpy installed.
    """

    def __init__(self, module: str, camera: str, server: str, port: int = 5810, identity: Optional[str] = None):
        self.table_path = f"/coprocessor/{module}/{camera}"
        self.server = server
        self.port = port
        self.identity = identity or f"{module}-{camera}"
        self._nt = None
        self._pubs: dict = {}

    def start(self) -> "NtPublisher":
        import ntcore  # deferred: see class docstring

        instance = ntcore.NetworkTableInstance.getDefault()
        instance.startClient4(self.identity)
        instance.setServer(self.server, self.port)
        table = instance.getTable(self.table_path)

        self._nt = instance
        self._pubs = {
            "count": table.getIntegerTopic("count").publish(),
            "yaw": table.getDoubleArrayTopic("yaw").publish(),
            "pitch": table.getDoubleArrayTopic("pitch").publish(),
            "distance": table.getDoubleArrayTopic("distance").publish(),
            "tx": table.getDoubleArrayTopic("tx").publish(),
            "ty": table.getDoubleArrayTopic("ty").publish(),
            "tz": table.getDoubleArrayTopic("tz").publish(),
            "confidence": table.getDoubleArrayTopic("confidence").publish(),
            "edge": table.getBooleanArrayTopic("edge").publish(),
            "sizeDistance": table.getDoubleArrayTopic("sizeDistance").publish(),
            "groundDistance": table.getDoubleArrayTopic("groundDistance").publish(),
            "suspect": table.getBooleanArrayTopic("suspect").publish(),
            "captureTimestamp": table.getDoubleTopic("captureTimestamp").publish(),
            "latencyMs": table.getDoubleTopic("latencyMs").publish(),
            "fps": table.getDoubleTopic("fps").publish(),
            "connected": table.getBooleanTopic("connected").publish(),
        }
        return self

    @property
    def connected(self) -> bool:
        return self._nt is not None and self._nt.isConnected()

    def publish(
        self,
        detections: Sequence[Detection],
        capture_timestamp: float,
        latency_ms: float,
        fps: float,
        source_connected: bool,
    ) -> None:
        if not self._pubs:
            return
        p = self._pubs
        p["yaw"].set([d.yaw_degrees for d in detections])
        p["pitch"].set([d.pitch_degrees for d in detections])
        p["distance"].set([d.distance_meters for d in detections])
        p["tx"].set([d.x for d in detections])
        p["ty"].set([d.y for d in detections])
        p["tz"].set([d.z for d in detections])
        p["confidence"].set([d.confidence for d in detections])
        p["edge"].set([d.edge for d in detections])
        p["sizeDistance"].set([d.size_distance_meters for d in detections])
        # NaN rather than a sentinel distance: a consumer that forgets to check gets arithmetic
        # that stays obviously broken instead of a plausible range it will happily drive to.
        p["groundDistance"].set(
            [
                float("nan") if d.ground_distance_meters is None else d.ground_distance_meters
                for d in detections
            ]
        )
        p["suspect"].set([d.suspect for d in detections])
        p["captureTimestamp"].set(capture_timestamp)
        p["latencyMs"].set(latency_ms)
        p["fps"].set(fps)
        p["connected"].set(source_connected)
        # Count last: a consumer that reads count first and then the arrays sees a length that
        # is never longer than what is actually there.
        p["count"].set(len(detections))
        if self._nt is not None:
            self._nt.flush()

    def stop(self) -> None:
        for pub in self._pubs.values():
            try:
                pub.close()
            except Exception:  # noqa: BLE001 - shutdown must not raise
                pass
        self._pubs.clear()
        if self._nt is not None:
            self._nt.stopClient()
            self._nt = None


class Rate:
    """Exponentially smoothed frames per second."""

    def __init__(self, smoothing: float = 0.2):
        self.smoothing = smoothing
        self._fps = 0.0
        self._last: Optional[float] = None

    def tick(self) -> float:
        now = time.monotonic()
        if self._last is not None:
            elapsed = now - self._last
            if elapsed > 0:
                instant = 1.0 / elapsed
                self._fps = instant if self._fps == 0.0 else (
                    self._fps + self.smoothing * (instant - self._fps)
                )
        self._last = now
        return self._fps

    @property
    def fps(self) -> float:
        return self._fps
