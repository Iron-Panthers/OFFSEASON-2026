"""Publishing detections to NetworkTables.

Parallel arrays rather than a struct: they are readable in Glass and AdvantageScope with no
schema, and trivial to consume from Java. The cost is that NT gives no cross-topic atomicity, so
everything for a frame is published and then flushed once, which puts it in a single NT4 packet in
practice.

"In practice" is not a guarantee: NT also sends on its own timer, which can land mid-frame. So each
frame is bracketed by a sequence number, ``sequenceBegin`` written first and ``sequence`` written
last. A consumer reads ``sequence``, then the arrays, then ``sequenceBegin``; if the two numbers
match, no newer frame had started arriving while it read, and the arrays all belong to that frame.
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
        self._sequence = 0

    def start(self) -> "NtPublisher":
        import ntcore  # deferred: see class docstring

        instance = ntcore.NetworkTableInstance.getDefault()
        instance.startClient4(self.identity)
        instance.setServer(self.server, self.port)
        table = instance.getTable(self.table_path)

        self._nt = instance
        self._pubs = {
            "sequenceBegin": table.getIntegerTopic("sequenceBegin").publish(),
            "sequence": table.getIntegerTopic("sequence").publish(),
            "u": table.getDoubleArrayTopic("u").publish(),
            "v": table.getDoubleArrayTopic("v").publish(),
            "frameWidth": table.getIntegerTopic("frameWidth").publish(),
            "frameHeight": table.getIntegerTopic("frameHeight").publish(),
            "frameTimestamp": table.getDoubleTopic("frameTimestamp").publish(),
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
        frame_timestamp: Optional[float] = None,
        frame_size: tuple[int, int] = (0, 0),
    ) -> None:
        """Publish one frame's detections.

        :param frame_timestamp: when the source says the scene was captured, in the source's clock
            (robot time, in simulation); NaN is published when the source gave none
        :param frame_size: ``(width, height)`` of the image the boxes are in, so a consumer
            rescales its intrinsics rather than assuming the declared resolution
        """
        if not self._pubs:
            return
        p = self._pubs
        self._sequence += 1
        p["sequenceBegin"].set(self._sequence)
        # Box centres, in pixels. Everything a consumer needs to redo the geometry with its own
        # calibrated lens model and its own pose at the capture instant; the ranges below are this
        # module's pinhole estimates, which know nothing about lens warp.
        p["u"].set([(d.bbox[0] + d.bbox[2]) / 2.0 for d in detections])
        p["v"].set([(d.bbox[1] + d.bbox[3]) / 2.0 for d in detections])
        p["frameWidth"].set(int(frame_size[0]))
        p["frameHeight"].set(int(frame_size[1]))
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
        p["frameTimestamp"].set(float("nan") if frame_timestamp is None else frame_timestamp)
        # Count after the arrays: a consumer that reads count first and then the arrays sees a
        # length that is never longer than what is actually there.
        p["count"].set(len(detections))
        # And the closing sequence number after everything, so it only ever names a frame whose
        # every value has already been written.
        p["sequence"].set(self._sequence)
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
