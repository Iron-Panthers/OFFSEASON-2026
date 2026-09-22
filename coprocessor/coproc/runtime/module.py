"""What a coprocessor module is.

A module turns one frame into detections and an annotated image. Everything else -- getting
frames, reconnecting, publishing, serving the annotated stream, timing -- belongs to the runtime,
so a new module is one file rather than a new pipeline.
"""

from __future__ import annotations

import argparse
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Optional, Sequence

from .intrinsics import Intrinsics


@dataclass(frozen=True)
class Detection:
    """One detected object, in the WPILib camera frame: X forward, Y left, Z up."""

    label: str
    confidence: float

    yaw_degrees: float
    """Bearing off the optical axis, counter-clockwise positive (left is positive)."""

    pitch_degrees: float
    """Elevation off the optical axis, up positive."""

    distance_meters: float
    """Range to the object's centre."""

    x: float
    y: float
    z: float
    """Camera-frame position, metres."""

    bbox: tuple[float, float, float, float]
    """``(x0, y0, x1, y1)`` in pixels, for drawing."""

    edge: bool
    """True when the box touches the frame border.

    A clipped silhouette is smaller than the real one, so the object reads as further away than
    it is. Consumers should distrust :attr:`distance_meters` when this is set rather than
    discovering the bias the hard way.
    """


@dataclass
class ModuleContext:
    """Everything the runtime hands a module at start-up."""

    intrinsics: Intrinsics
    args: argparse.Namespace
    extras: dict[str, Any] = field(default_factory=dict)


class CoprocessorModule(ABC):
    """Base class for a coprocessor module.

    Implementations must be importable without their heavy dependencies present at module scope:
    import Torch and friends inside :meth:`setup`, not at the top of the file, so that listing
    modules or running the geometry tests does not require a 2.5 GB install.
    """

    #: Name used on the command line and in the NetworkTables path.
    name: str = "module"

    @abstractmethod
    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        """Register this module's own command-line options."""

    @abstractmethod
    def setup(self, context: ModuleContext) -> None:
        """Load models and allocate anything expensive. Called once, before the first frame."""

    @abstractmethod
    def process(self, image: Any, context: ModuleContext) -> Sequence[Detection]:
        """Detect in one decoded BGR image.

        Must not mutate ``image``; the runtime reuses it for annotation.
        """

    def annotate(self, image: Any, detections: Sequence[Detection]) -> Optional[Any]:
        """Draw detections for the outbound stream.

        Only called when somebody is actually watching. Returning None means "no annotated
        output", and the runtime falls back to the raw frame.
        """
        return None
