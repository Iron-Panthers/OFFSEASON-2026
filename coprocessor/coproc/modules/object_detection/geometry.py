"""Turning a bounding box around a sphere into a position in the camera frame.

Pure math: no OpenCV, no model, no network. That is what makes it testable, and this is the part
worth testing, because a sign error here produces results that look entirely plausible and are
mirrored.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from ...runtime.intrinsics import Intrinsics, angle_between
from ...runtime.module import Detection

# How close to the border counts as clipped. A box genuinely touching the edge usually lands a
# pixel or two inside it, so an exact comparison misses most real clipping.
_EDGE_TOLERANCE_PX = 2.0

# Fractional gap between the two range estimates that means the box is wrong.
#
# Not a guess. A box wrong along one axis -- merged across a neighbour, or clipped down one side
# by something in front -- moves the size-based range by a factor of exactly 3/2 or 2/3, because
# the angular radius averages the two half-extents and only one of them changed. That is a 0.33
# gap, and it is 0.33 at every range, so the threshold only has to sit below it with enough room
# left for the honest error: box jitter on the size estimate, and the ground plane's own bias,
# which reaches about 0.10 at the far end of useful range. 0.20 separates the two cleanly.
DISAGREEMENT_TOLERANCE = 0.20


@dataclass(frozen=True)
class Mounting:
    """Where the camera sits above the floor, and how it is tilted relative to gravity.

    The only piece of robot state the coprocessor carries, and it is carried for one reason: a
    ball on the floor has a known centre height, which turns the bearing to it into a range
    without reference to how big its box is. Yaw is deliberately absent -- rotating the camera
    about the vertical changes nothing about where its rays meet the floor, so asking for it
    would only invite someone to supply a stale one.

    Angles follow the convention in ``ObjectDetectionConstants``: **positive pitch is nose
    down**, matching WPILib's ``Rotation3d``.
    """

    height_meters: float
    """Camera origin above the floor."""

    pitch_radians: float
    """Downward tilt, positive nose down."""

    roll_radians: float = 0.0
    """Rotation about the optical axis. Zero for anything mounted square."""

    def vertical_component(self, ray: tuple[float, float, float]) -> float:
        """How fast a camera-frame ray descends, per unit length, in the gravity-aligned frame.

        The third row of the camera-to-level rotation, applied to the ray. Negative means the ray
        is heading towards the floor. Yaw drops out of that row entirely, which is why this needs
        no heading.
        """
        sp, cp = math.sin(self.pitch_radians), math.cos(self.pitch_radians)
        sr, cr = math.sin(self.roll_radians), math.cos(self.roll_radians)
        return -sp * ray[0] + cp * sr * ray[1] + cp * cr * ray[2]


def ground_plane_range(
    ray: tuple[float, float, float], mounting: Mounting, target_height_meters: float
) -> Optional[float]:
    """Distance along ``ray`` to where it meets the plane the target's centre lives on.

    A ball resting on the floor has its centre at exactly its own radius, so that plane is known
    without measuring anything. Because this reads the bearing rather than the silhouette, a box
    that is twice too wide does not move it at all -- which is the entire point, since a box
    merged across neighbouring balls is the common failure in a dense pile.

    Returns None when the ray never reaches the plane: at or above the horizon, or from a camera
    mounted below the target's centre.
    """
    drop = mounting.height_meters - target_height_meters
    if drop <= 0.0:
        return None  # the camera is at or below the ball; there is no downward intersection

    descent = mounting.vertical_component(ray)
    if descent >= -1e-9:
        return None  # at or above the horizon, so it never meets the floor

    return drop / -descent


def angular_radius(
    bbox: tuple[float, float, float, float], intrinsics: Intrinsics
) -> float:
    """Half-angle subtended by the box, in radians.

    Computed from actual ray directions rather than from ``pixels / focal_length``. The cheap
    version is a small-angle approximation that is fine in the centre of the image and wrong at
    the corners, which is exactly where a wide-angle lens puts most of its field.
    """
    x0, y0, x1, y1 = bbox
    u_c = (x0 + x1) / 2.0
    v_c = (y0 + y1) / 2.0

    horizontal = angle_between(
        intrinsics.unit_ray(x0, v_c), intrinsics.unit_ray(x1, v_c)
    ) / 2.0
    vertical = angle_between(
        intrinsics.unit_ray(u_c, y0), intrinsics.unit_ray(u_c, y1)
    ) / 2.0
    return (horizontal + vertical) / 2.0


def touches_edge(
    bbox: tuple[float, float, float, float], intrinsics: Intrinsics
) -> bool:
    """True when the box runs into the frame border, so its size understates the object."""
    x0, y0, x1, y1 = bbox
    return (
        x0 <= _EDGE_TOLERANCE_PX
        or y0 <= _EDGE_TOLERANCE_PX
        or x1 >= intrinsics.width - _EDGE_TOLERANCE_PX
        or y1 >= intrinsics.height - _EDGE_TOLERANCE_PX
    )


def locate_sphere(
    bbox: tuple[float, float, float, float],
    intrinsics: Intrinsics,
    radius_meters: float,
    label: str,
    confidence: float,
    mounting: Optional[Mounting] = None,
    disagreement_tolerance: float = DISAGREEMENT_TOLERANCE,
) -> Detection:
    """Locate a sphere of known radius from its bounding box.

    Two independent ranges are computed where the mounting allows it.

    *From apparent size*: a sphere at distance ``d`` presents a silhouette of half-angle
    ``theta`` where ``sin(theta) = r / d``. Exact for a sphere viewed from any angle, which is
    why the renderer drawing fuel as analytic spheres rather than as a faceted mesh matters: the
    silhouette it produces is the one this inverts. It reads the box *extent*, so every error in
    the box lands on it undiluted -- a box merged across two balls halves the range, a clipped
    one doubles it, and neither failure announces itself.

    *From the ground plane*: the bearing through the box centre, intersected with the plane the
    ball's centre sits on. It reads the box *centre*, which the same failures barely move.

    The ground-plane range becomes the reported one when it is available, because a ball on the
    floor is the case this module exists for and its error stays bounded where the size-based
    error does not. The size-based range is kept alongside it, and the two disagreeing by more
    than ``disagreement_tolerance`` sets :attr:`Detection.suspect`. That catches occlusion away
    from the frame border, which no single method can see.

    The bearing comes from the box centre. For an off-axis sphere the silhouette is very slightly
    elliptical and its centre is not exactly the projection of the sphere's centre; the error is
    far below the model's own box jitter and is ignored.
    """
    x0, y0, x1, y1 = bbox
    if x1 <= x0 or y1 <= y0:
        raise ValueError(f"degenerate bounding box {bbox}")

    theta = angular_radius(bbox, intrinsics)
    if theta <= 0.0:
        raise ValueError("bounding box subtends no angle")

    size_distance = radius_meters / math.sin(theta)

    u_c = (x0 + x1) / 2.0
    v_c = (y0 + y1) / 2.0
    ray = intrinsics.unit_ray(u_c, v_c)
    dx, dy, dz = ray

    ground_distance = (
        None if mounting is None else ground_plane_range(ray, mounting, radius_meters)
    )

    distance = size_distance if ground_distance is None else ground_distance

    suspect = False
    if ground_distance is not None:
        # Relative to the ground-plane estimate, since that is the one being trusted.
        suspect = abs(size_distance - ground_distance) / ground_distance > disagreement_tolerance

    return Detection(
        label=label,
        confidence=confidence,
        yaw_degrees=math.degrees(math.atan2(dy, dx)),
        pitch_degrees=math.degrees(math.atan2(dz, math.hypot(dx, dy))),
        distance_meters=distance,
        x=dx * distance,
        y=dy * distance,
        z=dz * distance,
        bbox=(x0, y0, x1, y1),
        edge=touches_edge(bbox, intrinsics),
        size_distance_meters=size_distance,
        ground_distance_meters=ground_distance,
        suspect=suspect,
    )


def sphere_bbox(
    x: float,
    y: float,
    z: float,
    intrinsics: Intrinsics,
    radius_meters: float,
    samples: int = 512,
) -> tuple[float, float, float, float]:
    """Project a sphere at a known camera-frame position to its exact silhouette bounding box.

    The forward model for :func:`locate_sphere`. Exact rather than approximate: the silhouette of
    a sphere is the cone of rays tangent to it, so this walks that cone and takes the extent of
    what it projects to. Off-axis the result is an ellipse whose bounding box is neither square
    nor centred on the projection of the sphere's centre, and an approximate forward model here
    would only re-test the estimator's own assumptions.

    Sampling rather than solving: the closed form for the extremes exists but is long, and at 512
    samples the error is several orders of magnitude below anything that matters.

    Also useful beyond the tests -- the simulation knows where every ball really is, so this is
    what turns that into labelled boxes for training.
    """
    distance = math.sqrt(x * x + y * y + z * z)
    if distance <= radius_meters:
        raise ValueError("camera is inside the sphere")

    axis = (x / distance, y / distance, z / distance)
    theta = math.asin(radius_meters / distance)

    # Any two directions perpendicular to the axis; which two is irrelevant, the cone is
    # symmetric. Pick the seed that is least parallel to the axis so the cross product is stable.
    seed = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (0.0, 1.0, 0.0)
    e1 = _normalise(_cross(axis, seed))
    e2 = _cross(axis, e1)

    us: list[float] = []
    vs: list[float] = []
    for i in range(samples):
        phi = 2.0 * math.pi * i / samples
        direction = tuple(
            math.cos(theta) * axis[k]
            + math.sin(theta) * (math.cos(phi) * e1[k] + math.sin(phi) * e2[k])
            for k in range(3)
        )
        if direction[0] <= 0.0:
            continue  # behind the image plane; the sphere is clipped by the horizon of the lens
        u, v = intrinsics.project(*direction)
        us.append(u)
        vs.append(v)

    if not us:
        raise ValueError("sphere projects entirely behind the camera")
    return (min(us), min(vs), max(us), max(vs))


def _cross(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalise(v: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    return (v[0] / norm, v[1] / norm, v[2] / norm)
