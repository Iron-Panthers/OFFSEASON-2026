"""Turning a bounding box around a sphere into a position in the camera frame.

Pure math: no OpenCV, no model, no network. That is what makes it testable, and this is the part
worth testing, because a sign error here produces results that look entirely plausible and are
mirrored.
"""

from __future__ import annotations

import math

from ...runtime.intrinsics import Intrinsics, angle_between
from ...runtime.module import Detection

# How close to the border counts as clipped. A box genuinely touching the edge usually lands a
# pixel or two inside it, so an exact comparison misses most real clipping.
_EDGE_TOLERANCE_PX = 2.0


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
) -> Detection:
    """Locate a sphere of known radius from its bounding box.

    A sphere at distance ``d`` presents a silhouette of half-angle ``theta`` where
    ``sin(theta) = r / d``, so ``d = r / sin(theta)``. This is exact for a sphere viewed from any
    angle, which is why the renderer drawing fuel as analytic spheres rather than as a faceted
    mesh matters: the silhouette it produces is the one this inverts.

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

    distance = radius_meters / math.sin(theta)

    u_c = (x0 + x1) / 2.0
    v_c = (y0 + y1) / 2.0
    dx, dy, dz = intrinsics.unit_ray(u_c, v_c)

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
