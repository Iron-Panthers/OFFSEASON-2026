"""Pinhole camera intrinsics, and the ray directions that follow from them.

Kept free of any imaging dependency so the geometry tests run without OpenCV or a model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole model for one camera at one resolution.

    Pixel coordinates are the usual image convention: ``u`` right, ``v`` down, origin at the
    top-left corner of the top-left pixel.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @staticmethod
    def from_diagonal_fov(width: int, height: int, fov_degrees: float) -> "Intrinsics":
        """Build intrinsics from the one number a datasheet actually gives you.

        Matches ``VisionConstants.SIM_CAMERA_FOV_DIAGONAL_DEGREES`` on the robot side, so the
        detector and the renderer agree on what the camera sees without either hardcoding a
        focal length.
        """
        if width <= 0 or height <= 0:
            raise ValueError(f"resolution must be positive, got {width}x{height}")
        if not 0.0 < fov_degrees < 180.0:
            raise ValueError(f"diagonal FOV must be in (0, 180), got {fov_degrees}")

        diagonal_px = math.hypot(width, height)
        focal = (diagonal_px / 2.0) / math.tan(math.radians(fov_degrees) / 2.0)
        return Intrinsics(
            width=width,
            height=height,
            fx=focal,
            fy=focal,
            cx=width / 2.0,
            cy=height / 2.0,
        )

    def scaled_to(self, width: int, height: int) -> "Intrinsics":
        """Rescale to a different resolution of the same optics.

        The stream may not arrive at the resolution the intrinsics were declared for, and
        silently using the wrong focal length biases every range estimate.
        """
        if width == self.width and height == self.height:
            return self
        sx = width / self.width
        sy = height / self.height
        return Intrinsics(
            width=width,
            height=height,
            fx=self.fx * sx,
            fy=self.fy * sy,
            cx=self.cx * sx,
            cy=self.cy * sy,
        )

    def unit_ray(self, u: float, v: float) -> tuple[float, float, float]:
        """Unit direction through a pixel, in the WPILib camera frame.

        WPILib camera convention: X forward out of the lens, Y left, Z up. The image axes are
        both flipped relative to that, which is where the two minus signs come from.
        """
        x_n = (u - self.cx) / self.fx
        y_n = (v - self.cy) / self.fy
        x, y, z = 1.0, -x_n, -y_n
        norm = math.sqrt(x * x + y * y + z * z)
        return (x / norm, y / norm, z / norm)

    def project(self, x: float, y: float, z: float) -> tuple[float, float]:
        """Inverse of :meth:`unit_ray`: a camera-frame point back to a pixel.

        Only meaningful for points in front of the camera. Used by the tests to build a bbox
        from a known ball position and check the estimator recovers it.
        """
        if x <= 0.0:
            raise ValueError(f"point is not in front of the camera (x={x})")
        return (self.cx - (y / x) * self.fx, self.cy - (z / x) * self.fy)


def angle_between(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    """Angle in radians between two unit vectors, numerically safe at small angles."""
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    cross = (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )
    cross_norm = math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2)
    # atan2 rather than acos: acos loses all its precision exactly where these angles live.
    return math.atan2(cross_norm, dot)
