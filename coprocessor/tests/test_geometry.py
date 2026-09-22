"""Geometry round trips and sign conventions.

No model, no network, no OpenCV: this is the part where a mistake is invisible in a demo and
wrong on the field, so it is the part that gets tested properly.
"""

import math

import pytest

from coproc.modules.object_detection import geometry
from coproc.runtime.intrinsics import Intrinsics, angle_between

RADIUS = 0.075  # 2026 Fuel


@pytest.fixture
def cam() -> Intrinsics:
    return Intrinsics.from_diagonal_fov(640, 400, 70.0)


def test_diagonal_fov_is_actually_the_diagonal(cam: Intrinsics):
    """The corner-to-corner angle must come back out as the FOV we asked for."""
    corner_to_corner = angle_between(cam.unit_ray(0, 0), cam.unit_ray(cam.width, cam.height))
    assert math.degrees(corner_to_corner) == pytest.approx(70.0, abs=1e-6)


def test_principal_ray_points_straight_ahead(cam: Intrinsics):
    assert cam.unit_ray(cam.cx, cam.cy) == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)


def test_signs_follow_wpilib_camera_convention(cam: Intrinsics):
    """X forward, Y left, Z up. Getting this wrong mirrors the field and looks plausible."""
    left_of_centre = cam.unit_ray(cam.cx - 100, cam.cy)
    assert left_of_centre[1] > 0, "a target left in the image must be +Y"

    above_centre = cam.unit_ray(cam.cx, cam.cy - 100)
    assert above_centre[2] > 0, "a target high in the image must be +Z"


def test_project_inverts_unit_ray(cam: Intrinsics):
    for u, v in [(0, 0), (320, 200), (639, 399), (100, 350)]:
        x, y, z = cam.unit_ray(u, v)
        assert cam.project(x, y, z) == pytest.approx((u, v), abs=1e-9)


@pytest.mark.parametrize(
    "position",
    [
        (1.0, 0.0, 0.0),
        (3.0, 0.0, 0.0),
        (6.0, 0.0, 0.0),
        (2.0, 0.6, 0.0),
        (2.0, -0.6, 0.0),
        (2.0, 0.0, 0.35),
        (4.0, 1.1, -0.5),
    ],
)
def test_sphere_round_trip(cam: Intrinsics, position):
    """Project a ball at a known place, then recover it from its box."""
    x, y, z = position
    bbox = geometry.sphere_bbox(x, y, z, cam, RADIUS)
    found = geometry.locate_sphere(bbox, cam, RADIUS, "fuel", 0.9)

    expected_distance = math.sqrt(x * x + y * y + z * z)
    # 1% covers the circular-silhouette approximation used off-axis; the model's own box jitter
    # is far larger than this.
    assert found.distance_meters == pytest.approx(expected_distance, rel=0.01)
    assert found.x == pytest.approx(x, rel=0.02, abs=0.01)
    assert found.y == pytest.approx(y, rel=0.02, abs=0.01)
    assert found.z == pytest.approx(z, rel=0.02, abs=0.01)


def test_yaw_and_pitch_signs(cam: Intrinsics):
    left = geometry.locate_sphere(
        geometry.sphere_bbox(3.0, 1.0, 0.0, cam, RADIUS), cam, RADIUS, "fuel", 1.0
    )
    assert left.yaw_degrees > 0

    right = geometry.locate_sphere(
        geometry.sphere_bbox(3.0, -1.0, 0.0, cam, RADIUS), cam, RADIUS, "fuel", 1.0
    )
    assert right.yaw_degrees < 0

    high = geometry.locate_sphere(
        geometry.sphere_bbox(3.0, 0.0, 0.5, cam, RADIUS), cam, RADIUS, "fuel", 1.0
    )
    assert high.pitch_degrees > 0


def test_distance_scales_with_radius(cam: Intrinsics):
    """Radius is a pure scale factor on range, so a wrong ball size is a proportional error."""
    bbox = geometry.sphere_bbox(3.0, 0.0, 0.0, cam, RADIUS)
    single = geometry.locate_sphere(bbox, cam, RADIUS, "fuel", 1.0)
    double = geometry.locate_sphere(bbox, cam, RADIUS * 2, "fuel", 1.0)
    assert double.distance_meters == pytest.approx(single.distance_meters * 2, rel=1e-9)


def test_further_balls_look_smaller(cam: Intrinsics):
    near = geometry.sphere_bbox(2.0, 0.0, 0.0, cam, RADIUS)
    far = geometry.sphere_bbox(6.0, 0.0, 0.0, cam, RADIUS)
    assert (far[2] - far[0]) < (near[2] - near[0])


def test_edge_flag_marks_clipped_boxes(cam: Intrinsics):
    assert geometry.touches_edge((0.0, 100.0, 60.0, 160.0), cam)
    assert geometry.touches_edge((300.0, 0.0, 360.0, 60.0), cam)
    assert geometry.touches_edge((580.0, 100.0, 640.0, 160.0), cam)
    assert not geometry.touches_edge((300.0, 180.0, 340.0, 220.0), cam)


def test_clipped_box_reads_as_too_far(cam: Intrinsics):
    """The bias the edge flag exists to warn about, pinned down so it cannot drift."""
    whole = geometry.sphere_bbox(2.0, 0.0, 0.0, cam, RADIUS)
    x0, y0, x1, y1 = whole
    clipped = (x0 + (x1 - x0) / 2, y0, x1, y1)  # half the width visible

    assert (
        geometry.locate_sphere(clipped, cam, RADIUS, "fuel", 1.0).distance_meters
        > geometry.locate_sphere(whole, cam, RADIUS, "fuel", 1.0).distance_meters
    )


def test_occlusion_away_from_the_border_is_not_flagged(cam: Intrinsics):
    """A documented blind spot, asserted so nobody mistakes the edge flag for occlusion cover.

    A ball half hidden behind another robot in the middle of the frame reads as too far away and
    carries no warning. Only clipping against the frame border is detectable from a box alone.
    """
    x0, y0, x1, y1 = geometry.sphere_bbox(2.0, 0.0, 0.0, cam, RADIUS)
    half_hidden = (x0 + (x1 - x0) / 2, y0, x1, y1)
    assert not geometry.touches_edge(half_hidden, cam)


def test_degenerate_box_is_rejected(cam: Intrinsics):
    with pytest.raises(ValueError):
        geometry.locate_sphere((10.0, 10.0, 10.0, 20.0), cam, RADIUS, "fuel", 1.0)


def test_rescaling_intrinsics_preserves_angles(cam: Intrinsics):
    """A stream at an unexpected size must not change what a direction means."""
    half = cam.scaled_to(320, 200)
    assert half.unit_ray(half.cx, half.cy) == pytest.approx(cam.unit_ray(cam.cx, cam.cy))
    assert half.unit_ray(0, 0) == pytest.approx(cam.unit_ray(0, 0), abs=1e-12)


def test_rescaled_intrinsics_recover_the_same_distance(cam: Intrinsics):
    half = cam.scaled_to(320, 200)
    at_full = geometry.locate_sphere(
        geometry.sphere_bbox(3.0, 0.4, 0.0, cam, RADIUS), cam, RADIUS, "fuel", 1.0
    )
    at_half = geometry.locate_sphere(
        geometry.sphere_bbox(3.0, 0.4, 0.0, half, RADIUS), half, RADIUS, "fuel", 1.0
    )
    assert at_half.distance_meters == pytest.approx(at_full.distance_meters, rel=1e-6)
    assert at_half.yaw_degrees == pytest.approx(at_full.yaw_degrees, rel=1e-6)


def test_bad_intrinsics_are_rejected():
    with pytest.raises(ValueError):
        Intrinsics.from_diagonal_fov(0, 400, 70.0)
    with pytest.raises(ValueError):
        Intrinsics.from_diagonal_fov(640, 400, 180.0)


@pytest.mark.parametrize(
    "position",
    [(3.0, 1.2, 0.8), (2.0, 0.9, 0.6), (5.0, 1.9, -1.1), (4.0, 1.1, -0.5)],
)
def test_off_axis_range_error_stays_bounded(cam: Intrinsics, position):
    """Pin the cost of the estimator's one approximation, at the corners of the lens.

    ``locate_sphere`` averages the horizontal and vertical angular half-extents of the box, which
    is exact on axis and slightly under-reads the elliptical silhouette off it. Measured against
    the true silhouette the error stays under 1% out to the edge of a 70 degree lens -- far below
    the box jitter of any detector. This test exists so that if someone changes the estimator,
    the number they are trading against is visible.
    """
    x, y, z = position
    bbox = geometry.sphere_bbox(x, y, z, cam, RADIUS)
    found = geometry.locate_sphere(bbox, cam, RADIUS, "fuel", 1.0)

    truth = math.sqrt(x * x + y * y + z * z)
    assert abs(found.distance_meters / truth - 1.0) < 0.01


def test_bearing_is_accurate_off_axis(cam: Intrinsics):
    """Range degrades off axis; bearing must not, because that is what a robot steers on."""
    x, y, z = 4.0, 1.4, -0.7
    bbox = geometry.sphere_bbox(x, y, z, cam, RADIUS)
    found = geometry.locate_sphere(bbox, cam, RADIUS, "fuel", 1.0)

    assert found.yaw_degrees == pytest.approx(math.degrees(math.atan2(y, x)), abs=0.15)
    assert found.pitch_degrees == pytest.approx(
        math.degrees(math.atan2(z, math.hypot(x, y))), abs=0.15
    )
