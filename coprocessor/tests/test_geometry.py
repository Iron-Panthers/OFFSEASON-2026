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
    """Asserted so nobody mistakes the edge flag for occlusion cover.

    A ball half hidden behind another robot in the middle of the frame touches no border, so the
    edge flag stays silent about it: clipping against the frame is the only thing a box alone can
    reveal. That is still true and still the limit of this flag. What covers the case now is the
    range cross-check, which needs a second, independent estimate to see it at all --
    ``test_occlusion_away_from_the_border_is_now_caught``.
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


# --- Ground-plane ranging -------------------------------------------------------------------
#
# The object-detection camera, from ObjectDetectionConstants.ROBOT_TO_CAMERA: 0.60 m up, tilted
# 20 degrees down. Positive pitch is nose down, matching WPILib's Rotation3d.
MOUNT = geometry.Mounting(height_meters=0.60, pitch_radians=math.radians(20.0))


def ball_on_floor(distance_m: float, lateral_m: float = 0.0):
    """A ball resting on the floor, in the camera frame of :data:`MOUNT`.

    The camera looks down by its pitch, so a ball that is ``distance_m`` away along the floor
    sits below and ahead of it, rotated into the camera's own axes.
    """
    drop = MOUNT.height_meters - RADIUS
    pitch = MOUNT.pitch_radians
    # Level frame: forward along the floor, down by the full drop. Rotate into the camera frame,
    # which is the level frame pitched nose-down, i.e. rotate the point back up by the pitch.
    forward, down = distance_m, -drop
    x = forward * math.cos(pitch) - down * math.sin(pitch)
    z = forward * math.sin(pitch) + down * math.cos(pitch)
    return x, lateral_m, z


@pytest.mark.parametrize("distance", [0.8, 1.5, 2.5, 4.0, 6.0])
def test_ground_plane_recovers_a_ball_on_the_floor(cam: Intrinsics, distance):
    """The whole point: range from the bearing, with no reference to the box's size."""
    x, y, z = ball_on_floor(distance)
    bbox = geometry.sphere_bbox(x, y, z, cam, RADIUS)
    found = geometry.locate_sphere(bbox, cam, RADIUS, "fuel", 1.0, MOUNT)

    truth = math.sqrt(x * x + y * y + z * z)
    assert found.ground_distance_meters == pytest.approx(truth, rel=0.01)
    assert found.distance_meters == pytest.approx(truth, rel=0.01)
    assert not found.suspect, "the two estimates agree on a clean box"


def test_ground_plane_is_the_reported_range_when_a_mounting_is_given(cam: Intrinsics):
    x, y, z = ball_on_floor(2.0)
    bbox = geometry.sphere_bbox(x, y, z, cam, RADIUS)
    found = geometry.locate_sphere(bbox, cam, RADIUS, "fuel", 1.0, MOUNT)
    assert found.distance_meters == found.ground_distance_meters
    # and the position follows the reported range, not the discarded one
    assert math.sqrt(found.x**2 + found.y**2 + found.z**2) == pytest.approx(
        found.ground_distance_meters, rel=1e-9
    )


def test_without_a_mounting_nothing_changes(cam: Intrinsics):
    """The old behaviour is still exactly the old behaviour."""
    x, y, z = ball_on_floor(2.0)
    bbox = geometry.sphere_bbox(x, y, z, cam, RADIUS)
    found = geometry.locate_sphere(bbox, cam, RADIUS, "fuel", 1.0)

    assert found.ground_distance_meters is None
    assert not found.suspect
    assert found.distance_meters == found.size_distance_meters


def test_a_box_merged_across_two_balls_is_caught(cam: Intrinsics):
    """The failure that carries no warning today.

    Two balls side by side in one box: the silhouette is twice as wide, so the size-based range
    halves. The centre barely moves, so the ground-plane range does not. That disagreement is the
    only signal available that the box is wrong.
    """
    x, y, z = ball_on_floor(2.5)
    x0, y0, x1, y1 = geometry.sphere_bbox(x, y, z, cam, RADIUS)
    merged = (x0, y0, x1 + (x1 - x0), y1)  # a neighbouring ball swallowed into the same box

    found = geometry.locate_sphere(merged, cam, RADIUS, "fuel", 1.0, MOUNT)

    truth = math.sqrt(x * x + y * y + z * z)
    assert found.size_distance_meters < 0.75 * truth, "size-based range should have collapsed"
    assert found.ground_distance_meters == pytest.approx(truth, rel=0.15)
    assert found.suspect


def test_occlusion_away_from_the_border_is_now_caught(cam: Intrinsics):
    """Counterpart to ``test_occlusion_away_from_the_border_is_not_flagged``.

    A ball half hidden behind a robot mid-frame has a box too small and does not touch any edge,
    so ``edge`` stays false. Size-based range pushes it into the distance; ground-plane range
    does not follow, and the cross-check fires.
    """
    x, y, z = ball_on_floor(2.0)
    x0, y0, x1, y1 = geometry.sphere_bbox(x, y, z, cam, RADIUS)
    half = (x0, y0, x0 + (x1 - x0) / 2.0, y1)  # left half of the ball still visible

    found = geometry.locate_sphere(half, cam, RADIUS, "fuel", 1.0, MOUNT)

    assert not found.edge, "nothing here touches the frame border"
    assert found.suspect


def test_a_ball_above_the_horizon_has_no_ground_range(cam: Intrinsics):
    """A ball in flight is not on the floor, and the estimator must not pretend otherwise."""
    # Well above the optical axis, which is already tilted down: this ray climbs.
    bbox = geometry.sphere_bbox(3.0, 0.0, 2.5, cam, RADIUS)
    found = geometry.locate_sphere(bbox, cam, RADIUS, "fuel", 1.0, MOUNT)

    assert found.ground_distance_meters is None
    assert found.distance_meters == found.size_distance_meters
    assert not found.suspect, "no second opinion exists, so there is nothing to disagree with"


def test_yaw_cannot_affect_the_ground_range():
    """Mounting carries no heading on purpose; rotating about the vertical must be a no-op.

    Guards the derivation: the third row of the camera-to-level rotation drops yaw entirely. If
    somebody reintroduces a heading term, a lateral ball's range starts moving with it.
    """
    ray = (0.90, 0.30, -0.32)
    norm = math.sqrt(sum(c * c for c in ray))
    ray = tuple(c / norm for c in ray)

    expected = (MOUNT.height_meters - RADIUS) / -MOUNT.vertical_component(ray)
    assert geometry.ground_plane_range(ray, MOUNT, RADIUS) == pytest.approx(expected)


def test_a_camera_below_the_ball_centre_has_no_ground_range():
    low = geometry.Mounting(height_meters=0.05, pitch_radians=math.radians(20.0))
    assert geometry.ground_plane_range((0.9, 0.0, -0.4), low, RADIUS) is None
