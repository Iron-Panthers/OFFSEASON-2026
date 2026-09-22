"""
Measure drivetrain and vision fidelity between a simulated log and a real one.

The signals that diverge worst between sim and real are all drivetrain ones, and none of the
existing metrics say *why*. These do, and every one is defined identically on both sides, so a
sim/real comparison means something:

  slip      How badly the four modules disagree about the robot's rigid-body motion. Fit
            (vx, vy, omega) to the four module states by least squares; a robot whose wheels all
            grip reads near zero, and slip or scrub shows up as residual. Uses nothing but module
            states, so it is immune to pose drift.

  gyro      Gyro yaw rate minus the yaw rate the modules imply. Slip again, from an independent
            sensor, so it catches the case where all four wheels slip together.

  odometry  Path length by wheel odometry over path length by the pose estimator. Wheels that slip
            travel further than the robot does, so this sits above 1 on a real robot.

  vision    Tag counts, per-camera visibility and target distance. Vision accuracy falls off with
            range, so a simulation seeing tags at the wrong distances cannot have the right pose
            error even with a perfect camera model.

  precision Disagreement between two cameras that reported a pose on the SAME loop, binned by how
            far away the tags were. This is the honest measure of vision accuracy: both cameras saw
            the same robot at the same instant, so the pose estimator, its lag and its tuning are
            all out of the picture and what is left is vision error alone. It is also the number
            that has to GROW with distance, because that is what real vision does.

Usage:
    python scripts/drive_vision_fidelity.py <sim.wpilog> <real.wpilog>
    python scripts/drive_vision_fidelity.py <log.wpilog>
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from log_compare import find_match_window, resample  # noqa: E402
from wpilog_to_csv import read_log  # noqa: E402

GRID_DT = 0.02

# Metres from robot centre, NWU. DriveConstants names the front-to-back distance `trackWidth` and
# the side-to-side one `trackLength`, which reads backwards but is self-consistent there: modules
# 0 and 1 are the +x (front) pair.
HALF_X = 0.5 * 19.75 * 0.0254
HALF_Y = 0.5 * 24.25 * 0.0254
MODULES = [(HALF_X, HALF_Y), (HALF_X, -HALF_Y), (-HALF_X, HALF_Y), (-HALF_X, -HALF_Y)]

# Below this the robot is parked, and every ratio here becomes noise over noise.
MOVING_MPS = 0.25

CAMERA_COUNT = 3


def _keys():
    keys = [
        "DriverStation/Enabled",
        "Swerve/Gyro/YawVelocityRadPerSec",
        "RealOutputs/Robot State/Estimated Pose",
        "RealOutputs/Vision/GetMultiTags",
    ]
    for i in range(4):
        keys += [
            f"Swerve/Module{i}/DriveVelocityMetersPerSec",
            f"Swerve/Module{i}/DrivePositionMeters",
            f"RealOutputs/Swerve/Module{i}/Steer Setpoint",
            f"RealOutputs/Swerve/Module{i}/SteerError",
        ]
    for i in range(CAMERA_COUNT):
        keys.append(f"RealOutputs/Vision/Camera{i}/Average Distance")
        keys.append(f"RealOutputs/Vision/Camera{i}/Accepted Poses")
    return keys


# Two cameras disagreeing by more than this are not both working; it is a bad solve the rejection
# filter happened to let through, and averaging it in swamps the precision measurement.
MAX_PLAUSIBLE_DISAGREEMENT_M = 5.0


def _camera_precision(series, rows):
    """
    Disagreement between cameras that reported a pose on the same loop, binned by target distance.

    Both cameras saw the same robot at the same instant, so every difference between their answers
    is vision error -- no pose estimator, no lag, no filter tuning. Binning by distance is the
    point: real vision degrades with range, and a simulation whose error is flat in distance is not
    simulating vision at all, however small that error is.
    """
    bins = {}
    for row in range(rows):
        seen = []
        for i in range(CAMERA_COUNT):
            poses = series[f"RealOutputs/Vision/Camera{i}/Accepted Poses"][row]
            distance = series[f"RealOutputs/Vision/Camera{i}/Average Distance"][row]
            if poses and distance and distance > 0.01:
                seen.append((poses[0], distance))
        for a in range(len(seen)):
            for b in range(a + 1, len(seen)):
                (pose_a, dist_a), (pose_b, dist_b) = seen[a], seen[b]
                error = math.hypot(pose_a[0] - pose_b[0], pose_a[1] - pose_b[1])
                if error > MAX_PLAUSIBLE_DISAGREEMENT_M:
                    continue
                bins.setdefault(int(0.5 * (dist_a + dist_b)), []).append(error)
    return {k: _stats(v) for k, v in bins.items() if len(v) >= 30}


def _fit_rigid_body(speeds, angles):
    """
    Least-squares (vx, vy, omega) from four module states, and the residual.

    Each module contributes v_i = (vx - omega*y_i, vy + omega*x_i). Four modules give eight
    equations for three unknowns, and the leftover is exactly the part of the wheels' motion that
    no rigid-body motion can explain -- which is slip, scrub, or a bad steer angle.
    """
    a = np.zeros((8, 3))
    b = np.zeros(8)
    for i, (x, y) in enumerate(MODULES):
        a[2 * i] = (1.0, 0.0, -y)
        a[2 * i + 1] = (0.0, 1.0, x)
        b[2 * i] = speeds[i] * math.cos(angles[i])
        b[2 * i + 1] = speeds[i] * math.sin(angles[i])
    solution, _, _, _ = np.linalg.lstsq(a, b, rcond=None)
    residual = float(np.sqrt(np.mean((a @ solution - b) ** 2)))
    return solution, residual


def _stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        "n": len(ordered),
        "mean": sum(ordered) / len(ordered),
        "p50": ordered[len(ordered) // 2],
        "p95": ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))],
        "max": ordered[-1],
    }


# A single 20 ms step longer than this is not the robot moving. In a replay the pose is snapped
# back to the logged one every few seconds, and those teleports were adding metres of phantom path
# -- enough to put the simulated wheel/pose ratio at 0.87, i.e. wheels apparently travelling LESS
# than the robot, which is physically impossible.
MAX_STEP_M = 0.15


def _path_length(poses):
    total = 0.0
    jumps = 0
    for a, b in zip(poses, poses[1:]):
        try:
            step = math.hypot(b[0] - a[0], b[1] - a[1])
        except (TypeError, IndexError):
            continue
        if step > MAX_STEP_M:
            jumps += 1
            continue
        total += step
    return total, jumps


def analyse(path):
    data = read_log(path, keys=_keys())
    start, end = find_match_window(data)
    if start is None:
        raise SystemExit(f"no enabled window in {path}")
    if end is None:
        # A scripted-teleop run ends while still enabled, so there is no closing disable record.
        # Run to the last sample instead of refusing to analyse the log.
        end = max((ts for s in data.values() for ts, _ in s), default=start)
    grid = [start + i * GRID_DT for i in range(int((end - start) / GRID_DT))]
    series = {k: resample(data.get(k, []), grid) for k in _keys()}

    slips, gyro_residuals, speeds_seen = [], [], []
    for row in range(len(grid)):
        speeds, angles = [], []
        for i in range(4):
            v = series[f"Swerve/Module{i}/DriveVelocityMetersPerSec"][row]
            setpoint = series[f"RealOutputs/Swerve/Module{i}/Steer Setpoint"][row]
            error = series[f"RealOutputs/Swerve/Module{i}/SteerError"][row]
            if v is None or setpoint is None or error is None:
                break
            speeds.append(v)
            angles.append(setpoint - error)
        if len(speeds) != 4:
            continue
        (vx, vy, omega), residual = _fit_rigid_body(speeds, angles)
        speed = math.hypot(vx, vy)
        if speed < MOVING_MPS:
            continue
        speeds_seen.append(speed)
        # Normalised by speed, so it reads as a fraction rather than scaling with how fast the
        # robot happens to be going and telling us only that it moved.
        slips.append(residual / speed)
        gyro = series["Swerve/Gyro/YawVelocityRadPerSec"][row]
        if gyro is not None:
            gyro_residuals.append(abs(gyro - omega))

    # Path length: wheels against the pose estimator, both at full rate so the teleport filter
    # in _path_length can tell a jump from fast driving.
    wheel_path = 0.0
    for i in range(4):
        pos = [v for v in series[f"Swerve/Module{i}/DrivePositionMeters"] if v is not None]
        wheel_path += sum(abs(b - a) for a, b in zip(pos, pos[1:])) / 4.0
    poses = [p for p in series["RealOutputs/Robot State/Estimated Pose"] if p is not None]
    pose_path, pose_jumps = _path_length(poses)

    vision = {}
    for i in range(CAMERA_COUNT):
        seen = series[f"RealOutputs/Vision/Camera{i}/Average Distance"]
        # Zero means "no target this frame", not "a tag at zero metres".
        vision[i] = [v for v in seen if isinstance(v, (int, float)) and v > 0.01]

    multitags = [v for v in series["RealOutputs/Vision/GetMultiTags"] if v is not None]

    return {
        "grid": len(grid),
        "moving": len(slips),
        "speed": _stats(speeds_seen),
        "slip": _stats(slips),
        "gyro": _stats(gyro_residuals),
        "wheel_path": wheel_path,
        "pose_path": pose_path,
        "pose_jumps": pose_jumps,
        "vision": vision,
        "multitags": _stats(multitags),
        "precision": _camera_precision(series, len(grid)),
    }


def report(label, r):
    lines = [f"--- {label} ---", f"  moving samples        {r['moving']} of {r['grid']}"]
    if r["speed"]:
        lines.append(
            f"  speed while moving    mean {r['speed']['mean']:.2f}"
            f"  p95 {r['speed']['p95']:.2f}  max {r['speed']['max']:.2f} m/s"
        )
    if r["slip"]:
        lines.append(
            f"  module disagreement   mean {r['slip']['mean']:.4f}"
            f"  p50 {r['slip']['p50']:.4f}  p95 {r['slip']['p95']:.4f}  (fraction of speed)"
        )
    if r["gyro"]:
        lines.append(
            f"  gyro minus modules    mean {r['gyro']['mean']:.4f}"
            f"  p95 {r['gyro']['p95']:.4f} rad/s"
        )
    if r["pose_path"] > 1.0:
        lines.append(
            f"  wheel path/pose path  {r['wheel_path'] / r['pose_path']:.4f}"
            f"  ({r['wheel_path']:.1f} m / {r['pose_path']:.1f} m,"
            f" {r['pose_jumps']} teleports dropped)"
        )
    if r["multitags"]:
        lines.append(f"  multi-tag frames      mean {r['multitags']['mean']:.3f}")
    for i, dist in sorted(r["vision"].items()):
        if not dist:
            lines.append(f"  camera{i} target range  never sees a tag")
            continue
        ordered = sorted(dist)
        lines.append(
            f"  camera{i} target range  n {len(ordered):>5}"
            f"  mean {sum(ordered) / len(ordered):.2f}"
            f"  p50 {ordered[len(ordered) // 2]:.2f}"
            f"  p95 {ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]:.2f} m"
        )
    if r["precision"]:
        lines.append("  vision error between two cameras on the same loop:")
        for b in sorted(r["precision"]):
            s = r["precision"][b]
            lines.append(
                f"    {b}-{b + 1} m  n {s['n']:>5}  mean {s['mean']:.3f}"
                f"  p50 {s['p50']:.3f}  p95 {s['p95']:.3f} m"
            )
    return "\n".join(lines)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    for arg, label in zip(sys.argv[1:], ("sim", "real")):
        print(report(label, analyse(arg)))
        print()
