"""
Baseline chart bundles, one per --investigate category.

These deliberately mirror INVESTIGATE_KEYS in wpilog_to_csv.py: the same keys
the text report reads, drawn. If a key is added there it belongs here too, so
the picture and the prose never disagree about what was examined.

Every chart here is ``required=False``. A bundle is generic across all logs and a
vision log has no flywheel, so a missing series is dropped and a chart left with
no series disappears. Charts named explicitly in a findings file are the opposite
(see spec.ChartSpec) -- there, a missing key is a mistake worth stopping for.
"""

from __future__ import annotations

from dataclasses import replace

from .spec import ChartSpec, SeriesSpec

# Key fragments repeated across bundles. Spelled out once so a rename is one edit.
_ENABLED = "DriverStation/Enabled"
_AUTONOMOUS = "DriverStation/Autonomous"
_FLYWHEEL_ACTUAL = "RealOutputs/Shooter/Shooter Flywheels/Current Velocity"
_FLYWHEEL_TARGET = "RealOutputs/Shooter/Shooter Flywheels/Target Velocity"
_SHOOTER_STATE = "RealOutputs/Shooter/Target State"
_UP_TO_SPEED = "RealOutputs/Shooter/Flywheels Up To Speed"
_HOOD_TARGET = "RealOutputs/Shooter/Shooter Hood/Target Position"
_PREDICTED_ANGLE = "RealOutputs/RobotState/Target Shooting State/Shooter Angle"
_ACCELERATOR = "RealOutputs/Shooter/Shooter Accelerator/Target Velocity"
_RACK_TARGET = "RealOutputs/Intake/Intake Rack/Target"
_RACK_REACHED = "RealOutputs/Intake/Intake Rack/Reached Target"
_RACK_ROTATIONS = "RealOutputs/Intake/Intake Rack/Position Target Rotations"
_ROLLERS_TARGET = "RealOutputs/Intake/Intake Rollers/Target"
_ROLLERS_VELOCITY = "RealOutputs/Intake/Intake Rollers/Target Velocity"
_FUEL_COUNT = "RealOutputs/Field Simulation/Fuel Count"
_SWERVE_POSITION = "RealOutputs/Swerve/Current Position"
_PATH_TARGET = "RealOutputs/Path Planner/Target Pose"
_PATH_ERROR = "RealOutputs/Swerve/Distance From Setpoint"
_VISION_DISTANCE = "RealOutputs/Vision/Camera0/Average Distance"
_VISION_TAGS = "RealOutputs/Vision/GetMultiTags"


def _s(key: str, label: str) -> SeriesSpec:
    return SeriesSpec(key=key, label=label)


def _timeline(title: str, series: list[SeriesSpec]) -> ChartSpec:
    return ChartSpec(form="timeline", title=title, series=tuple(series), required=False)


def _timeseries(title, series, unit="", include_zero=True, tolerance=None, note="") -> ChartSpec:
    return ChartSpec(
        form="timeseries",
        title=title,
        series=tuple(series),
        unit=unit,
        include_zero=include_zero,
        tolerance_pct=tolerance,
        required=False,
        note=note,
    )


def _path(title, series, error_key=None, note="") -> ChartSpec:
    return ChartSpec(
        form="path",
        title=title,
        series=tuple(series),
        unit="m",
        error_key=error_key,
        required=False,
        note=note,
    )


_MATCH_LANES = [_s(_ENABLED, "Enabled"), _s(_AUTONOMOUS, "Autonomous")]


BUNDLES: dict[str, tuple[ChartSpec, ...]] = {
    "auto": (
        _timeline(
            "Match state and mechanism targets",
            _MATCH_LANES
            + [
                _s(_SHOOTER_STATE, "Shooter state"),
                _s(_RACK_TARGET, "Intake rack"),
                _s(_ROLLERS_TARGET, "Intake rollers"),
            ],
        ),
        _path(
            "Driven path vs PathPlanner target",
            [_s(_SWERVE_POSITION, "Driven"), _s(_PATH_TARGET, "Commanded")],
            error_key=_PATH_ERROR,
        ),
        _timeseries("Fuel remaining on field", [_s(_FUEL_COUNT, "Fuel count")]),
        _timeseries(
            "Flywheel velocity",
            [_s(_FLYWHEEL_ACTUAL, "Actual"), _s(_FLYWHEEL_TARGET, "Target")],
            unit="rot/s",
            tolerance=0.05,
        ),
    ),
    "shooter": (
        _timeseries(
            "Flywheel velocity vs setpoint",
            [_s(_FLYWHEEL_ACTUAL, "Actual"), _s(_FLYWHEEL_TARGET, "Target")],
            unit="rot/s",
            tolerance=0.05,
            note="Shaded band is 5% either side of the setpoint.",
        ),
        _timeline(
            "Shooter state and readiness",
            [_s(_SHOOTER_STATE, "Shooter state"), _s(_UP_TO_SPEED, "Up to speed")],
        ),
        _timeseries(
            "Hood angle: commanded vs predicted",
            [_s(_HOOD_TARGET, "Hood target"), _s(_PREDICTED_ANGLE, "Predicted angle")],
            unit="rad",
            include_zero=False,
        ),
        _timeseries("Accelerator target velocity", [_s(_ACCELERATOR, "Accelerator")], unit="rot/s"),
    ),
    "intake": (
        _timeline(
            "Intake state",
            [
                _s(_RACK_TARGET, "Rack target"),
                _s(_ROLLERS_TARGET, "Rollers target"),
                _s(_RACK_REACHED, "Rack reached target"),
            ],
        ),
        _timeseries("Rack position target", [_s(_RACK_ROTATIONS, "Rack rotations")], unit="rot"),
        _timeseries("Rollers target velocity", [_s(_ROLLERS_VELOCITY, "Rollers")], unit="rot/s"),
        _timeseries(
            "Fuel remaining on field",
            [_s(_FUEL_COUNT, "Fuel count")],
            note="A flat line through an intake window means nothing was picked up.",
        ),
    ),
    "drive": (
        _path(
            "Driven path vs PathPlanner target",
            [_s(_SWERVE_POSITION, "Driven"), _s(_PATH_TARGET, "Commanded")],
            error_key=_PATH_ERROR,
        ),
        _timeseries(
            "Distance from path setpoint",
            [_s(_PATH_ERROR, "Path error")],
            unit="m",
            note="Sustained values above 0.25m are the anomaly threshold the text report flags.",
        ),
        _timeline("Match state", _MATCH_LANES),
    ),
    "vision": (
        _timeseries("Average tag distance", [_s(_VISION_DISTANCE, "Camera 0")], unit="m"),
        _timeseries("Multi-tag observations", [_s(_VISION_TAGS, "Tags")]),
        _timeline("Match state", _MATCH_LANES),
    ),
}

BUNDLE_NAMES = frozenset(BUNDLES)


def bundle_charts(name: str) -> tuple[ChartSpec, ...]:
    """Charts for one bundle, each already marked optional."""
    return tuple(replace(chart, required=False) for chart in BUNDLES[name])


def bundle_keys(names) -> set[str]:
    """Every key the named bundles could use, for a single read of the log."""
    keys: set[str] = set()
    for name in names:
        for chart in BUNDLES.get(name, ()):
            keys.update(series.key for series in chart.series)
            if chart.error_key:
                keys.add(chart.error_key)
    return keys
