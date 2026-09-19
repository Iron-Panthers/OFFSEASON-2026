"""
The findings-JSON contract.

log_charts.py is driven by a JSON document describing what an analysis concluded
and which charts support it. This module turns that document into validated
dataclasses, or refuses it with a message naming the offending path.

Refusing loudly is the whole point. A chart that silently renders empty still
*looks* like evidence, which is worse than a crash: it invites someone to trust
a conclusion nothing is backing. So a misspelled key, an unknown chart form or a
backwards time window all stop the run.

The document also has a fixed shape, enforced here rather than left to habit.
Every finding is conclusion, then evidence, then the reading of that evidence:

    finding.detail    the conclusion -- what is wrong, stated in full
    finding.charts    the graphs that show it
    chart.reads_as    why that particular graph supports the conclusion

All three are required on a finding, because a chart with no stated reading is
the failure mode this format exists to prevent: a picture dropped next to a
sentence, leaving the reader to guess which line in it was the point.

Prose fields (``verdict``, ``detail``, ``reads_as``, stat and chart ``note``)
accept either a string -- one paragraph -- or an array. In an array the first
entry is the lead paragraph and every entry after it is a bullet, so an
explanation that needs to enumerate can do so without being crammed into one
run-on sentence.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

CHART_FORMS = ("timeseries", "timeline", "path")
SEVERITIES = ("info", "good", "warning", "serious", "critical")
DEFAULT_SEVERITY = "info"


class SpecError(ValueError):
    """Malformed findings JSON. The message names the JSON path that is wrong."""


@dataclass(frozen=True)
class Prose:
    """
    A block of explanation: one lead paragraph, then optional bullets.

    Findings prose is meant to be read by someone who was not in the pit when
    the log was taken, so it is allowed -- encouraged -- to be long. The bullet
    form exists so that "long" does not have to mean "one sentence with four
    semicolons in it".
    """

    lead: str = ""
    bullets: tuple[str, ...] = field(default_factory=tuple)

    def __bool__(self) -> bool:
        return bool(self.lead or self.bullets)


@dataclass(frozen=True)
class SeriesSpec:
    key: str
    label: str


@dataclass(frozen=True)
class ChartSpec:
    """
    One chart.

    ``required`` separates the two sources of charts. A chart named in a findings
    file is required: the analyst asked for that key by name, so its absence is a
    mistake worth stopping for. A chart from a preset bundle is not: bundles are
    generic across every log, and a vision log legitimately has no flywheel, so
    missing series are dropped instead.

    The same split governs ``reads_as``, the sentence-or-more saying what in this
    chart backs the finding. A chart the analyst chose has to carry one; a
    bundle chart is context nobody claimed anything about, so it does not.
    """

    form: str
    title: str
    series: tuple[SeriesSpec, ...]
    unit: str = ""
    window: tuple[float, float] | None = None
    tolerance_pct: float | None = None
    error_key: str | None = None
    include_zero: bool = False
    required: bool = True
    note: str = ""
    reads_as: Prose = field(default_factory=Prose)


@dataclass(frozen=True)
class StatSpec:
    """A headline scalar, for when a chart is the wrong form for one number."""

    label: str
    value: str
    unit: str = ""
    severity: str = DEFAULT_SEVERITY
    note: str = ""


@dataclass(frozen=True)
class Finding:
    title: str
    detail: Prose = field(default_factory=Prose)
    severity: str = DEFAULT_SEVERITY
    window: tuple[float, float] | None = None
    charts: tuple[ChartSpec, ...] = field(default_factory=tuple)


@dataclass(frozen=True)
class Report:
    log: str
    question: str = ""
    verdict: Prose = field(default_factory=Prose)
    confidence: str = ""
    label: str | None = None
    bundles: tuple[str, ...] = field(default_factory=tuple)
    stats: tuple[StatSpec, ...] = field(default_factory=tuple)
    findings: tuple[Finding, ...] = field(default_factory=tuple)

    def keys_used(self) -> set[str]:
        """Every log key the report needs, so the log is read exactly once."""
        keys: set[str] = set()
        for finding in self.findings:
            for chart in finding.charts:
                keys.update(series.key for series in chart.series)
                if chart.error_key:
                    keys.add(chart.error_key)
        return keys


# --- parsing helpers -------------------------------------------------------
# Each takes the JSON path so the error message can point at the real location
# rather than saying "invalid input" and leaving the caller to guess.


def _as_dict(value, path: str) -> dict:
    if not isinstance(value, dict):
        raise SpecError(f"{path}: expected an object, got {type(value).__name__}")
    return value


def _as_list(value, path: str) -> list:
    if not isinstance(value, list):
        raise SpecError(f"{path}: expected an array, got {type(value).__name__}")
    return value


def _string(source: dict, key: str, path: str, default: str | None = None) -> str:
    if key not in source or source[key] is None:
        if default is None:
            raise SpecError(f"{path}.{key}: required")
        return default
    value = source[key]
    if not isinstance(value, str):
        raise SpecError(f"{path}.{key}: expected a string, got {type(value).__name__}")
    return value


def _prose(value, path: str, *, required: bool = False, because: str = "") -> Prose:
    """
    Parse a prose field: a string paragraph, or an array of lead + bullets.

    ``because`` is appended to the "required" error so the message says what the
    missing text is for rather than only that something is missing -- the whole
    reason these fields are mandatory is that they are easy to skip.
    """
    if value is None or value == "" or value == []:
        if required:
            raise SpecError(f"{path}: required{because}")
        return Prose()

    if isinstance(value, str):
        return Prose(lead=value.strip())

    items = _as_list(value, path)
    for index, item in enumerate(items):
        if not isinstance(item, str):
            raise SpecError(
                f"{path}[{index}]: expected a string, got {type(item).__name__}"
            )
    cleaned = [item.strip() for item in items if item.strip()]
    if not cleaned:
        if required:
            raise SpecError(f"{path}: required{because}")
        return Prose()
    # First entry leads, the rest are bullets. A one-entry array is just a
    # paragraph, so a findings file never has to choose the form up front.
    return Prose(lead=cleaned[0], bullets=tuple(cleaned[1:]))


def _severity(source: dict, path: str) -> str:
    value = source.get("severity", DEFAULT_SEVERITY)
    if value not in SEVERITIES:
        raise SpecError(
            f"{path}.severity: {value!r} is not one of {', '.join(SEVERITIES)}"
        )
    return value


def _window(value, path: str) -> tuple[float, float] | None:
    if value is None:
        return None
    items = _as_list(value, path)
    if len(items) != 2:
        raise SpecError(f"{path}: expected [start, end], got {len(items)} item(s)")
    for index, item in enumerate(items):
        if not isinstance(item, (int, float)) or isinstance(item, bool):
            raise SpecError(f"{path}[{index}]: expected a number, got {item!r}")
    start, end = float(items[0]), float(items[1])
    if end <= start:
        raise SpecError(f"{path}: end ({end}) must be greater than start ({start})")
    return (start, end)


def _series(value, path: str) -> tuple[SeriesSpec, ...]:
    items = _as_list(value, path)
    out = []
    for index, item in enumerate(items):
        item_path = f"{path}[{index}]"
        entry = _as_dict(item, item_path)
        key = _string(entry, "key", item_path)
        # Defaulting the label to the key's leaf keeps short findings files
        # readable without making every chart legend say the full log path.
        out.append(SeriesSpec(key=key, label=_string(entry, "label", item_path, key.rsplit("/", 1)[-1])))
    return tuple(out)


def _chart(value, path: str) -> ChartSpec:
    entry = _as_dict(value, path)
    form = _string(entry, "form", path)
    if form not in CHART_FORMS:
        raise SpecError(f"{path}.form: {form!r} is not one of {', '.join(CHART_FORMS)}")

    series = _series(entry.get("series"), f"{path}.series")
    if not series:
        raise SpecError(f"{path}.series: at least one series is required")
    if form == "path" and len(series) > 2:
        raise SpecError(
            f"{path}.series: a path chart takes 1 series (actual) or 2 "
            f"(actual, target), got {len(series)}"
        )

    tolerance = entry.get("tolerance_pct")
    if tolerance is not None:
        if not isinstance(tolerance, (int, float)) or isinstance(tolerance, bool):
            raise SpecError(f"{path}.tolerance_pct: expected a number, got {tolerance!r}")
        if not 0 < tolerance < 1:
            raise SpecError(
                f"{path}.tolerance_pct: expected a fraction between 0 and 1 "
                f"(0.05 for 5%), got {tolerance}"
            )
        if form != "timeseries":
            raise SpecError(f"{path}.tolerance_pct: only meaningful on a timeseries chart")
        if len(series) < 2:
            raise SpecError(
                f"{path}.tolerance_pct: needs a second series to band around "
                f"(the setpoint), got {len(series)}"
            )

    error_key = entry.get("error_key")
    if error_key is not None:
        if not isinstance(error_key, str):
            raise SpecError(f"{path}.error_key: expected a string, got {error_key!r}")
        if form != "path":
            raise SpecError(f"{path}.error_key: only meaningful on a path chart")

    return ChartSpec(
        form=form,
        title=_string(entry, "title", path, series[0].label),
        series=series,
        unit=_string(entry, "unit", path, ""),
        window=_window(entry.get("window"), f"{path}.window"),
        tolerance_pct=float(tolerance) if tolerance is not None else None,
        error_key=error_key,
        include_zero=bool(entry.get("include_zero", False)),
        required=True,
        note=_string(entry, "note", path, ""),
        reads_as=_prose(
            entry.get("reads_as"),
            f"{path}.reads_as",
            required=True,
            because=(
                " -- say what in this chart supports the finding: the signal, "
                "the moment, and the number a reader should come away with"
            ),
        ),
    )


def _stat(value, path: str) -> StatSpec:
    entry = _as_dict(value, path)
    raw = entry.get("value")
    if raw is None:
        raise SpecError(f"{path}.value: required")
    # Numbers are accepted and stringified so a findings file can say 3 or "3".
    text = raw if isinstance(raw, str) else f"{raw:,.2f}".rstrip("0").rstrip(".")
    return StatSpec(
        label=_string(entry, "label", path),
        value=text,
        unit=_string(entry, "unit", path, ""),
        severity=_severity(entry, path),
        note=_string(entry, "note", path, ""),
    )


def _finding(value, path: str) -> Finding:
    entry = _as_dict(value, path)
    charts = tuple(
        _chart(item, f"{path}.charts[{index}]")
        for index, item in enumerate(_as_list(entry.get("charts", []), f"{path}.charts"))
    )
    window = _window(entry.get("window"), f"{path}.window")
    # A finding window is the default window for its own charts: the analyst
    # already said which slice of the match this finding is about, and repeating
    # it on every chart is noise that drifts out of sync.
    if window is not None:
        charts = tuple(
            chart if chart.window is not None else replace(chart, window=window)
            for chart in charts
        )
    return Finding(
        title=_string(entry, "title", path),
        detail=_prose(
            entry.get("detail"),
            f"{path}.detail",
            required=True,
            because=" -- the conclusion the charts below are evidence for",
        ),
        severity=_severity(entry, path),
        window=window,
        charts=charts,
    )


def parse_report(raw, known_bundles=frozenset()) -> Report:
    """
    Validate a decoded findings document into a Report.

    ``known_bundles`` is passed in rather than imported so this module stays free
    of the bundle catalogue, which depends on it.
    """
    document = _as_dict(raw, "<root>")

    bundles = tuple(
        _as_list(document.get("bundles", []), "bundles")
    )
    for index, name in enumerate(bundles):
        if not isinstance(name, str):
            raise SpecError(f"bundles[{index}]: expected a string, got {name!r}")
        if known_bundles and name not in known_bundles:
            raise SpecError(
                f"bundles[{index}]: unknown bundle {name!r} "
                f"(known: {', '.join(sorted(known_bundles))})"
            )

    report = Report(
        log=_string(document, "log", "<root>"),
        question=_string(document, "question", "<root>", ""),
        verdict=_prose(document.get("verdict"), "<root>.verdict"),
        confidence=_string(document, "confidence", "<root>", ""),
        label=document.get("label") or None,
        bundles=tuple(bundles),
        stats=tuple(
            _stat(item, f"stats[{index}]")
            for index, item in enumerate(_as_list(document.get("stats", []), "stats"))
        ),
        findings=tuple(
            _finding(item, f"findings[{index}]")
            for index, item in enumerate(_as_list(document.get("findings", []), "findings"))
        ),
    )

    if report.label is not None and not isinstance(report.label, str):
        raise SpecError("<root>.label: expected a string")
    if not report.findings and not report.bundles:
        raise SpecError(
            "<root>: nothing to render -- give at least one entry in "
            "'findings' or one preset name in 'bundles'"
        )
    return report
