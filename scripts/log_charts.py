#!/usr/bin/env python3
"""
Render an analysis findings document into a single self-contained HTML page.

This is the charting half of log analysis. The text tools in wpilog_to_csv.py are
untouched and still do all the reading and diagnosing; this script takes what
that investigation concluded -- written down as a findings JSON -- and draws it.

    python scripts/log_charts.py findings.json

Output lands in build/artifacts/<run-id>/index.html, where run-id comes from the
log's own match metadata (2026cada-qm12) or, for a sim log that has none, its
filename and timestamp (sim-20260918-143022). Re-running for the same log
replaces that directory: the id identifies the match, not the attempt.

The page needs no network and no server -- open the file, or put it on a USB
stick and open it in the pit.

See .claude/commands/log-analysis.md for the findings-JSON schema, and
scripts/charting/spec.py for the authoritative validation rules.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import replace
from datetime import datetime
from difflib import get_close_matches
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from charting import bundles as bundle_catalogue  # noqa: E402
from charting.marks import RENDERERS  # noqa: E402
from charting.page import render_page  # noqa: E402
from charting.palette import SlotRegistry  # noqa: E402
from charting.spec import ChartSpec, Prose, SeriesSpec, SpecError, parse_report  # noqa: E402
from wpilog_to_csv import read_log  # noqa: E402

DEFAULT_OUT_ROOT = Path("build/artifacts")

# WPILib DriverStation match types, as logged by AdvantageKit.
MATCH_TYPES = {0: "", 1: "p", 2: "qm", 3: "e"}

METADATA_KEYS = (
    "DriverStation/EventName",
    "DriverStation/MatchType",
    "DriverStation/MatchNumber",
    "DriverStation/ReplayNumber",
    "DriverStation/Enabled",
)


def slugify(text: str) -> str:
    """Lowercase, hyphen-separated, filesystem-safe."""
    cleaned = re.sub(r"[^a-z0-9]+", "-", str(text).lower()).strip("-")
    return cleaned or "log"


def _last_value(data, key):
    series = data.get(key) or []
    return series[-1][1] if series else None


def resolve_run_id(data, log_path: Path, label: str | None = None) -> str:
    """
    Name the artifact directory.

    Real match logs carry event name, match type and match number, which give a
    directory a human can find later: 2026cada-qm12. Sim logs carry none of
    that, so they fall back to the filename plus the log's own mtime -- which
    still sorts chronologically and still points at one specific run.
    """
    if label:
        return slugify(label)

    event = _last_value(data, "DriverStation/EventName")
    match_type = _last_value(data, "DriverStation/MatchType")
    match_number = _last_value(data, "DriverStation/MatchNumber")

    type_code = MATCH_TYPES.get(int(match_type), "") if isinstance(match_type, int) else ""
    has_match = bool(type_code) and isinstance(match_number, int) and match_number > 0
    if event and str(event).strip() and has_match:
        return slugify(f"{event}-{type_code}{match_number}")
    if has_match:
        return slugify(f"{type_code}{match_number}")

    stamp = datetime.fromtimestamp(log_path.stat().st_mtime).strftime("%Y%m%d-%H%M%S")
    return slugify(f"{log_path.stem}-{stamp}")


def expand_charts(charts):
    """
    Turn spec-level conveniences into the charts actually drawn.

    Only one expansion so far: a path chart with an ``error_key`` becomes the
    path plus a separate error timeseries beneath it. That is small multiples
    rather than a second y-scale on the path plot -- encoding metres-of-error
    into a chart already using both axes for metres-of-position is exactly the
    dual-axis mistake.

    The derived chart gets a derived reading. It is not the analyst's claim --
    it just says what the second chart is measuring -- but the page's rule is
    that no chart appears without one, and generating it here is better than
    making every path finding hand-write the same sentence.
    """
    out = []
    for chart in charts:
        out.append(chart)
        if chart.form == "path" and chart.error_key:
            out.append(
                ChartSpec(
                    form="timeseries",
                    title=f"{chart.title} — error",
                    series=(SeriesSpec(key=chart.error_key, label="Path error"),),
                    unit="m",
                    window=chart.window,
                    include_zero=True,
                    required=chart.required,
                    reads_as=Prose(
                        lead=(
                            "The path above shows where the robot went; this shows how "
                            "far that was from where it was told to be at each instant. "
                            "Read the peaks against the turns in the path."
                        )
                    ),
                )
            )
    return out


def check_keys(charts, data, log_path: Path):
    """
    Fail on a required chart whose key is not in the log.

    A typo in a 60-character key name would otherwise render as an empty chart,
    which still looks like evidence. Missing keys on bundle charts are fine --
    bundles are generic and a vision log has no flywheel.
    """
    available = set(data)
    problems = []
    for chart in charts:
        if not chart.required:
            continue
        wanted = [series.key for series in chart.series]
        if chart.error_key:
            wanted.append(chart.error_key)
        for key in wanted:
            if key not in available:
                hint = get_close_matches(key, available, n=1, cutoff=0.6)
                suggestion = f"  closest key in this log: {hint[0]!r}" if hint else ""
                problems.append(f"  {chart.title!r} wants {key!r}, which is not in the log.{suggestion}")
    if problems:
        raise SpecError(
            "keys missing from {}:\n{}\n\nList what the log does contain with:\n"
            "  python scripts/wpilog_to_csv.py {} --summary".format(
                log_path.name, "\n".join(problems), log_path
            )
        )


def render_charts(charts, data, t_end, registry):
    rendered = []
    for chart in charts:
        renderer = RENDERERS[chart.form]
        if chart.form == "timeline":
            rendered.append(renderer(chart, data, t_end, registry))
        else:
            rendered.append(renderer(chart, data, t_end))
    return rendered


def resolve_log_path(raw: str, findings_path: Path) -> Path:
    """Accept a log path relative to the cwd or to the findings file."""
    direct = Path(raw)
    if direct.exists():
        return direct
    beside = findings_path.parent / raw
    if beside.exists():
        return beside
    raise SpecError(
        f"<root>.log: {raw!r} not found (looked in the working directory and "
        f"beside {findings_path.name})"
    )


def build(findings_path: Path, out_dir: Path | None, out_root: Path) -> Path:
    try:
        document = json.loads(findings_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise SpecError(f"{findings_path}: not valid JSON -- {error}") from error

    report = parse_report(document, known_bundles=bundle_catalogue.BUNDLE_NAMES)
    log_path = resolve_log_path(report.log, findings_path)

    finding_charts = [expand_charts(finding.charts) for finding in report.findings]
    baseline_charts = expand_charts(
        [chart for name in report.bundles for chart in bundle_catalogue.bundle_charts(name)]
    )

    # One read for every key any chart could want, plus the metadata that names
    # the output directory. Reading the log once keeps a 150s match cheap.
    wanted = set(METADATA_KEYS)
    for group in finding_charts + [baseline_charts]:
        for chart in group:
            wanted.update(series.key for series in chart.series)
            if chart.error_key:
                wanted.add(chart.error_key)
    data = read_log(str(log_path), keys=sorted(wanted))

    for group in finding_charts:
        check_keys(group, data, log_path)

    t_end = max((series[-1][0] for series in data.values() if series), default=None)
    t_start = min((series[0][0] for series in data.values() if series), default=None)

    # One registry for the whole page: once SHOOT is a given colour it keeps it
    # in every lane and every chart, so a reader who learns a colour keeps it.
    registry = SlotRegistry()
    rendered_findings = [
        (finding, render_charts(charts, data, t_end, registry))
        for finding, charts in zip(report.findings, finding_charts)
    ]
    rendered_baseline = render_charts(baseline_charts, data, t_end, registry)

    run_id = resolve_run_id(data, log_path, report.label)
    target = out_dir if out_dir is not None else out_root / run_id
    target.mkdir(parents=True, exist_ok=True)

    duration = f"{t_end - t_start:.1f}s" if t_start is not None and t_end is not None else "unknown duration"
    subtitle_bits = [log_path.name, duration]
    if report.bundles:
        subtitle_bits.append(f"bundles: {', '.join(report.bundles)}")
    subtitle_bits.append(f"{sum(len(charts) for _, charts in rendered_findings) + len(rendered_baseline)} charts")

    html = render_page(
        title=run_id,
        subtitle_bits=subtitle_bits,
        report=report,
        findings=rendered_findings,
        baseline=rendered_baseline,
        generated=datetime.now().strftime("%Y-%m-%d %H:%M"),
    )
    output = target / "index.html"
    output.write_text(html, encoding="utf-8")

    # Keep the findings beside the page: the page is the argument, this is the
    # input that produced it, and six months later you want both.
    (target / "findings.json").write_text(
        json.dumps(document, indent=2) + "\n", encoding="utf-8"
    )
    return output


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render a log-analysis findings JSON into a self-contained HTML page.",
        epilog="Schema: see .claude/commands/log-analysis.md",
    )
    parser.add_argument("findings", help="Path to the findings JSON document")
    parser.add_argument(
        "-o",
        "--out",
        help="Write to this exact directory instead of <out-root>/<run-id>",
    )
    parser.add_argument(
        "--out-root",
        default=str(DEFAULT_OUT_ROOT),
        help=f"Parent directory for the run folder (default: {DEFAULT_OUT_ROOT})",
    )
    args = parser.parse_args()

    findings_path = Path(args.findings)
    if not findings_path.exists():
        print(f"ERROR: findings file not found: {findings_path}", file=sys.stderr)
        return 1

    try:
        output = build(
            findings_path,
            Path(args.out) if args.out else None,
            Path(args.out_root),
        )
    except SpecError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1

    size_kb = output.stat().st_size / 1024
    print(f"{output}  ({size_kb:,.0f} KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
