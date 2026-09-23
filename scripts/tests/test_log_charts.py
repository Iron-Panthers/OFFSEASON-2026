import json
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from charting.scales import (
    Scale,
    clip_series,
    coalesce_blocks,
    decimate,
    format_number,
    nice_axis,
    nice_step,
    pad_domain,
    sample_series,
)


# --- scales ---------------------------------------------------------------


def test_nice_step_picks_allowed_mantissas():
    assert nice_step(60.0, 6) == 10.0
    assert nice_step(12.0, 6) == 2.0
    assert nice_step(0.6, 6) == 0.1


def test_nice_step_rejects_degenerate_span():
    assert nice_step(0.0) == 1.0
    assert nice_step(float("nan")) == 1.0
    assert nice_step(-5.0) == 1.0


def test_nice_axis_snaps_outward_to_tick_boundaries():
    lo, hi, ticks = nice_axis(46.2, 50.0)
    assert lo <= 46.2 and hi >= 50.0
    assert ticks[0] == lo and ticks[-1] == hi


def test_nice_axis_ticks_are_evenly_spaced_without_float_drift():
    _, _, ticks = nice_axis(0.0, 3.0)
    steps = {round(b - a, 10) for a, b in zip(ticks, ticks[1:])}
    assert len(steps) == 1


def test_nice_axis_handles_a_constant_signal():
    # A setpoint that never moves is normal in robot logs and must not divide by zero.
    lo, hi, ticks = nice_axis(50.0, 50.0)
    assert lo < 50.0 < hi
    assert len(ticks) >= 2


def test_nice_axis_handles_constant_zero():
    lo, hi, _ = nice_axis(0.0, 0.0)
    assert lo < 0.0 < hi


def test_nice_axis_include_zero_extends_the_baseline():
    lo, _, _ = nice_axis(46.0, 50.0, include_zero=True)
    assert lo == 0.0


def test_pad_domain_orders_reversed_input():
    assert pad_domain(10.0, 2.0) == (2.0, 10.0)


def test_scale_maps_domain_onto_pixels():
    scale = Scale(0.0, 10.0, 100.0, 300.0)
    assert scale.px(0.0) == 100.0
    assert scale.px(10.0) == 300.0
    assert scale.px(5.0) == 200.0


def test_scale_supports_an_inverted_axis():
    # y axes pass bottom as p0 and top as p1; that is not a special case.
    scale = Scale(0.0, 10.0, 300.0, 100.0)
    assert scale.px(10.0) == 100.0


def test_scale_clamps_outliers_into_the_plot():
    scale = Scale(0.0, 10.0, 300.0, 100.0)
    assert scale.clamp_px(99.0) == 100.0
    assert scale.clamp_px(-99.0) == 300.0


def test_scale_with_zero_width_domain_does_not_divide_by_zero():
    assert Scale(5.0, 5.0, 0.0, 100.0).px(5.0) == 0.0


# --- number formatting ----------------------------------------------------


def test_format_number_follows_the_tick_step():
    assert format_number(1.0, 0.5) == "1.0"
    assert format_number(1000.0, 500.0) == "1,000"


def test_format_number_gives_a_two_point_five_step_an_extra_place():
    # Without the correction these ticks round to 0 / 2 / 5 / 8.
    assert [format_number(v, 2.5) for v in (0.0, 2.5, 5.0)] == ["0.0", "2.5", "5.0"]


def test_format_number_rejects_non_numbers():
    assert format_number(None) == "-"
    assert format_number(float("nan")) == "-"
    assert format_number(True) == "-"


# --- series reshaping -----------------------------------------------------


def test_coalesce_blocks_merges_repeated_values():
    blocks = coalesce_blocks([(1.0, "A"), (2.0, "A"), (3.0, "B")], 5.0)
    assert blocks == [(1.0, 3.0, "A"), (3.0, 5.0, "B")]


def test_coalesce_blocks_extends_the_final_block_to_the_log_end():
    # The last state is often the one that matters; without t_end it renders as
    # a hairline at the right edge.
    assert coalesce_blocks([(1.0, "SHOOT")], 40.0) == [(1.0, 40.0, "SHOOT")]


def test_coalesce_blocks_gives_a_lone_sample_nonzero_width():
    start, end, _ = coalesce_blocks([(1.0, "STOW")], None)[0]
    assert end > start


def test_coalesce_blocks_of_empty_series():
    assert coalesce_blocks([], 10.0) == []


def test_coalesce_blocks_compares_booleans_by_rendered_text():
    assert coalesce_blocks([(0.0, True), (1.0, True), (2.0, False)], 3.0) == [
        (0.0, 2.0, "True"),
        (2.0, 3.0, "False"),
    ]


def test_decimate_preserves_a_spike_that_stride_sampling_would_drop():
    flat = [(i * 0.02, 5.0) for i in range(500)]
    series = flat + [(10.0, 99.0)] + [(10.02 + i * 0.02, 5.0) for i in range(500)]
    reduced = decimate(series, 40)
    assert max(value for _, value in reduced) == 99.0
    assert len(reduced) <= 40


def test_decimate_leaves_short_series_alone():
    series = [(0.0, 1.0), (1.0, 2.0)]
    assert decimate(series, 1200) == series


def test_decimate_output_stays_in_chronological_order():
    series = [(i * 0.02, (i % 7) * 1.5) for i in range(4000)]
    stamps = [ts for ts, _ in decimate(series, 100)]
    assert stamps == sorted(stamps)


def test_clip_series_keeps_the_sample_straddling_the_window_start():
    # Without it the line starts at the window edge instead of where the signal
    # actually was, which reads as a jump that never happened.
    clipped = clip_series([(0.0, 1.0), (5.0, 2.0), (10.0, 3.0)], 6.0, 11.0)
    assert clipped[0] == (5.0, 2.0)


def test_sample_series_thins_to_the_interval_and_keeps_the_last_sample():
    series = [(i * 0.1, float(i)) for i in range(50)]
    sampled = sample_series(series, 1.0)
    assert len(sampled) < len(series)
    assert sampled[-1] == series[-1]


# --- spec validation ------------------------------------------------------

from charting.bundles import BUNDLES, BUNDLE_NAMES, bundle_charts, bundle_keys  # noqa: E402
from charting.spec import SpecError, parse_report  # noqa: E402


# A finding owes a conclusion and every chart in it owes a reading, so the
# minimal *valid* document is not the minimal document: these two fields are
# required and the fixture carries them.
def _report(**overrides):
    document = {
        "log": "x.wpilog",
        "findings": [
            {
                "title": "t",
                "detail": "d",
                "charts": [
                    {
                        "form": "timeseries",
                        "title": "c",
                        "series": [{"key": "A"}, {"key": "B"}],
                        "reads_as": "r",
                    }
                ],
            }
        ],
    }
    document.update(overrides)
    return document


def test_parse_report_accepts_a_minimal_document():
    report = parse_report(_report())
    assert report.log == "x.wpilog"
    assert report.findings[0].charts[0].series[0].key == "A"


def test_series_label_defaults_to_the_key_leaf():
    document = _report()
    document["findings"][0]["charts"][0]["series"][0]["key"] = "RealOutputs/Shooter/Target State"
    report = parse_report(document)
    assert report.findings[0].charts[0].series[0].label == "Target State"


def test_unknown_chart_form_is_rejected_by_path():
    document = _report()
    document["findings"][0]["charts"][0]["form"] = "sparkline"
    with pytest.raises(SpecError, match=r"findings\[0\]\.charts\[0\]\.form"):
        parse_report(document)


def test_backwards_window_is_rejected():
    document = _report()
    document["findings"][0]["window"] = [20.0, 5.0]
    with pytest.raises(SpecError, match="must be greater than start"):
        parse_report(document)


def test_finding_window_cascades_to_its_charts():
    document = _report()
    document["findings"][0]["window"] = [11.0, 14.0]
    chart = parse_report(document).findings[0].charts[0]
    assert chart.window == (11.0, 14.0)


def test_an_explicit_chart_window_beats_the_finding_window():
    document = _report()
    document["findings"][0]["window"] = [11.0, 14.0]
    document["findings"][0]["charts"][0]["window"] = [1.0, 2.0]
    assert parse_report(document).findings[0].charts[0].window == (1.0, 2.0)


def test_tolerance_must_be_a_fraction_not_a_percentage():
    document = _report()
    document["findings"][0]["charts"][0]["tolerance_pct"] = 5
    with pytest.raises(SpecError, match="fraction between 0 and 1"):
        parse_report(document)


def test_tolerance_needs_a_setpoint_series_to_band_around():
    document = _report()
    document["findings"][0]["charts"][0]["series"] = [{"key": "A"}]
    document["findings"][0]["charts"][0]["tolerance_pct"] = 0.05
    with pytest.raises(SpecError, match="needs a second series"):
        parse_report(document)


def test_path_chart_rejects_more_than_two_series():
    document = _report()
    document["findings"][0]["charts"][0]["form"] = "path"
    document["findings"][0]["charts"][0]["series"] = [{"key": "A"}, {"key": "B"}, {"key": "C"}]
    with pytest.raises(SpecError, match="path chart takes"):
        parse_report(document)


def test_unknown_bundle_is_rejected_and_lists_the_known_ones():
    with pytest.raises(SpecError, match="unknown bundle"):
        parse_report(_report(bundles=["hopper"]), known_bundles=BUNDLE_NAMES)


def test_unknown_severity_is_rejected():
    document = _report()
    document["findings"][0]["severity"] = "catastrophic"
    with pytest.raises(SpecError, match="severity"):
        parse_report(document)


def test_a_document_with_nothing_to_render_is_rejected():
    with pytest.raises(SpecError, match="nothing to render"):
        parse_report({"log": "x.wpilog"})


def test_missing_log_is_rejected():
    with pytest.raises(SpecError, match="log"):
        parse_report({"findings": []})


def test_keys_used_collects_every_key_including_error_keys():
    document = _report()
    document["findings"][0]["charts"].append(
        {"form": "path", "title": "p", "series": [{"key": "P"}], "error_key": "E", "reads_as": "r"}
    )
    assert parse_report(document).keys_used() == {"A", "B", "P", "E"}


# --- the conclusion / evidence / reading contract -------------------------


def test_a_finding_without_a_conclusion_is_rejected():
    document = _report()
    del document["findings"][0]["detail"]
    with pytest.raises(SpecError, match=r"findings\[0\]\.detail: required"):
        parse_report(document)


def test_a_chart_without_a_reading_is_rejected():
    # The failure this format exists to prevent: a picture dropped beside a
    # sentence, with nothing saying which line in it was the point.
    document = _report()
    del document["findings"][0]["charts"][0]["reads_as"]
    with pytest.raises(SpecError, match=r"charts\[0\]\.reads_as: required"):
        parse_report(document)


def test_the_missing_reading_error_says_what_to_write():
    document = _report()
    document["findings"][0]["charts"][0]["reads_as"] = ""
    with pytest.raises(SpecError, match="the signal, the moment, and the number"):
        parse_report(document)


def test_a_bundle_chart_needs_no_reading():
    # Bundles are context nobody made a claim about, so nothing has to be said
    # about them -- unlike a chart the analyst chose by name.
    for chart in bundle_charts("shooter"):
        assert not chart.reads_as


def test_prose_from_a_string_is_one_paragraph():
    document = _report()
    document["findings"][0]["detail"] = "One sentence."
    detail = parse_report(document).findings[0].detail
    assert detail.lead == "One sentence."
    assert detail.bullets == ()


def test_prose_from_an_array_leads_then_bullets():
    document = _report()
    document["findings"][0]["detail"] = ["Lead.", "First point.", "Second point."]
    detail = parse_report(document).findings[0].detail
    assert detail.lead == "Lead."
    assert detail.bullets == ("First point.", "Second point.")


def test_a_one_entry_array_is_just_a_paragraph():
    document = _report()
    document["findings"][0]["detail"] = ["Only this."]
    assert parse_report(document).findings[0].detail.bullets == ()


def test_prose_rejects_a_non_string_entry_by_index():
    document = _report()
    document["findings"][0]["detail"] = ["Lead.", 12]
    with pytest.raises(SpecError, match=r"findings\[0\]\.detail\[1\]"):
        parse_report(document)


def test_a_verdict_may_be_bulleted_and_stays_optional():
    assert not parse_report(_report()).verdict
    verdict = parse_report(_report(verdict=["Lead.", "A.", "B."])).verdict
    assert verdict.lead == "Lead." and verdict.bullets == ("A.", "B.")


# --- bundles --------------------------------------------------------------


def test_every_bundle_is_renderable_and_optional():
    from charting.marks import RENDERERS

    for name, charts in BUNDLES.items():
        assert charts, f"{name} bundle is empty"
        for chart in charts:
            assert chart.form in RENDERERS
            assert chart.required is False, f"{name}/{chart.title} must tolerate missing keys"


def test_bundle_keys_are_collected_for_a_single_log_read():
    keys = bundle_keys(["shooter"])
    assert "RealOutputs/Shooter/Shooter Flywheels/Current Velocity" in keys


def test_bundle_keys_ignores_unknown_names():
    assert bundle_keys(["nope"]) == set()


# --- run id ---------------------------------------------------------------

from log_charts import expand_charts, resolve_run_id, slugify  # noqa: E402


def test_run_id_from_match_metadata():
    data = {
        "DriverStation/EventName": [(1.0, "2026cada")],
        "DriverStation/MatchType": [(1.0, 2)],
        "DriverStation/MatchNumber": [(1.0, 12)],
    }
    assert resolve_run_id(data, _FakePath("x.wpilog")) == "2026cada-qm12"


def test_run_id_uses_the_practice_and_elimination_codes():
    def run(match_type):
        data = {
            "DriverStation/EventName": [(1.0, "evt")],
            "DriverStation/MatchType": [(1.0, match_type)],
            "DriverStation/MatchNumber": [(1.0, 3)],
        }
        return resolve_run_id(data, _FakePath("x.wpilog"))

    assert run(1) == "evt-p3"
    assert run(3) == "evt-e3"


def test_run_id_falls_back_to_filename_and_timestamp_for_a_sim_log():
    # Sim logs carry no match metadata at all, which is the common case.
    run_id = resolve_run_id({}, _FakePath("sim-run.wpilog", mtime=1_600_000_000))
    assert run_id.startswith("sim-run-")
    assert len(run_id) > len("sim-run-")


def test_run_id_ignores_match_metadata_with_no_match_number():
    data = {
        "DriverStation/EventName": [(1.0, "evt")],
        "DriverStation/MatchType": [(1.0, 0)],
        "DriverStation/MatchNumber": [(1.0, 0)],
    }
    assert resolve_run_id(data, _FakePath("fallback.wpilog")).startswith("fallback-")


def test_an_explicit_label_wins():
    data = {
        "DriverStation/EventName": [(1.0, "evt")],
        "DriverStation/MatchType": [(1.0, 2)],
        "DriverStation/MatchNumber": [(1.0, 12)],
    }
    assert resolve_run_id(data, _FakePath("x.wpilog"), "Practice Field Test") == "practice-field-test"


def test_slugify_strips_characters_that_are_hostile_to_a_path():
    assert slugify("2026 CA/DA #12!") == "2026-ca-da-12"


def test_slugify_never_returns_an_empty_name():
    assert slugify("///") == "log"


class _FakePath:
    """Stands in for a Path so run-id tests need no file on disk."""

    def __init__(self, name, mtime=1_700_000_000):
        self.stem = name.rsplit(".", 1)[0]
        self._mtime = mtime

    def stat(self):
        class _Stat:
            st_mtime = self._mtime

        _Stat.st_mtime = self._mtime
        return _Stat()


# --- chart expansion ------------------------------------------------------


def test_path_error_key_expands_into_its_own_chart():
    # Encoding error onto the path plot would be a second scale on a chart
    # already using both axes for position -- the dual-axis mistake.
    document = _report()
    document["findings"][0]["charts"] = [
        {
            "form": "path",
            "title": "Driven path",
            "series": [{"key": "P"}],
            "error_key": "E",
            "reads_as": "r",
        }
    ]
    charts = expand_charts(parse_report(document).findings[0].charts)
    assert [chart.form for chart in charts] == ["path", "timeseries"]
    assert charts[1].series[0].key == "E"
    assert charts[1].include_zero is True


def test_expansion_leaves_a_path_without_an_error_key_alone():
    document = _report()
    document["findings"][0]["charts"] = [
        {"form": "path", "title": "p", "series": [{"key": "P"}], "reads_as": "r"}
    ]
    assert len(expand_charts(parse_report(document).findings[0].charts)) == 1


# --- rendering ------------------------------------------------------------

from charting.marks import Rendered, text_width, timeline, timeseries  # noqa: E402
from charting.palette import MAX_SLOTS, SlotRegistry, ink_on  # noqa: E402
from charting.spec import ChartSpec, SeriesSpec  # noqa: E402


def _chart(form, series, **kwargs):
    return ChartSpec(
        form=form,
        title="t",
        series=tuple(SeriesSpec(key=key, label=key) for key in series),
        **kwargs,
    )


def test_timeseries_renders_a_line_and_a_table():
    data = {"A": [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0)]}
    result = timeseries(_chart("timeseries", ["A"]), data, 2.0)
    assert "<polyline" in result.svg
    assert "<table" in result.table_html
    assert not result.empty_reason


def test_timeseries_with_no_numeric_samples_reports_why():
    result = timeseries(_chart("timeseries", ["A"]), {"A": [(0.0, "STOW")]}, 1.0)
    assert result.empty_reason
    assert result.svg == ""


def test_timeseries_omits_a_legend_for_a_single_series():
    data = {"A": [(0.0, 1.0), (1.0, 2.0)]}
    assert len(timeseries(_chart("timeseries", ["A"]), data, 1.0).legend) == 1


def test_timeseries_tolerance_band_renders_as_a_wash_not_a_line():
    data = {"A": [(0.0, 48.0), (1.0, 50.0)], "B": [(0.0, 50.0), (1.0, 50.0)]}
    chart = _chart("timeseries", ["A", "B"], tolerance_pct=0.05)
    result = timeseries(chart, data, 1.0)
    assert "vz-band" in result.svg


def test_timeseries_window_is_padded_so_it_reads_as_an_interval():
    data = {"A": [(i * 0.1, float(i)) for i in range(200)]}
    result = timeseries(_chart("timeseries", ["A"], window=(5.0, 10.0)), data, 20.0)
    assert "vz-window" in result.svg
    # The padding means the drawn range is wider than the window itself.
    assert "4." in result.subtitle or "4" in result.subtitle


def test_timeline_labels_a_wide_block_but_not_a_narrow_one():
    wide = timeline(_chart("timeline", ["A"]), {"A": [(0.0, "INTAKE")]}, 40.0)
    assert "INTAKE" in wide.svg
    # Same state, but squeezed into a slice far too narrow for the text.
    busy = {"A": [(i * 0.05, f"S{i}") for i in range(200)]}
    narrow = timeline(_chart("timeline", ["A"]), busy, 10.0)
    assert "vz-block-label" not in narrow.svg


def test_timeline_draws_false_as_an_empty_track():
    result = timeline(_chart("timeline", ["A"]), {"A": [(0.0, False), (5.0, True)]}, 10.0)
    assert "vz-block-off" in result.svg


def test_timeline_shares_one_colour_per_state_across_lanes():
    registry = SlotRegistry()
    data = {"A": [(0.0, "SHOOT")], "B": [(0.0, "STOW"), (1.0, "SHOOT")]}
    chart = _chart("timeline", ["A", "B"])
    timeline(chart, data, 5.0, registry)
    assigned = registry.assigned()
    assert assigned["SHOOT"] == 1  # first seen, first slot -- and only one slot
    assert len(set(assigned.values())) == len(assigned)


def test_registry_folds_past_eight_values_into_one_other_slot():
    registry = SlotRegistry()
    slots = [registry.slot(f"S{i}") for i in range(MAX_SLOTS + 4)]
    assert slots[:MAX_SLOTS] == list(range(1, MAX_SLOTS + 1))
    # A ninth generated hue is indistinguishable under CVD, so they share one.
    assert set(slots[MAX_SLOTS:]) == {0}


def test_registry_is_stable_for_a_repeated_value():
    registry = SlotRegistry()
    assert registry.slot("SHOOT") == registry.slot("SHOOT")


def test_text_width_grows_with_length_and_size():
    assert text_width("AAAA", 11) > text_width("AA", 11)
    assert text_width("AA", 22) > text_width("AA", 11)
    assert text_width("", 11) == 0


def test_ink_on_a_fill_is_chosen_per_theme():
    # Slot 7 needs white on its light step and near-black on its dark one; that
    # is exactly why the choice is computed rather than hardcoded.
    assert ink_on("#4a3aa7") == "#ffffff"
    assert ink_on("#9085e9") == "#0b0b0b"


def test_rendered_svg_escapes_markup_in_a_state_name():
    result = timeline(_chart("timeline", ["A"]), {"A": [(0.0, "<script>x</script>")]}, 10.0)
    assert "<script>" not in result.svg
    assert "&lt;script&gt;" in result.svg


# --- end to end -----------------------------------------------------------

FIXTURE = os.path.join(
    os.path.dirname(__file__), "..", "..", "build", "test-fixtures", "synthetic-2.wpilog"
)


@pytest.mark.skipif(not os.path.exists(FIXTURE), reason="fixture log not present")
def test_end_to_end_page_is_standalone(tmp_path):
    from log_charts import build

    findings = tmp_path / "f.json"
    findings.write_text(
        json.dumps(
            {
                "log": os.path.abspath(FIXTURE),
                "verdict": "Smoke.",
                "findings": [
                    {
                        "title": "Match state",
                        "detail": "The robot enabled once and stayed enabled.",
                        "charts": [
                            {
                                "form": "timeline",
                                "title": "State",
                                "series": [{"key": "DriverStation/Enabled", "label": "Enabled"}],
                                "reads_as": [
                                    "One unbroken enabled block, no dropouts.",
                                    "The lane never returns to false mid-match.",
                                ],
                            }
                        ],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    output = build(findings, tmp_path / "out", tmp_path)
    html = output.read_text(encoding="utf-8")

    assert html.startswith("<!DOCTYPE html>")
    assert html.count("<svg") >= 1
    # Self-contained: nothing may be fetched at open time, in a pit with no
    # network. The SVG xmlns is a namespace identifier, never fetched, so the
    # check looks for things that actually load: src, href and url().
    assert not re.search(r"""(?:src|href)\s*=\s*["']\s*(?:https?:)?//""", html)
    assert not re.search(r"url\(\s*['\"]?\s*(?:https?:)?//", html)
    assert "<script" in html
    # The findings that produced the page are kept beside it.
    assert (output.parent / "findings.json").exists()


@pytest.mark.skipif(not os.path.exists(FIXTURE), reason="fixture log not present")
def test_end_to_end_rejects_a_key_that_is_not_in_the_log(tmp_path):
    from log_charts import build

    findings = tmp_path / "f.json"
    findings.write_text(
        json.dumps(
            {
                "log": os.path.abspath(FIXTURE),
                "findings": [
                    {
                        "title": "Typo",
                        "detail": "d",
                        "charts": [
                            {
                                "form": "timeline",
                                "title": "State",
                                "series": [{"key": "DriverStation/Enable"}],
                                "reads_as": "r",
                            }
                        ],
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(SpecError, match="closest key in this log"):
        build(findings, tmp_path / "out", tmp_path)


# --- page structure -------------------------------------------------------


def _prose_of(value):
    from charting.spec import _prose

    return _prose(value, "<test>")


def _page(finding_detail, chart_reads_as, verdict=""):
    """Render one finding with one stub chart, bypassing the log entirely."""
    from charting.marks import Rendered
    from charting.page import render_page
    from charting.spec import Finding

    finding = Finding(
        title="Flywheel never recovered",
        detail=_prose_of(finding_detail),
        severity="critical",
    )
    rendered = Rendered(
        title="Flywheel velocity",
        svg="<svg></svg>",
        table_html="<table></table>",
        reads_as=_prose_of(chart_reads_as),
    )
    return render_page(
        title="run",
        subtitle_bits=["x.wpilog"],
        report=parse_report(_report(verdict=verdict)) if verdict else parse_report(_report()),
        findings=[(finding, [rendered])],
        baseline=[],
        generated="now",
    )


def test_a_finding_reads_conclusion_then_evidence_then_reading():
    html = _page("It plateaued low.", "The actual line sits below the target line.")
    order = [
        html.index(">Conclusion<"),
        html.index(">Evidence<"),
        html.index(">Why this supports the conclusion<"),
    ]
    assert order == sorted(order)
    # The reading follows the picture it is about, not the caption above it.
    assert html.index("<svg>") < html.index(">Why this supports the conclusion<")


def test_bulleted_prose_becomes_a_list_in_the_page():
    html = _page(["Lead.", "First.", "Second."], "Reading.")
    assert "<p>Lead.</p>" in html
    assert "<li>First.</li><li>Second.</li>" in html


def test_a_chart_with_no_reading_prints_no_empty_label():
    # Bundle charts land here. A label with nothing under it reads as an
    # omission the reader has to wonder about.
    html = _page("Conclusion text.", "")
    assert "Why this supports the conclusion" not in html


def test_a_bulleted_verdict_renders_as_a_list():
    html = _page("d", "r", verdict=["Lead.", "Point one."])
    assert ">Verdict<" in html and "<li>Point one.</li>" in html


def test_prose_is_escaped_in_the_page():
    html = _page("<b>not markup</b>", "Reading.")
    assert "<b>not markup</b>" not in html
    assert "&lt;b&gt;not markup&lt;/b&gt;" in html
