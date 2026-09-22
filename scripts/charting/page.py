"""
The HTML shell: tokens, layout, and the interaction layer.

Everything is inlined. The output has to open from a USB stick in a pit with no
network, so there are no CDN links, no web fonts and no external stylesheets --
which also means a chart cannot silently lose its styling six months from now
because a CDN moved.

Colours are CSS custom properties declared for both themes, so the single SVG
render that marks.py produced serves light and dark without re-rendering.
"""

from __future__ import annotations

import json

from .marks import esc
from .palette import (
    AXIS,
    GRID,
    HAIRLINE,
    INK_MUTED,
    INK_PRIMARY,
    INK_SECONDARY,
    MAX_SLOTS,
    PLANE,
    SERIES_DARK,
    SERIES_LIGHT,
    STATUS,
    SURFACE,
    ink_on,
)

SEVERITY_ICONS = {
    # Status colour never travels alone: every severity ships an icon and the
    # word, so the meaning survives greyscale, CVD and forced-colors.
    "critical": "◆",
    "serious": "▲",
    "warning": "▲",
    "good": "●",
    "info": "○",
}


def _theme_vars(mode: int) -> str:
    """Token block for one theme. mode 0 = light, 1 = dark."""
    series = SERIES_LIGHT if mode == 0 else SERIES_DARK
    lines = [
        f"color-scheme: {'light' if mode == 0 else 'dark'};",
        f"--surface-1: {SURFACE[mode]};",
        f"--plane: {PLANE[mode]};",
        f"--text-primary: {INK_PRIMARY[mode]};",
        f"--text-secondary: {INK_SECONDARY[mode]};",
        f"--text-muted: {INK_MUTED[mode]};",
        f"--grid: {GRID[mode]};",
        f"--axis: {AXIS[mode]};",
        f"--hairline: {HAIRLINE[mode]};",
        "--series-other: var(--text-muted);",
    ]
    for index, hex_color in enumerate(series, start=1):
        lines.append(f"--series-{index}: {hex_color};")
        # Computed per theme rather than hardcoded: slot 7 needs white ink on the
        # light step and near-black on the dark one.
        lines.append(f"--on-series-{index}: {ink_on(hex_color)};")
    for name, hex_color in STATUS.items():
        lines.append(f"--status-{name}: {hex_color};")
    return "\n    ".join(lines)


def _css() -> str:
    light = _theme_vars(0)
    dark = _theme_vars(1)
    return f"""
:root {{
    {light}
}}
@media (prefers-color-scheme: dark) {{
  :root:where(:not([data-theme="light"])) {{
    {dark}
  }}
}}
:root[data-theme="dark"] {{
    {dark}
}}

* {{ box-sizing: border-box; }}
html, body {{ margin: 0; padding: 0; }}
body {{
  background: var(--plane);
  color: var(--text-primary);
  font: 15px/1.55 system-ui, -apple-system, "Segoe UI", sans-serif;
  padding: 0 16px 72px;
}}
.wrap {{ max-width: 1080px; margin: 0 auto; }}

header.top {{ padding: 32px 0 8px; display: flex; gap: 16px; align-items: flex-start; flex-wrap: wrap; }}
header.top .grow {{ flex: 1 1 320px; min-width: 0; }}
h1 {{ font-size: 24px; line-height: 1.25; margin: 0 0 6px; font-weight: 600; letter-spacing: -0.01em; }}
.meta {{ color: var(--text-secondary); font-size: 13px; margin: 0; }}
.meta code {{ font-size: 12px; word-break: break-all; }}

button.theme {{
  border: 1px solid var(--hairline); background: var(--surface-1); color: var(--text-secondary);
  border-radius: 8px; padding: 7px 12px; font: inherit; font-size: 13px; cursor: pointer;
}}
button.theme:hover {{ color: var(--text-primary); }}

.card {{
  background: var(--surface-1); border: 1px solid var(--hairline);
  border-radius: 12px; padding: 20px 22px; margin: 16px 0;
}}
.verdict {{ font-size: 17px; line-height: 1.5; margin: 0; }}
.verdict-label {{
  font-size: 11px; letter-spacing: 0.09em; text-transform: uppercase;
  color: var(--text-muted); margin: 0 0 8px;
}}
.confidence {{ color: var(--text-secondary); font-size: 13px; margin: 12px 0 0; }}

.question {{ color: var(--text-secondary); font-size: 14px; font-style: italic; margin: 0 0 14px; }}

/* Stat tiles: proportional figures, never tabular -- tabular-nums makes a
   standalone 121 look loose at display sizes. */
.tiles {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 12px; margin: 16px 0; }}
.tile {{ background: var(--surface-1); border: 1px solid var(--hairline); border-radius: 12px; padding: 14px 16px; }}
.tile .label {{ color: var(--text-secondary); font-size: 12px; margin: 0 0 6px; }}
.tile .value {{ font-size: 28px; font-weight: 600; letter-spacing: -0.02em; line-height: 1.1; }}
.tile .unit {{ font-size: 15px; font-weight: 400; color: var(--text-secondary); margin-left: 3px; }}
.tile .note {{ color: var(--text-muted); font-size: 12px; margin: 6px 0 0; }}

.chip {{
  display: inline-flex; align-items: center; gap: 6px; font-size: 12px; font-weight: 500;
  border: 1px solid var(--hairline); border-radius: 999px; padding: 3px 10px;
  color: var(--text-secondary); white-space: nowrap;
}}
.chip .dot {{ font-size: 11px; line-height: 1; }}
.chip[data-sev="critical"] .dot {{ color: var(--status-critical); }}
.chip[data-sev="serious"] .dot {{ color: var(--status-serious); }}
.chip[data-sev="warning"] .dot {{ color: var(--status-warning); }}
.chip[data-sev="good"] .dot {{ color: var(--status-good); }}
.chip[data-sev="info"] .dot {{ color: var(--text-muted); }}

h2 {{ font-size: 18px; margin: 34px 0 4px; font-weight: 600; }}
h3 {{ font-size: 16px; margin: 0; font-weight: 600; }}
.finding-head {{ display: flex; gap: 10px; align-items: baseline; flex-wrap: wrap; margin-bottom: 8px; }}
.detail {{ color: var(--text-secondary); margin: 0 0 4px; }}
.section-note {{ color: var(--text-secondary); font-size: 13px; margin: 4px 0 0; }}

figure.chart {{ margin: 20px 0 0; }}
figure.chart + figure.chart {{ border-top: 1px solid var(--hairline); padding-top: 18px; }}
figcaption {{ margin-bottom: 8px; }}
.chart-title {{ font-size: 14px; font-weight: 600; }}
.chart-sub {{ color: var(--text-muted); font-size: 12px; }}
.chart-note {{ color: var(--text-secondary); font-size: 12px; margin-top: 3px; }}

.legend {{ display: flex; gap: 14px; flex-wrap: wrap; margin: 6px 0 2px; }}
.legend span {{ display: inline-flex; align-items: center; gap: 6px; font-size: 12px; color: var(--text-secondary); }}
.legend i {{ width: 11px; height: 11px; border-radius: 3px; display: inline-block; flex: none; }}

/* The plot area grows with its content so the x-axis band is never cut off
   into a nested scrollbar. */
.plot {{ position: relative; }}
.vz-svg {{ display: block; width: 100%; height: auto; overflow: visible; }}
.plot:focus-visible {{ outline: 2px solid var(--series-1); outline-offset: 3px; border-radius: 6px; }}

.vz-grid, .vz-grid-x {{ stroke: var(--grid); stroke-width: 1; fill: none; }}
.vz-axis {{ stroke: var(--axis); stroke-width: 1; }}
.vz-frame {{ stroke: var(--grid); stroke-width: 1; fill: none; }}
.vz-line {{ fill: none; stroke-width: 2; stroke-linejoin: round; stroke-linecap: round; }}
.vz-band {{ fill-opacity: 0.1; stroke: none; }}
.vz-window {{ fill: var(--text-muted); fill-opacity: 0.09; }}
.vz-dot {{ stroke: var(--surface-1); stroke-width: 2; }}
.vz-tick {{ fill: var(--text-muted); font-size: 11px; font-variant-numeric: tabular-nums; }}
.vz-tick-y {{ text-anchor: end; }}
.vz-tick-x {{ text-anchor: middle; }}
.vz-axis-label {{ fill: var(--text-muted); font-size: 11px; }}
.vz-lane {{ fill: var(--text-secondary); font-size: 12px; text-anchor: end; }}
.vz-track {{ fill: var(--text-muted); fill-opacity: 0.07; }}
.vz-block {{ cursor: crosshair; }}
.vz-block:focus-visible {{ outline: 2px solid var(--text-primary); }}
.vz-block-off {{ fill: var(--text-muted); fill-opacity: 0.1; }}
.vz-block-label {{ font-size: 11px; text-anchor: middle; pointer-events: none; }}
.vz-end-label {{ fill: var(--text-secondary); font-size: 11px; }}
.vz-cursor {{ stroke: var(--text-muted); stroke-width: 1; pointer-events: none; }}
.vz-cursor-dot {{ stroke: var(--surface-1); stroke-width: 2; pointer-events: none; }}

#tip {{
  position: fixed; z-index: 20; pointer-events: none; opacity: 0;
  transition: opacity 90ms ease; background: var(--surface-1);
  border: 1px solid var(--hairline); border-radius: 8px; padding: 8px 10px;
  font-size: 12px; color: var(--text-primary); max-width: 300px;
  box-shadow: 0 6px 20px rgba(0,0,0,0.16);
}}
#tip.on {{ opacity: 1; }}
#tip .row {{ display: flex; align-items: center; gap: 7px; white-space: nowrap; }}
#tip .row i {{ width: 9px; height: 9px; border-radius: 2px; display: inline-block; flex: none; }}
#tip .row b {{ font-weight: 400; color: var(--text-secondary); }}
#tip .row span {{ margin-left: auto; font-variant-numeric: tabular-nums; }}
#tip .when {{ color: var(--text-muted); margin-bottom: 4px; font-variant-numeric: tabular-nums; }}

details.tv {{ margin-top: 10px; }}
details.tv summary {{
  cursor: pointer; font-size: 12px; color: var(--text-secondary);
  padding: 3px 0; width: fit-content;
}}
details.tv summary:hover {{ color: var(--text-primary); }}
.tv-scroll {{ max-height: 340px; overflow: auto; margin-top: 8px; border: 1px solid var(--hairline); border-radius: 8px; }}
table.vz-table {{ border-collapse: collapse; width: 100%; font-size: 12px; font-variant-numeric: tabular-nums; }}
table.vz-table caption {{ caption-side: top; text-align: left; color: var(--text-muted); font-size: 12px; padding: 8px 10px; font-variant-numeric: normal; }}
table.vz-table th, table.vz-table td {{ text-align: right; padding: 4px 10px; border-bottom: 1px solid var(--hairline); white-space: nowrap; }}
table.vz-table thead th {{ position: sticky; top: 0; background: var(--surface-1); color: var(--text-secondary); font-weight: 500; }}
table.vz-table tbody th {{ color: var(--text-secondary); font-weight: 400; }}
table.vz-table td:first-of-type, table.vz-table th:first-child {{ text-align: left; }}

.empty {{ color: var(--text-muted); font-size: 13px; font-style: italic; margin: 8px 0 0; }}
footer {{ color: var(--text-muted); font-size: 12px; margin-top: 40px; border-top: 1px solid var(--hairline); padding-top: 14px; }}

@media print {{
  body {{ background: #fff; padding: 0; }}
  .card {{ break-inside: avoid; }}
  button.theme {{ display: none; }}
  details.tv {{ display: none; }}
}}
""".strip()


_SCRIPT = r"""
(function () {
  var root = document.documentElement;
  var btn = document.getElementById('theme');
  function apply(mode) {
    root.setAttribute('data-theme', mode);
    btn.textContent = mode === 'dark' ? 'Light mode' : 'Dark mode';
    btn.setAttribute('aria-label', 'Switch to ' + (mode === 'dark' ? 'light' : 'dark') + ' mode');
  }
  var startDark = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;
  apply(startDark ? 'dark' : 'light');
  btn.addEventListener('click', function () {
    apply(root.getAttribute('data-theme') === 'dark' ? 'light' : 'dark');
  });

  var tip = document.getElementById('tip');
  function showTip(html, clientX, clientY) {
    tip.innerHTML = html;
    tip.classList.add('on');
    var box = tip.getBoundingClientRect();
    var x = clientX + 14, y = clientY + 14;
    if (x + box.width > window.innerWidth - 8) x = clientX - box.width - 14;
    if (y + box.height > window.innerHeight - 8) y = clientY - box.height - 14;
    tip.style.left = Math.max(8, x) + 'px';
    tip.style.top = Math.max(8, y) + 'px';
  }
  function hideTip() { tip.classList.remove('on'); }

  function fmt(v) {
    if (v === null || v === undefined || isNaN(v)) return '-';
    var a = Math.abs(v);
    return v.toFixed(a >= 100 ? 0 : a >= 10 ? 1 : 2);
  }
  function swatch(slot) {
    var c = slot > 0 ? 'var(--series-' + slot + ')' : 'var(--series-other)';
    return '<i style="background:' + c + '"></i>';
  }

  // Blocks and any other per-mark target carry their text in data-tip. Focus
  // shows the same thing as hover, so a keyboard reader is not left out.
  document.querySelectorAll('[data-tip]').forEach(function (el) {
    function at(ev) {
      var r = el.getBoundingClientRect();
      showTip('<div class="row"><b>' + el.getAttribute('data-tip') + '</b></div>',
              ev && ev.clientX ? ev.clientX : r.left + r.width / 2,
              ev && ev.clientY ? ev.clientY : r.top);
    }
    el.addEventListener('pointerenter', at);
    el.addEventListener('pointermove', at);
    el.addEventListener('pointerleave', hideTip);
    el.addEventListener('focus', at);
    el.addEventListener('blur', hideTip);
  });

  document.querySelectorAll('.plot[data-chart]').forEach(function (plot) {
    var cfg;
    try { cfg = JSON.parse(plot.getAttribute('data-chart')); } catch (e) { return; }
    var svg = plot.querySelector('svg');
    if (!svg || !cfg.series || !cfg.series.length) return;

    var ns = 'http://www.w3.org/2000/svg';
    var layer = document.createElementNS(ns, 'g');
    svg.appendChild(layer);

    function toPx(t) { return cfg.px0 + (t - cfg.x0) / (cfg.x1 - cfg.x0) * (cfg.px1 - cfg.px0); }
    function toPy(v) { return cfg.py0 + (v - cfg.y0) / (cfg.y1 - cfg.y0) * (cfg.py1 - cfg.py0); }

    var index = 0;
    function clear() { while (layer.firstChild) layer.removeChild(layer.firstChild); }

    function drawX(t, clientX, clientY) {
      clear();
      var line = document.createElementNS(ns, 'line');
      line.setAttribute('class', 'vz-cursor');
      line.setAttribute('x1', toPx(t)); line.setAttribute('x2', toPx(t));
      line.setAttribute('y1', cfg.py1); line.setAttribute('y2', cfg.py0);
      layer.appendChild(line);
      var rows = '<div class="when">' + t.toFixed(2) + 's</div>';
      cfg.series.forEach(function (s) {
        var best = null;
        for (var i = 0; i < s.pts.length; i++) {
          if (s.pts[i][0] <= t) best = s.pts[i]; else break;
        }
        if (!best) return;
        var dot = document.createElementNS(ns, 'circle');
        dot.setAttribute('class', 'vz-cursor-dot');
        dot.setAttribute('cx', toPx(best[0])); dot.setAttribute('cy', toPy(best[1]));
        dot.setAttribute('r', 4);
        dot.setAttribute('fill', s.slot > 0 ? 'var(--series-' + s.slot + ')' : 'var(--series-other)');
        layer.appendChild(dot);
        rows += '<div class="row">' + swatch(s.slot) + '<b>' + s.label + '</b><span>' +
                fmt(best[1]) + (cfg.unit ? ' ' + cfg.unit : '') + '</span></div>';
      });
      showTip(rows, clientX, clientY);
    }

    function drawXY(px, py, clientX, clientY) {
      clear();
      var bestPoint = null, bestSeries = null, bestDist = Infinity;
      cfg.series.forEach(function (s) {
        for (var i = 0; i < s.pts.length; i++) {
          var dx = toPx(s.pts[i][1]) - px, dy = toPy(s.pts[i][2]) - py;
          var d = dx * dx + dy * dy;
          if (d < bestDist) { bestDist = d; bestPoint = s.pts[i]; bestSeries = s; }
        }
      });
      if (!bestPoint) return;
      var dot = document.createElementNS(ns, 'circle');
      dot.setAttribute('class', 'vz-cursor-dot');
      dot.setAttribute('cx', toPx(bestPoint[1])); dot.setAttribute('cy', toPy(bestPoint[2]));
      dot.setAttribute('r', 5);
      dot.setAttribute('fill', 'var(--series-' + bestSeries.slot + ')');
      layer.appendChild(dot);
      showTip('<div class="when">' + bestPoint[0].toFixed(2) + 's</div>' +
              '<div class="row">' + swatch(bestSeries.slot) + '<b>' + bestSeries.label +
              '</b><span>' + bestPoint[1].toFixed(2) + ', ' + bestPoint[2].toFixed(2) + ' m</span></div>',
              clientX, clientY);
    }

    function locate(ev) {
      var r = svg.getBoundingClientRect();
      var k = cfg.vw / r.width;
      return { x: (ev.clientX - r.left) * k, y: (ev.clientY - r.top) * k };
    }

    svg.addEventListener('pointermove', function (ev) {
      var p = locate(ev);
      if (p.x < cfg.px0 - 10 || p.x > cfg.px1 + 10) { clear(); hideTip(); return; }
      if (cfg.mode === 'xy') { drawXY(p.x, p.y, ev.clientX, ev.clientY); return; }
      var t = cfg.x0 + (p.x - cfg.px0) / (cfg.px1 - cfg.px0) * (cfg.x1 - cfg.x0);
      drawX(t, ev.clientX, ev.clientY);
    });
    svg.addEventListener('pointerleave', function () { clear(); hideTip(); });

    // Arrow keys step the cursor, so the values behind the crosshair are
    // reachable without a pointer.
    plot.setAttribute('tabindex', '0');
    var first = cfg.series[0].pts;
    plot.addEventListener('keydown', function (ev) {
      var step = ev.key === 'ArrowRight' ? 1 : ev.key === 'ArrowLeft' ? -1 : 0;
      if (!step) return;
      ev.preventDefault();
      index = Math.max(0, Math.min(first.length - 1, index + step * (ev.shiftKey ? 25 : 1)));
      var point = first[index];
      var r = svg.getBoundingClientRect();
      var cx = r.left + (toPx(point[0]) / cfg.vw) * r.width;
      if (cfg.mode === 'xy') drawXY(toPx(point[1]), toPy(point[2]), cx, r.top + r.height / 2);
      else drawX(point[0], cx, r.top + r.height / 2);
    });
    plot.addEventListener('blur', function () { clear(); hideTip(); });
  });
})();
""".strip()


def _legend(entries) -> str:
    """A legend is always present for two or more series, never for exactly one."""
    if len(entries) < 2:
        return ""
    items = []
    for label, slot in entries:
        color = f"var(--series-{slot})" if slot > 0 else "var(--series-other)"
        items.append(f'<span><i style="background:{color}"></i>{esc(label)}</span>')
    return f'<div class="legend">{"".join(items)}</div>'


def _chip(severity: str) -> str:
    icon = SEVERITY_ICONS.get(severity, SEVERITY_ICONS["info"])
    return (
        f'<span class="chip" data-sev="{esc(severity)}">'
        f'<span class="dot" aria-hidden="true">{icon}</span>{esc(severity)}</span>'
    )


def render_chart(chart) -> str:
    """One chart figure: caption, legend, plot, table view."""
    if chart.empty_reason:
        return (
            f'<figure class="chart"><figcaption><span class="chart-title">'
            f'{esc(chart.title)}</span></figcaption>'
            f'<p class="empty">Not rendered: {esc(chart.empty_reason)}.</p></figure>'
        )
    caption = f'<span class="chart-title">{esc(chart.title)}</span>'
    if chart.subtitle:
        caption += f' <span class="chart-sub">{esc(chart.subtitle)}</span>'
    if chart.note:
        caption += f'<div class="chart-note">{esc(chart.note)}</div>'
    data_attr = ""
    if chart.hover:
        data_attr = f" data-chart='{esc(json.dumps(chart.hover, separators=(',', ':')))}'"
    return (
        f'<figure class="chart"><figcaption>{caption}</figcaption>'
        f"{_legend(chart.legend)}"
        f'<div class="plot"{data_attr}>{chart.svg}</div>'
        f'<details class="tv"><summary>Table view</summary>'
        f'<div class="tv-scroll">{chart.table_html}</div></details>'
        f"</figure>"
    )


def render_page(*, title, subtitle_bits, report, findings, baseline, generated) -> str:
    """
    Assemble the whole document.

    ``findings`` is [(Finding, [Rendered, ...])] and ``baseline`` is [Rendered].
    """
    parts = [
        "<!DOCTYPE html>",
        '<html lang="en"><head><meta charset="utf-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1">',
        f"<title>{esc(title)}</title>",
        f"<style>{_css()}</style>",
        "</head><body>",
        '<div id="tip" role="status" aria-live="polite"></div>',
        '<div class="wrap">',
        '<header class="top"><div class="grow">',
        f"<h1>{esc(title)}</h1>",
        f'<p class="meta">{" &middot; ".join(esc(bit) for bit in subtitle_bits)}</p>',
        "</div>",
        '<button class="theme" id="theme" type="button">Dark mode</button>',
        "</header>",
    ]

    if report.verdict or report.question:
        parts.append('<section class="card">')
        if report.question:
            parts.append(f'<p class="question">{esc(report.question)}</p>')
        if report.verdict:
            parts.append('<p class="verdict-label">Verdict</p>')
            parts.append(f'<p class="verdict">{esc(report.verdict)}</p>')
        if report.confidence:
            parts.append(f'<p class="confidence">Confidence: {esc(report.confidence)}</p>')
        parts.append("</section>")

    if report.stats:
        tiles = []
        for stat in report.stats:
            unit = f'<span class="unit">{esc(stat.unit)}</span>' if stat.unit else ""
            note = f'<p class="note">{esc(stat.note)}</p>' if stat.note else ""
            chip = _chip(stat.severity) if stat.severity != "info" else ""
            tiles.append(
                f'<div class="tile"><p class="label">{esc(stat.label)}</p>'
                f'<div class="value">{esc(stat.value)}{unit}</div>{note}'
                f"{chip}</div>"
            )
        parts.append(f'<div class="tiles">{"".join(tiles)}</div>')

    if findings:
        parts.append("<h2>Findings</h2>")
        for finding, charts in findings:
            parts.append('<section class="card">')
            parts.append(
                f'<div class="finding-head"><h3>{esc(finding.title)}</h3>'
                f"{_chip(finding.severity)}</div>"
            )
            if finding.detail:
                parts.append(f'<p class="detail">{esc(finding.detail)}</p>')
            for chart in charts:
                parts.append(render_chart(chart))
            parts.append("</section>")

    if baseline:
        parts.append("<h2>Baseline</h2>")
        parts.append(
            '<p class="section-note">The standard preset charts, drawn whether or not '
            "a finding points at them. Collapsed because they are context, not the answer.</p>"
        )
        parts.append('<details class="card"><summary style="cursor:pointer">Show baseline charts</summary>')
        for chart in baseline:
            parts.append(render_chart(chart))
        parts.append("</details>")

    parts.append(
        f'<footer>Generated {esc(generated)} by scripts/log_charts.py &middot; '
        f"every chart plots raw logged keys, so a chart that contradicts the prose "
        f"above is the chart telling the truth.</footer>"
    )
    parts.append("</div>")
    parts.append(f"<script>{_SCRIPT}</script>")
    parts.append("</body></html>")
    return "\n".join(parts)
