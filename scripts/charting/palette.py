"""
The chart palette -- validated, not eyeballed.

Slot order and hex values come from the data-viz reference palette and were run
through its validator before being pasted here:

    8 slots, adjacent pairlist  ->  PASS light, PASS dark
    3 slots, all-pairs          ->  PASS light, PASS dark   (path/scatter forms)

Two consequences are baked into the rest of the renderer and must not be undone
casually:

* Light-mode aqua, yellow and magenta sit below 3:1 against the surface, so the
  validator raises the "relief rule": every chart ships direct labels and a
  table view. That is why table views are not optional here.
* There is no ninth slot. Past eight distinct values, identity folds into
  ``OTHER_SLOT`` and leans on the block's own label, because a generated ninth
  hue is indistinguishable from an existing slot under colour-vision deficiency.

Charts emit ``var(--series-N)`` rather than hex so that one render serves light
and dark mode. The hexes below exist so the CSS can be generated, and so the
ink colour for text sitting *inside* a filled block can be computed per theme
instead of guessed.

Re-run the validator if any hex here changes.
"""

from __future__ import annotations

# Categorical slots, in the validated order. Index 0 is slot 1.
SERIES_LIGHT = (
    "#2a78d6",  # 1 blue
    "#eb6834",  # 2 orange
    "#1baf7a",  # 3 aqua
    "#eda100",  # 4 yellow
    "#e87ba4",  # 5 magenta
    "#008300",  # 6 green
    "#4a3aa7",  # 7 violet
    "#e34948",  # 8 red
)
SERIES_DARK = (
    "#3987e5",
    "#d95926",
    "#199e70",
    "#c98500",
    "#d55181",
    "#008300",
    "#9085e9",
    "#e66767",
)

MAX_SLOTS = len(SERIES_LIGHT)

# Status palette -- fixed, never themed, never reused as a series colour.
STATUS = {
    "good": "#0ca30c",
    "warning": "#fab219",
    "serious": "#ec835a",
    "critical": "#d03b3b",
}

# Chart chrome. (light, dark)
SURFACE = ("#fcfcfb", "#1a1a19")
PLANE = ("#f9f9f7", "#0d0d0d")
INK_PRIMARY = ("#0b0b0b", "#ffffff")
INK_SECONDARY = ("#52514e", "#c3c2b7")
INK_MUTED = ("#898781", "#898781")
GRID = ("#e1e0d9", "#2c2c2a")
AXIS = ("#c3c2b7", "#383835")
HAIRLINE = ("rgba(11,11,11,0.10)", "rgba(255,255,255,0.10)")

# The slot used once a timeline runs out of distinct hues. Deliberately not a
# categorical slot: it reads as "not individually coloured", and the block label
# carries the identity.
OTHER_SLOT = 0


def series_var(slot: int) -> str:
    """CSS variable for a 1-based categorical slot; slot 0 is the 'Other' fill."""
    if slot <= 0 or slot > MAX_SLOTS:
        return "var(--series-other)"
    return f"var(--series-{slot})"


def on_series_var(slot: int) -> str:
    """CSS variable for text drawn *on top of* a filled slot."""
    if slot <= 0 or slot > MAX_SLOTS:
        return "var(--text-primary)"
    return f"var(--on-series-{slot})"


def _channel(component: float) -> float:
    c = component / 255.0
    return c / 12.92 if c <= 0.03928 else ((c + 0.055) / 1.055) ** 2.4


def relative_luminance(hex_color: str) -> float:
    """WCAG relative luminance of an #rrggbb string."""
    h = hex_color.lstrip("#")
    r, g, b = (int(h[i : i + 2], 16) for i in (0, 2, 4))
    return 0.2126 * _channel(r) + 0.7152 * _channel(g) + 0.0722 * _channel(b)


def contrast_ratio(a: str, b: str) -> float:
    la, lb = relative_luminance(a), relative_luminance(b)
    lighter, darker = max(la, lb), min(la, lb)
    return (lighter + 0.05) / (darker + 0.05)


def ink_on(fill_hex: str) -> str:
    """
    Pick white or near-black for text sitting inside a filled block.

    Computed rather than hardcoded because the light and dark steps of a slot
    differ enough that the right answer is not always the same in both themes.
    """
    return "#ffffff" if contrast_ratio("#ffffff", fill_hex) >= contrast_ratio("#0b0b0b", fill_hex) else "#0b0b0b"


class SlotRegistry:
    """
    Assigns a stable colour slot to each distinct discrete value on a page.

    Colour follows the entity, not its rank: once ``SHOOT`` is slot 2 it is slot
    2 in every lane and every chart on the page, so a reader who learns a colour
    keeps it. Assignment is by order of first request, which for timelines means
    order of first appearance in the log -- deterministic for a given log, and
    therefore reproducible.

    Past ``MAX_SLOTS`` distinct values, everything else shares the 'Other' fill
    rather than getting a generated hue. Timeline blocks are always labelled, so
    identity survives the fold.
    """

    def __init__(self) -> None:
        self._slots: dict[str, int] = {}

    def slot(self, value: str) -> int:
        if value not in self._slots:
            taken = len(self._slots)
            self._slots[value] = taken + 1 if taken < MAX_SLOTS else OTHER_SLOT
        return self._slots[value]

    def assigned(self) -> dict[str, int]:
        return dict(self._slots)
