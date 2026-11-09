"""Order-insensitive comparison of two robots (used by the self-check and tests)."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections import Counter

from .model import Robot


def _norm_value(value: str):
    tokens = value.split()
    if not tokens:
        return ""
    try:
        numbers = [float(t) for t in tokens]
    except ValueError:
        return " ".join(tokens)
    return tuple(0.0 if abs(x) < 1e-12 else round(x, 9) for x in numbers)


def canonical(element: ET.Element):
    """Hashable form of an element: attribute/child order and number formatting ignored."""
    text = (element.text or "").strip()
    return (
        element.tag,
        tuple(sorted((k, _norm_value(v)) for k, v in element.attrib.items())),
        _norm_value(text) if text else "",
        tuple(sorted((canonical(c) for c in element), key=repr)),
    )


def diff_robots(a: Robot, b: Robot, limit: int = 20) -> list[str]:
    """Human-readable differences between two robots; empty list means identical."""
    diffs: list[str] = []
    if a.name != b.name:
        diffs.append(f"robot name differs: '{a.name}' vs '{b.name}'")
    for kind, ta, tb in (("link", a.links, b.links), ("joint", a.joints, b.joints)):
        missing = sorted(ta.keys() - tb.keys())
        extra = sorted(tb.keys() - ta.keys())
        if missing:
            diffs.append(f"{kind}s missing after rebuild: {', '.join(missing)}")
        if extra:
            diffs.append(f"unexpected {kind}s after rebuild: {', '.join(extra)}")
        for name in sorted(ta.keys() & tb.keys()):
            if canonical(ta[name].element) != canonical(tb[name].element):
                diffs.append(f"{kind} '{name}' differs")
    extras_a = Counter(repr(canonical(e)) for e in a.extras)
    extras_b = Counter(repr(canonical(e)) for e in b.extras)
    if extras_a != extras_b:
        diffs.append("top-level elements (gazebo/transmission/ros2_control/material) differ")
    return diffs[:limit]
