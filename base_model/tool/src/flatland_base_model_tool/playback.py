"""Playback schedule for the in-process replay (`CommandPlayback` plugin): drives -> one JSON file."""
from __future__ import annotations

import json
from pathlib import Path

from .drives import DT, Drive

FIRST_START_S = 1.0
GAP_S = 6.0      # rest between drives: the shaped state reaches exactly zero and the robot is back at the origin
TAIL_S = 0.5     # recorded after the last grid step, so every hop up to 25 steps is covered


def _on_grid(t: float) -> float:
    return round(t / DT) * DT


def schedule(drives: list[Drive]) -> dict:
    """Drives in the given order, each starting at a simulated time on the step grid."""
    out, start = [], FIRST_START_S
    for d in drives:
        length = _on_grid(d.n * DT + TAIL_S)
        events = [[float(t), float(v), float(w)] for t, v, w in zip(d.ev_t, d.ev_v, d.ev_w)]
        out.append({"name": d.name, "start_s": _on_grid(start), "length_s": length, "events": events})
        start = start + length + GAP_S
    return {"version": 1, "drives": out}


def write(drives: list[Drive], path) -> dict:
    doc = schedule(drives)
    Path(path).write_text(json.dumps(doc, separators=(",", ":")) + "\n")
    return doc
