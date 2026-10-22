"""Where the acceptance bounds of the plugin-in-flatland comparisons come from.

The plugin and the offline stepper run the same maths in double precision (the test vectors prove it to
1e-9). In flatland they differ only through what happens after the velocity is set:

1. **Box2D arithmetic.** The body velocity is a `float`, the rotation is `cos`/`sin` of a `float` angle, and the
   solver integrates position and angle in `float` (`c += h * v`, `a += h * w`). `float32_track` repeats that
   arithmetic. Its error grows with the distance from the world origin (the `float` spacing is 3.8e-6 m between 32
   and 64 m); the replay puts every drive's start at the origin.
2. **Command timing** is not a model difference but a property of the replay path: a command reaches flatland through
   a topic and is applied a fixed whole number of steps later than the offline model assumes. `fbm validate hop`
   measures that number from a run with the identity model, at each speed, and the offline model then includes it.
   Nothing here covers a hop that varies within a run: that would show as a failure, and the fix is in the replay
   path, not in the bounds.
3. **Not different:** the zero snap (same rule, same input), step quantisation of the delay (same function),
   Box2D position correction (the welded bodies get one rigid-body velocity, so the joints have nothing to correct)
   and damping (zero in the models).

Bounds are set from these effects before any flatland run. `derive` prints them; `validate` applies the constants named there; a test checks, on synthetic drives, that the constants are at least the modelled effect times the margin.
"""
from __future__ import annotations

import math

import numpy as np

from . import scoring, stepper
from .drives import DT, Drive

SAFETY = 10.0         # margin over the modelled effect for what is not modelled (float sin/cos tables, solver order)


def float32_track(v: np.ndarray, w: np.ndarray, dt: float):
    """Box2D's single-precision integration of per-step (v, w), starting at the origin. Pose at the start of each step."""
    f = np.float32
    h = f(dt)
    x = y = a = f(0.0)
    n = len(v)
    ox, oy, oh = np.zeros(n), np.zeros(n), np.zeros(n)
    for k in range(n):
        ox[k], oy[k], oh[k] = x, y, a
        c, s = np.cos(a), np.sin(a)
        vk, wk = f(v[k]), f(w[k])
        x = f(x + h * (c * vk))
        y = f(y + h * (s * vk))
        a = f(a + h * wk)
    return ox, oy, oh


def _commands(drive: Drive, dt: float):
    cv, cw = np.zeros(drive.n), np.zeros(drive.n)
    stepper.events_to_steps(drive.ev_t, drive.ev_v, drive.ev_w, dt, drive.n, cv, cw)
    return cv, cw


def float32_delta(drives: list[Drive], p: np.ndarray, dt: float = DT, hop: int = 0) -> np.ndarray:
    """Per-drive objective of the offline (double) track scored against the single-precision track."""
    out = []
    for d in drives:
        cv, cw = _commands(d, dt)
        ov, ow = np.zeros(d.n), np.zeros(d.n)
        stepper.simulate(p, cv, cw, dt, ov, ow)
        sv, sw = np.zeros(d.n), np.zeros(d.n)
        sv[hop:], sw[hop:] = ov[:d.n - hop], ow[:d.n - hop]
        ref = float32_track(sv, sw, dt)
        out.append(scoring.track_objective(ref, scoring.offline_track(d, p, dt, hop)))
    return np.array(out)


def round_up(x: float, digits: int = 0) -> float:
    """Round up to `digits + 1` significant figures: 0.00437 -> 0.005."""
    e = math.floor(math.log10(x)) - digits
    return math.ceil(x / 10 ** e) * 10 ** e


def derive(drives: list[Drive], p: np.ndarray, dt: float = DT) -> dict:
    """The modelled effects and the bounds built from them (SAFETY times the effect, rounded up)."""
    f32 = float32_delta(drives, p, dt)
    return {
        "float32_mean": float(f32.mean()), "float32_max": float(f32.max()),
        "plugin_vs_offline_mean": round_up(SAFETY * f32.mean()),
        "plugin_vs_offline_max": round_up(SAFETY * f32.max()),
    }
