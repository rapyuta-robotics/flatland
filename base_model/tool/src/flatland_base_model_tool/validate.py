"""Flatland validation: score tracks recorded from flatland against the reference and the offline model.

Tracks are JSON files named after the drive, as written by `base_model_replay.py` (or built from any
other source): `{"name", "t": [s from the start of the replay], "x", "y", "yaw"}` in the world frame.
The reference of the window scores is the drive's scaled odometry (`Drive.odom`). A scan-matched track, if attached,
is for the long-horizon checks (`longhorizon`), reported next to these gates, not as one of them.

All comparisons use the objective of `scoring` (2 s windows, p95, position plus half the heading).

  estimate_hop        steps between a command and the body in the replay path (identity-model run)
  plugin_vs_offline   the offline model's track scored against the plugin's track as the reference, per drive
  fast_vs_slow        plugin_vs_offline at both speeds, each with its own measured hop
  negative_control    per-drive paired objective of a baseline track minus the plugin track, against a reference
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import scoring, stepper
from .drives import DT, Drive, grid_track

# Acceptance bounds of the comparisons below, fixed in code: the functions report them and never change them.
# They are derived in `bounds` from the single-precision arithmetic of the physics engine, times a margin of 10, and
# hold for any step-synchronous replay once the hop of the replay path is measured.
PLUGIN_VS_OFFLINE_MEAN_MAX = 7e-5
PLUGIN_VS_OFFLINE_DRIVE_MAX = 3e-4
MAX_HOP = 3
NEGATIVE_CONTROL_MIN_FRACTION = 0.8
BOOTSTRAP_SAMPLES = 10000
BOOTSTRAP_SEED = 0


def load_track(path, drive: Drive):
    """A replayed track on the grid of `drive`. The replay clock starts at the drive's step 0."""
    raw = json.loads(Path(path).read_text())
    return grid_track(raw["t"], raw["x"], raw["y"], raw["yaw"], 0.0, drive.n)


def track_dir(directory, drives: list[Drive]) -> dict:
    """name -> track for every drive that has a file in `directory`; a missing drive is an error."""
    out, missing = {}, []
    for d in drives:
        p = Path(directory) / f"{d.name}.json"
        if p.exists():
            out[d.name] = load_track(p, d)
        else:
            missing.append(d.name)
    if missing:
        raise FileNotFoundError(f"no track in {directory} for: {', '.join(missing)}")
    return out


@dataclass
class GateResult:
    name: str
    passed: bool
    detail: dict

    def line(self) -> str:
        return f"{'PASS' if self.passed else 'FAIL'} {self.name}: " + ", ".join(
            f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}" for k, v in self.detail.items())


def plugin_vs_offline(drives: list[Drive], plugin: dict, p: np.ndarray, dt: float = DT, hop: int = 0) -> GateResult:
    """Offline model with `hop` extra steps, scored against the plugin's track. Bounds: mean and max per drive."""
    deltas = np.array([scoring.track_objective(plugin[d.name], scoring.offline_track(d, p, dt, hop)) for d in drives])
    mean, worst = float(deltas.mean()), float(deltas.max())
    return GateResult("plugin vs offline", mean <= PLUGIN_VS_OFFLINE_MEAN_MAX and worst <= PLUGIN_VS_OFFLINE_DRIVE_MAX,
                      {"hop_steps": hop, "drives": len(drives), "mean": mean, "max": worst,
                       "mean_bound": PLUGIN_VS_OFFLINE_MEAN_MAX, "max_bound": PLUGIN_VS_OFFLINE_DRIVE_MAX,
                       "worst_drive": drives[int(deltas.argmax())].name})


def estimate_hop(drives: list[Drive], identity: dict, dt: float = DT, max_hop: int = MAX_HOP) -> GateResult:
    """Steps between a command and the body in the replay path, from a run with the identity model: the offline
    identity model with `hop` extra steps that matches the track best. Passes if that match is within the
    plugin-vs-offline bounds, i.e. one whole number of steps explains every drive."""
    p = stepper.vector(stepper.IDENTITY)
    means = []
    for hop in range(max_hop + 1):
        deltas = np.array([scoring.track_objective(identity[d.name], scoring.offline_track(d, p, dt, hop)) for d in drives])
        means.append((float(deltas.mean()), float(deltas.max()), hop))
    mean, worst, hop = min(means)
    ok = mean <= PLUGIN_VS_OFFLINE_MEAN_MAX and worst <= PLUGIN_VS_OFFLINE_DRIVE_MAX
    return GateResult("replay hop (identity model)", ok, {"hop_steps": hop, "mean": mean, "max": worst,
                                                          "next_best_mean": sorted(means)[1][0]})


def fast_vs_slow(drives: list[Drive], slow: dict, fast: dict, p: np.ndarray, hop_slow: int, hop_fast: int,
                 dt: float = DT) -> list[GateResult]:
    """Rate scaling must not change the model: the plugin-vs-offline comparison passes for both runs, each
    with the hop measured for its speed. The direct difference between the two tracks is reported."""
    a = plugin_vs_offline(drives, slow, p, dt, hop_slow)
    b = plugin_vs_offline(drives, fast, p, dt, hop_fast)
    a.name, b.name = "1x: " + a.name, "fast: " + b.name
    direct = np.array([scoring.track_objective(slow[d.name], fast[d.name]) for d in drives])
    info = GateResult("fast vs 1x, direct difference (information)", True,
                      {"mean": float(direct.mean()), "max": float(direct.max()), "hop_slow": hop_slow, "hop_fast": hop_fast})
    return [a, b, info]


def bootstrap_mean_ci(values: np.ndarray, level: float = 0.95, n: int = BOOTSTRAP_SAMPLES, seed: int = BOOTSTRAP_SEED):
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, len(values), size=(n, len(values)))
    means = values[idx].mean(axis=1)
    lo, hi = np.percentile(means, [(1 - level) / 2 * 100, (1 + level) / 2 * 100])
    return float(lo), float(hi)


def negative_control(drives: list[Drive], reference: dict, baseline: dict, plugin: dict, label: str) -> GateResult:
    """The baseline (stock flatland model) must score worse than the plugin on at least 80 % of drives, and the 95 %
    bootstrap interval of the paired mean difference (baseline minus plugin) must exclude 0, on the side
    where the baseline is worse."""
    diff = np.array([scoring.track_objective(reference[d.name], baseline[d.name])
                     - scoring.track_objective(reference[d.name], plugin[d.name]) for d in drives])
    frac = float((diff > 0).mean())
    lo, hi = bootstrap_mean_ci(diff)
    ok = frac >= NEGATIVE_CONTROL_MIN_FRACTION and lo > 0
    return GateResult(f"negative control ({label})", ok,
                      {"drives": len(drives), "fraction_baseline_worse": frac, "mean_diff": float(diff.mean()),
                       "ci_low": lo, "ci_high": hi, "fraction_bound": NEGATIVE_CONTROL_MIN_FRACTION})
