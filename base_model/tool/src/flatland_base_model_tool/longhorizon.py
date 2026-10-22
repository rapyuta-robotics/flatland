"""Long-horizon checks against the attached scan-matched tracks, per drive.

The 2 s window metric re-anchors at the start of every window, so the noise of a track's start pose enters twice.
These checks use the whole drive instead and need no re-anchoring beyond the drive's own start pose:

- final position error (m): distance between the model's and the track's final position;
- heading drift per metre (rad/m): the final heading error over the driven track length;
- distance ratio: the model's path length over the track's path length (1 is right; above 1 when the odometry
  over-reads distance and the model was fitted to it).

They are reported next to the window scores for the fitted model and for the baselines, not as a pass/fail gate.
"""
from __future__ import annotations

import math

import numpy as np

from . import scoring
from .drives import DT, Drive


def path_length(x, y) -> float:
    return float(np.hypot(np.diff(x), np.diff(y)).sum())


def check(drive: Drive, p: np.ndarray, dt: float = DT, hop: int = 0) -> dict:
    """The three numbers for one drive. Needs `drive.track`."""
    if drive.track is None:
        raise ValueError(f"drive {drive.name} has no attached track")
    sx, sy, sh = scoring.offline_track(drive, p, dt, hop)
    tx, ty, th = drive.track
    length = path_length(tx, ty)
    yaw_err = (sh[-1] - th[-1] + math.pi) % (2 * math.pi) - math.pi
    return {"final_position_error_m": float(math.hypot(sx[-1] - tx[-1], sy[-1] - ty[-1])),
            "heading_drift_rad_per_m": abs(yaw_err) / length if length > 0 else float("nan"),
            "distance_ratio": path_length(sx, sy) / length if length > 0 else float("nan")}


def summary(drives: list[Drive], p: np.ndarray, dt: float = DT, hop: int = 0) -> dict:
    """Per-drive checks reduced to median and mean (the mean of the distance ratio is by metres)."""
    rows = [check(d, p, dt, hop) for d in drives if d.track is not None]
    if not rows:
        raise ValueError("no drive has an attached track")
    col = lambda k: np.array([r[k] for r in rows])
    lengths = np.array([path_length(d.track[0], d.track[1]) for d in drives if d.track is not None])
    sim = col("distance_ratio") * lengths
    return {"drives": len(rows),
            "final_position_error_m": {"median": float(np.median(col("final_position_error_m"))),
                                       "mean": float(col("final_position_error_m").mean())},
            "heading_drift_rad_per_m": {"median": float(np.nanmedian(col("heading_drift_rad_per_m"))),
                                        "mean": float(np.nanmean(col("heading_drift_rad_per_m")))},
            "distance_ratio": {"median": float(np.median(col("distance_ratio"))),
                               "by_metres": float(sim.sum() / lengths.sum())}}


def lines(label: str, s: dict) -> str:
    return (f"{label:22s} final position error m: median {s['final_position_error_m']['median']:.3f}, mean "
            f"{s['final_position_error_m']['mean']:.3f} | heading drift rad/m: median "
            f"{s['heading_drift_rad_per_m']['median']:.4f}, mean {s['heading_drift_rad_per_m']['mean']:.4f} | "
            f"distance ratio: median {s['distance_ratio']['median']:.4f}, by metres {s['distance_ratio']['by_metres']:.4f}"
            f"  ({s['drives']} drives)")
