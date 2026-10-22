"""Hold stretches: periods when the platform is not under command control (for example manual handling), so that it
does not follow `/cmd_vel`.

A hold stretch is a time during which any configured source is true. A source is a topic and a way to read it:
- `bool:<topic>:<field>`: a message whose boolean field (a dotted attribute path, `data` for std_msgs/Bool) is true;
- `active:<topic>`: an actionlib GoalStatusArray with any goal in status ACTIVE (1).

A message holds its state until the next message of its topic, at most `MAX_HOLD_S` past the last message of a run.
Stretches closer than `MERGE_GAP_S` merge. The settle time after a stretch is derived from the data by `settle_curve`:
the first bin of time since a stretch ended whose median angular-velocity error, and that of the next two bins, is
inside the band of normal driving.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

ACTIVE = 1                 # actionlib_msgs/GoalStatus ACTIVE
# Generic defaults, not tuned to any platform; change them with `holds.configure` (the options of `fbm extract`).
MAX_HOLD_S = 1.0           # a state is not held longer than this past the last message of a run
MERGE_GAP_S = 1.0          # stretches closer than this are one stretch
LAG_S = 0.2                # command-to-odometry alignment of the settle statistic
NORMAL_AFTER_S = 30.0      # normal driving: at least this long after a stretch ends and before the next starts


def configure(max_hold_s=None, merge_gap_s=None) -> None:
    """Override the defaults above (None keeps the current value)."""
    global MAX_HOLD_S, MERGE_GAP_S
    if max_hold_s is not None:
        MAX_HOLD_S = float(max_hold_s)
    if merge_gap_s is not None:
        MERGE_GAP_S = float(merge_gap_s)


@dataclass(frozen=True)
class HoldSource:
    topic: str
    kind: str              # "bool" or "active"
    field: str = ""

    @staticmethod
    def parse(text: str) -> "HoldSource":
        kind, _, rest = text.partition(":")
        if kind == "bool":
            topic, _, field = rest.rpartition(":")
            if not topic.startswith("/") or not field:
                raise ValueError(f"hold source {text!r}: expected bool:/topic:field")
            return HoldSource(topic, "bool", field)
        if kind == "active" and rest.startswith("/"):
            return HoldSource(rest, "active")
        raise ValueError(f"hold source {text!r}: expected bool:/topic:field or active:/topic")


def _runs(times: np.ndarray, state: np.ndarray) -> list[tuple[float, float]]:
    """One interval per true message: from its time to the next message of the topic, at most MAX_HOLD_S later."""
    out = []
    for k in np.flatnonzero(state):
        nxt = times[k + 1] if k + 1 < len(times) else np.inf
        out.append((float(times[k]), float(min(nxt, times[k] + MAX_HOLD_S))))
    return out


def merge(intervals, gap: float = MERGE_GAP_S) -> list[tuple[float, float]]:
    out: list[list[float]] = []
    for a, b in sorted(intervals):
        if out and a - out[-1][1] <= gap:
            out[-1][1] = max(out[-1][1], b)
        else:
            out.append([a, b])
    return [(float(a), float(b)) for a, b in out]


def read_holds(bag, sources) -> dict:
    """Hold stretches of one bag file in record time, and which source topics the file has."""
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    sources = [s if isinstance(s, HoldSource) else HoldSource.parse(s) for s in sources]
    by_topic = {s.topic: s for s in sources}
    store = get_typestore(Stores.ROS1_NOETIC)
    series = {s.topic: [] for s in sources}
    if not sources:
        return {"stretches": [], "topics_present": {}}
    with Reader(Path(bag)) as r:
        conns = [c for c in r.connections if c.topic in by_topic]
        for c in conns:
            try:
                store.register(get_types_from_msg(c.msgdef.data, c.msgtype))
            except Exception:       # a type that the ROS 1 store already has
                pass
        for conn, t_ns, raw in (r.messages(connections=conns) if conns else ()):
            m = store.deserialize_ros1(raw, conn.msgtype)
            src = by_topic[conn.topic]
            if src.kind == "bool":
                value = m
                for part in src.field.split("."):
                    value = getattr(value, part)
                state = bool(value)
            else:
                state = any(int(g.status) == ACTIVE for g in m.status_list)
            series[conn.topic].append((t_ns * 1e-9, state))
    stretches = []
    for rows in series.values():
        if rows:
            a = np.array(rows, dtype=float)
            a = a[np.argsort(a[:, 0], kind="stable")]
            stretches += _runs(a[:, 0], a[:, 1] > 0.5)
    return {"stretches": merge(stretches), "topics_present": {t: bool(rows) for t, rows in series.items()}}


def overlaps(stretches, a: float, b: float, extend_end: float = 0.0) -> bool:
    """True if any stretch, with its end extended by `extend_end`, overlaps [a, b]."""
    return any(s0 <= b and s1 + extend_end >= a for s0, s1 in stretches)


# ----------------------------------------------------------------------------------------------
# Settle-time derivation (the criterion is fixed before the data are looked at)


def delta_series(odom: np.ndarray, cmd: np.ndarray, lag: float = LAG_S):
    """For each odometry sample: (time, |odom w(t) - cmd w(t - lag)|, command moving at t - lag).
    `odom` columns: header stamp, x, y, yaw, v, w; `cmd` columns: record time, v, w (sorted)."""
    t = odom[:, 0]
    idx = np.searchsorted(cmd[:, 0], t - lag, side="right") - 1
    ok = idx >= 0
    idx = np.clip(idx, 0, None)
    cv, cw = cmd[idx, 1], cmd[idx, 2]
    moving = ok & ((np.abs(cv) > 0.02) | (np.abs(cw) > 0.05))
    return t, np.abs(odom[:, 5] - cw), moving


def event_samples(stretches, t, delta, moving, horizon: float = 20.0):
    """For each stretch end: (time since the end, delta) of the moving samples before the next stretch starts."""
    out = []
    for i, (_, end) in enumerate(stretches):
        nxt = stretches[i + 1][0] if i + 1 < len(stretches) else np.inf
        m = moving & (t >= end) & (t < min(end + horizon, nxt))
        if m.any():
            out.append((t[m] - end, delta[m]))
    return out


def normal_samples(stretches, t, delta, moving) -> np.ndarray:
    """Delta of the moving samples at least NORMAL_AFTER_S after every stretch end and before every next start."""
    ok = moving.copy()
    for s0, s1 in stretches:
        ok &= ~((t >= s0 - NORMAL_AFTER_S) & (t <= s1 + NORMAL_AFTER_S))
    return delta[ok]


def settle_curve(events, normal: np.ndarray, bin_s: float = 0.5, max_s: float = 20.0, n_boot: int = 2000,
                 seed: int = 0) -> dict:
    """The settle time by the fixed criterion.

    `events`: list of (time since the end, delta) per stretch end. For every bin of `bin_s`: the median delta of the
    pooled samples and a band edge, the 95th percentile of the median of the pooled samples of E windows of `bin_s`
    drawn at random from the normal samples (E = the events with samples in the bin; the normal series has the same
    sampling interval as the event samples, so a window is `bin_s` of consecutive samples). A bin with fewer than 3
    events is untested and counts as inside the band only if its neighbours on both sides are.
    The settle time is the start of the first bin such that it and the next two bins are inside the band; `max_s` if
    none is. Returns the curve too.
    """
    rng = np.random.default_rng(seed)
    edges = np.arange(0.0, max_s + 1e-9, bin_s)
    nb = len(edges) - 1
    # window length in samples from the event samples' own spacing
    dts = [np.median(np.diff(t)) for t, _ in events if len(t) > 2]
    win = max(1, int(round(bin_s / float(np.median(dts))))) if dts else 25
    starts = np.arange(0, max(len(normal) - win, 1), win)
    medians, bands, counts = np.full(nb, np.nan), np.full(nb, np.nan), np.zeros(nb, int)
    for b in range(nb):
        pooled, e = [], 0
        for t, d in events:
            m = (t >= edges[b]) & (t < edges[b + 1])
            if m.any():
                pooled.append(d[m])
                e += 1
        counts[b] = e
        if not pooled:
            continue
        medians[b] = float(np.median(np.concatenate(pooled)))
        if e >= 3:
            meds = np.empty(n_boot)
            for k in range(n_boot):
                pick = rng.choice(starts, size=e)
                meds[k] = np.median(np.concatenate([normal[s:s + win] for s in pick]))
            bands[b] = float(np.percentile(meds, 95))
    inside = np.full(nb, False)
    tested = ~np.isnan(bands)
    inside[tested] = medians[tested] <= bands[tested]
    for b in range(nb):
        if not tested[b]:
            inside[b] = 0 < b < nb - 1 and tested[b - 1] and inside[b - 1] and tested[b + 1] and inside[b + 1]
    settle = max_s
    for b in range(nb - 2):
        if inside[b] and inside[b + 1] and inside[b + 2]:
            settle = float(edges[b])
            break
    return {"settle_s": settle, "edges": edges.tolist(), "medians": medians.tolist(), "bands": bands.tolist(),
            "events_per_bin": counts.tolist(), "inside": inside.tolist()}
