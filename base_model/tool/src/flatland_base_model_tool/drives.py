"""Drives: bags -> drive files -> arrays on the 50 Hz grid.

A drive file is JSON:

    {"name", "t0", "t1", "cmd": [[t, v, w], ...], "odom": [[t, x, y, yaw, v, w], ...], "meta": {...}}

`cmd` times are bag record times. `odom` times are header stamps (when the message was produced): a record time can
lag the stamp by a varying amount, which would show up as false position error. t0 and t1 include a pad.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Generic defaults for the drive extraction, not tuned to any platform; change them with `configure` (the options of
# `fbm extract`).
DT = 0.02
PAD_S = 2.0
MOVING_V, MOVING_W = 0.02, 0.05      # odometry speeds above which the robot counts as moving
GAP_S = 1.0                          # a pause this long ends a drive
MIN_DIST_M, MIN_TURN_RAD = 3.0, 1.5  # smaller moves are not drives
MAX_STITCH_GAP_S = 0.1               # largest /odom header and record-time gap at a split boundary that is stitched
EMBARGO_S = 60.0                     # drives this close after the last fit drive are in neither set
MIN_SCORED_S = 4.0                   # commanded motion a manually interrupted drive must keep (two scoring windows)
MIN_ODOM_SAMPLES = 10
DEFAULT_MANUAL_TOPIC = None             # topic with a manual-control flag, if the platform has one (none by default)
DEFAULT_MANUAL_FIELD = "data"
DEFAULT_LATCHED = ("/map", "/tf_static")   # latched topics a reference-track builder may need; reported, not read here


def configure(pad_s=None, moving_v=None, moving_w=None, gap_s=None, min_dist_m=None, min_turn_rad=None,
              max_stitch_gap_s=None, min_scored_s=None) -> None:
    """Override the defaults above (None keeps the current value)."""
    global PAD_S, MOVING_V, MOVING_W, GAP_S, MIN_DIST_M, MIN_TURN_RAD, MAX_STITCH_GAP_S, MIN_SCORED_S
    if pad_s is not None:
        PAD_S = float(pad_s)
    if moving_v is not None:
        MOVING_V = float(moving_v)
    if moving_w is not None:
        MOVING_W = float(moving_w)
    if gap_s is not None:
        GAP_S = float(gap_s)
    if min_dist_m is not None:
        MIN_DIST_M = float(min_dist_m)
    if min_turn_rad is not None:
        MIN_TURN_RAD = float(min_turn_rad)
    if max_stitch_gap_s is not None:
        MAX_STITCH_GAP_S = float(max_stitch_gap_s)
    if min_scored_s is not None:
        MIN_SCORED_S = float(min_scored_s)


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read_bag(path, manual_topic=DEFAULT_MANUAL_TOPIC, manual_field=DEFAULT_MANUAL_FIELD):
    """Return (odom, cmd, manual, has_manual) arrays from one ROS 1 bag."""
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    odom, cmd, manual = [], [], []
    store = get_typestore(Stores.ROS1_NOETIC)
    with Reader(Path(path)) as reader:
        topics = {c.topic for c in reader.connections}
        wanted = {"/odom", "/cmd_vel"} | ({manual_topic} if manual_topic else set())
        conns = [c for c in reader.connections if c.topic in wanted]
        for c in conns:   # only the wanted types are registered: other topics' types may clash with the store
            try:
                store.register(get_types_from_msg(c.msgdef.data, c.msgtype))
            except Exception:   # a type that the ROS 1 store already has
                pass
        for conn, t_ns, raw in reader.messages(connections=conns):
            m = store.deserialize_ros1(raw, conn.msgtype)
            if conn.topic == "/odom":
                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                p = m.pose.pose
                odom.append((stamp, p.position.x, p.position.y, _yaw(p.orientation),
                             m.twist.twist.linear.x, m.twist.twist.angular.z))
            elif conn.topic == "/cmd_vel":
                cmd.append((t_ns * 1e-9, m.linear.x, m.angular.z))
            else:
                manual.append((t_ns * 1e-9, int(getattr(m, manual_field))))
    has_manual = bool(manual_topic) and manual_topic in topics
    return (np.array(odom).reshape(-1, 6), np.array(cmd).reshape(-1, 3),
            np.array(manual).reshape(-1, 2), has_manual)


def segment(odom: np.ndarray):
    """Moving runs of the odometry as dicts, in time order. Odometry is sorted by header stamp first."""
    odom = odom[np.argsort(odom[:, 0], kind="stable")]
    moving = (np.abs(odom[:, 4]) > MOVING_V) | (np.abs(odom[:, 5]) > MOVING_W)
    idx = np.flatnonzero(moving)
    if not len(idx):
        return []
    max_gap = int(round(GAP_S / DT))
    segs = []
    for run in np.split(idx, np.flatnonzero(np.diff(idx) > max_gap) + 1):
        if len(run) < MIN_ODOM_SAMPLES:
            continue
        i0, i1 = int(run[0]), int(run[-1])
        xy = odom[i0:i1 + 1, 1:3]
        dist = float(np.hypot(*np.diff(xy, axis=0).T).sum())
        turn = float(np.abs(np.diff(np.unwrap(odom[i0:i1 + 1, 3]))).sum())
        segs.append({"i0": i0, "i1": i1, "t0": float(odom[i0, 0]), "t1": float(odom[i1, 0]), "dist": dist,
                     "abs_turn": turn})
    return segs


def latched_sources(group: list[dict], topics) -> dict:
    """For each latched topic, the first file of the recording that holds it, or None. Recorders that save latched
    topics only in the first split leave later splits without them; the stream of a recording carries them forward
    from that file. A topic is never taken from a file outside the recording."""
    return {t: next((p["bag"].name for p in group if t in p["topics"]), None) for t in topics}


def stitch(parts: list[dict]) -> list[list[dict]]:
    """Group per-file streams into recordings. Files are ordered by their first /odom record time; a file
    continues the recording of the one before it when the gap at the boundary is at most MAX_STITCH_GAP_S
    in /odom header stamps and in record time. Otherwise it starts a new recording. Every boundary is
    reported in the returned groups' `boundary` field."""
    parts = sorted(parts, key=lambda p: p["first_record"])
    groups: list[list[dict]] = []
    for p in parts:
        if groups:
            q = groups[-1][-1]
            header_gap = p["odom"][0, 0] - q["odom"][-1, 0]
            record_gap = p["first_record"] - q["last_record"]
            p["boundary"] = {"header_gap_s": float(header_gap), "record_gap_s": float(record_gap)}
            if 0 <= header_gap <= MAX_STITCH_GAP_S and abs(record_gap) <= MAX_STITCH_GAP_S:
                groups[-1].append(p)
                continue
        groups.append([p])
    return groups


HOLD_SETTLE_S = None   # seconds after a hold stretch that are left out; derive it from your data (`holds.settle_curve`)


def extract(bags, out_dir, manual_topic=DEFAULT_MANUAL_TOPIC, manual_field=DEFAULT_MANUAL_FIELD,
            latched=DEFAULT_LATCHED, hold_sources=(), hold_settle_s=HOLD_SETTLE_S) -> dict:
    """Write one drive file per usable drive into `out_dir`; return the index.

    Consecutive bag splits of one recording are read as one stream, in time order, without writing merged
    files (see `stitch`). A drive is cut only if the robot is moving at the first sample of the first file
    or the last sample of the last file of a recording. A drive window extends PAD_S before and after the
    moving run but never past the middle of the pause to the neighbouring run, so the robot is at rest at
    both edges.

    Excluded: cut drives; drives that start under manual control, or whose manual-control onset leaves
    less than MIN_SCORED_S of commanded motion; drives without commands; drives whose commanded distance
    does not explain the driven distance. Manual control that starts after a drive's moving run, or inside it
    with enough motion before it, ends the scored part at its onset (`score_end_s`).

    Hold rule (`hold_sources` and `hold_settle_s`, in seconds; off when there are no sources): hold
    stretches (see `holds`) are long and usually touch the pads of a drive, so the
    drive is kept and each stretch that overlaps its window, with its end extended by the settle time, is recorded in
    the drive file as `holds` (seconds from the window start). Scoring leaves out every window that overlaps one,
    and drops a drive that has no moving window left (`load_dir`).
    """
    from rosbags.rosbag1 import Reader  # noqa: F401  (import error early if rosbags is missing)

    if hold_sources and hold_settle_s is None:
        raise ValueError("hold sources need a settle time (hold_settle_s)")
    if not hold_sources:
        hold_settle_s = None

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    parts = []
    for bag in bags:
        odom, cmd, manual, has_manual = read_bag(bag, manual_topic, manual_field)
        if not len(odom):
            continue
        odom = odom[np.argsort(odom[:, 0], kind="stable")]
        cmd = cmd[np.argsort(cmd[:, 0], kind="stable")] if len(cmd) else cmd
        first_record, last_record, topics = _bag_info(bag)
        part = {"bag": Path(bag), "odom": odom, "cmd": cmd, "manual": manual, "has_manual": has_manual,
                "first_record": first_record, "last_record": last_record, "topics": topics}
        if hold_settle_s is not None:
            from . import holds
            part["holds"] = holds.read_holds(bag, hold_sources)
        parts.append(part)
    index = {"max_stitch_gap_s": MAX_STITCH_GAP_S, "hold_settle_s": hold_settle_s, "recordings": []}
    for group in stitch(parts):
        index["recordings"].append(_extract_recording(group, out_dir, latched, hold_settle_s))
    Path(out_dir, "index.json").write_text(json.dumps(index, indent=1))
    return index


def _bag_info(bag) -> tuple[float, float, set]:
    """Record times of the first and last message of a bag file, and the topics that it holds."""
    from rosbags.rosbag1 import Reader
    with Reader(Path(bag)) as r:
        return r.start_time * 1e-9, r.end_time * 1e-9, {c.topic for c in r.connections}


def _extract_recording(group: list[dict], out_dir: Path, latched=(), hold_settle_s=None) -> dict:
    odom = np.concatenate([p["odom"] for p in group])
    odom = odom[np.argsort(odom[:, 0], kind="stable")]
    cmd = np.concatenate([p["cmd"] for p in group if len(p["cmd"])]) if any(len(p["cmd"]) for p in group) else np.zeros((0, 3))
    cmd = cmd[np.argsort(cmd[:, 0], kind="stable")] if len(cmd) else cmd
    mans = [p["manual"] for p in group if len(p["manual"])]
    manual = np.concatenate(mans) if mans else np.zeros((0, 2))
    manual = manual[np.argsort(manual[:, 0], kind="stable")] if len(manual) else manual
    stem = group[0]["bag"].stem
    entry = {"files": [p["bag"].name for p in group],
             "boundaries": [p["boundary"] for p in group[1:]],
             "manual_topic_present": all(p["has_manual"] for p in group),
             "latched": latched_sources(group, latched), "kept": [], "excluded": []}
    stretches = []
    if hold_settle_s is not None:
        from . import holds
        stretches = holds.merge([iv for p in group for iv in p["holds"]["stretches"]])
        entry["holds"] = {"stretches": len(stretches), "list": stretches,
                          "topics_present": {k: any(p["holds"]["topics_present"][k] for p in group)
                                             for k in group[0]["holds"]["topics_present"]}}
    n = len(odom)
    segs = segment(odom)
    for i, s in enumerate(segs):
        if s["dist"] < MIN_DIST_M and s["abs_turn"] < MIN_TURN_RAD:
            continue
        rec = {"index": i, "t0": s["t0"], "dist": s["dist"], "abs_turn": s["abs_turn"]}
        if s["i0"] == 0 or s["i1"] == n - 1:
            edge = odom[0] if s["i0"] == 0 else odom[-1]
            entry["excluded"].append({**rec, "reason": "cut by the start or end of the recording",
                                      "edge_speed": float(abs(edge[4])), "edge_yaw_rate": float(abs(edge[5]))})
            continue
        t0 = max(s["t0"] - PAD_S, (segs[i - 1]["t1"] + s["t0"]) / 2) if i else s["t0"] - PAD_S
        t1 = min(s["t1"] + PAD_S, (s["t1"] + segs[i + 1]["t0"]) / 2) if i + 1 < len(segs) else s["t1"] + PAD_S
        score_end_s = None
        run_end = t1
        if len(manual):
            k = np.searchsorted(manual[:, 0], s["t0"], side="right")
            if k > 0 and manual[k - 1, 1] != 0:
                entry["excluded"].append({**rec, "reason": "manual control at the start"})
                continue
            after = manual[(manual[:, 0] > s["t0"]) & (manual[:, 0] <= t1) & (manual[:, 1] != 0), 0]
            if len(after):
                onset = float(after[0])
                if onset <= s["t1"] and onset - s["t0"] < MIN_SCORED_S:
                    entry["excluded"].append({**rec, "reason": "manual control takes over early"})
                    continue
                score_end_s = onset - t0
                run_end = onset
        cc = cmd[(cmd[:, 0] >= t0) & (cmd[:, 0] <= t1)] if len(cmd) else cmd
        if not len(cc):
            entry["excluded"].append({**rec, "reason": "no /cmd_vel"})
            continue
        cs = cc[cc[:, 0] <= run_end]
        cmd_dist = float(np.sum(np.abs(cs[:-1, 1]) * np.diff(cs[:, 0]))) if len(cs) > 1 else 0.0
        driven = s["dist"] if score_end_s is None or run_end >= s["t1"] else _dist_until(odom, s, run_end)
        if abs(cmd_dist - driven) > 0.25 * driven + 0.5:
            entry["excluded"].append({**rec, "cmd_dist": cmd_dist, "reason": "commanded path does not explain real path"})
            continue
        oo = odom[(odom[:, 0] >= t0) & (odom[:, 0] <= t1)]
        name = f"{stem}_{i:02d}"
        hold_iv = None
        if hold_settle_s is not None:
            hold_iv = [[max(a, t0) - t0, min(b + hold_settle_s, t1) - t0] for a, b in stretches
                       if a <= t1 and b + hold_settle_s >= t0]
        (out_dir / f"{name}.json").write_text(json.dumps({
            "name": name, "t0": t0, "t1": t1, "run_t0": s["t0"], "run_t1": s["t1"], "cmd": cc.tolist(), "odom": oo.tolist(),
            "score_end_s": score_end_s, **({"holds": hold_iv} if hold_iv is not None else {}),
            "meta": {"files": entry["files"], "dist": s["dist"], "abs_turn": s["abs_turn"], "cmd_dist": cmd_dist,
                     "edge_speed": float(max(abs(oo[0, 4]), abs(oo[-1, 4])))}}))
        entry["kept"].append({"name": name, **rec, "cmd_dist": cmd_dist,
                              "edge_speed": float(max(abs(oo[0, 4]), abs(oo[-1, 4]))),
                              **({"hold_seconds": float(sum(b - a for a, b in hold_iv))} if hold_iv is not None else {})})
    return entry


def _dist_until(odom: np.ndarray, s: dict, t: float) -> float:
    m = (odom[:, 0] >= s["t0"]) & (odom[:, 0] <= t)
    xy = odom[m, 1:3]
    return float(np.hypot(*np.diff(xy, axis=0).T).sum()) if len(xy) > 1 else 0.0


def extract_all(bags, out_dir, **kw) -> dict:
    return extract(bags, out_dir, **kw)


def edge_report(index: dict) -> str:
    """What `extract` saw at the edges: the boundaries it stitched, the drives cut at a recording end, and
    the speed at the edges of the kept drive windows (which must be at rest)."""
    lines = []
    for r in index["recordings"]:
        gaps = [(b["header_gap_s"], b["record_gap_s"]) for b in r["boundaries"]]
        lines.append(f"recording of {len(r['files'])} file(s), {len(r['kept'])} drives kept, boundaries (header gap, "
                     f"record gap, s): " + (", ".join(f"({h:.3f}, {c:.3f})" for h, c in gaps) or "none"))
    for r in index["recordings"]:
        for topic, src in r["latched"].items():
            lines.append(f"latched {topic}: " + (f"from {src}" if src else "MISSING in every file of this recording"))
    for r in index["recordings"]:
        h = r.get("holds")
        if h is not None:
            touched = [k for k in r["kept"] if k.get("hold_seconds", 0) > 0]
            missing = [k for k, v in h["topics_present"].items() if not v]
            lines.append(f"holds: {h['stretches']} stretch(es), settle {index['hold_settle_s']:g} s; {len(touched)} of "
                         f"{len(r['kept'])} drives have excluded windows ({sum(k['hold_seconds'] for k in touched):.0f} s of drive time)"
                         + (f"; WARNING hold topic(s) absent: {', '.join(missing)}" if missing else ""))
    kept = [k["edge_speed"] for r in index["recordings"] for k in r["kept"]]
    cut = [e for r in index["recordings"] for e in r["excluded"] if e["reason"].startswith("cut")]
    if kept:
        lines.append(f"edge speed of kept drive windows: max {max(kept):.3f} m/s")
    for e in cut:
        lines.append(f"cut: drive {e['index']:02d} ({e['dist']:.1f} m) moving at the recording edge: "
                     f"{e['edge_speed']:.3f} m/s, {e['edge_yaw_rate']:.3f} rad/s")
    return "\n".join(lines)


@dataclass
class Drive:
    """A drive on the model grid. Tracks are in the frame of the pose at t0.

    `x`, `y`, `h` are the primary reference that fits and scores use: the attached scan-matched track if
    `reference == "track"`, else the odometry with its displacement scaled by `odom_scale`. `odom` is always the
    scaled odometry; `track` is the attached track or None.
    """
    name: str
    t_start: float          # absolute time of grid step 0
    n: int
    dist: float
    ev_t: np.ndarray        # command times, s from t_start
    ev_v: np.ndarray
    ev_w: np.ndarray
    x: np.ndarray
    y: np.ndarray
    h: np.ndarray
    reference: str = "odom"
    odom: tuple | None = None
    track: tuple | None = None
    exclude: np.ndarray | None = None   # bool per grid step: inside a hold stretch (or its settle time); None = none


def grid_track(t, x, y, yaw, t0: float, n: int):
    """Samples with absolute stamps -> track on the model grid, in the frame of the pose at t0."""
    t = np.asarray(t, float) - t0
    keep = np.concatenate([[True], np.diff(t) > 0])
    t, x, y, yaw = t[keep], np.asarray(x, float)[keep], np.asarray(y, float)[keep], np.unwrap(np.asarray(yaw, float)[keep])
    tg = np.arange(n) * DT
    gx, gy, gh = np.interp(tg, t, x), np.interp(tg, t, y), np.interp(tg, t, yaw)
    x0, y0, h0 = np.interp(0, t, x), np.interp(0, t, y), np.interp(0, t, yaw)
    c, s = math.cos(h0), math.sin(h0)
    return c * (gx - x0) + s * (gy - y0), -s * (gx - x0) + c * (gy - y0), gh - h0


def prepare(raw: dict, odom_scale: float = 1.0, reference: str = "odom", odom_yaw_scale: float = 1.0,
            holds: bool = True) -> Drive:
    """Drive file contents -> Drive. `odom_scale` multiplies the odometry displacement and `odom_yaw_scale` its heading change (1 by default:
    only the displacement is corrected). `reference` is "odom" or "track"; "track" needs the drive file to carry a `ref_track`."""
    if reference not in ("odom", "track"):
        raise ValueError(f"reference must be 'odom' or 'track', not {reference!r}")
    t0 = raw["t0"]
    ev = [(float(t) - t0, float(v), float(w)) for t, v, w in raw["cmd"]]
    if ev[-1][1] != 0.0 or ev[-1][2] != 0.0:
        ev.append((ev[-1][0], 0.0, 0.0))   # a drive ends at rest
    ev = np.array(ev)
    n = int(math.floor((raw["t1"] - t0) / DT)) + 1
    if raw.get("score_end_s") is not None:
        n = min(n, int(math.floor(raw["score_end_s"] / DT)) + 1)
    o = np.array(raw["odom"])
    ox, oy, oh = grid_track(o[:, 0], o[:, 1], o[:, 2], o[:, 3], t0, n)
    odom = (odom_scale * ox, odom_scale * oy, odom_yaw_scale * oh)
    track = None
    if raw.get("ref_track"):
        r = raw["ref_track"]
        track = grid_track(r["t"], r["x"], r["y"], r["yaw"], t0, n)
    if reference == "track":
        if track is None:
            raise ValueError(f"drive {raw['name']} has no reference track")
        primary = track
    else:
        primary = odom
    dist = float(np.hypot(np.diff(primary[0]), np.diff(primary[1])).sum())
    exclude = None
    if holds and raw.get("holds"):
        tg = np.arange(n) * DT
        exclude = np.zeros(n, dtype=bool)
        for a, b in raw["holds"]:
            exclude |= (tg >= a - 1e-9) & (tg <= b + 1e-9)
    return Drive(raw["name"], t0, n, dist, ev[:, 0], ev[:, 1], ev[:, 2], primary[0], primary[1], primary[2],
                 reference, odom, track, exclude)


def has_moving_window(drive: Drive) -> bool:
    """True if some 2 s scoring window of the drive lies outside every hold stretch and has more than 0.2 m of path."""
    from . import scoring
    mask = np.zeros(drive.n, np.uint8) if drive.exclude is None else drive.exclude.astype(np.uint8)
    return scoring.count_moving_windows(drive.x, drive.y, mask) > 0


def load_dir(directory, odom_scale: float = 1.0, reference: str = "odom", odom_yaw_scale: float = 1.0,
             holds: bool = True) -> list[Drive]:
    """All drive files of a directory, in time order. With reference "track", drives without an attached
    track are left out; `last_skipped` of the module names them."""
    global last_skipped
    raws = [json.loads(p.read_text()) for p in sorted(Path(directory).glob("*.json")) if p.name != "index.json"]
    raws.sort(key=lambda r: r["t0"])
    last_skipped = []
    if reference == "track":
        last_skipped = [r["name"] for r in raws if not r.get("ref_track")]
        raws = [r for r in raws if r.get("ref_track")]
    out = [prepare(r, odom_scale, reference, odom_yaw_scale, holds) for r in raws]
    dropped = [d.name for d in out if d.exclude is not None and not has_moving_window(d)]
    if dropped:
        last_skipped = last_skipped + dropped
        out = [d for d in out if d.name not in dropped]
    return out


last_skipped: list[str] = []


def read_track_file(path) -> dict:
    """A reference track: `.npz` (arrays t, x, y, yaw, absolute stamps; optional sigma_xy, sigma_yaw) or `.json`."""
    p = Path(path)
    if p.suffix == ".npz":
        z = np.load(p, allow_pickle=True)
        out = {k: np.asarray(z[k], float) for k in ("t", "x", "y", "yaw")}
        for k in ("sigma_xy", "sigma_yaw"):
            if k in z.files:
                out[k] = np.asarray(z[k], float)
        return out
    raw = json.loads(p.read_text())
    return {k: np.asarray(raw[k], float) for k in ("t", "x", "y", "yaw")}


def attach_references(drive_dir, track_files, tolerance_s: float = 0.1) -> dict:
    """Write the matching track into each drive file as `ref_track`. A track matches a drive if its time range
    covers the drive's moving run (to within `tolerance_s`); the pads, where the robot is at rest, may be partly
    uncovered (the grid holds the first or last pose there). The tightest covering track wins. Returns
    {"attached": [...], "missing": [...]}."""
    tracks = []
    for f in track_files:
        tr = read_track_file(f)
        tracks.append((Path(f).name, tr, float(tr["t"][0]), float(tr["t"][-1])))
    report = {"attached": [], "missing": []}
    for p in sorted(Path(drive_dir).glob("*.json")):
        if p.name == "index.json":
            continue
        raw = json.loads(p.read_text())
        t0, t1 = raw["t0"], raw["t1"]
        r0, r1 = raw.get("run_t0", t0), raw.get("run_t1", t1)
        hits = [(name, tr) for name, tr, lo, hi in tracks if lo <= r0 + tolerance_s and hi >= r1 - tolerance_s]
        if not hits:
            raw.pop("ref_track", None)
            report["missing"].append(raw["name"])
        else:
            name, tr = min(hits, key=lambda h: h[1]["t"][-1] - h[1]["t"][0])   # the tightest cover
            m = (tr["t"] >= t0 - 1.0) & (tr["t"] <= t1 + 1.0)
            ref = {"source": name, **{k: tr[k][m].tolist() for k in ("t", "x", "y", "yaw")}}
            if "sigma_xy" in tr:
                ref["sigma_xy_median"] = float(np.median(tr["sigma_xy"][m]))
            raw["ref_track"] = ref
            report["attached"].append(raw["name"])
        p.write_text(json.dumps(raw))
    return report


def time_split(drives: list[Drive], fit_fraction: float = 0.5, embargo_s: float = EMBARGO_S):
    """Split in time order, balanced by driven distance: the earliest drives, up to `fit_fraction` of the
    metres, fit; drives that start within `embargo_s` of the end of the last fit drive are dropped from both
    sets; the rest hold out. Returns (fit, hold-out, dropped)."""
    drives = sorted(drives, key=lambda d: d.t_start)
    total = sum(d.dist for d in drives)
    done, k = 0.0, 0
    while k < len(drives) - 1 and done < fit_fraction * total:
        done += drives[k].dist
        k += 1
    fit = drives[:k]
    boundary = fit[-1].t_start + fit[-1].n * DT
    rest = drives[k:]
    hold = [d for d in rest if d.t_start >= boundary + embargo_s]
    dropped = [d for d in rest if d.t_start < boundary + embargo_s]
    if not fit or not hold:
        raise ValueError(f"split leaves {len(fit)} fit and {len(hold)} hold-out drives")
    return fit, hold, dropped


DERIVE_WINDOW_S = 10.0
DERIVE_MIN_CHORD_M = 2.0


def derive_scale(drives: list[Drive]) -> dict:
    """Odometry scale against the attached tracks, from windows of `DERIVE_WINDOW_S`.

    Linear: the sum of track chord lengths over the sum of odometry chord lengths of every window whose odometry
    chord is at least `DERIVE_MIN_CHORD_M`. Chords, not path lengths: noise of a few centimetres adds length to a
    path, and the chord of a 2 m or longer window carries it as a small relative error. The per-drive spread is
    reported so that a scale that holds for some drives only is visible.
    Yaw: the sum of track heading changes over the sum of odometry heading changes, in windows whose odometry turns
    at least 0.3 rad (same sign only counts through the sum). Informational: the fits do not use it by default.
    """
    step = int(round(DERIVE_WINDOW_S / DT))
    lin_t = lin_o = yaw_t = yaw_o = 0.0
    per_drive = []
    for d in drives:
        if d.track is None:
            continue
        dt_, do_ = 0.0, 0.0
        for a in range(0, d.n - step, step):
            b = a + step
            co = math.hypot(d.odom[0][b] - d.odom[0][a], d.odom[1][b] - d.odom[1][a])
            ct = math.hypot(d.track[0][b] - d.track[0][a], d.track[1][b] - d.track[1][a])
            if co >= DERIVE_MIN_CHORD_M:
                dt_ += ct
                do_ += co
            ho, ht = d.odom[2][b] - d.odom[2][a], d.track[2][b] - d.track[2][a]
            if abs(ho) >= 0.3:
                yaw_t += ht
                yaw_o += ho
        lin_t += dt_
        lin_o += do_
        if do_ > 0:
            per_drive.append(dt_ / do_)
    if not per_drive:
        raise ValueError("no drive has an attached track with a 10 s window that moves at least 2 m")
    return {"linear": lin_t / lin_o, "linear_drive_median": float(np.median(per_drive)),
            "linear_drive_p10_p90": [float(np.percentile(per_drive, 10)), float(np.percentile(per_drive, 90))],
            "metres": lin_o, "drives": len(per_drive), "yaw": (yaw_t / yaw_o) if abs(yaw_o) > 0 else None}
