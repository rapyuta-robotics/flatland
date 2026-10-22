"""Extraction on synthetic bags: stitched splits, cut drives, window padding, manual control, latched topics."""
import json

import numpy as np
import pytest

from flatland_base_model_tool import drives, stepper
from test_drives_fit import synth_drive, write_bag

REST = 250   # 5 s of rest rows


def rest_rows(t, n=REST):
    return [[t + 0.02 * k, 0, 0, 0, 0, 0] for k in range(n)]


def recording(t0=5000.0, seeds=(1, 2), gap_s=4.0, cut_end=False):
    """Odometry and commands of rest, drive, rest, drive, ... A drive without a tail ends the recording moving."""
    odom, cmd, t = [], [], t0
    starts = []
    for i, seed in enumerate(seeds):
        odom += rest_rows(t)
        starts.append(t + 5.0)
        last = cut_end and i == len(seeds) - 1
        p = synth_drive(f"x{i}", t + 5.0, seed, seconds=30.0, tail_s=0.0 if last else gap_s, last_v=1.2 if last else None)
        odom += p["odom"]
        cmd += p["cmd"]
        t += 5.0 + 30.0 + (0.0 if last else gap_s) + 0.5
    return odom, cmd, starts


def split(odom, cmd, at, tmp_path, names=("a_00.bag", "a_01.bag"), skip=0.0, **kw):
    """Cut a recording into two bag files at time `at`; `skip` seconds of odometry are lost at the boundary."""
    o1, o2 = [r for r in odom if r[0] < at], [r for r in odom if r[0] >= at + skip]
    c1, c2 = [r for r in cmd if r[0] < at], [r for r in cmd if r[0] >= at + skip]
    paths = [tmp_path / n for n in names]
    write_bag(paths[0], o1, c1, latched=kw.get("latched_first", ()), manual=kw.get("manual1"))
    write_bag(paths[1], o2, c2, latched=kw.get("latched_second", ()), manual=kw.get("manual2"))
    return paths


def test_split_in_the_middle_of_a_drive_is_stitched_not_cut(tmp_path):
    odom, cmd, starts = recording()
    paths = split(odom, cmd, starts[0] + 15.0, tmp_path)   # the boundary is inside the first drive
    index = drives.extract(paths, tmp_path / "d", manual_topic=None)
    (rec,) = index["recordings"]
    assert len(rec["files"]) == 2 and len(rec["boundaries"]) == 1
    assert rec["boundaries"][0]["header_gap_s"] == pytest.approx(0.02, abs=1e-6)
    assert len(rec["kept"]) == 2 and not rec["excluded"]
    d0 = json.loads((tmp_path / "d" / f"{rec['kept'][0]['name']}.json").read_text())
    t = np.array(d0["odom"])[:, 0]
    assert np.diff(t).max() < 0.021 and t[0] < starts[0] < t[-1] - 30   # one continuous drive across the boundary


def test_boundary_with_a_gap_is_not_stitched_and_cuts_the_drive(tmp_path):
    odom, cmd, starts = recording()
    paths = split(odom, cmd, starts[0] + 15.0, tmp_path, skip=1.0)   # 1 s of odometry lost: above the bound
    index = drives.extract(paths, tmp_path / "d", manual_topic=None)
    assert len(index["recordings"]) == 2
    assert [r["boundaries"] for r in index["recordings"]][0] == []
    cut = [e for r in index["recordings"] for e in r["excluded"] if e["reason"].startswith("cut")]
    assert len(cut) == 2 and all(e["edge_speed"] > 0.0 for e in cut)   # the drive is cut at both of its ends


def test_only_the_recording_ends_cut_a_drive(tmp_path):
    odom, cmd, starts = recording(cut_end=True)
    paths = split(odom, cmd, starts[0] + 15.0, tmp_path)
    index = drives.extract(paths, tmp_path / "d", manual_topic=None)
    (rec,) = index["recordings"]
    assert len(rec["kept"]) == 1
    assert [e["reason"] for e in rec["excluded"]] == ["cut by the start or end of the recording"]
    assert "cut: drive" in drives.edge_report(index)


def test_window_padding_stops_in_the_pause_between_drives(tmp_path):
    odom, cmd, starts = recording(gap_s=1.6)   # a 2.1 s pause between runs: the 2 s pads would overlap
    paths = split(odom, cmd, starts[0] + 15.0, tmp_path)
    index = drives.extract(paths, tmp_path / "d", manual_topic=None)
    kept = index["recordings"][0]["kept"]
    assert len(kept) == 2 and max(k["edge_speed"] for k in kept) < drives.MOVING_V


def test_latched_topics_only_in_the_first_split_are_carried_forward(tmp_path):
    odom, cmd, starts = recording()
    paths = split(odom, cmd, starts[0] + 15.0, tmp_path, latched_first=("/map", "/tf_static"))
    index = drives.extract(paths, tmp_path / "d", manual_topic=None)
    assert index["recordings"][0]["latched"] == {"/map": "a_00.bag", "/tf_static": "a_00.bag"}
    assert "MISSING" not in drives.edge_report(index)


def test_latched_topic_is_never_borrowed_from_another_recording(tmp_path):
    odom, cmd, starts = recording()
    paths = split(odom, cmd, starts[0] + 15.0, tmp_path, skip=1.0, latched_first=("/map",))
    index = drives.extract(paths, tmp_path / "d", manual_topic=None, latched=("/map",))
    first, second = index["recordings"]
    assert first["latched"] == {"/map": "a_00.bag"} and second["latched"] == {"/map": None}
    assert "MISSING" in drives.edge_report(index)


def test_manual_control_at_start_excludes_and_late_onset_trims(tmp_path):
    odom, cmd, starts = recording()
    t_a = starts[0] + 1.0    # manual already on when the first drive starts
    t_b = starts[1] + 15.0   # manual takes over in the middle of the second drive
    p = tmp_path / "m.bag"
    write_bag(p, odom, cmd, manual=[(starts[0] - 3.0, 0), (t_a - 3.0, 1), (t_a - 2.0, 0), (t_a - 1.5, 1),
                                    (starts[1] - 3.0, 0), (t_b, 1)])
    index = drives.extract([p], tmp_path / "d", manual_topic="/manual", manual_field="data")
    (rec,) = index["recordings"]
    reasons = [e["reason"] for e in rec["excluded"]]
    assert reasons == ["manual control at the start"]
    (kept,) = rec["kept"]
    raw = json.loads((tmp_path / "d" / f"{kept['name']}.json").read_text())
    assert raw["score_end_s"] == pytest.approx(t_b - raw["t0"], abs=1e-6)


def test_early_manual_takeover_leaves_too_little_motion(tmp_path):
    odom, cmd, starts = recording(seeds=(1,))
    p = tmp_path / "m.bag"
    write_bag(p, odom, cmd, manual=[(starts[0] - 3.0, 0), (starts[0] + 2.0, 1)])
    index = drives.extract([p], tmp_path / "d", manual_topic="/manual", manual_field="data")
    assert [e["reason"] for e in index["recordings"][0]["excluded"]] == ["manual control takes over early"]


def test_attach_reference_matches_by_time_and_drives_prefer_the_track(tmp_path):
    odom, cmd, starts = recording()
    p = tmp_path / "r.bag"
    write_bag(p, odom, cmd)
    index = drives.extract([p], tmp_path / "d", manual_topic=None)
    names = [k["name"] for k in index["recordings"][0]["kept"]]
    # a "track" that is the odometry shifted by 1 m in x, covering only the first drive
    first = json.loads((tmp_path / "d" / f"{names[0]}.json").read_text())
    o = np.array(first["odom"])
    np.savez(tmp_path / "trk.npz", t=o[:, 0], x=o[:, 1] + 1.0, y=o[:, 2], yaw=o[:, 3], sigma_xy=np.full(len(o), 0.04))
    r = drives.attach_references(tmp_path / "d", [tmp_path / "trk.npz"])
    assert r["attached"] == [names[0]] and r["missing"] == [names[1]]
    default = drives.load_dir(tmp_path / "d")
    assert all(d.reference == "odom" and d.track is not None or d.name == names[1] for d in default)   # odometry is the default
    tracked = drives.load_dir(tmp_path / "d", reference="track")
    assert [d.name for d in tracked] == [names[0]] and drives.last_skipped == [names[1]]
    d = tracked[0]
    assert d.track is not None and np.allclose(d.x, d.track[0]) and np.allclose(d.odom[0], d.track[0])  # start frame
    with pytest.raises(ValueError):
        drives.prepare(json.loads((tmp_path / "d" / f"{names[1]}.json").read_text()), reference="track")


def test_odom_scale_scales_displacement_not_heading(tmp_path):
    odom, cmd, _ = recording(seeds=(3,))
    p = tmp_path / "s.bag"
    write_bag(p, odom, cmd)
    drives.extract([p], tmp_path / "d", manual_topic=None)
    one = drives.load_dir(tmp_path / "d")[0]
    scaled = drives.load_dir(tmp_path / "d", odom_scale=0.9)[0]
    assert np.allclose(scaled.x, 0.9 * one.x) and np.allclose(scaled.h, one.h)


def test_derive_scale_recovers_a_known_odometry_error(tmp_path):
    """Tracks that are the odometry scaled by 0.99 in displacement: the derivation returns 0.99 and a yaw scale of 1."""
    odom, cmd, starts = recording(seeds=(1, 2, 3))
    p = tmp_path / "r.bag"
    write_bag(p, odom, cmd)
    index = drives.extract([p], tmp_path / "d", manual_topic=None)
    files = []
    for k in index["recordings"][0]["kept"]:
        raw = json.loads((tmp_path / "d" / f"{k['name']}.json").read_text())
        o = np.array(raw["odom"])
        x0, y0 = o[0, 1], o[0, 2]
        np.savez(tmp_path / f"{k['name']}.npz", t=o[:, 0], x=x0 + 0.99 * (o[:, 1] - x0), y=y0 + 0.99 * (o[:, 2] - y0), yaw=o[:, 3])
        files.append(tmp_path / f"{k['name']}.npz")
    assert drives.attach_references(tmp_path / "d", files)["missing"] == []
    r = drives.derive_scale(drives.load_dir(tmp_path / "d", reference="track"))
    assert r["linear"] == pytest.approx(0.99, abs=1e-6) and r["yaw"] == pytest.approx(1.0, abs=1e-6)
    scaled = drives.load_dir(tmp_path / "d", odom_scale=r["linear"])
    assert np.allclose(scaled[0].x, drives.load_dir(tmp_path / "d", reference="track")[0].track[0], atol=1e-9)


def test_long_horizon_prefers_the_right_model(tmp_path):
    from flatland_base_model_tool import longhorizon
    from test_drives_fit import TRUE
    ds = []
    for i in range(3):
        raw = synth_drive(f"s{i}", 1000.0 + 100 * i, i)
        o = np.array(raw["odom"])
        raw["ref_track"] = {"t": o[:, 0].tolist(), "x": o[:, 1].tolist(), "y": o[:, 2].tolist(), "yaw": o[:, 3].tolist()}
        ds.append(drives.prepare(raw, reference="track"))
    good = longhorizon.summary(ds, stepper.vector(TRUE))
    bad = longhorizon.summary(ds, stepper.vector(stepper.IDENTITY))
    assert good["final_position_error_m"]["mean"] < 0.01 < bad["final_position_error_m"]["mean"]
    assert good["distance_ratio"]["by_metres"] == pytest.approx(1.0, abs=1e-3)
    with pytest.raises(ValueError):
        longhorizon.check(drives.prepare(synth_drive("n", 1.0, 1)), stepper.vector(TRUE))
