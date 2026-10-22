"""Hold stretches (the platform ignores commands) and the hold rule of the extractor."""
import json

import numpy as np
import pytest

from flatland_base_model_tool import drives, holds
from test_drives_fit import synth_drive, write_bag
from test_stitch import recording, split

HOLD_DEF = "bool engaged\n"
HOLD_MSG = "test_msgs/msg/Hold"
S_FLAG = "bool:/example/hold_flag:engaged"     # a custom message with a boolean field
S_BOOL = "bool:/example/hold_bool:data"             # std_msgs/Bool
S_ACTIVE = "active:/example/task_status"            # actionlib GoalStatusArray
SOURCES = [S_FLAG, S_BOOL, S_ACTIVE]


def write_hold_bag(path, odom, cmd, flag=(), bool_flag=(), task=()):
    """A bag with /odom and /cmd_vel plus hold topics. flag: [(t, engaged)], bool_flag: [(t, bool)],
    task: [(t, [status ints])]."""
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    import test_drives_fit as t

    ts = get_typestore(Stores.ROS1_NOETIC)
    ts.register(get_types_from_msg(HOLD_DEF, HOLD_MSG))
    T = ts.types
    write_bag(path, odom, cmd)       # odom and cmd first; the hold topics are appended by a second pass below
    raw = path.read_bytes()
    path.unlink()
    # rebuild in one writer so the file is a single valid bag
    with Writer(path) as w:
        _copy_all(w, ts, odom, cmd)
        if flag:
            c = w.add_connection("/example/hold_flag", HOLD_MSG, typestore=ts)
            for tt, rel in flag:
                w.write(c, int(tt * 1e9), ts.serialize_ros1(T[HOLD_MSG](rel), HOLD_MSG))
        if bool_flag:
            c = w.add_connection("/example/hold_bool", "std_msgs/msg/Bool", typestore=ts)
            for tt, b in bool_flag:
                w.write(c, int(tt * 1e9), ts.serialize_ros1(T["std_msgs/msg/Bool"](b), "std_msgs/msg/Bool"))
        if task:
            c = w.add_connection("/example/task_status", "actionlib_msgs/msg/GoalStatusArray", typestore=ts)
            for tt, states in task:
                hdr = T["std_msgs/msg/Header"](0, T["builtin_interfaces/msg/Time"](int(tt), int((tt % 1) * 1e9)), "")
                gid = T["actionlib_msgs/msg/GoalID"](T["builtin_interfaces/msg/Time"](0, 0), "g")
                lst = [T["actionlib_msgs/msg/GoalStatus"](gid, s, "") for s in states]
                w.write(c, int(tt * 1e9), ts.serialize_ros1(T["actionlib_msgs/msg/GoalStatusArray"](hdr, lst),
                                                            "actionlib_msgs/msg/GoalStatusArray"))


def _copy_all(w, ts, odom, cmd):
    import math
    T = ts.types
    co = w.add_connection("/odom", "nav_msgs/msg/Odometry", typestore=ts)
    cc = w.add_connection("/cmd_vel", "geometry_msgs/msg/Twist", typestore=ts)
    v3 = T["geometry_msgs/msg/Vector3"]
    for row in odom:
        sec = int(row[0])
        hdr = T["std_msgs/msg/Header"](0, T["builtin_interfaces/msg/Time"](sec, int((row[0] - sec) * 1e9)), "odom")
        q = T["geometry_msgs/msg/Quaternion"](0.0, 0.0, math.sin(row[3] / 2), math.cos(row[3] / 2))
        pose = T["geometry_msgs/msg/PoseWithCovariance"](
            T["geometry_msgs/msg/Pose"](T["geometry_msgs/msg/Point"](row[1], row[2], 0.0), q), np.zeros(36))
        tw = T["geometry_msgs/msg/TwistWithCovariance"](
            T["geometry_msgs/msg/Twist"](v3(row[4], 0.0, 0.0), v3(0.0, 0.0, row[5])), np.zeros(36))
        w.write(co, int(row[0] * 1e9) + 5_000_000,
                ts.serialize_ros1(T["nav_msgs/msg/Odometry"](hdr, "base", pose, tw), "nav_msgs/msg/Odometry"))
    for t_, v, wz in cmd:
        w.write(cc, int(t_ * 1e9),
                ts.serialize_ros1(T["geometry_msgs/msg/Twist"](v3(v, 0.0, 0.0), v3(0.0, 0.0, wz)), "geometry_msgs/msg/Twist"))


def test_read_holds_from_all_three_sources(tmp_path):
    odom, cmd, starts = recording(seeds=(1,))
    t0 = starts[0] - 4.0
    p = tmp_path / "h.bag"
    write_hold_bag(p, odom, cmd,
                   flag=[(t0 + 0.1 * k, 1 <= k <= 20) for k in range(40)],                 # flag true for 2 s
                   bool_flag=[(t0 + 10.0, True), (t0 + 10.5, False)],               # a 0.5 s true state
                   task=[(t0 + 20.0 + 0.2 * k, [1] if k < 10 else [3]) for k in range(20)])  # ACTIVE for 2 s, then SUCCEEDED
    h = holds.read_holds(p, SOURCES)
    iv = h["stretches"]
    assert len(iv) == 3
    (a0, a1), (b0, b1), (c0, c1) = iv
    assert a0 == pytest.approx(t0 + 0.1, abs=1e-3) and a1 == pytest.approx(t0 + 2.1, abs=0.11)
    assert b0 == pytest.approx(t0 + 10.0, abs=1e-3) and b1 == pytest.approx(t0 + 10.5, abs=0.51)
    assert c0 == pytest.approx(t0 + 20.0, abs=1e-3) and c1 == pytest.approx(t0 + 22.0, abs=0.21)
    assert h["topics_present"] == {"/example/hold_flag": True, "/example/hold_bool": True, "/example/task_status": True}


def test_read_holds_merges_close_stretches_and_ignores_other_states(tmp_path):
    odom, cmd, starts = recording(seeds=(1,))
    t0 = starts[0]
    p = tmp_path / "m.bag"
    write_hold_bag(p, odom, cmd, task=[(t0 + 0.2 * k, [1] if k in (0, 1, 2, 5, 6, 7) else [3, 4]) for k in range(15)])
    h = holds.read_holds(p, SOURCES)
    assert len(h["stretches"]) == 1                     # a 0.4 s pause between two ACTIVE runs merges (gap <= 1 s)
    p2 = tmp_path / "n.bag"
    write_hold_bag(p2, odom, cmd, task=[(t0 + 0.2 * k, [2, 3, 4]) for k in range(10)])
    assert holds.read_holds(p2, SOURCES)["stretches"] == []      # preempted, succeeded, aborted: not ACTIVE


def test_missing_hold_topics_are_reported_not_silent(tmp_path):
    odom, cmd, _ = recording(seeds=(1,))
    p = tmp_path / "x.bag"
    write_bag(p, odom, cmd)
    h = holds.read_holds(p, SOURCES)
    assert h["stretches"] == [] and h["topics_present"] == {"/example/hold_flag": False, "/example/hold_bool": False, "/example/task_status": False}


def test_extractor_records_hold_intervals_instead_of_dropping_drives(tmp_path):
    """Stretches are long and usually touch the pads of a drive: the extractor keeps the drive and records the
    time stretches (each extended by the settle time) that overlap its window, relative to the window start."""
    odom, cmd, starts = recording(seeds=(1, 2, 3))
    p = tmp_path / "e.bag"
    # a hold stretch during the second drive; a stretch that ends 2.5 s before the third drive starts
    write_hold_bag(p, odom, cmd, flag=[(starts[1] + 10.0 + 0.1 * k, k < 20) for k in range(30)],
                   task=[(starts[2] - 12.5 + 0.2 * k, [1] if k < 50 else [3]) for k in range(70)])
    index = drives.extract([p], tmp_path / "d", manual_topic=None, hold_sources=SOURCES, hold_settle_s=4.0)
    (rec,) = index["recordings"]
    assert len(rec["kept"]) == 3 and not rec["excluded"] and rec["holds"]["stretches"] == 2
    raws = [json.loads((tmp_path / "d" / f"{k['name']}.json").read_text()) for k in rec["kept"]]
    assert "holds" not in raws[0] or raws[0]["holds"] == []                    # the first drive has no stretch near it
    iv = raws[1]["holds"][0]                                                     # the flag, inside the second window
    assert len(raws[1]["holds"]) == 2                                            # and a second stretch that starts at its end
    assert iv[0] == pytest.approx(starts[1] + 10.0 - raws[1]["t0"], abs=1e-6)
    assert iv[1] == pytest.approx(starts[1] + 12.0 + 4.0 - raws[1]["t0"], abs=0.11)   # end of the last true message + settle
    iv3 = raws[2]["holds"][-1]                                                   # the stretch and its settle time reach into the run
    assert iv3[1] > starts[2] - raws[2]["t0"]
    off = drives.extract([p], tmp_path / "d2", manual_topic=None, hold_sources=(), hold_settle_s=None)
    assert all("holds" not in json.loads(f.read_text()) for f in (tmp_path / "d2").glob("*_*.json") if f.name != "index.json")


def test_settle_time_moves_the_end_of_the_excluded_interval(tmp_path):
    odom, cmd, starts = recording(seeds=(1, 2))
    p = tmp_path / "s.bag"
    write_hold_bag(p, odom, cmd, flag=[(starts[1] - 4.0 + 0.1 * k, k < 15) for k in range(25)])
    ends = {}
    for settle in (1.0, 4.0):
        index = drives.extract([p], tmp_path / f"d{settle}", manual_topic=None, hold_sources=SOURCES, hold_settle_s=settle)
        k = index["recordings"][0]["kept"][1]["name"]
        ends[settle] = json.loads((tmp_path / f"d{settle}" / f"{k}.json").read_text())["holds"][0][1]
    assert ends[4.0] - ends[1.0] == pytest.approx(3.0, abs=1e-6)


def test_excluded_windows_do_not_count_in_the_score(tmp_path):
    from flatland_base_model_tool import scoring, stepper
    from test_drives_fit import TRUE
    raw = synth_drive("w", 1000.0, 3, seconds=40.0, tail_s=4.0)
    clean = drives.prepare(raw)
    broken = dict(raw)
    o = np.array(raw["odom"])
    o[int(20 / drives.DT):, 1] += 3.0                           # the real robot is shoved 3 m at t = 20 s
    broken["odom"] = o.tolist()
    p = stepper.vector(TRUE)
    bad = scoring.objective(scoring.run(scoring.pack([drives.prepare(broken)]), p))
    masked = drives.prepare({**broken, "holds": [[19.5, 20.5 + 3.0]]})
    good = scoring.objective(scoring.run(scoring.pack([masked]), p))
    ref = scoring.objective(scoring.run(scoring.pack([clean]), p))
    assert masked.exclude is not None and masked.exclude.sum() > 100
    assert bad > 3 * good and good == pytest.approx(ref, abs=0.02)     # windows with the shove no longer count


def test_a_drive_with_no_moving_window_left_is_dropped_and_reported(tmp_path):
    raw = synth_drive("w", 1000.0, 3, seconds=20.0, tail_s=2.0)
    (tmp_path / "a.json").write_text(json.dumps({**raw, "name": "a", "holds": [[-5.0, 100.0]]}))
    raw2 = synth_drive("v", 2000.0, 4, seconds=20.0, tail_s=2.0)
    (tmp_path / "b.json").write_text(json.dumps({**raw2, "name": "b", "holds": [[0.0, 3.0]]}))
    ds = drives.load_dir(tmp_path)
    assert [d.name for d in ds] == ["b"] and drives.last_skipped == ["a"]
    assert [d.name for d in drives.load_dir(tmp_path, holds=False)] == ["a", "b"]      # rule off at load: nothing dropped


def test_holds_are_carried_across_stitched_splits(tmp_path):
    odom, cmd, starts = recording(seeds=(1, 2))
    at = starts[0] + 15.0
    flag = [(starts[1] + 5.0 + 0.1 * k, k < 30) for k in range(40)]
    o1, o2 = [r for r in odom if r[0] < at], [r for r in odom if r[0] >= at]
    c1, c2 = [r for r in cmd if r[0] < at], [r for r in cmd if r[0] >= at]
    a, b = tmp_path / "r_00.bag", tmp_path / "r_01.bag"
    write_hold_bag(a, o1, c1)
    write_hold_bag(b, o2, c2, flag=flag)
    index = drives.extract([a, b], tmp_path / "d", manual_topic=None, hold_sources=SOURCES, hold_settle_s=4.0)
    (rec,) = index["recordings"]
    assert rec["holds"]["topics_present"]["/example/hold_flag"] is True      # present in any file of the recording
    assert len(rec["kept"]) == 2 and rec["holds"]["stretches"] == 1


def test_settle_curve_finds_a_known_settle_time():
    rng = np.random.default_rng(0)
    normal = np.abs(rng.normal(0.0, 0.05, 20000)) + 0.02
    events = []
    for e in range(40):
        t = np.arange(0.0, 20.0, 0.02)
        delta = np.abs(rng.normal(0.0, 0.05, len(t))) + 0.02 + 0.5 * np.exp(-t / 1.5) * (t < 4.0)
        events.append((t, delta))
    out = holds.settle_curve(events, normal, bin_s=0.5, max_s=20.0, n_boot=500, seed=0)
    assert 2.5 <= out["settle_s"] <= 5.0
    assert out["medians"][0] > out["bands"][0] and out["medians"][-1] <= out["bands"][-1]
    flat = [(t, np.abs(rng.normal(0.0, 0.05, len(t))) + 0.02) for t, _ in events]
    assert holds.settle_curve(flat, normal, 0.5, 20.0, 500, 0)["settle_s"] == 0.0
    never = [(t, d + 1.0) for t, d in events]
    assert holds.settle_curve(never, normal, 0.5, 20.0, 500, 0)["settle_s"] == 20.0
