import json
import math

import numpy as np
import pytest

from flatland_base_model_tool import cli, drives, fit, params, scoring, stepper
from limits import LIMITED, TRUE_PARAMS

TRUE = TRUE_PARAMS


def synth_drive(name, t0, seed, values=TRUE, seconds=40.0, tail_s=0.0, last_v=None):
    """A drive whose odometry is the reference stepper's own track for random piecewise-constant commands,
    followed by `tail_s` seconds of zero command."""
    rng = np.random.default_rng(seed)
    n = int((seconds + tail_s) / drives.DT)
    cv, cw = np.zeros(n), np.zeros(n)
    for a in range(0, int(seconds / drives.DT), 40):
        cv[a:a + 40] = rng.uniform(-0.3, 1.2)
        cw[a:a + 40] = rng.uniform(-1.2, 1.2)
    if last_v is not None:
        cv[int(seconds / drives.DT) - 40:int(seconds / drives.DT)] = last_v
    ev, ew = stepper.run(stepper.vector(values), cv, cw, drives.DT)
    x, y, h = np.zeros(n + 1), np.zeros(n + 1), np.zeros(n + 1)
    stepper.integrate(ev, ew, drives.DT, x, y, h)
    t = t0 + np.arange(n) * drives.DT
    cmd = [[float(t[k]), float(cv[k]), float(cw[k])] for k in range(n)]
    odom = [[float(t[k]), float(x[k]), float(y[k]), float(h[k]), float(ev[k]), float(ew[k])] for k in range(n)]
    return {"name": name, "t0": float(t0), "t1": float(t[-1]), "cmd": cmd, "odom": odom}


@pytest.fixture(scope="module")
def drive_dir(tmp_path_factory):
    d = tmp_path_factory.mktemp("drives")
    for i in range(4):
        (d / f"s_{i:02d}.json").write_text(json.dumps(synth_drive(f"s_{i:02d}", 1000.0 + 100 * i, i)))
    return d


def test_true_parameters_score_near_zero_and_the_limits_only_model_does_not(drive_dir):
    ds = drives.load_dir(drive_dir)
    packed = scoring.pack(ds)
    # the stepper's own track starts one step "ahead" of the grid reference only by interpolation error
    true = scoring.objective(scoring.run(packed, stepper.vector(TRUE)))
    firm = scoring.objective(scoring.run(packed, stepper.vector(LIMITED)))
    assert true < 0.02
    assert firm > 3 * true


def test_time_split_orders_by_time(drive_dir):
    ds = drives.load_dir(drive_dir)
    a, b, dropped = drives.time_split(ds, 0.5, embargo_s=0.0)
    assert not dropped and [d.name for d in a + b] == ["s_00", "s_01", "s_02", "s_03"]   # fit is the earlier part
    assert sum(d.dist for d in a) >= 0.5 * sum(d.dist for d in ds) > sum(d.dist for d in a[:-1])
    # drives start 100 s apart and last ~40 s: a 50 s embargo drops nothing, a 75 s one leaves no hold-out
    assert not drives.time_split(ds, 0.5, embargo_s=50.0)[2]
    with pytest.raises(ValueError, match="hold-out"):
        drives.time_split(ds, 0.5, embargo_s=75.0)


def test_fit_picks_seed_on_fit_only_and_ships_whole_step_delay(drive_dir):
    ds = drives.load_dir(drive_dir)
    fit_d, hold_d, _ = drives.time_split(ds, 0.5, embargo_s=0.0)
    r = fit.fit(fit_d, hold_d, seeds=(1, 2), n_trials=60, jobs=1, base={"jerk_lin": 20.0, "v_max": 1.5})
    assert r.seed == min(r.seeds, key=lambda s: s.fit_objective).seed
    assert r.fit_objective == pytest.approx(min(s.fit_objective for s in r.seeds))
    assert abs(r.values["delay_s"] / drives.DT - round(r.values["delay_s"] / drives.DT)) < 1e-9
    assert r.values["jerk_lin"] == 20.0 and r.values["v_max"] == 1.5 and r.values["w_max"] == float("inf")
    assert r.holdout_objective < scoring.objective(
        scoring.run(scoring.pack(hold_d), stepper.vector(stepper.IDENTITY)))


def test_fit_cli_writes_valid_file(drive_dir, tmp_path, capsys):
    out = tmp_path / "p.yaml"
    rc = cli.main(["fit", str(drive_dir), "--out", str(out), "--name", "t", "--robot", "r", "--site", "s",
                   "--platform-version", "v1", "--seeds", "1", "--trials", "20", "--jobs", "1", "--embargo-s", "0"])
    assert rc == 0
    doc = params.load(out)
    prov = doc["provenance"]
    assert prov["fit_drives"] + prov["holdout_drives"] == 4 and prov["optuna_version"] == "5.0.0"
    assert prov["chosen_seed"] == 1 and doc["calibrated_step_s"] == 0.02
    assert cli.main(["score", str(out), str(drive_dir)]) == 0
    assert "hold-out" in capsys.readouterr().out


def test_metrics_are_zero_for_identical_tracks():
    n = 400
    t = np.arange(n) * 0.02
    x, y, h = t * 1.0, np.zeros(n), np.zeros(n)
    out = np.zeros(6)
    scoring.metrics(x, y, h, x, y, h, np.zeros(len(x), np.uint8), out)
    assert np.all(out[:5] == 0.0) and out[5] > 0


def write_bag(path, odom, cmd, manual=None, latched=()):
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS1_NOETIC)
    T = ts.types
    with Writer(path) as w:
        co = w.add_connection("/odom", "nav_msgs/msg/Odometry", typestore=ts)
        cc = w.add_connection("/cmd_vel", "geometry_msgs/msg/Twist", typestore=ts)
        for row in odom:
            sec = int(row[0])
            hdr = T["std_msgs/msg/Header"](0, T["builtin_interfaces/msg/Time"](sec, int((row[0] - sec) * 1e9)), "odom")
            q = T["geometry_msgs/msg/Quaternion"](0.0, 0.0, math.sin(row[3] / 2), math.cos(row[3] / 2))
            pose = T["geometry_msgs/msg/PoseWithCovariance"](
                T["geometry_msgs/msg/Pose"](T["geometry_msgs/msg/Point"](row[1], row[2], 0.0), q), np.zeros(36))
            v3 = T["geometry_msgs/msg/Vector3"]
            tw = T["geometry_msgs/msg/TwistWithCovariance"](
                T["geometry_msgs/msg/Twist"](v3(row[4], 0.0, 0.0), v3(0.0, 0.0, row[5])), np.zeros(36))
            m = T["nav_msgs/msg/Odometry"](hdr, "base", pose, tw)
            w.write(co, int(row[0] * 1e9) + 5_000_000, ts.serialize_ros1(m, "nav_msgs/msg/Odometry"))
        for topic in latched:
            lc = w.add_connection(topic, "std_msgs/msg/String", typestore=ts, latching=1)
            w.write(lc, int(odom[0][0] * 1e9), ts.serialize_ros1(T["std_msgs/msg/String"](topic), "std_msgs/msg/String"))
        if manual:
            cm = w.add_connection("/manual", "std_msgs/msg/Int32", typestore=ts)
            for t, state in manual:
                w.write(cm, int(t * 1e9), ts.serialize_ros1(T["std_msgs/msg/Int32"](state), "std_msgs/msg/Int32"))
        for t, v, wz in cmd:
            v3 = T["geometry_msgs/msg/Vector3"]
            m = T["geometry_msgs/msg/Twist"](v3(v, 0.0, 0.0), v3(0.0, 0.0, wz))
            w.write(cc, int(t * 1e9), ts.serialize_ros1(m, "geometry_msgs/msg/Twist"))
