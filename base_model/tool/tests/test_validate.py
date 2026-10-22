import json

import numpy as np
import pytest

from flatland_base_model_tool import bounds, cli, drives, scoring, stepper, validate
from limits import LIMITED
from test_drives_fit import TRUE, synth_drive


def make_dir(tmp_path, n=4):
    d = tmp_path / "drives"
    d.mkdir()
    for i in range(n):
        (d / f"s_{i:02d}.json").write_text(json.dumps(synth_drive(f"s_{i:02d}", 1000.0 + 100 * i, i)))
    return d


def write_track(path, drive, track, shift=0.0):
    x, y, h = track
    t = np.arange(drive.n) * drives.DT
    path.write_text(json.dumps({"name": drive.name, "t": t.tolist(), "x": (x + shift * t).tolist(), "y": y.tolist(),
                                "yaw": h.tolist()}))


def model_tracks(tmp_path, name, ds, values, hop=0, **kw):
    out = tmp_path / name
    out.mkdir()
    p = stepper.vector(values)
    for d in ds:
        write_track(out / f"{d.name}.json", d, scoring.offline_track(d, p, hop=hop), **kw)
    return out


def test_plugin_equal_to_offline_passes_with_the_right_hop(tmp_path):
    ds = drives.load_dir(make_dir(tmp_path))
    plugin = validate.track_dir(model_tracks(tmp_path, "plugin", ds, TRUE, hop=1), ds)
    p = stepper.vector(TRUE)
    right = validate.plugin_vs_offline(ds, plugin, p, hop=1)
    assert right.passed and right.detail["max"] < 1e-9
    wrong = validate.plugin_vs_offline(ds, plugin, p, hop=0)
    assert not wrong.passed and wrong.detail["mean"] > 100 * validate.PLUGIN_VS_OFFLINE_MEAN_MAX


def test_hop_is_measured_from_the_identity_model(tmp_path):
    ds = drives.load_dir(make_dir(tmp_path))
    for hop in (0, 1, 2):
        ident = validate.track_dir(model_tracks(tmp_path, f"id{hop}", ds, stepper.IDENTITY, hop=hop), ds)
        r = validate.estimate_hop(ds, ident)
        assert r.passed and r.detail["hop_steps"] == hop and r.detail["next_best_mean"] > r.detail["mean"]


def test_gate_fails_for_a_different_model(tmp_path):
    ds = drives.load_dir(make_dir(tmp_path))
    plugin = validate.track_dir(model_tracks(tmp_path, "plugin", ds, TRUE), ds)
    assert not validate.plugin_vs_offline(ds, plugin, stepper.vector(LIMITED)).passed


def test_fast_vs_slow_uses_each_speeds_own_hop(tmp_path):
    ds = drives.load_dir(make_dir(tmp_path))
    p = stepper.vector(TRUE)
    slow = validate.track_dir(model_tracks(tmp_path, "slow", ds, TRUE, hop=1), ds)
    fast = validate.track_dir(model_tracks(tmp_path, "fast", ds, TRUE, hop=2), ds)
    r = validate.fast_vs_slow(ds, slow, fast, p, 1, 2)
    assert [x.passed for x in r] == [True, True, True] and r[2].detail["mean"] > 0
    assert not validate.fast_vs_slow(ds, slow, fast, p, 1, 1)[1].passed   # a wrong hop at fast speed is caught


def test_negative_control_direction_and_bootstrap(tmp_path):
    ds = drives.load_dir(make_dir(tmp_path, 6))
    ref = {d.name: d.odom for d in ds}
    good = validate.track_dir(model_tracks(tmp_path, "good", ds, TRUE), ds)
    stock = validate.track_dir(model_tracks(tmp_path, "stock", ds, stepper.IDENTITY), ds)
    ok = validate.negative_control(ds, ref, stock, good, "odom")
    assert ok.passed and ok.detail["fraction_baseline_worse"] == 1.0 and ok.detail["ci_low"] > 0
    swapped = validate.negative_control(ds, ref, good, stock, "odom")
    assert not swapped.passed and swapped.detail["ci_high"] < 0


def test_bootstrap_is_deterministic():
    v = np.array([0.1, 0.2, -0.05, 0.3, 0.15])
    assert validate.bootstrap_mean_ci(v) == validate.bootstrap_mean_ci(v)


def test_missing_track_is_an_error(tmp_path):
    ds = drives.load_dir(make_dir(tmp_path))
    empty = tmp_path / "none"
    empty.mkdir()
    with pytest.raises(FileNotFoundError, match="s_00"):
        validate.track_dir(empty, ds)


def test_bounds_cover_the_modelled_effect_and_scale_with_it():
    """The constants in validate are the rounded-up derivation of `bounds`, on a drive set with long drives."""
    ds = drives.load_dir(make_dir_cached())
    p = stepper.vector(TRUE)
    d = bounds.derive(ds, p)
    assert d["float32_mean"] < validate.PLUGIN_VS_OFFLINE_MEAN_MAX / 5      # margin is real
    assert d["float32_max"] < validate.PLUGIN_VS_OFFLINE_DRIVE_MAX / 5
    assert validate.PLUGIN_VS_OFFLINE_MEAN_MAX < 1e-3


_CACHE = {}


def make_dir_cached():
    import tempfile
    from pathlib import Path
    if "d" not in _CACHE:
        _CACHE["d"] = make_dir(Path(tempfile.mkdtemp()))
    return _CACHE["d"]


def test_cli_exit_codes(tmp_path, capsys):
    d = make_dir(tmp_path)
    ds = drives.load_dir(d)
    plugin = model_tracks(tmp_path, "plugin", ds, TRUE, hop=1)
    stock = model_tracks(tmp_path, "stock", ds, stepper.IDENTITY, hop=1)
    pf = tmp_path / "p.yaml"
    from flatland_base_model_tool import params
    params.save(params.make("t", TRUE, 0.02, {}), pf)
    split = ["--set", "all", "--embargo-s", "0"]
    assert cli.main(["validate", "plugin-vs-offline", str(pf), str(d), str(plugin), "--hop", "1", *split]) == 0
    assert cli.main(["validate", "plugin-vs-offline", str(pf), str(d), str(plugin), "--hop", "0", *split]) == 1
    assert cli.main(["validate", "hop", str(d), str(stock), *split]) == 0
    assert cli.main(["validate", "negative-control", str(d), str(plugin), str(stock), *split]) == 0
    assert cli.main(["validate", "negative-control", str(d), str(stock), str(plugin), *split]) == 1
    assert "PASS replay hop" in capsys.readouterr().out


def test_playback_schedule_is_on_the_step_grid_and_matches_events_to_steps(tmp_path):
    from flatland_base_model_tool import playback
    ds = drives.load_dir(make_dir(tmp_path))
    doc = playback.schedule(ds)
    assert doc["version"] == 1 and [d["name"] for d in doc["drives"]] == [d.name for d in ds]
    end = playback.FIRST_START_S - playback.GAP_S
    for sched, d in zip(doc["drives"], ds):
        assert sched["start_s"] >= end + playback.GAP_S - 1e-9
        end = sched["start_s"] + sched["length_s"]
        for v in (sched["start_s"], sched["length_s"]):
            assert abs(v / drives.DT - round(v / drives.DT)) < 1e-9
        assert sched["length_s"] >= d.n * drives.DT
        # the plugin's lookup (last event with time <= step start + 1e-9) is the stepper's events_to_steps
        ev = sched["events"]
        cv, cw = np.zeros(d.n), np.zeros(d.n)
        stepper.events_to_steps(d.ev_t, d.ev_v, d.ev_w, drives.DT, d.n, cv, cw)
        for k in range(0, d.n, 7):
            last = [e for e in ev if e[0] <= k * drives.DT + 1e-9]
            assert (last[-1][1] if last else 0.0) == cv[k] and (last[-1][2] if last else 0.0) == cw[k]


def test_playback_cli_and_hop_information(tmp_path, capsys):
    d = make_dir(tmp_path)
    out = tmp_path / "sched.json"
    assert cli.main(["playback-schedule", str(d), "--out", str(out), "--set", "all", "--embargo-s", "0"]) == 0
    assert json.loads(out.read_text())["drives"]
    ds = drives.load_dir(d)
    bad = model_tracks(tmp_path, "jitter", ds, TRUE, hop=1)   # not the identity model: no hop explains it
    args = ["validate", "hop", str(d), str(bad), "--set", "all", "--embargo-s", "0"]
    assert cli.main(args) == 1
    assert cli.main([*args, "--information"]) == 0
