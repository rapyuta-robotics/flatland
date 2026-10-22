"""Wheel-space limit option of the reference stepper (`wheel_base_m` > 0).

The command is converted to wheel speeds (left = v - w b, right = v + w b); each wheel is rate limited (speeding up
uses a_lin_up, slowing down a_lin_dn, by the wheel's own direction of travel, with jerk_lin); the wheels are
converted back. `a_ang` is not used in this mode. Lags, delay, yaw scale, speed caps on the command, and the zero snap
are unchanged.
"""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from flatland_base_model_tool import params, stepper, vectors
from limits import LIMITED

B = 0.2
DT = 0.02
WHEEL_VECTORS = Path(__file__).resolve().parents[2] / "test_vectors" / "stepper_wheel.json"


def run(values, v, w, dt=DT):
    return stepper.run(stepper.vector({**stepper.IDENTITY, **values}), v, w, dt)


def test_option_is_optional_and_off_by_default():
    assert stepper.vector(LIMITED)[stepper.P_WHEEL] == 0.0
    stepper.validate_params({**LIMITED, "wheel_base_m": B})
    for bad in (-0.1, math.nan, math.inf):
        with pytest.raises(ValueError, match="wheel_base_m"):
            stepper.validate_params({**LIMITED, "wheel_base_m": bad})


def test_parameter_file_accepts_and_roundtrips_the_option(tmp_path):
    doc = params.make("w", {**LIMITED, "wheel_base_m": B}, 0.02, {})
    assert doc["parameters"]["wheel_base_m"] == B
    path = tmp_path / "w.yaml"
    params.save(doc, path)
    assert params.load(path)["parameters"]["wheel_base_m"] == B
    assert "wheel_base_m" not in params.make("n", LIMITED, 0.02, {})["parameters"]


def test_unlimited_wheel_mode_is_the_identity():
    rng = np.random.default_rng(3)
    v, w = rng.uniform(-0.5, 1.5, 300), rng.uniform(-3, 3, 300)
    ov, ow = run({"wheel_base_m": B}, v, w)
    assert np.allclose(ov, v, atol=1e-12) and np.allclose(ow, w, atol=1e-12)


def test_equal_wheel_braking_keeps_the_turn_rate_until_a_wheel_stops():
    """A hard stop while turning: v 1.2 -> 0 and w 0.6 -> 0 on one command. Wheel braking at 1.2 m/s^2 each keeps
    the wheel speed difference, so w stays at 0.6 while both wheels move, and falls only when the slower stops."""
    n = 250
    cv = np.r_[np.full(100, 1.2), np.zeros(n - 100)]
    cw = np.r_[np.full(100, 0.6), np.zeros(n - 100)]
    ov, ow = run({"wheel_base_m": B, "a_lin_up": math.inf, "a_lin_dn": 1.2}, cv, cw)
    assert ow[99] == pytest.approx(0.6, abs=1e-12)
    k = 100 + 25                       # 0.5 s after the stop command: both wheels still moving (slower one: 0.9 s)
    assert ow[k] == pytest.approx(0.6, abs=1e-9)
    assert ov[k] == pytest.approx(1.2 - 1.2 * 0.5 - 0.02 * 1.2 * 0, abs=0.04)
    # the slower (left) wheel stops after (1.2 - 0.6*B)/1.2 s; after that w falls
    stop_left = 100 + int(math.ceil((1.2 - 0.6 * B) / 1.2 / DT))
    assert ow[stop_left + 10] < 0.55
    assert ov[-1] == 0.0 and ow[-1] == 0.0                       # exact zero at rest
    ov2, ow2 = run({"a_lin_up": math.inf, "a_lin_dn": 1.2, "a_ang": 1.6}, cv, cw)   # the body-space model
    assert ow2[k] < 0.15 and ow[k] > 5 * ow2[k]


def test_in_place_turn_accelerates_both_wheels_at_the_wheel_limit():
    n = 40
    ov, ow = run({"wheel_base_m": B, "a_lin_up": 0.8, "a_lin_dn": 1.2}, np.zeros(n), np.full(n, 2.0))
    assert ow[0] == pytest.approx(DT * 0.8 / B, rel=1e-12)       # each wheel gains a*dt, w = a*dt / b
    assert np.all(ov == 0.0)                                      # symmetric wheels: no drift in v
    assert ow[-1] == pytest.approx(2.0)


def test_reverse_travel_uses_the_same_wheel_limits_mirrored():
    n = 120
    ov, _ = run({"wheel_base_m": B, "a_lin_up": 0.8, "a_lin_dn": 1.2}, np.full(n, -0.6), np.zeros(n))
    assert ov[0] == pytest.approx(-DT * 0.8)                      # speeding up in reverse is a_lin_up
    ov2, _ = run({"wheel_base_m": B, "a_lin_up": 0.8, "a_lin_dn": 1.2}, np.r_[np.full(80, -0.6), np.zeros(n - 80)], np.zeros(n))
    assert ov2[80] - ov2[79] == pytest.approx(DT * 1.2)           # slowing down is a_lin_dn


def test_lag_delay_and_yaw_scale_still_apply():
    n = 200
    p = {"wheel_base_m": B, "delay_s": 0.1, "tau_ang_s": 0.2, "yaw_scale": 0.9}
    ov, ow = run(p, np.zeros(n), np.full(n, 1.0))
    first = int(np.argmax(ow > 0))
    assert first == 5                                              # whole-step delay
    assert ow[-1] == pytest.approx(0.9)                            # scale on the output, lag settled
    assert ow[first + 9] == pytest.approx(0.9 * (1 - math.exp(-0.2 / 0.2)), rel=1e-9)   # ten lag updates = 0.2 s, exact discretisation


def test_zero_snap_in_wheel_mode():
    n = 1000
    cmd = np.r_[np.full(50, 1.0), np.zeros(n - 50)]
    ov, ow = run({"wheel_base_m": B, "tau_lin_s": 0.3, "tau_ang_s": 0.3}, cmd, cmd)
    assert ov[-1] == 0.0 and ow[-1] == 0.0


def test_wheel_vectors_file_is_current_and_matches():
    on_disk = json.loads(WHEEL_VECTORS.read_text())
    assert on_disk == json.loads(json.dumps(vectors.generate_wheel()))
    assert all(c["params"]["wheel_base_m"] == B for c in on_disk["cases"]) and len(on_disk["cases"]) == 30
    for case in on_disk["cases"]:
        p = stepper.vector(vectors.decode_params(case["params"]))
        ov, ow = stepper.run(p, case["cmd_v"], case["cmd_w"], case["dt"])
        assert np.max(np.abs(ov - case["out_v"])) <= on_disk["tolerance"], case["name"]
        assert np.max(np.abs(ow - case["out_w"])) <= on_disk["tolerance"], case["name"]


def test_scoring_uses_the_wheel_option():
    """The scorer applies the wheel option: its objective differs from the one without the option."""
    from flatland_base_model_tool import drives, scoring
    from test_drives_fit import TRUE, synth_drive
    d = drives.prepare(synth_drive("w", 1000.0, 5))
    pk = scoring.pack([d])
    plain = stepper.vector(TRUE)
    wheel = stepper.vector({**TRUE, "wheel_base_m": B})
    assert scoring.objective(scoring.run(pk, plain)) != scoring.objective(scoring.run(pk, wheel))
