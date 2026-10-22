"""Per-wheel acceleration cap on top of the axis limiter.

Mode: `wheel_base_m` > 0 and `wheel_acc_max` > 0. The axis limiter runs exactly as in the calibrated model; its output
(v, w) is converted to wheel speeds (left = v - w b, right = v + w b); each wheel then follows its target with
an acceleration cap (`wheel_acc_max` while its speed moves away from zero, `wheel_dec_max` while it moves toward zero;
`wheel_dec_max` 0 means the same cap in both directions); the wheels are converted back. Lags, delay and the
yaw scale are unchanged. A cap large enough never binds, and then the model is the calibrated one.
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
CAP_VECTORS = Path(__file__).resolve().parents[2] / "test_vectors" / "stepper_cap.json"
AXIS = {**stepper.IDENTITY, "a_lin_up": 0.8, "a_lin_dn": 1.2, "a_ang": 1.6}


def run(values, v, w, dt=DT):
    return stepper.run(stepper.vector({**AXIS, **values}), v, w, dt)


def stop_while_turning(n=300):
    cv = np.r_[np.full(100, 1.2), np.zeros(n - 100)]
    cw = np.r_[np.full(100, 0.6), np.zeros(n - 100)]
    return cv, cw


def test_options_validate():
    stepper.validate_params({**LIMITED, "wheel_base_m": B, "wheel_acc_max": 1.0})
    stepper.validate_params({**LIMITED, "wheel_base_m": B, "wheel_acc_max": 1.0, "wheel_dec_max": 1.0})
    for bad, text in (({"wheel_acc_max": 1.0}, "wheel_base_m"),
                      ({"wheel_base_m": B, "wheel_dec_max": 1.0}, "wheel_acc_max"),
                      ({"wheel_base_m": B, "wheel_acc_max": -1.0}, "wheel_acc_max"),
                      ({"wheel_base_m": B, "wheel_acc_max": math.nan}, "wheel_acc_max"),
                      ({"wheel_base_m": B, "wheel_acc_max": 1.0, "wheel_dec_max": math.inf}, "wheel_dec_max")):
        with pytest.raises(ValueError, match=text):
            stepper.validate_params({**LIMITED, **bad})


def test_parameter_file_roundtrip(tmp_path):
    values = {**LIMITED, "wheel_base_m": B, "wheel_acc_max": 1.0, "wheel_dec_max": 1.1}
    path = tmp_path / "c.yaml"
    params.save(params.make("c", values, 0.02, {}), path)
    got = params.load(path)["parameters"]
    assert got["wheel_acc_max"] == 1.0 and got["wheel_dec_max"] == 1.1


def test_a_cap_that_never_binds_is_the_axis_model():
    rng = np.random.default_rng(4)
    v, w = rng.uniform(-0.5, 1.5, 400), rng.uniform(-3, 3, 400)
    base = run({}, v, w)
    capped = run({"wheel_base_m": B, "wheel_acc_max": 1e6}, v, w)
    assert np.allclose(base[0], capped[0], atol=1e-12) and np.allclose(base[1], capped[1], atol=1e-12)


def test_wheel_speeds_never_change_faster_than_the_cap():
    cv, cw = stop_while_turning()
    ov, ow = run({"wheel_base_m": B, "wheel_acc_max": 1.0}, cv, cw)
    left, right = ov - ow * B, ov + ow * B
    for wheel in (left, right):
        assert np.max(np.abs(np.diff(wheel))) <= 1.0 * DT + 1e-12


def test_hard_stop_while_turning_keeps_more_turn_than_the_axis_model():
    cv, cw = stop_while_turning()
    axis = run({}, cv, cw)
    capped = run({"wheel_base_m": B, "wheel_acc_max": 1.0}, cv, cw)
    heading = lambda o: np.sum(o[1]) * DT
    assert heading(capped) > heading(axis) + 0.1        # the extra turn of the stop
    k = 100 + 25
    assert capped[1][k] > 3 * axis[1][k]                # w holds while the wheels brake


def test_cap_applies_to_each_direction_separately():
    n = 200
    up_v = np.full(n, 1.0)
    free = run({"wheel_base_m": B, "wheel_acc_max": 5.0, "wheel_dec_max": 0.5}, up_v, np.zeros(n))
    assert free[0][0] == pytest.approx(DT * 0.8)        # speeding up is the axis limit (0.8), cap 5.0 does not bind
    cv = np.r_[np.full(100, 1.0), np.zeros(n - 100)]
    slow = run({"wheel_base_m": B, "wheel_acc_max": 5.0, "wheel_dec_max": 0.5}, cv, np.zeros(n))
    assert slow[0][100] - slow[0][99] == pytest.approx(-DT * 0.5)   # braking is cap-limited
    both = run({"wheel_base_m": B, "wheel_acc_max": 0.5}, np.full(n, 1.0), np.zeros(n))
    assert both[0][0] == pytest.approx(DT * 0.5)        # the single value caps both directions


def test_in_place_turn_is_limited_per_wheel():
    n = 60
    ov, ow = run({"wheel_base_m": B, "wheel_acc_max": 0.5, "a_ang": math.inf}, np.zeros(n), np.full(n, 2.0))
    assert ow[0] == pytest.approx(DT * 0.5 / B, rel=1e-12)
    assert np.all(ov == 0.0)


def test_exact_zero_at_rest_and_lag_delay_scale_apply():
    cv, cw = stop_while_turning(500)
    ov, ow = run({"wheel_base_m": B, "wheel_acc_max": 1.0, "tau_ang_s": 0.15, "tau_lin_s": 0.05, "delay_s": 0.06,
                  "yaw_scale": 0.92}, cv, cw)
    assert ov[-1] == 0.0 and ow[-1] == 0.0
    assert int(np.argmax(ov > 0)) == 3                  # whole-step delay
    plain = run({"tau_ang_s": 0.15, "tau_lin_s": 0.05, "delay_s": 0.06, "yaw_scale": 0.92}, cv, cw)
    assert not np.array_equal(plain[1], ow)


def test_binding_steps_are_reported_and_mostly_not_at_gentle_commands():
    cv, cw = stop_while_turning()
    p = stepper.vector({**AXIS, "wheel_base_m": B, "wheel_acc_max": 1.3})   # above what accelerating needs (1.12), below what the hard stop needs (1.52)
    flags = stepper.cap_binding_steps(p, cv, cw, DT)
    assert flags.dtype == np.bool_ and flags.sum() > 10 and not flags[:50].any()
    gentle = stepper.cap_binding_steps(p, np.full(300, 0.3), np.full(300, 0.1), DT)
    assert gentle.sum() == 0
    off = stepper.cap_binding_steps(stepper.vector({**AXIS, "wheel_base_m": B, "wheel_acc_max": 1e6}), cv, cw, DT)
    assert off.sum() == 0


def test_cap_vectors_file_is_current_and_matches():
    on_disk = json.loads(CAP_VECTORS.read_text())
    assert on_disk == json.loads(json.dumps(vectors.generate_cap()))
    assert len(on_disk["cases"]) == 60 and all(c["params"]["wheel_acc_max"] > 0 for c in on_disk["cases"])
    for case in on_disk["cases"]:
        p = stepper.vector(vectors.decode_params(case["params"]))
        ov, ow = stepper.run(p, case["cmd_v"], case["cmd_w"], case["dt"])
        assert np.max(np.abs(ov - case["out_v"])) <= on_disk["tolerance"], case["name"]
        assert np.max(np.abs(ow - case["out_w"])) <= on_disk["tolerance"], case["name"]
