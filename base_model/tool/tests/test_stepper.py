import json
import math
from pathlib import Path

import numpy as np
import pytest

from flatland_base_model_tool import stepper, vectors
from limits import LIMITED

VECTORS = Path(__file__).resolve().parents[2] / "test_vectors" / "stepper.json"


def run(values, v, w, dt=0.02):
    return stepper.run(stepper.vector({**LIMITED, **values}), v, w, dt)


def test_vectors_file_is_current():
    """The checked-in vectors are exactly what the reference stepper produces now."""
    on_disk = json.loads(VECTORS.read_text())
    assert on_disk == json.loads(json.dumps(vectors.generate()))


def test_vectors_match_stepper():
    doc = json.loads(VECTORS.read_text())
    assert len(doc["cases"]) == 60
    for case in doc["cases"]:
        p = stepper.vector(vectors.decode_params(case["params"]))
        ov, ow = stepper.run(p, case["cmd_v"], case["cmd_w"], case["dt"])
        assert np.max(np.abs(ov - case["out_v"])) <= doc["tolerance"], case["name"]
        assert np.max(np.abs(ow - case["out_w"])) <= doc["tolerance"], case["name"]


def test_vectors_cover_three_step_sizes():
    doc = json.loads(VECTORS.read_text())
    assert {c["dt"] for c in doc["cases"]} == {0.02, 0.04, 0.08}


def test_compiled_and_plain_python_agree():
    """numba and the interpreter run the same source; the fit and the tests must not diverge."""
    simulate_py = getattr(stepper.simulate, "py_func", None)
    if simulate_py is None:
        pytest.skip("numba not active")
    doc = json.loads(VECTORS.read_text())
    for case in doc["cases"][::7]:
        p = stepper.vector(vectors.decode_params(case["params"]))
        n = len(case["cmd_v"])
        ov, ow = np.zeros(n), np.zeros(n)
        stepper.simulate.py_func(p, np.array(case["cmd_v"]), np.array(case["cmd_w"]), case["dt"], ov, ow)
        assert np.max(np.abs(ov - case["out_v"])) <= 1e-12


def test_identity_reproduces_command_exactly():
    rng = np.random.default_rng(1)
    v, w = rng.uniform(-3, 3, 200), rng.uniform(-5, 5, 200)
    ov, ow = stepper.run(stepper.IDENTITY, v, w, 0.02)
    assert np.array_equal(ov[v != 0], v[v != 0]) and np.array_equal(ow[w != 0], w[w != 0])


def test_rate_limit_ramp():
    n = 100
    ov, _ = run({"a_lin_up": 1.0, "jerk_lin": math.inf}, np.full(n, 1.0), np.zeros(n))
    assert ov[0] == pytest.approx(0.02)
    assert ov[24] == pytest.approx(0.5)
    assert ov[49] == pytest.approx(1.0)
    assert ov[-1] == 1.0


def test_decel_limit_and_exact_zero_at_rest():
    cmd = np.r_[np.full(100, 1.0), np.zeros(200)]
    ov, _ = run({"a_lin_up": math.inf, "a_lin_dn": 1.4, "jerk_lin": math.inf}, cmd, np.zeros(300))
    assert ov[99] == 1.0
    assert ov[100] == pytest.approx(1.0 - 1.4 * 0.02)
    assert ov[100 + 66] == 0.0 and ov[-1] == 0.0


def test_lag_tail_snaps_to_exact_zero():
    n = 1000
    cmd = np.r_[np.full(50, 1.0), np.zeros(n - 50)]
    ov, ow = run({"tau_lin_s": 0.3, "tau_ang_s": 0.3, "a_lin_up": math.inf, "a_lin_dn": math.inf,
                  "jerk_lin": math.inf}, cmd, cmd)
    assert (ov == 0.0).any() and ov[-1] == 0.0 and ow[-1] == 0.0
    first_zero = int(np.argmax((ov == 0.0) & (np.arange(n) > 50)))
    assert np.all(ov[first_zero:] == 0.0)
    assert 1e-3 <= ov[first_zero - 1] < 1.5e-3   # the step after the last output at or above the threshold snaps


def test_no_snap_while_command_nonzero():
    cmd = np.full(300, 1e-4)
    ov, _ = run({"a_lin_up": math.inf, "jerk_lin": math.inf}, cmd, np.zeros(300))
    assert ov[-1] == 1e-4


def test_delay_rounds_to_whole_steps():
    cmd = np.r_[np.zeros(5), np.ones(60)]
    for delay, dt, steps in ((0.1, 0.02, 5), (0.11, 0.02, 6), (0.03, 0.04, 1), (0.019, 0.04, 0), (0.1, 0.08, 1)):
        n = len(cmd)
        ov, _ = run({"delay_s": delay, "a_lin_up": math.inf, "jerk_lin": math.inf}, cmd, np.zeros(n), dt)
        assert stepper.delay_steps(delay, dt) == steps
        assert int(np.argmax(ov > 0)) == 5 + steps


def test_yaw_scale_applies_to_output_only():
    n = 200
    _, ow = run({"yaw_scale": 0.9, "a_ang": math.inf, "jerk_ang": math.inf}, np.zeros(n), np.full(n, 2.0))
    assert ow[-1] == pytest.approx(1.8)


def test_speed_caps():
    n = 400
    ov, ow = run({"a_lin_up": math.inf, "jerk_lin": math.inf, "a_ang": math.inf, "jerk_ang": math.inf,
                  "v_max": 0.5, "v_min": -0.2, "w_max": 1.0}, np.full(n, 3.0), np.full(n, -7.0))
    assert ov[-1] == 0.5 and ow[-1] == -1.0
    ov, _ = run({"a_lin_dn": math.inf, "a_lin_up": math.inf, "jerk_lin": math.inf, "v_min": -0.2},
                np.full(n, -3.0), np.zeros(n))
    assert ov[-1] == -0.2


def test_step_size_independence_of_continuous_parameters():
    """Exact discretisation: a pure lag reaches the same value at the same time for any dt."""
    values = {"tau_lin_s": 0.2, "a_lin_up": math.inf, "a_lin_dn": math.inf, "jerk_lin": math.inf}
    t_probe = 0.4
    results = []
    for dt in (0.02, 0.04, 0.08):
        n = int(round(1.0 / dt))
        ov, _ = run(values, np.ones(n), np.zeros(n), dt)
        k = int(round(t_probe / dt)) - 1
        results.append(ov[k])
    assert max(results) - min(results) < 1e-12
    assert results[0] == pytest.approx(1 - math.exp(-t_probe / 0.2), abs=1e-12)


def test_events_to_steps_uses_next_step():
    et, ev, ew = np.array([0.0, 0.05, 0.1]), np.array([1.0, 2.0, 0.0]), np.zeros(3)
    cv, cw = np.zeros(8), np.zeros(8)
    stepper.events_to_steps(et, ev, ew, 0.02, 8, cv, cw)
    # step k begins at k*dt; the 0.05 s command first applies at k=3 (0.06 s); the 0.1 s one at k=5
    assert cv.tolist() == [1.0, 1.0, 1.0, 2.0, 2.0, 0.0, 0.0, 0.0]


def test_integrate_straight_and_turn():
    n = 50
    ox, oy, oh = np.zeros(n + 1), np.zeros(n + 1), np.zeros(n + 1)
    stepper.integrate(np.full(n, 1.0), np.zeros(n), 0.02, ox, oy, oh)
    assert ox[-1] == pytest.approx(1.0) and oy[-1] == 0.0
    stepper.integrate(np.zeros(n), np.full(n, math.pi), 0.02, ox, oy, oh)
    assert oh[-1] == pytest.approx(math.pi)


@pytest.mark.parametrize("bad, text", [
    ({"tau_lin_s": -0.1}, "tau_lin_s"),
    ({"delay_s": float("nan")}, "NaN"),
    ({"a_lin_up": 0.0}, "a_lin_up"),
    ({"yaw_scale": 0.0}, "yaw_scale"),
    ({"v_min": 0.5}, "v_min"),
    ({"v_max": -0.5}, "v_max"),
    ({"w_max": 0.0}, "w_max"),
    ({"delay_s": math.inf}, "delay_s"),
])
def test_validate_rejects(bad, text):
    with pytest.raises(ValueError, match=text):
        stepper.validate_params({**LIMITED, **bad})


def test_validate_rejects_unknown_and_missing():
    with pytest.raises(ValueError, match="unknown"):
        stepper.validate_params({**LIMITED, "yaw_bias": 0.1})
    short = dict(LIMITED)
    del short["a_ang"]
    with pytest.raises(ValueError, match="missing"):
        stepper.validate_params(short)
