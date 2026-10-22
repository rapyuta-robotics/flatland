"""Guard: every model option must change the output of every path that runs the model.

Every model option must reach every path that runs the model (the stepper and the scorer, which are compiled
separately). This suite
drives a sequence that makes every option bind (speed caps exceeded, a reversal, a hard stop while turning) and
asserts that perturbing each option changes the stepper output and the scored track. A compiled cache that is out of date fails here.
"""
import numpy as np
import pytest

from flatland_base_model_tool import drives, scoring, stepper

DT = 0.02
BASE = {"delay_s": 0.06, "tau_lin_s": 0.1, "tau_ang_s": 0.1, "a_lin_up": 0.8, "a_lin_dn": 1.2, "a_ang": 1.6,
        "jerk_lin": 5.0, "jerk_ang": 20.0, "yaw_scale": 0.9, "v_min": -0.3, "v_max": 1.2, "w_max": 0.8,
        "wheel_base_m": 0.0, "wheel_acc_max": 0.0, "wheel_dec_max": 0.0}
CHANGES = {"delay_s": 0.1, "tau_lin_s": 0.2, "tau_ang_s": 0.2, "a_lin_up": 0.6, "a_lin_dn": 1.0, "a_ang": 0.5,
           "jerk_lin": 2.0, "jerk_ang": 5.0, "yaw_scale": 0.8, "v_min": -0.2, "v_max": 1.0, "w_max": 0.6,
           "wheel_base_m": 0.2, "wheel_acc_max": 0.8, "wheel_dec_max": 0.6}
# the options that are only valid together with another one are perturbed from a base that has it
BASE_FOR = {"wheel_acc_max": {"wheel_base_m": 0.2},
            "wheel_dec_max": {"wheel_base_m": 0.2, "wheel_acc_max": 1.3}}


def base_for(option):
    return {**BASE, **BASE_FOR.get(option, {})}


def commands():
    """Accelerate while turning beyond both caps, reverse hard while turning the other way, then stop while turning."""
    segs = [(3.0, 1.5, 1.2), (1.5, -0.6, -1.0), (2.0, 1.0, 0.6), (3.0, 0.0, 0.0)]
    v, w = [], []
    for seconds, cv, cw in segs:
        v += [cv] * int(seconds / DT)
        w += [cw] * int(seconds / DT)
    return np.array(v), np.array(w)


def drive():
    cv, cw = commands()
    raw = {"name": "g", "t0": 0.0, "t1": len(cv) * DT, "cmd": [[k * DT, float(cv[k]), float(cw[k])] for k in range(len(cv))],
           "odom": [[k * DT, 0.0, 0.0, 0.0, 0.0, 0.0] for k in range(len(cv))]}
    return drives.prepare(raw)


def test_options_listed_here_are_all_the_options():
    assert set(CHANGES) == set(stepper.PARAM_NAMES) | set(stepper.OPTIONAL_NAMES)


@pytest.mark.parametrize("option", sorted(CHANGES))
def test_each_option_changes_the_stepper_output(option):
    cv, cw = commands()
    a = stepper.run(stepper.vector(base_for(option)), cv, cw, DT)
    b = stepper.run(stepper.vector({**base_for(option), option: CHANGES[option]}), cv, cw, DT)
    assert not (np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])), option


@pytest.mark.parametrize("option", sorted(CHANGES))
def test_each_option_changes_the_scored_track(option):
    """The path of the fit and the scores (events -> model -> pose), not only the stepper."""
    d = drive()
    a = scoring.offline_track(d, stepper.vector(base_for(option)))
    b = scoring.offline_track(d, stepper.vector({**base_for(option), option: CHANGES[option]}))
    assert any(not np.array_equal(x, y) for x, y in zip(a, b)), option
    pk = scoring.pack([d])
    ref = scoring.run(pk, stepper.vector(base_for(option)))
    alt = scoring.run(pk, stepper.vector({**base_for(option), option: CHANGES[option]}))
    assert not np.array_equal(ref, alt), option


def test_wheel_space_differs_from_axis_limits_when_turning_while_braking():
    cv = np.r_[np.full(100, 1.2), np.zeros(100)]
    cw = np.r_[np.full(100, 0.6), np.zeros(100)]
    free = {**stepper.IDENTITY, "a_lin_dn": 1.2, "a_ang": 1.6}
    axis = stepper.run(stepper.vector(free), cv, cw, DT)
    wheel = stepper.run(stepper.vector({**free, "wheel_base_m": 0.2}), cv, cw, DT)
    assert np.max(np.abs(axis[1] - wheel[1])) > 0.3


def test_wheel_cap_differs_from_the_axis_model_when_turning_while_braking():
    cv = np.r_[np.full(100, 1.2), np.zeros(100)]
    cw = np.r_[np.full(100, 0.6), np.zeros(100)]
    free = {**stepper.IDENTITY, "a_lin_dn": 1.2, "a_ang": 1.6}
    axis = stepper.run(stepper.vector(free), cv, cw, DT)
    capped = stepper.run(stepper.vector({**free, "wheel_base_m": 0.2, "wheel_acc_max": 1.0}), cv, cw, DT)
    assert np.max(np.abs(axis[1] - capped[1])) > 0.2
