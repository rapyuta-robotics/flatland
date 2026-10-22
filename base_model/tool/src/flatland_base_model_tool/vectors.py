"""Shared test vectors: command sequences and the outputs of the reference stepper.

`generate()` is deterministic. The file it writes is checked in at `test_vectors/stepper.json`; the Python
tests and the C++ gtests both replay it and must match within TOLERANCE. Infinite parameters are the
strings ".inf" / "-.inf" so that the file is plain JSON.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import stepper

TOLERANCE = 1e-9
DTS = (0.02, 0.04, 0.08)
DURATION_S = 8.0
VERSION = 1

PARAM_SETS = {
    # made-up, round values: these are test inputs, not the parameters of any robot
    "example": {
        "delay_s": 0.1, "tau_lin_s": 0.1, "tau_ang_s": 0.2, "a_lin_up": 1.0, "a_lin_dn": 2.0,
        "a_ang": 2.0, "jerk_lin": 20.0, "jerk_ang": 100.0, "yaw_scale": 0.9,
        "v_min": -0.5, "v_max": 1.5, "w_max": 2.0,
    },
    "limits_only": {
        "delay_s": 0.0, "tau_lin_s": 0.0, "tau_ang_s": 0.0, "a_lin_up": 0.8, "a_lin_dn": 1.2,
        "a_ang": 1.5, "jerk_lin": 10.0, "jerk_ang": 50.0, "yaw_scale": 1.0,
        "v_min": -0.5, "v_max": 1.5, "w_max": 2.0,
    },
    "identity": dict(stepper.IDENTITY),
    "slow_jerk": {
        "delay_s": 0.05, "tau_lin_s": 0.1, "tau_ang_s": 0.3, "a_lin_up": 2.0, "a_lin_dn": 2.0,
        "a_ang": 3.0, "jerk_lin": 5.0, "jerk_ang": 20.0, "yaw_scale": 1.05,
        "v_min": -1.0, "v_max": 1.0, "w_max": 2.0,
    },
}


def _piecewise(t, pieces):
    """pieces: [(start_s, value)], sorted; value of the last piece with start <= t."""
    out = 0.0
    for start, value in pieces:
        if t >= start - 1e-9:
            out = value
    return out


def _lcg_pieces(seed, n, hold_s, scale_v, scale_w):
    state = seed
    pv, pw = [], []
    for i in range(n):
        state = (1103515245 * state + 12345) % (2 ** 31)
        pv.append((i * hold_s, ((state >> 8) % 41 - 20) / 20.0 * scale_v))
        state = (1103515245 * state + 12345) % (2 ** 31)
        pw.append((i * hold_s, ((state >> 8) % 41 - 20) / 20.0 * scale_w))
    return pv, pw


def _profiles():
    rnd_v, rnd_w = _lcg_pieces(7, 30, 0.3, 1.4, 1.5)
    return {
        "step_stop": ([(0.2, 1.0), (3.0, 0.0)], [(1.0, 0.8), (4.0, 0.0)]),
        "reversal": ([(0.0, 1.2), (2.5, -0.7), (5.0, 0.0)], [(2.0, -1.0), (3.5, 0.0)]),
        "saturate": ([(1.0, 3.0), (4.0, -2.0), (6.0, 0.0)], [(1.0, 6.0), (3.0, -6.0), (5.0, 0.0)]),
        "pseudo_random": (rnd_v, rnd_w),
        "snap_tail": ([(0.5, 1.0), (1.5, 0.0)], [(0.5, 2.0), (1.0, 0.0)]),
    }


def _encode(x: float):
    if x == float("inf"):
        return ".inf"
    if x == float("-inf"):
        return "-.inf"
    return x


def generate() -> dict:
    cases = []
    for pname, pvals in PARAM_SETS.items():
        p = stepper.vector(pvals)
        for prof, (pv, pw) in _profiles().items():
            for dt in DTS:
                n = int(round(DURATION_S / dt))
                cv = np.array([_piecewise(k * dt, pv) for k in range(n)])
                cw = np.array([_piecewise(k * dt, pw) for k in range(n)])
                ov, ow = stepper.run(p, cv, cw, dt)
                cases.append({
                    "name": f"{pname}/{prof}/dt{dt:g}",
                    "dt": dt,
                    "params": {k: _encode(float(pvals[k])) for k in stepper.PARAM_NAMES},
                    "cmd_v": cv.tolist(), "cmd_w": cw.tolist(),
                    "out_v": ov.tolist(), "out_w": ow.tolist(),
                })
    return {"version": VERSION, "tolerance": TOLERANCE, "cases": cases}


WHEEL_BASE_M = 0.2    # made-up
WHEEL_SETS = ("example", "limits_only", "slow_jerk")


def generate_wheel() -> dict:
    """Vectors of the wheel-space mode (`wheel_base_m`), in their own file: the C++ model does not implement it
    yet, and the main file stays the contract that it does."""
    cases = []
    for pname in WHEEL_SETS:
        pvals = {**PARAM_SETS[pname], "wheel_base_m": WHEEL_BASE_M}
        p = stepper.vector(pvals)
        for prof, (pv, pw) in _profiles().items():
            for dt in (0.02, 0.04):
                n = int(round(DURATION_S / dt))
                cv = np.array([_piecewise(k * dt, pv) for k in range(n)])
                cw = np.array([_piecewise(k * dt, pw) for k in range(n)])
                ov, ow = stepper.run(p, cv, cw, dt)
                cases.append({"name": f"wheel/{pname}/{prof}/dt{dt:g}", "dt": dt,
                              "params": {k: _encode(float(pvals[k])) for k in stepper.PARAM_NAMES + ("wheel_base_m",)},
                              "cmd_v": cv.tolist(), "cmd_w": cw.tolist(), "out_v": ov.tolist(), "out_w": ow.tolist()})
    return {"version": VERSION, "tolerance": TOLERANCE, "cases": cases}


def generate_cap() -> dict:
    """Vectors of the per-wheel cap mode (`wheel_acc_max`), in their own file like the wheel-space ones."""
    cases = []
    for pname in WHEEL_SETS:
        for cap, dec in ((1.25, 0.0), (0.9, 1.1)):
            pvals = {**PARAM_SETS[pname], "wheel_base_m": WHEEL_BASE_M, "wheel_acc_max": cap, "wheel_dec_max": dec}
            p = stepper.vector(pvals)
            for prof, (pv, pw) in _profiles().items():
                for dt in (0.02, 0.04):
                    n = int(round(DURATION_S / dt))
                    cv = np.array([_piecewise(k * dt, pv) for k in range(n)])
                    cw = np.array([_piecewise(k * dt, pw) for k in range(n)])
                    ov, ow = stepper.run(p, cv, cw, dt)
                    cases.append({"name": f"cap/{pname}/{cap:g}-{dec:g}/{prof}/dt{dt:g}", "dt": dt,
                                  "params": {k: _encode(float(pvals[k])) for k in stepper.PARAM_NAMES + stepper.OPTIONAL_NAMES},
                                  "cmd_v": cv.tolist(), "cmd_w": cw.tolist(), "out_v": ov.tolist(), "out_w": ow.tolist()})
    return {"version": VERSION, "tolerance": TOLERANCE, "cases": cases}


def write_cap(path) -> None:
    Path(path).write_text(json.dumps(generate_cap(), separators=(",", ":")) + "\n")


def write(path) -> None:
    Path(path).write_text(json.dumps(generate(), separators=(",", ":")) + "\n")


def write_wheel(path) -> None:
    Path(path).write_text(json.dumps(generate_wheel(), separators=(",", ":")) + "\n")


def decode_params(raw: dict) -> dict:
    return {k: float(v.replace(".inf", "inf")) if isinstance(v, str) else float(v) for k, v in raw.items()}
