"""Reference stepper: the discrete maths of the BaseModelDrive plugin.

The C++ plugin (`flatland_plugins/include/flatland_plugins/base_model_shaper.h`) implements the same
steps in the same order. `test_vectors/stepper.json` is produced by this module; both
implementations must reproduce it within 1e-9.

Per step of length `dt`, from the command `cmd` that is current at the step:

1. Delay. The model sees the command from `n = floor(delay_s / dt + 0.5)` steps ago (zero before the start).
2. Lag. Per axis, `lag += a * (u - lag)` with `a = 1 - exp(-dt / tau)` (`a = 1` and `lag = u` if `tau <= 0`).
3. Rate limits. Per axis, the lagged target is clipped to the speed caps, then moved toward by an
   acceleration, clipped by the signed acceleration limits and then by the jerk limit. A step that
   reaches the target ends on it exactly.
4. Zero snap. An axis whose delayed input is exactly 0 and whose speed is below 1e-3 is set to exactly 0
   (with its lag and acceleration state).
5. Output `(v, yaw_scale * w)`.

The same source runs under numba for the fit and as plain Python for the tests.
"""
from __future__ import annotations

import math

import numpy as np

try:
    from numba import njit
except ImportError:  # pragma: no cover - numba is a declared dependency
    def njit(*args, **kwargs):
        if args and callable(args[0]):
            return args[0]
        return lambda f: f

PARAM_NAMES = (
    "delay_s", "tau_lin_s", "tau_ang_s", "a_lin_up", "a_lin_dn", "a_ang",
    "jerk_lin", "jerk_ang", "yaw_scale", "v_min", "v_max", "w_max",
)
(P_DELAY, P_TAU_LIN, P_TAU_ANG, P_A_LIN_UP, P_A_LIN_DN, P_A_ANG,
 P_JERK_LIN, P_JERK_ANG, P_YAW_SCALE, P_V_MIN, P_V_MAX, P_W_MAX) = range(12)
# Optional: wheel_base_m > 0 switches the rate limits to wheel space (half the wheel separation, in m).
# `wheel_acc_max` > 0 (needs wheel_base_m) keeps the axis limiter and adds a per-wheel acceleration cap after it;
# `wheel_dec_max` > 0 (needs wheel_acc_max) caps wheels that move toward zero separately.
OPTIONAL_NAMES = ("wheel_base_m", "wheel_acc_max", "wheel_dec_max")
P_WHEEL, P_WACC, P_WDEC = 12, 13, 14

INF = math.inf
SNAP_V = 1e-3   # m/s
SNAP_W = 1e-3   # rad/s

# No shaping: the model applies the command unchanged. Also the neutral starting point of a fit.
IDENTITY = {
    "delay_s": 0.0, "tau_lin_s": 0.0, "tau_ang_s": 0.0,
    "a_lin_up": INF, "a_lin_dn": INF, "a_ang": INF,
    "jerk_lin": INF, "jerk_ang": INF,
    "yaw_scale": 1.0, "v_min": -INF, "v_max": INF, "w_max": INF,
}


def validate_params(values: dict) -> None:
    """Raise ValueError unless `values` holds exactly PARAM_NAMES, each acceptable. Same rules as the plugin."""
    unknown = sorted(set(values) - set(PARAM_NAMES) - set(OPTIONAL_NAMES))
    missing = [k for k in PARAM_NAMES if k not in values]
    if unknown:
        raise ValueError(f"unknown parameter(s): {', '.join(unknown)}")
    if missing:
        raise ValueError(f"missing parameter(s): {', '.join(missing)}")
    v = {k: float(values[k]) for k in PARAM_NAMES + tuple(k for k in OPTIONAL_NAMES if k in values)}
    for k, x in v.items():
        if math.isnan(x):
            raise ValueError(f"{k} is NaN")
    for k in ("delay_s", "tau_lin_s", "tau_ang_s", "yaw_scale"):
        if not math.isfinite(v[k]):
            raise ValueError(f"{k} must be finite")
    for k in ("delay_s", "tau_lin_s", "tau_ang_s"):
        if v[k] < 0.0:
            raise ValueError(f"{k} must be >= 0")
    for k in ("a_lin_up", "a_lin_dn", "a_ang", "jerk_lin", "jerk_ang", "yaw_scale", "w_max"):
        if not v[k] > 0.0:
            raise ValueError(f"{k} must be > 0")
    if "wheel_base_m" in v and not (math.isfinite(v["wheel_base_m"]) and v["wheel_base_m"] >= 0.0):
        raise ValueError("wheel_base_m must be finite and >= 0")
    for k in ("wheel_acc_max", "wheel_dec_max"):
        if k in v and not (math.isfinite(v[k]) and v[k] >= 0.0):
            raise ValueError(f"{k} must be finite and >= 0")
    if v.get("wheel_acc_max", 0.0) > 0.0 and not v.get("wheel_base_m", 0.0) > 0.0:
        raise ValueError("wheel_acc_max needs wheel_base_m > 0")
    if v.get("wheel_dec_max", 0.0) > 0.0 and not v.get("wheel_acc_max", 0.0) > 0.0:
        raise ValueError("wheel_dec_max needs wheel_acc_max > 0")
    if v["v_min"] > 0.0:
        raise ValueError("v_min must be <= 0")
    if v["v_max"] < 0.0:
        raise ValueError("v_max must be >= 0")


def vector(values: dict) -> np.ndarray:
    """Parameter dict -> float64 vector in PARAM_NAMES order (validated)."""
    validate_params(values)
    return np.array([float(values[k]) for k in PARAM_NAMES] + [float(values.get(k, 0.0)) for k in OPTIONAL_NAMES],
                    dtype=np.float64)


def delay_steps(delay_s: float, dt: float) -> int:
    return int(math.floor(delay_s / dt + 0.5))


@njit(cache=True)
def _axis(x, a, target, dt, acc_pos, acc_neg, jerk, lo, hi):
    """One rate-limited axis step. Returns (x_new, a_new)."""
    if target < lo:
        target = lo
    elif target > hi:
        target = hi
    a_raw = (target - x) / dt
    a_des = a_raw
    if a_des > acc_pos:
        a_des = acc_pos
    elif a_des < -acc_neg:
        a_des = -acc_neg
    if math.isinf(jerk):
        a_new = a_des
    else:
        da = a_des - a
        lim = jerk * dt
        if da > lim:
            da = lim
        elif da < -lim:
            da = -lim
        a_new = a + da
    x_new = x + a_new * dt
    if (x < target <= x_new) or (x_new <= target < x) or a_new == a_raw:
        x_new = target
    if x_new < lo:
        x_new = lo
    elif x_new > hi:
        x_new = hi
    return x_new, (x_new - x) / dt


@njit(cache=True)
def _wheel_axis(x, a, target, dt, up, dn, jerk):
    """One wheel: speeding up (the wheel's speed grows in its direction of travel) is limited by `up`, slowing down
    (including reversing through zero within the step) by `dn`. At rest both directions use `up`."""
    if x > 0.0:
        pos, neg = up, dn
    elif x < 0.0:
        pos, neg = dn, up
    else:
        pos, neg = up, up
    return _axis(x, a, target, dt, pos, neg, jerk, -INF, INF)


@njit(cache=True)
def _simulate_wheels(p, cmd_v, cmd_w, dt, out_v, out_w):
    n = len(cmd_v)
    b = p[P_WHEEL]
    nd = int(math.floor(p[P_DELAY] / dt + 0.5))
    lag_v = 0.0
    lag_w = 0.0
    vl = 0.0
    vr = 0.0
    al = 0.0
    ar = 0.0
    kl = 1.0 if p[P_TAU_LIN] <= 0.0 else 1.0 - math.exp(-dt / p[P_TAU_LIN])
    ka = 1.0 if p[P_TAU_ANG] <= 0.0 else 1.0 - math.exp(-dt / p[P_TAU_ANG])
    for k in range(n):
        j = k - nd
        u_v = cmd_v[j] if j >= 0 else 0.0
        u_w = cmd_w[j] if j >= 0 else 0.0
        if p[P_TAU_LIN] <= 0.0:
            lag_v = u_v
        else:
            lag_v += kl * (u_v - lag_v)
        if p[P_TAU_ANG] <= 0.0:
            lag_w = u_w
        else:
            lag_w += ka * (u_w - lag_w)
        tv = min(max(lag_v, p[P_V_MIN]), p[P_V_MAX])
        tw = min(max(lag_w, -p[P_W_MAX]), p[P_W_MAX])
        vl, al = _wheel_axis(vl, al, tv - tw * b, dt, p[P_A_LIN_UP], p[P_A_LIN_DN], p[P_JERK_LIN])
        vr, ar = _wheel_axis(vr, ar, tv + tw * b, dt, p[P_A_LIN_UP], p[P_A_LIN_DN], p[P_JERK_LIN])
        v = 0.5 * (vl + vr)
        if u_v == 0.0 and abs(v) < SNAP_V:
            lag_v = 0.0
            vl -= v
            vr -= v
            al = 0.0
            ar = 0.0
        w = (vr - vl) / (2.0 * b)
        if u_w == 0.0 and abs(w) < SNAP_W:
            lag_w = 0.0
            m = 0.5 * (vl + vr)
            vl = m
            vr = m
            al = 0.0
            ar = 0.0
        out_v[k] = 0.5 * (vl + vr)
        out_w[k] = p[P_YAW_SCALE] * (vr - vl) / (2.0 * b)


@njit(cache=True)
def _wheel_follow(x, target, dt, cap_up, cap_dn):
    """A wheel follows its target with an acceleration cap: `cap_up` while its speed moves away from zero (or
    crosses it), `cap_dn` while it moves toward zero. Returns (new speed, whether the cap clipped the step)."""
    d = target - x
    cap = cap_dn if x * d < 0.0 else cap_up
    lim = cap * dt
    if d > lim:
        return x + lim, True
    if d < -lim:
        return x - lim, True
    return target, False


@njit(cache=True)
def simulate(p, cmd_v, cmd_w, dt, out_v, out_w):
    """Run the model over per-step commands. `cmd_*[k]` is the command current at step k.

    Writes the shaped (v, w) of each step to `out_v`, `out_w` (the velocity set for that step and, in the
    plugin, the odometry twist after it).
    """
    flags = np.zeros(len(cmd_v), dtype=np.bool_)
    _simulate_axes(p, cmd_v, cmd_w, dt, out_v, out_w, flags)


@njit(cache=True)
def _simulate_axes(p, cmd_v, cmd_w, dt, out_v, out_w, flags):
    if p[P_WACC] <= 0.0 and p[P_WHEEL] > 0.0:
        _simulate_wheels(p, cmd_v, cmd_w, dt, out_v, out_w)
        return
    cap_mode = p[P_WACC] > 0.0
    cap_up = p[P_WACC]
    cap_dn = p[P_WDEC] if p[P_WDEC] > 0.0 else p[P_WACC]
    b = p[P_WHEEL]
    wl = 0.0
    wr = 0.0
    n = len(cmd_v)
    nd = int(math.floor(p[P_DELAY] / dt + 0.5))
    lag_v = 0.0
    lag_w = 0.0
    v = 0.0
    av = 0.0
    w = 0.0
    aw = 0.0
    al = 1.0 if p[P_TAU_LIN] <= 0.0 else 1.0 - math.exp(-dt / p[P_TAU_LIN])
    aa = 1.0 if p[P_TAU_ANG] <= 0.0 else 1.0 - math.exp(-dt / p[P_TAU_ANG])
    for k in range(n):
        j = k - nd
        u_v = cmd_v[j] if j >= 0 else 0.0
        u_w = cmd_w[j] if j >= 0 else 0.0
        if p[P_TAU_LIN] <= 0.0:
            lag_v = u_v
        else:
            lag_v += al * (u_v - lag_v)
        if p[P_TAU_ANG] <= 0.0:
            lag_w = u_w
        else:
            lag_w += aa * (u_w - lag_w)
        v, av = _axis(v, av, lag_v, dt, p[P_A_LIN_UP], p[P_A_LIN_DN], p[P_JERK_LIN], p[P_V_MIN], p[P_V_MAX])
        w, aw = _axis(w, aw, lag_w, dt, p[P_A_ANG], p[P_A_ANG], p[P_JERK_ANG], -p[P_W_MAX], p[P_W_MAX])
        if u_v == 0.0 and abs(v) < SNAP_V:
            lag_v = 0.0
            v = 0.0
            av = 0.0
        if u_w == 0.0 and abs(w) < SNAP_W:
            lag_w = 0.0
            w = 0.0
            aw = 0.0
        if cap_mode:
            wl, cl = _wheel_follow(wl, v - w * b, dt, cap_up, cap_dn)
            wr, cr = _wheel_follow(wr, v + w * b, dt, cap_up, cap_dn)
            flags[k] = cl or cr
            out_v[k] = 0.5 * (wl + wr)
            out_w[k] = p[P_YAW_SCALE] * (wr - wl) / (2.0 * b)
        else:
            out_v[k] = v
            out_w[k] = p[P_YAW_SCALE] * w


def cap_binding_steps(p, cmd_v, cmd_w, dt):
    """Boolean array: the steps where the per-wheel cap clipped a wheel (all False without the cap)."""
    cv = np.ascontiguousarray(cmd_v, dtype=np.float64)
    cw = np.ascontiguousarray(cmd_w, dtype=np.float64)
    ov, ow = np.zeros(len(cv)), np.zeros(len(cv))
    flags = np.zeros(len(cv), dtype=np.bool_)
    _simulate_axes(p, cv, cw, float(dt), ov, ow, flags)
    return flags


def run(values_or_vector, cmd_v, cmd_w, dt):
    """Convenience wrapper: (params dict or vector, per-step commands, dt) -> (v, w) arrays."""
    p = values_or_vector if isinstance(values_or_vector, np.ndarray) else vector(values_or_vector)
    cv = np.ascontiguousarray(cmd_v, dtype=np.float64)
    cw = np.ascontiguousarray(cmd_w, dtype=np.float64)
    ov = np.zeros(len(cv))
    ow = np.zeros(len(cv))
    simulate(p, cv, cw, float(dt), ov, ow)
    return ov, ow


@njit(cache=True)
def events_to_steps(et, ev, ew, dt, n, cmd_v, cmd_w):
    """Time-stamped commands -> per-step commands, with flatland's own path: a command received in
    `spinOnce` is applied at the next `BeforePhysicsStep`, i.e. at the first step k with k*dt >= t.
    `et` must be sorted. Commands before the first event are zero."""
    i = -1
    for k in range(n):
        tq = k * dt
        while i + 1 < len(et) and et[i + 1] <= tq + 1e-9:
            i += 1
        cmd_v[k] = ev[i] if i >= 0 else 0.0
        cmd_w[k] = ew[i] if i >= 0 else 0.0


@njit(cache=True)
def integrate(v, w, dt, ox, oy, oh):
    """Unicycle pose after each step from the per-step (v, w), starting at the origin. `ox[0]` is the
    start; `ox[k + 1]` the pose after step k. Arrays have len(v) + 1 entries."""
    x = 0.0
    y = 0.0
    h = 0.0
    ox[0] = 0.0
    oy[0] = 0.0
    oh[0] = 0.0
    for k in range(len(v)):
        x += v[k] * math.cos(h) * dt
        y += v[k] * math.sin(h) * dt
        h += w[k] * dt
        ox[k + 1] = x
        oy[k + 1] = y
        oh[k + 1] = h
