"""Scoring: simulate drives with the reference stepper and compare them with the recorded track.

Timing is the plugin's: a recorded command is applied at the first step that starts at or after its time
(flatland applies a command received in `spinOnce` at the next `BeforePhysicsStep`); the model's own
delay is `delay_s` rounded to whole steps; the velocity set in step k moves the body during step k.

The score of a drive is taken over 2 s windows every 0.5 s, anchored at the window start so a slow
heading drift does not count against the whole drive: the p95 over moving windows of the largest
position error and of the largest heading error in the window. The objective is the mean over drives of
`pos_p95 + 0.5 * yaw_p95` (metres plus half a radian).
"""
from __future__ import annotations

import math

import numpy as np

from . import stepper
from .drives import DT, Drive

try:
    from numba import njit
except ImportError:  # pragma: no cover
    def njit(*args, **kwargs):
        if args and callable(args[0]):
            return args[0]
        return lambda f: f

WIN, WSTEP = 100, 25      # 2 s windows every 0.5 s at 50 Hz
MOVING_PATH_M = 0.2       # a window counts when the reference moves more than this
YAW_WEIGHT = 0.5
METRIC_NAMES = ("win_pos_p95", "win_yaw_p95", "final_pos", "final_yaw", "win_pos_max", "n_moving")


@njit(cache=True)
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@njit(cache=True)
def count_moving_windows(rx, ry, mask) -> int:
    """Scoring windows that lie outside `mask` (nonzero = excluded) and have more than MOVING_PATH_M of reference path."""
    n = len(rx)
    cnt = 0
    for a in range(0, n - WIN, WSTEP):
        masked = False
        path = 0.0
        for k in range(a, a + WIN + 1):
            if mask[k] != 0:
                masked = True
                break
            if k > a:
                path += math.hypot(rx[k] - rx[k - 1], ry[k] - ry[k - 1])
        if not masked and path > MOVING_PATH_M:
            cnt += 1
    return cnt


@njit(cache=True)
def metrics(rx, ry, rh, sx, sy, sh, mask, out):
    """Compare a simulated track (s) with the reference (r); fill `out` as METRIC_NAMES. Windows that touch a
    nonzero `mask` step (a hold stretch) do not count."""
    n = len(rx)
    nwin = 0
    for a in range(0, n - WIN, WSTEP):
        nwin += 1
    pm = np.zeros(nwin)
    ym = np.zeros(nwin)
    mv = np.zeros(nwin, dtype=np.bool_)
    valid = np.ones(nwin, dtype=np.bool_)
    wi = 0
    for a in range(0, n - WIN, WSTEP):
        b = a + WIN
        for k in range(a, b + 1):
            if mask[k] != 0:
                valid[wi] = False
                break
        cr, sr = math.cos(rh[a]), math.sin(rh[a])
        cs, ss = math.cos(sh[a]), math.sin(sh[a])
        pmax = 0.0
        ymax = 0.0
        path = 0.0
        for k in range(a, b + 1):
            dxr, dyr = rx[k] - rx[a], ry[k] - ry[a]
            dxs, dys = sx[k] - sx[a], sy[k] - sy[a]
            xr, yr = cr * dxr + sr * dyr, -sr * dxr + cr * dyr
            xs, ys = cs * dxs + ss * dys, -ss * dxs + cs * dys
            e = math.hypot(xs - xr, ys - yr)
            if e > pmax:
                pmax = e
            ye = abs(_wrap((sh[k] - sh[a]) - (rh[k] - rh[a])))
            if ye > ymax:
                ymax = ye
            if k > a:
                path += math.hypot(rx[k] - rx[k - 1], ry[k] - ry[k - 1])
        pm[wi] = pmax
        ym[wi] = ymax
        mv[wi] = valid[wi] and path > MOVING_PATH_M
        wi += 1
    cnt = 0
    for i in range(nwin):
        if mv[i]:
            cnt += 1
    if cnt == 0:
        sel_p, sel_y = pm, ym
    else:
        sel_p = np.zeros(cnt)
        sel_y = np.zeros(cnt)
        c = 0
        for i in range(nwin):
            if mv[i]:
                sel_p[c] = pm[i]
                sel_y[c] = ym[i]
                c += 1
    out[0] = np.percentile(sel_p, 95)
    out[1] = np.percentile(sel_y, 95)
    out[2] = math.hypot(sx[n - 1] - rx[n - 1], sy[n - 1] - ry[n - 1])
    out[3] = _wrap(sh[n - 1] - rh[n - 1])
    out[4] = sel_p.max()
    out[5] = cnt


# no cache: numba does not invalidate a cached caller when its callee in another module changes
@njit
def sim_pose(p, et, ev, ew, n, dt, hop, ox, oy, oh):
    """Pose at the start of each of `n` steps (arrays of length n) for one drive's recorded commands.
    `hop` delays the velocity reaching the body by that many steps (0 for the plugin; 1 for a replay node
    that publishes after it sees the clock)."""
    cv = np.zeros(n)
    cw = np.zeros(n)
    stepper.events_to_steps(et, ev, ew, dt, n, cv, cw)
    ov = np.zeros(n)
    ow = np.zeros(n)
    stepper.simulate(p, cv, cw, dt, ov, ow)
    px = np.zeros(n + 1)
    py = np.zeros(n + 1)
    ph = np.zeros(n + 1)
    sv = np.zeros(n)
    sw = np.zeros(n)
    for k in range(hop, n):
        sv[k] = ov[k - hop]
        sw[k] = ow[k - hop]
    stepper.integrate(sv, sw, dt, px, py, ph)
    for k in range(n):
        ox[k] = px[k]
        oy[k] = py[k]
        oh[k] = ph[k]


def pack(drives: list[Drive]) -> dict:
    """Concatenate drives into flat arrays for the compiled scorer."""
    return {
        "ev_off": np.cumsum([0] + [len(d.ev_t) for d in drives]).astype(np.int64),
        "g_off": np.cumsum([0] + [d.n for d in drives]).astype(np.int64),
        "et": np.concatenate([d.ev_t for d in drives]), "ev": np.concatenate([d.ev_v for d in drives]),
        "ew": np.concatenate([d.ev_w for d in drives]),
        "rx": np.concatenate([d.x for d in drives]), "ry": np.concatenate([d.y for d in drives]),
        "rh": np.concatenate([d.h for d in drives]),
        "mask": np.concatenate([np.zeros(d.n, np.uint8) if d.exclude is None else d.exclude.astype(np.uint8) for d in drives]),
    }


@njit
def score_all(ev_off, g_off, et, ev, ew, rx, ry, rh, mask, p, dt, hop, out):
    for k in range(len(ev_off) - 1):
        n = g_off[k + 1] - g_off[k]
        ox = np.zeros(n)
        oy = np.zeros(n)
        oh = np.zeros(n)
        sim_pose(p, et[ev_off[k]:ev_off[k + 1]], ev[ev_off[k]:ev_off[k + 1]], ew[ev_off[k]:ev_off[k + 1]],
                 n, dt, hop, ox, oy, oh)
        metrics(rx[g_off[k]:g_off[k + 1]], ry[g_off[k]:g_off[k + 1]], rh[g_off[k]:g_off[k + 1]], ox, oy, oh,
                mask[g_off[k]:g_off[k + 1]], out[k])


def run(packed: dict, p: np.ndarray, dt: float = DT, hop: int = 0) -> np.ndarray:
    """Per-drive metrics, shape (n_drives, 6)."""
    out = np.zeros((len(packed["ev_off"]) - 1, len(METRIC_NAMES)))
    score_all(packed["ev_off"], packed["g_off"], packed["et"], packed["ev"], packed["ew"],
              packed["rx"], packed["ry"], packed["rh"], packed["mask"], p, float(dt), int(hop), out)
    return out


def per_drive_objective(out: np.ndarray, yaw_weight: float = YAW_WEIGHT) -> np.ndarray:
    return out[:, 0] + yaw_weight * out[:, 1]


def objective(out: np.ndarray, yaw_weight: float = YAW_WEIGHT) -> float:
    return float(np.mean(per_drive_objective(out, yaw_weight)))


def offline_track(drive: Drive, p: np.ndarray, dt: float = DT, hop: int = 0):
    """The offline model's track for one drive on the grid: (x, y, h) arrays of length drive.n."""
    ox, oy, oh = np.zeros(drive.n), np.zeros(drive.n), np.zeros(drive.n)
    sim_pose(p, drive.ev_t, drive.ev_v, drive.ev_w, drive.n, dt, hop, ox, oy, oh)
    return ox, oy, oh


def compare(ref, sim, mask=None) -> np.ndarray:
    """Metrics of the track `sim` against the track `ref`; each is an (x, y, h) triple on the grid."""
    out = np.zeros(len(METRIC_NAMES))
    m = np.zeros(len(ref[0]), np.uint8) if mask is None else np.asarray(mask, np.uint8)
    metrics(ref[0], ref[1], ref[2], sim[0], sim[1], sim[2], m, out)
    return out


def track_objective(ref, sim, yaw_weight: float = YAW_WEIGHT) -> float:
    m = compare(ref, sim)
    return float(m[0] + yaw_weight * m[1])
