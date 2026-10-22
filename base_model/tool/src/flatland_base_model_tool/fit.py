"""Offline fit: Optuna TPE over the model parameters, several seeds, the best chosen on the FIT set only.

The hold-out set is scored once, after the choice, and never used to pick a seed or a trial.
The jerk limits and the speed caps are not searched: they are given (`base`), by default unlimited.
"""
from __future__ import annotations

import concurrent.futures as cf
import json
import os
import sys
from dataclasses import dataclass, field

import numpy as np

from . import scoring, stepper
from .drives import DT, Drive

DEFAULT_SEEDS = (1, 2, 3, 4, 5)
DEFAULT_TRIALS = 3000

# Generic default search ranges, not tuned to any platform (override with `--range NAME=LOW:HIGH`).
# name -> (low, high, log scale)
RANGES = {
    "delay_s": (0.0, 0.30, False),
    "tau_lin_s": (0.0, 0.5, False),
    "tau_ang_s": (0.0, 0.5, False),
    "a_lin_up": (0.3, 5.0, True),
    "a_lin_dn": (0.3, 5.0, True),
    "a_ang": (0.2, 8.0, True),
    "yaw_scale": (0.8, 1.2, False),
}


# Optional parameters that a fit may search when asked (`--search`); the upper bound is large enough that the
# per-wheel cap never binds, so the fit can reject the option.
OPTIONAL_RANGES = {
    "wheel_acc_max": (0.5, 20.0, True),
    "wheel_dec_max": (0.5, 20.0, True),
}


@dataclass
class SeedResult:
    seed: int
    fit_objective: float
    values: dict
    trials: int = 0


@dataclass
class FitResult:
    values: dict
    seed: int
    dt: float
    seeds: list[SeedResult] = field(default_factory=list)
    fit_objective: float = 0.0
    holdout_objective: float = 0.0
    fit_per_drive: list[float] = field(default_factory=list)
    holdout_per_drive: list[float] = field(default_factory=list)


def _values(trial_values: dict, dt: float, fixed: dict | None = None, base: dict | None = None) -> dict:
    """Search values -> full parameter dict, with the delay as the plugin will use it (whole steps)."""
    v = dict(stepper.IDENTITY)
    v.update(base or {})
    v.update(trial_values)
    v.update(fixed or {})
    v["delay_s"] = stepper.delay_steps(v["delay_s"], dt) * dt
    return v


def fit_seed(packed_fit: dict, seed: int, n_trials: int, dt: float = DT, fixed: dict | None = None,
             search: tuple = (), base: dict | None = None, ranges: dict | None = None) -> SeedResult:
    import optuna

    optuna.logging.set_verbosity(optuna.logging.WARNING)

    def objective(trial):
        tv = {}
        for name, (lo, hi, log) in (ranges or RANGES).items():
            if name in (fixed or {}):
                continue
            tv[name] = trial.suggest_float(name, lo, hi, log=log)
        for name in search:
            lo, hi, log = OPTIONAL_RANGES[name]
            tv[name] = trial.suggest_float(name, lo, hi, log=log)
        p = stepper.vector(_values(tv, dt, fixed, base))
        return scoring.objective(scoring.run(packed_fit, p, dt))

    study = optuna.create_study(direction="minimize", sampler=optuna.samplers.TPESampler(seed=seed))
    study.optimize(objective, n_trials=n_trials)
    return SeedResult(seed, float(study.best_value), _values(study.best_params, dt, fixed, base), n_trials)


def _pin_worker(queue) -> None:
    """Pin this worker process to one CPU taken from the queue, and print its PID so the placement can be checked."""
    cpu = queue.get()
    os.sched_setaffinity(0, {cpu})
    print(f"fit worker pid {os.getpid()} pinned to cpu {cpu}", flush=True)


def _worker(args):
    packed, seed, n_trials, dt, fixed, search, base, ranges = args
    return fit_seed(packed, seed, n_trials, dt, fixed, search, base, ranges)


def fit(fit_drives: list[Drive], holdout_drives: list[Drive], seeds=DEFAULT_SEEDS, n_trials=DEFAULT_TRIALS,
        jobs: int | None = None, dt: float = DT, fixed: dict | None = None, pin_cpus=None, search: tuple = (), base: dict | None = None, ranges: dict | None = None) -> FitResult:
    packed_fit = scoring.pack(fit_drives)
    jobs = jobs or min(len(seeds), os.cpu_count() or 1)
    for k in fixed or {}:
        if k not in RANGES and k not in stepper.OPTIONAL_NAMES:
            raise ValueError(f"cannot fix {k}: the fit searches {', '.join(RANGES)}; options: {', '.join(stepper.OPTIONAL_NAMES)}")
    for k in search:
        if k not in OPTIONAL_RANGES:
            raise ValueError(f"cannot search {k}: options are {', '.join(OPTIONAL_RANGES)}")
    for k in ranges or {}:
        if k not in RANGES:
            raise ValueError(f"cannot set a range for {k}: the fit searches {', '.join(RANGES)}")
    work = [(packed_fit, s, n_trials, dt, fixed, tuple(search), base, {**RANGES, **(ranges or {})}) for s in seeds]
    if jobs <= 1:
        results = [_worker(w) for w in work]
    else:
        init = {}
        if pin_cpus:
            import multiprocessing as mp
            queue = mp.Queue()
            for c in list(pin_cpus)[:jobs]:
                queue.put(c)
            init = {"initializer": _pin_worker, "initargs": (queue,)}
        with cf.ProcessPoolExecutor(max_workers=jobs, **init) as pool:
            results = list(pool.map(_worker, work))
    best = min(results, key=lambda r: (r.fit_objective, r.seed))   # FIT objective only
    p = stepper.vector(best.values)
    fit_out = scoring.run(packed_fit, p, dt)
    hold_out = scoring.run(scoring.pack(holdout_drives), p, dt)
    return FitResult(best.values, best.seed, dt, results, scoring.objective(fit_out), scoring.objective(hold_out),
                     scoring.per_drive_objective(fit_out).tolist(), scoring.per_drive_objective(hold_out).tolist())


def summary(result: FitResult) -> str:
    lines = [f"seed {r.seed}: fit objective {r.fit_objective:.4f}" for r in result.seeds]
    lines.append(f"chosen seed {result.seed} (lowest fit objective): fit {result.fit_objective:.4f}, "
                 f"hold-out {result.holdout_objective:.4f}")
    lines.append(json.dumps({k: round(v, 5) for k, v in result.values.items()}))
    return "\n".join(lines)


if __name__ == "__main__":  # pragma: no cover
    sys.exit("use `fbm fit`")
