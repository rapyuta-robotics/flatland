"""`fbm`: calibration and validation command line.

    fbm extract BAG... --out DRIVES      bags -> drive files
    fbm attach-reference DRIVES TRACK...  attach scan-matched reference tracks (optional)
    fbm derive-scale DRIVES               odometry scale from the attached tracks
    fbm long-horizon PARAMS DRIVES        final position, heading drift, distance against the tracks
    fbm fit DRIVES --out PARAMS.yaml     offline fit -> parameter file
    fbm score PARAMS DRIVES              score a parameter file on the fit and hold-out drives
    fbm check-params PARAMS...           load-time validation, as the plugin does it
    fbm patch-model MODEL ...            swap DiffDrive for BaseModelDrive in a model file
    fbm playback-schedule DRIVES --out F  schedule for the in-process replay plugin
    fbm validate WHAT ...                score flatland tracks (hop, plugin-vs-offline, fast-vs-slow, negative-control)
    fbm vectors --out FILE               regenerate the shared test vectors
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
from importlib import metadata
from pathlib import Path

from . import params, patch, stepper, vectors


def _load(args):
    from . import drives

    ds = drives.load_dir(args.drives, args.odom_scale, args.reference, args.odom_yaw_scale)
    args.reference_used = args.reference
    if drives.last_skipped:
        print(f"left out {len(drives.last_skipped)} drive(s) (no attached reference track, or no moving window outside the hold stretches): {', '.join(drives.last_skipped)}")
    return ds


def _split(args, every):
    from . import drives

    fit, hold, dropped = drives.time_split(every, args.fit_fraction, args.embargo_s)
    args.dropped = len(dropped)
    return fit, hold


def _reference_note(args, ds) -> dict:
    if args.reference_used == "track":
        return {"kind": "scan-matched track attached to each drive", "drives_with_track": len(ds),
                "note": "the track follows odometry between scan matches: dynamics faster than about 0.2 s come from odometry"}
    note = {"kind": "odometry, header stamps, displacement scaled", "odom_scale": args.odom_scale}
    if args.odom_yaw_scale != 1.0:
        note["odom_yaw_scale"] = args.odom_yaw_scale
    return note


def _provenance(args, result, fit_drives, hold_drives) -> dict:
    index_path = Path(args.drives) / "index.json"
    bags = []
    if index_path.exists():
        bags = [f for r in json.loads(index_path.read_text())["recordings"] for f in r["files"]]
    return {
        "robot": args.robot, "site": args.site, "platform_version": args.platform_version,
        "fitted_on": datetime.date.today().isoformat(),
        "files": bags,
        "fit_drives": len(fit_drives), "fit_metres": round(sum(d.dist for d in fit_drives), 1),
        "holdout_drives": len(hold_drives), "holdout_metres": round(sum(d.dist for d in hold_drives), 1),
        "holdout_score": round(result.holdout_objective, 4),
        "split": f"time order, balanced by metres: the first {args.fit_fraction:g} of the metres fit; "
                 f"{args.dropped} drive(s) within {args.embargo_s:g} s after the last fit drive in neither set; "
                 "the rest hold out",
        "reference": _reference_note(args, fit_drives + hold_drives),
        "fixed_parameters": {name: {"value": value, "source": args.fix_note or "not stated"} for name, value in _fixed(args)},
        "holdout_reference": "header-stamped /odom of the hold-out drives",
        "fit_score": round(result.fit_objective, 4),
        "objective": "mean over drives of window position p95 (m) + 0.5 * window heading p95 (rad); "
                     "2 s windows every 0.5 s, windows with more than 0.2 m of reference path",
        "timing": "plugin: command applied at the next step, delay rounded to whole steps, no extra hop",
        "seeds": {str(r.seed): round(r.fit_objective, 4) for r in result.seeds},
        "chosen_seed": result.seed, "trials_per_seed": result.seeds[0].trials if result.seeds else 0,
        "tool_version": metadata.version("flatland-base-model-tool"),
        "optuna_version": metadata.version("optuna"),
    }


def cmd_extract(args) -> int:
    from . import drives, holds

    drives.configure(args.pad_s, args.moving_v, args.moving_w, args.gap_s, args.min_dist_m, args.min_turn_rad,
                     args.stitch_gap_s, args.min_scored_s)
    holds.configure(args.hold_max_s, args.hold_merge_gap_s)
    index = drives.extract(args.bags, args.out, manual_topic=args.manual_topic or None, manual_field=args.manual_field,
                           latched=args.latched, hold_sources=args.hold_source or (), hold_settle_s=args.hold_settle_s)
    for r in index["recordings"]:
        why = {}
        for e in r["excluded"]:
            why[e["reason"]] = why.get(e["reason"], 0) + 1
        note = "" if r["manual_topic_present"] else "  (no manual-control topic: manual drives not excluded)"
        print(f"recording {r['files'][0]} +{len(r['files']) - 1}: {len(r['kept'])} kept, excluded {why or 'none'}{note}")
    print(drives.edge_report(index))
    missing = [t for r in index["recordings"] for t, src in r["latched"].items() if src is None]
    return 1 if (args.require_latched and missing) else 0


def cmd_attach(args) -> int:
    from . import drives

    r = drives.attach_references(args.drives, args.tracks)
    print(f"attached to {len(r['attached'])} drive(s); no track covers {len(r['missing'])}: {', '.join(r['missing']) or '-'}")
    return 0


def _ranges(args) -> dict:
    """Search ranges from `--range NAME=LOW:HIGH`; the log scale of a parameter is kept."""
    from . import fit

    out = {}
    for item in args.range or []:
        name, _, rest = item.partition("=")
        lo, _, hi = rest.partition(":")
        if name not in fit.RANGES or not lo or not hi:
            raise SystemExit(f"--range {item!r}: expected NAME=LOW:HIGH with NAME one of {', '.join(fit.RANGES)}")
        out[name] = (float(lo), float(hi), fit.RANGES[name][2])
    return out


def _base(args) -> dict:
    """Parameters that the fit does not search, from the command line; unlimited when not given."""
    base = {}
    for name in ("jerk_lin", "jerk_ang", "v_min", "v_max", "w_max"):
        value = getattr(args, name, None)
        if value is not None:
            base[name] = value
    return base


def _fixed(args) -> list:
    out = []
    for item in args.fix or []:
        name, _, value = item.partition("=")
        out.append((name, float(value)))
    return out


def cmd_derive_scale(args) -> int:
    from . import drives

    ds = drives.load_dir(args.drives, reference="track")
    if drives.last_skipped:
        print(f"no track for {len(drives.last_skipped)} drive(s), left out of the derivation")
    r = drives.derive_scale(ds)
    print(f"linear odometry scale (track over odometry): {r['linear']:.4f} from {r['metres']:.0f} m in {r['drives']} drives; "
          f"per drive median {r['linear_drive_median']:.4f}, 10th to 90th percentile "
          f"{r['linear_drive_p10_p90'][0]:.4f} to {r['linear_drive_p10_p90'][1]:.4f}")
    if r["yaw"] is not None:
        print(f"yaw scale (information; fits use 1 unless told otherwise): {r['yaw']:.4f}")
    print(f"use: --odom-scale {r['linear']:.4f}")
    return 0


def cmd_long_horizon(args) -> int:
    from . import drives, longhorizon

    ds = drives.load_dir(args.drives, args.odom_scale, "track", args.odom_yaw_scale)
    fit_d, hold_d, _ = drives.time_split(ds, args.fit_fraction, args.embargo_s)
    doc = params.load(args.params)
    dt = doc["calibrated_step_s"] or drives.DT
    compared = [(doc["name"], stepper.vector(doc["parameters"])), ("identity", stepper.vector(stepper.IDENTITY))]
    for label, part in (("fit", fit_d), ("hold-out", hold_d)):
        print(f"[{label}]")
        for name, p in compared:
            print(longhorizon.lines(name, longhorizon.summary(part, p, dt)))
    return 0


def cmd_fit(args) -> int:
    from . import drives, fit

    every = _load(args)
    if len(every) < 2:
        print("need at least 2 drives for a fit and a hold-out", file=sys.stderr)
        return 2
    fit_drives, hold_drives = _split(args, every)
    result = fit.fit(fit_drives, hold_drives, args.seeds, args.trials, args.jobs, args.dt, dict(_fixed(args)), args.pin_cpus, tuple(args.search or ()), _base(args), _ranges(args))
    print(fit.summary(result))
    doc = params.make(args.name, result.values, args.dt, _provenance(args, result, fit_drives, hold_drives))
    params.save(doc, args.out)
    print(f"wrote {args.out}")
    return 0


def cmd_score(args) -> int:
    from . import drives, scoring

    doc = params.load(args.params)
    every = _load(args)
    fit_drives, hold_drives = _split(args, every)
    dt = doc["calibrated_step_s"] or drives.DT
    compared = [(doc["name"], stepper.vector(doc["parameters"])), ("identity", stepper.vector(stepper.IDENTITY))]
    for label, ds in (("fit", fit_drives), ("hold-out", hold_drives)):
        packed = scoring.pack(ds)
        for name, p in compared:
            out = scoring.run(packed, p, dt)
            print(f"{label:9s} {name:24s} objective {scoring.objective(out):.4f}  ({len(ds)} drives)")
    return 0


def cmd_check(args) -> int:
    status = 0
    for path in args.params:
        try:
            doc = params.load(path)
            print(f"ok   {path}: {doc['name']}")
        except params.ParamFileError as e:
            print(f"FAIL {path}: {e}")
            status = 1
    return status


def cmd_patch(args) -> int:
    path = Path(args.model)
    text = path.read_text()
    try:
        out = patch.patch_text(text, args.params_name, args.params_file)
    except patch.PatchError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    if args.in_place:
        path.write_text(out)
    elif args.out:
        Path(args.out).write_text(out)
    else:
        sys.stdout.write(patch.unified_diff(text, out, path.name))
    return 0


def _validation_drives(args):
    from . import drives

    every = _load(args)
    fit_drives, hold_drives = _split(args, every)
    return {"fit": fit_drives, "hold": hold_drives, "all": every}[args.set]


def cmd_validate(args) -> int:
    from . import validate

    ds = _validation_drives(args)
    results = []
    if args.what == "hop":
        results.append(validate.estimate_hop(ds, validate.track_dir(args.tracks, ds)))
    elif args.what == "plugin-vs-offline":
        doc = params.load(args.params)
        results.append(validate.plugin_vs_offline(ds, validate.track_dir(args.tracks, ds), stepper.vector(doc["parameters"]),
                                                  doc["calibrated_step_s"] or 0.02, args.hop))
    elif args.what == "fast-vs-slow":
        doc = params.load(args.params)
        results += validate.fast_vs_slow(ds, validate.track_dir(args.slow, ds), validate.track_dir(args.fast, ds),
                                         stepper.vector(doc["parameters"]), args.hop_slow, args.hop_fast,
                                         doc["calibrated_step_s"] or 0.02)
    else:
        plugin = validate.track_dir(args.plugin, ds)
        baseline = validate.track_dir(args.baseline, ds)
        results.append(validate.negative_control(ds, {d.name: d.odom for d in ds}, baseline, plugin,
                                                 "scaled odometry"))
    for r in results:
        print(r.line())
    if args.what == "hop" and args.information:
        return 0
    return 0 if all(r.passed for r in results) else 1


def cmd_playback_schedule(args) -> int:
    from . import playback

    ds = _validation_drives(args)
    doc = playback.write(ds, args.out)
    last = doc["drives"][-1]
    print(f"wrote {args.out}: {len(ds)} drives, {last['start_s'] + last['length_s']:.0f} s of simulated time")
    return 0


def cmd_vectors(args) -> int:
    vectors.write(args.out)
    print(f"wrote {args.out}")
    return 0


def _split_args(p) -> None:
    p.add_argument("--fit-fraction", type=float, default=0.5,
                   help="the earliest drives, up to this fraction of the metres, fit (default 0.5)")
    p.add_argument("--embargo-s", type=float, default=60.0,
                   help="drives starting this soon after the last fit drive are in neither set (default 60)")
    p.add_argument("--reference", choices=("odom", "track"), default="odom",
                   help="what fits and window scores compare with: the odometry (default; scale it with --odom-scale) "
                        "or the track attached to each drive. The 2 s window metric double-counts the noise of a "
                        "track's start pose: tracks are for the long-horizon checks (`fbm long-horizon`)")
    p.add_argument("--odom-yaw-scale", type=float, default=1.0,
                   help="multiply the odometry heading change by this (default 1: only the displacement is corrected)")
    p.add_argument("--odom-scale", type=float, default=1.0,
                   help="multiply the odometry displacement by this (heading unchanged), when an independent "
                        "reference shows that the odometry over- or under-reads distance (default 1)")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="fbm", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("extract", help="bags -> drive files")
    p.add_argument("bags", nargs="+")
    p.add_argument("--out", required=True)
    p.add_argument("--manual-topic", default="",
                   help="topic carrying a manual-control field; drives with any non-zero sample are excluded. "
                        "Empty string (the default) disables the check")
    p.add_argument("--manual-field", default="data")
    p.add_argument("--latched", nargs="*", default=list(("/map", "/tf_static")),
                   help="latched topics to look for in each recording (carried forward from earlier splits)")
    p.add_argument("--hold-source", nargs="*", metavar="SOURCE",
                   help="periods when the platform is not under command control (for example manual handling): "
                        "bool:/topic:field or active:/topic (an actionlib GoalStatusArray with an ACTIVE goal). "
                        "Scoring leaves out windows near these times; needs --hold-settle-s")
    p.add_argument("--hold-settle-s", type=float, default=None,
                   help="seconds after a hold stretch that are also left out; derive it from your data (holds.settle_curve)")
    p.add_argument("--require-latched", action="store_true", help="exit 1 if a recording lacks a latched topic")
    g = p.add_argument_group("extraction settings (generic defaults, not tuned to any platform)")
    g.add_argument("--pad-s", type=float, help="seconds of rest kept before and after a drive (default 2)")
    g.add_argument("--moving-v", type=float, help="odometry speed above which the robot counts as moving, m/s (default 0.02)")
    g.add_argument("--moving-w", type=float, help="odometry yaw rate above which the robot counts as moving, rad/s (default 0.05)")
    g.add_argument("--gap-s", type=float, help="a pause this long ends a drive, s (default 1)")
    g.add_argument("--min-dist-m", type=float, help="shorter moves are not drives, unless they turn enough, m (default 3)")
    g.add_argument("--min-turn-rad", type=float, help="smaller turns are not drives, unless they move far enough, rad (default 1.5)")
    g.add_argument("--stitch-gap-s", type=float, help="largest gap at a file boundary that is stitched, s (default 0.1)")
    g.add_argument("--min-scored-s", type=float, help="commanded motion a manually interrupted drive must keep, s (default 4)")
    g.add_argument("--hold-max-s", type=float, help="a hold state is not held longer than this past its last message, s (default 1)")
    g.add_argument("--hold-merge-gap-s", type=float, help="hold stretches closer than this are one, s (default 1)")
    p.set_defaults(func=cmd_extract)

    p = sub.add_parser("attach-reference", help="attach scan-matched reference tracks to drive files")
    p.add_argument("drives")
    p.add_argument("tracks", nargs="+", help=".npz (t, x, y, yaw) or .json track files with absolute stamps")
    p.set_defaults(func=cmd_attach)

    p = sub.add_parser("derive-scale", help="odometry scale against the attached tracks")
    p.add_argument("drives")
    p.set_defaults(func=cmd_derive_scale)

    p = sub.add_parser("long-horizon", help="final position, heading drift and distance against the attached tracks")
    p.add_argument("params")
    p.add_argument("drives")
    _split_args(p)
    p.set_defaults(func=cmd_long_horizon)

    p = sub.add_parser("fit", help="offline fit -> parameter file")
    p.add_argument("drives")
    p.add_argument("--out", required=True)
    p.add_argument("--name", required=True, help="parameter set name; also the file stem it should be shipped as")
    p.add_argument("--robot", required=True)
    p.add_argument("--site", required=True)
    p.add_argument("--platform-version", required=True, help="software or firmware version of the platform, recorded as provenance")
    for name in ("jerk-lin", "jerk-ang", "v-min", "v-max", "w-max"):
        p.add_argument(f"--{name}", type=float, default=None,
                       help=f"value of {name.replace('-', '_')} for the fit (not searched); default unlimited")
    p.add_argument("--range", nargs="*", metavar="NAME=LOW:HIGH",
                   help="override the generic search range of a parameter (default ranges are not tuned to any platform)")
    p.add_argument("--fix", nargs="*", metavar="NAME=VALUE",
                   help="hold these searched parameters at a value (for example yaw_scale=0.9)")
    p.add_argument("--search", nargs="*", metavar="NAME",
                   help="also search these optional parameters (wheel_acc_max, wheel_dec_max)")
    p.add_argument("--pin-cpus", type=int, nargs="*", help="pin worker i to CPU pin_cpus[i] (one CPU per worker)")
    p.add_argument("--fix-note", help="where the fixed values come from (recorded in the provenance)")
    p.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5])
    p.add_argument("--trials", type=int, default=3000)
    p.add_argument("--jobs", type=int, default=None, help="worker processes (default: one per seed, up to the CPUs)")
    _split_args(p)
    p.add_argument("--dt", type=float, default=0.02, help="step size the file is calibrated for")
    p.set_defaults(func=cmd_fit)

    p = sub.add_parser("score", help="score a parameter file on drives")
    p.add_argument("params")
    p.add_argument("drives")
    _split_args(p)
    p.set_defaults(func=cmd_score)

    p = sub.add_parser("check-params", help="validate parameter files")
    p.add_argument("params", nargs="+")
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("patch-model", help="swap DiffDrive for BaseModelDrive in a model file")
    p.add_argument("model")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--params-name", help="a parameter file shipped with the package, by name")
    g.add_argument("--params-file", help="a parameter file path (flatland expands $VARS in model files)")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--in-place", action="store_true")
    g.add_argument("--out", help="write the patched model here (default: print the diff)")
    p.set_defaults(func=cmd_patch)

    p = sub.add_parser("validate", help="score flatland tracks against the reference and the offline model")
    vs = p.add_subparsers(dest="what", required=True)
    for name in ("hop", "plugin-vs-offline", "fast-vs-slow", "negative-control"):
        q = vs.add_parser(name)
        q.add_argument("--set", choices=("fit", "hold", "all"), default="hold")
        _split_args(q)
        q.set_defaults(func=cmd_validate)
        if name == "hop":
            q.add_argument("--information", action="store_true",
                           help="print the result but exit 0 (for the ROS command path, whose jitter is not a gate)")
            q.add_argument("drives")
            q.add_argument("tracks", help="tracks recorded with the identity model")
        elif name == "plugin-vs-offline":
            q.add_argument("params")
            q.add_argument("drives")
            q.add_argument("tracks", help="directory of tracks recorded from the plugin")
            q.add_argument("--hop", type=int, required=True, help="the measured hop of the replay path (`validate hop`)")
        elif name == "fast-vs-slow":
            q.add_argument("params")
            q.add_argument("drives")
            q.add_argument("slow", help="tracks at 1x")
            q.add_argument("fast", help="tracks with rate scaling")
            q.add_argument("--hop-slow", type=int, required=True)
            q.add_argument("--hop-fast", type=int, required=True)
        else:
            q.add_argument("drives")
            q.add_argument("plugin", help="tracks recorded from the plugin")
            q.add_argument("baseline", help="tracks recorded from the stock flatland model")

    p = sub.add_parser("playback-schedule", help="schedule of drives for the in-process replay plugin")
    p.add_argument("drives")
    p.add_argument("--out", required=True)
    p.add_argument("--set", choices=("fit", "hold", "all"), default="hold")
    _split_args(p)
    p.set_defaults(func=cmd_playback_schedule)

    p = sub.add_parser("vectors", help="regenerate the shared test vectors")
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_vectors)
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
