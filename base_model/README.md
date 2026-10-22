# BaseModelDrive: a flatland drive that follows measured behaviour of a real base

`flatland_plugins::BaseModelDrive` makes a simulated differential-drive robot move like a real one. This directory holds the parameter files, the shared test vectors, the calibration and validation tool, and a validation world. The plugin itself is part of the `flatland_plugins` package (`flatland_plugins/src/base_model_drive.cpp`, its headers and gtests next to the other plugins). `DiffDrive` is unchanged.

The stock `DiffDrive` applies the commanded velocity at once. A real base reacts to a command after a delay, follows it through lags and acceleration limits, and may turn slightly less than commanded. `BaseModelDrive` puts those effects into the simulation, with the numbers taken from a parameter file. A calibration tool fits a parameter file from recordings of the real robot.

| Path | Content |
|---|---|
| `params/` | parameter files: `identity` (no shaping) and `example` (made-up values that show the format). Also reachable as `flatland_plugins/base_model_params` |
| `test_vectors/` | command sequences and expected outputs. The Python stepper and the C++ model must both match `stepper.json` within 1e-9 |
| `tool/` | uv project `flatland-base-model-tool`: the `fbm` command line, with the Python reference stepper |
| `validation/` | a blank world, two generic model files and launch files for replaying drives through flatland |
| `example/` | the model patch for a generic model, as a diff |

This directory has no `package.xml`, so catkin does not build it. Flatland loads model plugins through pluginlib, and `BaseModelDrive` is registered in `flatland_plugins.xml` like the other plugins.

## What the model does

At each physics step, with the step length `dt` of that step:

1. **Command delay.** The model uses the command from `delay_s` ago. The delay is rounded to whole steps.
2. **First-order lags** `tau_lin_s` and `tau_ang_s`, applied to the command, discretised exactly for the actual `dt`.
3. **Rate limits.** The velocity moves toward the lagged command with limited acceleration (separate limits for speeding up and slowing down in the linear axis), limited jerk, and speed caps.
4. **Yaw-rate scale** `yaw_scale`, applied to the output.
5. **Exact zero at rest.** An axis whose command is exactly 0 and whose speed is below 1e-3 is set to exactly 0. Without this, a lag tail never reaches 0, and software that tests for motion with `!= 0.0` sees a moving robot after every stop.

The plugin sets one rigid-body motion on every body of the model, so weld joints carry no relative motion and add no drag. The densities of the model file are not changed.

### Odometry twist changed

Stock `DiffDrive` reports the **commanded** twist in its odometry. `BaseModelDrive` reports the **velocity of the body after the step**, as a real robot does when its odometry is a measured velocity. Odometry is published after the physics step, and the message stamp is the time of the state it carries. If you depend on the commanded twist in the odometry, take it from `/cmd_vel`.

## Install

Build the `flatland_plugins` package of this repository with catkin (ROS 1 Noetic, yaml-cpp). The plugin is part of its library, `libflatland_plugins_lib`, and the parameter files are installed with it. Run the unit tests with `catkin build flatland_plugins --catkin-make-args run_tests`; the tests of the base model are `base_model_shaper_test`, `base_model_params_test` and `command_playback_schedule_test`.

## Use it in a model

Swap the drive block in the robot's model file. Two lines change:

```diff
-  - type: DiffDrive
+  - type: BaseModelDrive
+    params_name: example
     name: drive
     body: base
```

`example/` holds this diff for a generic model. The other keys of the block (`body`, `odom_pub`, `pub_rate`, noise, `enable_*`) keep their meaning. To apply the change to your own file and check the result:

    cd tool
    uv run fbm patch-model path/to/robot.model.yaml --params-name example          # print the diff
    uv run fbm patch-model path/to/robot.model.yaml --params-file my-robot.yaml --in-place

The script edits only the drive entry, parses the result, and refuses if anything else changed. It also refuses a block that has `linear_dynamics` or `angular_dynamics`: the plugin rejects those keys, so that nothing limits the command twice. Remove them first.

Plugin keys that are new:

| Key | Meaning |
|---|---|
| `params_name` | a parameter file of `params/` (installed with `flatland_plugins`), by name |
| `params_file` | a path to a parameter file (flatland expands `$VARS` in model files). Give exactly one of the two |
| `provenance_pub` | topic for the latched provenance message (default `base_model/provenance`) |

The plugin **refuses to load** with a clear error on a missing file, a bad schema, an unknown `schema_version`, a value out of range, or a `*_dynamics` block. A restart of simulated time resets the state.

## Parameter files

A file is YAML with `schema_version`, `name`, `calibrated_step_s`, the twelve `parameters` (`.inf` means unlimited) and an optional free-form `provenance` mapping. When present, the plugin logs the provenance at load and publishes it on a latched topic; it is never used to drive. A non-mapping `provenance` is refused. `fbm fit` writes a provenance block as an optional record. `identity` applies the command unchanged and keeps the rigid-body motion and the post-step odometry twist. `example` has made-up round values and shows the format.

Check files with `uv run fbm check-params params/*.yaml`. There are no defaults for the parameters of a robot: a file names every one of them.

### Fit your own

    cd tool
    uv run fbm extract bag1.bag bag2.bag --out drives/        # list the files of one recording together
    uv run fbm fit drives/ --out my-robot.yaml --name my-robot --robot R --site S --platform-version V --odom-scale 1.0
    uv run fbm score my-robot.yaml drives/ --odom-scale 1.0

- `extract` reads consecutive splits of one recording as one stream, in time order, and writes no merged file. A file continues the recording before it if the gap at the boundary is at most 0.1 s, in `/odom` header stamps and in record time. A larger gap starts a new recording, and the boundary is reported. Only the first and last sample of a recording can cut a drive. A drive window extends 2 s before and after the moving run, but never past the middle of the pause to the next run, so the robot is at rest at both edges.
- A drive is excluded if it is cut at the recording ends, if manual control (`--manual-topic`, `--manual-field`) is on when it starts or takes over within its first 4 s, if it has no commands, or if the commanded distance does not explain the driven distance. Manual control that starts later ends the scored part of the drive at its onset.
- Latched topics (`--latched`, default `/map` and `/tf_static`) are looked up per recording and carried forward from the first file that holds them. A missing topic is reported, never taken from another recording. `--require-latched` makes it an error.
- **Hold stretches.** Periods when the platform is not under command control (for example manual handling) do not follow `/cmd_vel`, and for some time afterwards they may not either. `--hold-source` names the topics that tell (`bool:/topic:field`, or `active:/topic` for an actionlib status array with an ACTIVE goal), `--hold-settle-s` the time after a stretch that is also left out. The extractor records the stretches in each drive file, and scoring leaves out every 2 s window that touches one. Derive the settle time from your data with `holds.settle_curve`: the first bin of time since a stretch ended whose median angular-velocity error, and that of the next two bins, is inside the band of normal driving. A hold rule removes more windows at the start of drives than elsewhere, so check how many drive starts are left.
- `fit` runs Optuna (version pinned in `tool/pyproject.toml`) with several seeds, picks the best seed **on the fit set only**, and then scores the hold-out set once. The split is in time order and balanced by driven metres: the earliest drives up to `--fit-fraction` (default 0.5) of the metres fit, drives that start within `--embargo-s` (default 60 s) after the last fit drive are in neither set, and the rest hold out. The fit uses the timing of the plugin: a command acts at the next step, the delay is whole steps, and there is no extra hop.
- The extraction settings (`--pad-s`, `--moving-v`, `--moving-w`, `--gap-s`, `--min-dist-m`, `--min-turn-rad`, `--stitch-gap-s`, `--min-scored-s`, `--hold-max-s`, `--hold-merge-gap-s`) have generic defaults that are not tuned to any platform; set them for yours.
- The fit searches the delay, the lags, the linear and angular acceleration limits and the yaw scale. The jerk limits and the speed caps are not searched: give them with `--jerk-lin --jerk-ang --v-min --v-max --w-max` (default unlimited). The search ranges are generic defaults, not tuned to any platform: override one with `--range NAME=LOW:HIGH`. `--fix NAME=VALUE --fix-note TEXT` holds a searched parameter at a value you know from another source and records where it comes from.

**Reference.** The fit and the 2 s window scores compare with the **odometry, with its displacement scaled** by `--odom-scale` (the heading is not changed; `--odom-yaw-scale` exists to test that). The parameter file records the scale.
- If the odometry over- or under-reads distance, a model fitted to it unscaled reproduces that error. Measure the scale against something independent (a map, tags, a surveyed path, or a scan-matched track).
- With scan-matched tracks: `fbm attach-reference drives/ tracks/*.npz` attaches them (by time), `fbm derive-scale drives/` computes the scale (track chord length over odometry chord length in 10 s windows of at least 2 m, with the per-drive spread), and you pass it to `fit` and `score`. `fbm long-horizon PARAMS drives/` then reports, per drive and for the fitted model and the identity model, the final position error, the heading drift per metre and the ratio of realised distance to the track's.
- The window metric re-anchors every 2 s window at its start pose, so the noise of a track enters twice. Use a track for the long-horizon checks, not for the window score. Between scan matches a track follows odometry, so dynamics faster than a few tenths of a second cannot be checked against it.

**Minimum data for a fit:** `/cmd_vel`, and `/odom` with **header stamps** (a record time can lag the stamp by a varying amount, which shows up as false position error). The robot must have moved: drives of at least 3 m or 1.5 rad, with in-place turns if you want the angular limit to be identified. Drives with few turns at the angular limit leave that limit poorly identified; fix it from another source if you have one.

## Validity envelope

- **Step size.** The fit is valid at the step size in `calibrated_step_s`. The plugin warns once when the step differs by more than 1 %. Flatland's server scales its step with `sim_speed` in some launch files: run at the calibrated step. The maths is exact for any `dt`, but the delay is rounded to whole steps.
- **Robot.** A file describes one base and one software version of it.
- **What the fit sees.** Commands and odometry only. It does not model wheel slip, load, floor type, battery state or collisions.
- **One yaw scale.** A single `yaw_scale` applies to every turn. If the real ratio of turn to command differs between turning in place and turning while driving, the model is right for neither exactly. The angular lag and the angular limit can absorb part of the shortfall, so judge a file by its scores, not by this one number.
- **Closed loop.** The model is validated against recorded open-loop drives. That it helps a particular navigation stack is not shown here.

### Experimental options of the reference stepper

The Python stepper has options for hard stops while turning: `wheel_base_m` alone applies the rate limits per wheel, and `wheel_base_m` with `wheel_acc_max` (and optionally `wheel_dec_max`) keeps the axis limiter and adds a per-wheel acceleration cap after it. They are not implemented in the plugin, and the plugin refuses a file that carries them. Their vectors are in `test_vectors/stepper_wheel.json` and `stepper_cap.json`. `fbm fit --search wheel_acc_max wheel_dec_max` fits the caps; keep an option only if a hold-out comparison, including a long-horizon check, supports it.

## Validate in flatland

`validation/` holds a blank 160 m by 160 m world, a generic model with `BaseModelDrive` and the same model with stock `DiffDrive`, and launch files. Both models also carry `CommandPlayback`, a test-only plugin. It replays the recorded drives from inside flatland's step loop (so ROS delivery cannot race the step, at any simulation speed), moves the robot to the origin before each drive, records the ground-truth pose after every step, writes one track file per drive, and ends the run. The base-model world reads the parameter file from the launch argument `params`.

    cd tool
    uv run fbm playback-schedule DRIVES --out schedule.json            # hold-out drives by default (--set)
    roslaunch validation/launch/validation.launch world:=base_model params:=my-robot.yaml schedule:=schedule.json out:=TRACKS_PLUGIN
    roslaunch validation/launch/validation.launch world:=stock      schedule:=schedule.json out:=TRACKS_STOCK
    roslaunch validation/launch/validation.launch world:=base_model params:=my-robot.yaml schedule:=schedule.json out:=TRACKS_FAST sim_speed:=4

`sim_speed` scales the update rate; the step stays 0.02 s. A command published in step k is applied in step k+1, a hop of one step. Measure it once per speed with a run that uses the identity model (`params:=../params/identity.yaml`), then use it. On the host:

    uv run fbm validate hop DRIVES TRACKS_IDENTITY
    uv run fbm validate plugin-vs-offline PARAMS DRIVES TRACKS_PLUGIN --hop N
    uv run fbm validate fast-vs-slow PARAMS DRIVES TRACKS_PLUGIN TRACKS_FAST --hop-slow N --hop-fast M
    uv run fbm validate negative-control DRIVES TRACKS_PLUGIN TRACKS_STOCK

For information only, `validation/launch/ros_path.launch` replays through a ROS node that publishes `/cmd_vel` from outside flatland. `fbm validate hop --information` on its tracks reports the hop and the jitter of an ordinary command path; it is not a gate.

Each prints PASS or FAIL against bounds that are fixed in the code, and the exit status is 1 on a failure. The bounds come from what can differ between the offline stepper and the plugin (single-precision Box2D arithmetic) times a margin of 10; `flatland_base_model_tool.bounds` derives them. `plugin-vs-offline` takes the flatland ground-truth track as the reference and the offline model (with the hop) as the simulation. The negative control needs the stock model to score worse than the plugin on at least 80 % of drives, with the 95 % bootstrap interval of the paired mean difference above 0, against the scaled odometry. By default the hold-out drives are used (`--set`, `--fit-fraction`, `--embargo-s`).

## Tests

- `cd tool && uv run pytest`: the stepper, the test vectors, parameter files, the model patch, the bag extraction, the hold rule and the fit.
- `catkin build flatland_plugins --catkin-make-args run_tests`: the same vectors and parameter rules in C++.
- `uv run fbm vectors --out ../test_vectors/stepper.json` (from `tool/`) regenerates the vectors. A Python test fails if the checked-in file is not what the stepper produces.

## Compiled code and its cache

The reference stepper and the scorer are compiled with numba. Numba does not invalidate a cached function when a function it calls, in another module, changes. The tool therefore caches only functions that call nothing outside their own module; the callers are compiled at every process start. If you edit the tool and see a model option have no effect, remove `tool/src/flatland_base_model_tool/__pycache__/*.nbi` and `*.nbc`. `tests/test_options_guard.py` fails if any model option stops changing the stepper output or the scored track.
