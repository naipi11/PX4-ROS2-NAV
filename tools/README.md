# Flight Tools

The scripts the acceptance flights are launched and read with. They are
evaluation tools: nothing here is built into the stack or run by a gate, and
none of them is a criterion (the mission check in
[`docs/testing.md`](../docs/testing.md) is). Run them from the repository
root on the host unless a line says "inside the container". What they write
(run logs, flags, series tables, recordings, replay binaries) goes to
`log/tools/`, which git ignores.

`log/tools/` is pruned like every log (`scripts/prune_sim_logs.sh`, a week):
an entry directly inside it goes when nothing in it was touched for seven
days. A directory holding a `.keep` file at its top is never deleted, and
that is how anything worth more than a week is kept:

| Kept directory | What it holds |
|---|---|
| `log/tools/journal/` | the goal journals (`goal_journal_*.md`) and the series tables |
| `log/tools/diagnostics/` | one-off scripts and data of single investigations |
| `log/tools/vio/`, `log/tools/replay/` | recorded flights for the offline replay (r575, r576; r779, r795) and the replay binaries |
| `log/tools/truth25/`, `log/tools/proof_leak/`, `log/tools/texture/`, `log/tools/stereo/` | truth grids and measurement data the documents cite |

A loose file cannot be kept: check logs (`run_rNNN.log`), gate logs and
flags go after a week. A new recording or result that has to stay goes into
a kept directory, or into a new directory with its own `.keep`.

Run IDs are unique labels such as `rNNN` or `p2_20261027_s01`. The series
keeps each flight under `log/runs/RUN/.keep`: `acceptance.log` is the same
output exposed at `log/tools/run_RUN.log`, so pruning the loose link does not
remove the retained evidence. `mission_check.json` is the existing checker's
result; `acceptance.json` combines it with the host verdict and command exits.

## Launching

| Tool | What it does | Use |
|---|---|---|
| `gates.sh` | The whole pre-commit gate in the dev container, detached: format, build, unit tests, script tests, quality. Verdict in `log/tools/gate_flag.txt` (`X=0` is green), output in `log/tools/gate_all.log`. | `./tools/gates.sh`, then poll the flag. Do not edit tracked files while it runs. |
| `series2.sh` | Acceptance flights one at a time on one clean commit. Rejects reused IDs and concurrent series; waits for the quiet host, stops the simulation before and after each flight, and stops at the first non-passing verdict. Keeps structured results and logs; appends the series journal. | `./tools/series2.sh r950 r951` (cameras); `./tools/series2.sh --lidar r954` (`--lidar` first); another scenario with `TARGET=sim-urban-light-lost-headless SMOKE_DURATION_S=400 ./tools/series2.sh r939`. |
| `host_verdict.py` | Whether a flight counts, by the host it was flown on (specification A7): `HOST valid` or `HOST VOID` with the reason, from the real-time factor at the median (0.82 on the stereo set, 0.97 on the lidar, of the factor the run asked for), the resource sampler's largest gap (2 s) and the age of the estimator's poses on their way to the autopilot (300 ms at the 95th percentile). The series launcher appends it to the series table after every flight. | `python3 tools/host_verdict.py r981` |
| `rtf_watchdog.py` | Whether a flight is being flown under a load that will void it, read while it flies: the median of the simulator's real-time factor over the last 30 sampled seconds (after the flight has got going) against the verdict's floor of the factor the run asked for. `WATCHDOG ok`, `slow` (exit 1) or `waiting`. | `python3 tools/rtf_watchdog.py r1098 --lidar --factor 1.0` |
| `fly_until_valid.sh` | Watches one series flight every 20 s. Retries only a watchdog-stopped or host-void flight, under a fresh run number; a valid failed flight or evidence error stops immediately. Preserves the series exit code; exhaustion exits 2. `TARGET`, `SMOKE_DURATION_S` and `REAL_TIME_FACTOR` pass through. Records `RESULT exit=...`, `VOID` or `WATCHDOG`, then `END`, in `log/tools/fly_until_valid.txt`. | `./tools/fly_until_valid.sh --lidar 1130 4`; `TARGET=sim-urban-low-battery-headless SMOKE_DURATION_S=1800 ./tools/fly_until_valid.sh 1140 4` |
| `mx.sh` | One flight of roadmap item 17's acceptance matrix by line number. Preserves the series exit code and records `DONE LINE RUN exit=CODE` in `log/tools/mx_flag.txt`; rejection records `REFUSED` with exit 3. | `./tools/mx.sh 5 r944 &` |
| `mxw.sh` | Waits up to 9.5 minutes for the exact matrix run ID. Prints `DONE exit=CODE` or `REFUSED exit=CODE` and returns that code; `RUNNING` only means the observation window ended. Never stops a flight. | `./tools/mxw.sh r944` |

Flights run strictly one at a time. A flight `host_verdict.py` calls void is
flown again, and a crash in it is not a defect. On a host that another task
loads from time to time, every flight goes through `fly_until_valid.sh` or
`record_until_pass.sh`: the load is noticed while the flight flies, the flight
is stopped, and it is flown again when the host is quiet; the foreign
processes are never touched (specification A4).

Series exit codes are documented in
[`docs/testing.md`](../docs/testing.md#retained-acceptance-results).
`HOST valid` alone is not a pass. A valid mission failure stops the series
and must be investigated, not retried into a green result.

## Recording

| Tool | What it does | Use |
|---|---|---|
| `record_flight.sh` | One flight of a named scenario (`point-to-point`, `light-lost`, `light-failure`, `low-battery`; the stereo set only; `long-failures` and `return-home` are set aside by the owner since 2026-10-08 and refused without `FLY_SET_ASIDE=1`, the owner's word) flown as its headless flight with the Gazebo window and RViz open, at a real-time factor of 0.6 unless `REAL_TIME_FACTOR` says otherwise, and recorded: two split videos in the directory, the world on the left and RViz on the right, third-person and top-down, each the whole flight at the flight's own pace. A recording whose window stood still, whose half was a slideshow while the vehicle moved, that is off the flight's clock by more than a second, that shows a frame further than half a second from its moment or that is shorter than a minute is `BAD`. The desktop is kept from going idle meanwhile; the windows open below the others and minimize themselves once they draw (the owner, 2026-10-08), and keep drawing minimized. The recorder's output is `log/tools/record_RUN.log`. | `./tools/record_flight.sh light-lost r1020 log/videos/2026-10-04` |
| `record_batch.sh` | Records the four allowed scenarios round robin. Only host-void flights or bad recordings are retried once at the end of the pass. A valid mission failure, evidence error or interruption stops the batch with a nonzero exit. Keeps videos, `index.md` and `batch.log` in the directory. | `./tools/record_batch.sh log/videos/2026-10-04 3 1020 &` |
| `record_index.py` | Indexes the retained `acceptance.json`, not an absence of `FAIL` text. Exit 0 (`OK`) requires a passing flight and two valid recordings; 2 (`RETRY`) means host void or a bad recording. Other verdicts print `STOP` and preserve the nonzero acceptance code. Missing evidence exits 3. | `python3 tools/record_index.py light-lost r1020 log/videos/2026-10-04` |
| `record_until_pass.sh` | Watches and records one scenario. Only watchdog/host-void flights or bad recordings are retried; valid failures, missing evidence and non-watchdog interruptions stop. The wrapper signals only its owned series and recorder, retaining partial evidence. Records each result in `log/tools/record_until_pass.txt`, then `END`. | `./tools/record_until_pass.sh point-to-point 1121 5 log/videos/2026-10-05 &` |
| `video_check.py` | A finished split video with no run beside it: its length, the different frames a second of each half in windows of 5 s (a slideshow shows about one, a smooth recording 19 to 24), and the sampled frames in which the world half holds the green of Gazebo's light gizmo. `VIDEO ok` or `VIDEO BAD` with the reasons. | `python3 tools/video_check.py log/videos/2026-10-05/*_follow.mp4` |
| `window_rig.sh` | The Gazebo window of a recording over a small test world, with no flight, inside the container: `rate SECONDS [NAME=value ...]` prints the window's redraws a second (24 is sound, 1 is a window waiting for a blank screen; the assignments try a remedy in its environment), `frame [SDF]` counts the bright green pixels of one frame (the gizmo of a light the window is told to visualize; the SDF goes inside the light). Both causes of the first batch's defects were found with it. | `bash /workspace/tools/window_rig.sh rate 20` |

A recorded flight the host's verdict counts is a flight like any other
(specification A11); a row that says `HOST VOID` is a demonstration. Lidar
flights are not recorded: the location is dark and the lidar carries no
light, so the world half of the picture shows nothing. A slowed flight of
any kind is `REAL_TIME_FACTOR=0.5 ./tools/series2.sh r1100`.

## Reading A Flight

| Tool | What it prints | Use |
|---|---|---|
| `mx_inspect.py` | Everything the acceptance asks of one flight: failing lines, speed, truth at the goal, the return's trigger and moment, light judgments, the least light flown in, gaze overshoots, tick time, real-time factor. | `python3 tools/mx_inspect.py r949 r950` |
| `mxs.sh` | The same, cut to the acceptance lines, with the count of failing lines. | `./tools/mxs.sh r949` |
| `homeway.py` | The way home of a returned flight: when and where the goal was given up, seconds and metres home, trail points, revocations. | `python3 tools/homeway.py r974` |
| `lost_inspect.py` | A light-lost flight by phase: dead reckoning, judgment, landing or contact, truth at each, the drift between. | `python3 tools/lost_inspect.py r939` |
| `crash_context.py` | State, readiness, executor and offboard lines in the seconds before `VEHICLE_DESTROYED`. | `python3 tools/crash_context.py r932 8` |
| `body_truth_clearance.py` | The true body's clearance to the true location over a flight (needs the 0.25 m truth grid in `log/tools/truth25`). | `python3 tools/body_truth_clearance.py r958` |
| `yaw_overshoot.py` | Gaze overshoots: the heading swinging more than 45 degrees past its target. | `python3 tools/yaw_overshoot.py r949` |
| `room_time.py`, `room_churn.py` | Seconds a flight spends in the shaft room at (53, -7), and the routes, holds and limiters inside it. Location-specific by design: they measure one register entry. | `python3 tools/room_time.py r958` |
| `sensor_range_compare.py`, `sensor_limit_why.py` | The braking contract's measured range and the speed it allowed; the ticks where the limit is low with the range high. | `python3 tools/sensor_range_compare.py r949` |

## Replaying The Visual-Inertial Estimator Offline (`vio/`)

A recorded flight (stereo frames, IMU, truth) replayed through the
repository's filter with no simulator: a change of the estimator is measured
on recordings before any flight.

| Tool | What it does | Use |
|---|---|---|
| `vio/record_flight.sh` | One flight with the recorders attached; the record lands in `log/tools/vio/RUN/`. Needs `build_recorder.sh` once. | `./tools/vio/record_flight.sh r575` |
| `vio/build_recorder.sh`, `vio/record_frames.cpp`, `vio/record_vio.py`, `vio/trim_record.py`, `vio/truth_from_run.py` | The recorder's parts: stereo frames from Gazebo transport, IMU and estimates from ROS, the trim to the flight, the truth from the run's pose capture. | inside the container: `bash /workspace/tools/vio/build_recorder.sh` |
| `vio/build_replay_src.sh`, `vio/replay_vio.cpp` | Builds the replay against the tree's `visual_inertial_odometry.cpp` and a tracker source. Options are `key=value`: `dark_from_s`, `dark_to_s` (frames blanked), `baro=1`, `drag=<1/s>`, `tag=<name>`; output `replay_<tag>.csv` beside the record. | inside the container: `bash /workspace/tools/vio/build_replay_src.sh /workspace/drone_city_nav/src/stereo_feature_tracker.cpp replay_vio` |
| `vio/drag_bench.sh`, `vio/drag_report.py` | Dark-window replays of two recorded flights with and without the rotor drag fusion (specification K21), and the drift table. | inside the container the bench, then `python3 tools/vio/drag_report.py` |
| `vio/dark_drift.py` | Position error of dark-window replays against the lit replay. | `python3 tools/vio/dark_drift.py log/tools/replay/r779` |
| `vio/convert_euroc.py` | Rectifies an ASL-format EuRoC stereo recording using its camera, IMU and Vicon calibration; retains original timing and exports Vicon only as a reference. Requires OpenCV, NumPy, SciPy and PyYAML. | Inside the container: `python3 tools/vio/convert_euroc.py log/tools/vio/real-data/V1_01_easy/mav0 log/tools/vio/real-data/v1_01_record` (output must not exist). |
| `vio/trajectory_report.py`, `vio/pair_reference.py` | Score a paired trajectory and associate a separately produced reference trajectory. Initial position/yaw alignment is optional; no scale is fitted. | `python3 tools/vio/trajectory_report.py RECORD/replay_euroc_full.csv --output RECORD/report.json` |
| `vio/replay_memory.py` | Runs actual ROS stereo depth and 3D obstacle memory on a calibrated record and healthy VIO estimates. Relays each received depth cloud once and captures the matching final raw snapshot; timeout fails rather than inventing an acknowledgement. | Inside the container, source `install/setup.bash`, then run `python3 tools/vio/replay_memory.py --record RECORD --estimate RECORD/replay_euroc_memory.csv --output-dir NEW_OUTPUT --max-frames 100`. |
| `lio/convert_ntu_viral.py` | Extracts original-stamp Ouster scans, VectorNav IMU and separate Leica prism positions from an NTU VIRAL ROS 1 bag. Requires NumPy, PyYAML and `rosbags` in the evaluation environment; `rosbags` is not a production dependency. | `python3 tools/lio/convert_ntu_viral.py BAG CALIBRATION_DIR NEW_RECORD` |
| `lio/build_replay.sh`, `lio/replay_lio.cpp`, `lio/compare_prism.py` | Builds the LIO-core replay, then compares the estimated prism position with the independently measured Leica position. The full-fit and prefix-fit holdout are reported separately. | Inside the container: `bash tools/lio/build_replay.sh replay_lio_ntu`; run `/workspace/log/tools/lio/replay_lio_ntu RECORD RECORD/lio_estimate.csv 0 2000000000`, then `python3 tools/lio/compare_prism.py RECORD/lio_estimate.csv RECORD/prism_reference.csv RECORD/metadata.json --report RECORD/lio_prism_report.json --paired RECORD/lio_prism_paired.csv`. |

For the real-recording Python tools in the project container, put the system
NumPy/SciPy packages first to avoid mixing an incompatible user-installed
NumPy with the distribution's SciPy:

```bash
export PYTHONPATH=/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}
```

For the EuRoC run reported in [`docs/localization.md`](../docs/localization.md),
build and run inside the container:

```bash
bash tools/vio/build_replay_src.sh /workspace/drone_city_nav/src/stereo_feature_tracker.cpp replay_vio_real
/workspace/log/tools/vio/replay_vio_real /workspace/log/tools/vio/real-data/v1_01_record \
  calibration=/workspace/log/tools/vio/real-data/v1_01_record/calibration.json \
  rest_start_s=2 start_s=4 tag=euroc_memory
```

The evaluated inputs are [EuRoC V1_01_easy](https://projects.asl.ethz.ch/datasets/euroc-mav)
(the [pinned archive mirror](https://huggingface.co/datasets/pepijn223/euroc-mirror/resolve/d05bd6a92a5f456562153f83d65805fea27cb57d/V1_01_easy.zip),
SHA-256 `a920fe5b5e69a6ad19b32f1cfaf90ac2f59d45dfd1ca18cc9722e07684ba45fa`)
and [NTU VIRAL eee_03](https://ntu-aris.github.io/ntu_viral_dataset/)
([pinned archive](https://huggingface.co/datasets/Yuan-Shenghai/NTU-VIRAL-DATASET/resolve/4a1b869551e7baff1eb6157a61c2f5c3e3d5ffbd/sequences/eee_03.zip),
SHA-256 `c842b269cbc9c31788b22095d49e49814499769ef22df9a0728c9e7f2a0f8b94`).

The two public recordings' own licenses restrict them to non-commercial use.
The reference results, dataset provenance and investigation scripts are kept on
the evaluation host in `log/tools/vio/real-data/.keep`; the datasets are not
part of the source tree. Both replays are open-loop and cannot prove flight
safety, camera braking range, or real-time performance.

## Measuring Stereo Visibility (`stereo/`)

These tools replay recorded images through `stereo_depth_node`; reference
depth is used only after the ROS node publishes. They do not run a flight or
establish a real-airframe safety guarantee.

- `prepare_middlebury.py` converts one official Middlebury Eval3 scene. Its
  PFM payload is already in disparity pixels: the official SDK uses the scale
  sign for byte order and does not multiply the values. Mask 0 is unknown,
  128 occluded, 255 nonoccluded. The right image is shifted without wrapping
  to align principal points; residual vertical rectification error is retained.
- `prepare_rgbnir.py` rectifies a recorded RGB-NIR pair, transforms the
  synchronized lidar into the left camera frame, and associates only points
  within 0.5 pixel per axis of a stereo sample. Missing lidar coverage is
  unknown; mutual stereo visibility of the sparse points is not assumed. The
  image-to-scan timestamp limit is explicit and does not deskew a lidar sweep.
- `replay_depth.py` runs the actual node with the prepared images and its
  explicit calibration. By default it restarts the node for each frame, so
  sparsely sampled records are not mistaken for continuous dark exposure.
  `--continuous` preserves history only for an actual continuous recording.
  A relaxed `--allowed-depth-error-m` is measurement-only and must not be
  used as a navigation guarantee.
- `evaluate_visibility.py` reports reference classes and sampled pixels
  separately, including hit and free-ray overreach. `calibrate_visibility.py`
  fits the worst calibration-recording p99 and reports held-out recording
  exceedances and missing observations; physical locations may overlap and
  the result is conditional, not a statistical bound. `build_braking_probe.sh`
  builds a probe of the existing C++ stopping contract using simulated x500
  parameters only.

For the retained RGB-NIR recordings, run inside the project container:

```bash
BASE=/workspace/log/tools/vio/depth_visibility_20261103
# h5py is an evaluation-only dependency; use the retained pinned wheel or
# install h5py 3.11.0 into a separate directory, not the production image.
export PYTHONPATH="$BASE/pydeps:/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
source install/setup.bash
python3 tools/stereo/prepare_rgbnir.py --scene-dir "$BASE/raw/01-27-21-26-23" \
  --output-dir "$BASE/repro/prepared" --max-lag-ms 20
python3 tools/stereo/replay_depth.py --input-dir "$BASE/repro/prepared" \
  --output-dir "$BASE/repro/replayed" --timeout-s 120
python3 tools/stereo/evaluate_visibility.py --input "$BASE/repro/replayed/21_26_20_089.npz" \
  --output "$BASE/repro/frame_report.json"
```

The evidence is retained under `log/tools/vio/depth_visibility_20261103/.keep`,
including the scene split and original input checksums. Middlebury grants use
and publication of its images and disparity maps with citation; the RGB-NIR
dataset card did not specify a license when inspected, so no redistribution or
commercial-use permission is inferred. The benchmark's static scenes do not
demonstrate moving-platform synchronization.

One-off diagnostics written for a single investigation stay in
`log/tools/diagnostics/` and are not tracked; the goal journals in
`log/tools/journal/` say which was used for what.
