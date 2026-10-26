# Build And Run Workflow

All build, test, quality, and simulation commands must use the container
workflow. Do not run ad-hoc top-level CMake commands on the host.

## Common Host Scripts

Run these from the repository root:

```bash
./scripts/bootstrap.sh
./scripts/build.sh
./scripts/test.sh
./scripts/sim_urban_point_to_point_gui.sh
./scripts/sim_urban_point_to_point_headless.sh
./scripts/sim_cooperative_traffic_urban_gui.sh
./scripts/sim_cooperative_traffic_urban_headless.sh
./scripts/stop_sim.sh
```

Use `./scripts/dev_shell.sh` when an interactive container shell is needed.
`./scripts/bootstrap.sh` prepares PX4, the workspace and urban assets, then starts
the point-to-point simulation. The frozen dev image must already be restored;
see `installation.md` and [Frozen Deployment Baselines](#frozen-deployment-baselines).

All host scripts go through `scripts/container_run.sh`. The container runner
sources the supported ROS 2 setup and the container-built `px4_msgs` setup
before running the requested command. Normal builds do not require manual
`source /opt/ros/...` or `source /opt/px4_msgs_ws/...` steps.

If you intentionally use a custom image or external dependency install, set:

```bash
ROS_SETUP_FILE=/path/to/ros/setup.bash
PX4_MSGS_SETUP_FILE=/path/to/px4_msgs/setup.bash
```

## Remote Simulation With Local Development

Keep the editable checkout on the development computer and use
`scripts/remote_sim.sh` to operate the separately provisioned Linux simulation
host. Source-controlled deployment entrypoints are in `deployment/windows/`
and `deployment/systemd/`; do not edit generated desktop adapters or installed
service copies as the source of truth.

On the dedicated simulation checkout, after restoring dependencies:

```bash
bash deployment/systemd/install.sh --managed-mirror
```

This installs the simulation, x11vnc and noVNC user services and marks this
checkout with the ignored `.remote-sim-managed` file. It does not start, stop,
enable services or change credentials. The checkout location is
`~/projects/px4-ros2-drone-nav`; run the installer as its non-root owner.

On Windows, regenerate the five desktop adapters from the editable checkout:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "\\wsl.localhost\Ubuntu\home\stack\projects\px4-ros2-drone-nav\deployment\windows\install.ps1"
```

The adapters invoke the tracked controller directly. Installer parameters allow
another desktop directory, WSL distribution, local checkout or SSH destination.
After the regenerated adapters work, the old desktop `remote_sim.ps1` is obsolete.

```bash
export DRONE_REMOTE_HOST=jd@192.168.110.35
export DRONE_REMOTE_REPO=/home/jd/projects/px4-ros2-drone-nav
# In WSL, reuse the Windows SSH keys when that is the configured SSH client:
export DRONE_REMOTE_SSH=/mnt/c/Windows/System32/OpenSSH/ssh.exe

./scripts/remote_sim.sh build
./scripts/remote_sim.sh start
./scripts/remote_sim.sh status
./scripts/remote_sim.sh logs
./scripts/remote_sim.sh stop
```

`start` stops the old flight, mirrors local source and starts the remote service;
the normal simulation target performs an incremental workspace build. `build`
also stops and syncs, but only builds. Standalone `sync` refuses while the
simulation service is active. The local source is authoritative: sync includes
Git-indexed source (including staged additions) and portable Git identity/object
metadata. It deletes obsolete tracked source but preserves untracked remote
dependencies, evidence, caches and private files. Run `git add` for a new source
file before syncing; untracked edits are deliberately not deployed.

Host Git configuration, hooks, reflogs, local excludes, alternate-object paths,
`.colcon/` and the remote marker are not mirrored. Generated binaries stay on
their own host. SSH uses existing key authentication; no login or sudo password
is stored in source.

For Windows OpenSSH through WSL, the helper uses blocking rsync transport and
closes stdin for control-only SSH calls. These settings avoid intermittent
`write failed 11` transfer failures and interactive command hangs.

The GUI service inherits `DISPLAY` and `XAUTHORITY` from the remote systemd user
manager. Keep that desktop session logged in. Render Gazebo and RViz on the
simulation host, not through X11 forwarding to the development computer.

The local Windows deployment provides desktop launchers for start, stop,
sync/build, status and viewing. Its authenticated noVNC viewer reaches remote
loopback port 6087 through an SSH tunnel on local port 16087. The separate VNC
credential is stored privately on the simulation host and printed by the viewer
launcher, never embedded in a browser URL. Closing the viewer/tunnel does not
stop the remote simulation; use the stop launcher or command explicitly.

## Frozen Deployment Baselines

The first deployment baseline is named `baseline-20261026`, on development branch
`deployment/baseline-20261026`, derived from upstream `v0.5.0`
(`3973331c463f1c96c78c47bb9d12b441675b96b8`). It preserves the deployed navigation
algorithms and parameters; it is not a claim of completing the ten-flight
navigation acceptance series.

| Item | Versioned source | Generated or host-private state |
|---|---|---|
| Navigation, scenarios and models | `drone_city_nav/`, `environments/` | `build*/`, `install*/`, `log*/`, Python caches |
| Container | `docker/Dockerfile`, `docker/dev-image.lock` | Saved image archive and installed Docker image |
| Deployment | `scripts/remote_sim.sh`, `deployment/windows/`, `deployment/systemd/` | Desktop adapters, installed units, `.remote-sim-managed` |
| PX4 and scene assets | Upstream pins/manifests in setup scripts and environment definitions | `external/PX4-Autopilot`, installed/derived assets, download caches |
| Host customization and secrets | No credentials in source | `.colcon/`, `deployment/local/`, `.env*`, SSH keys, private VNC password files |

The lock identifies the existing image
`sha256:66a6f4056db34af7b3e0e2d2f73d0822be00dae059866976a2d6a53aea6bcdce`.
Wrappers use that ID, not whichever image a mutable `latest` tag names.
`DRONE_GAZEBO_DEV_IMAGE` remains an explicit non-baseline override; a tag override
is resolved once before launch. Run manifests and resource evidence record the
actual ID.

After checks, commit the source and stop the flight. Capture on the simulation
host, outside the checkout, with no concurrent source or dependency edits:

```bash
python3 scripts/deployment_baseline.py capture \
  /home/jd/px4-nav-baselines/baseline-20261026 \
  --run-directory log/runs/RUN_ID
python3 scripts/deployment_baseline.py verify \
  /home/jd/px4-nav-baselines/baseline-20261026
```

Use the real run directory in place of `RUN_ID`; omit `--run-directory` only for
a snapshot without flight evidence. Capture refuses dirty source, a running
simulation, an unavailable locked image, or evidence from another commit/image.
It never overwrites a previous snapshot. A partial directory without
`baseline.json` is not a completed baseline.

The snapshot contains:

- `source.tar.gz`: tracked source plus the Git history and tags.
- `dependencies.tar.gz`: PX4 source/submodule state and installed/derived scene
  assets, excluding the PX4 build tree. No network fetch is needed to restore
  these bytes.
- `dev-image.tar.gz`: the actual immutable Docker image, not a Dockerfile rebuild.
- `packages.tsv`, `px4-submodules.txt`, optional per-host colcon defaults, and
  `baseline.json` with commit, image ID, host versions, PX4 status, sizes and
  SHA-256 hashes.
- When requested, small flight evidence records and the observed mission-result
  lines. The complete original run is retained with `.keep`; absence of a mission
  result is recorded as absence, never as success.

PX4 is pinned at `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`. The existing checkout
has partial/different submodules for unused hardware and alternate simulators;
the snapshot records and preserves that state rather than falsely labelling the
entire PX4 tree clean. The Gazebo SITL dependencies actually consumed are at their
parent-pinned commits. This baseline does not certify other PX4 board targets.

### Restore Without Re-downloading Dependencies

Verify the snapshot first. Extract into a **new empty directory**, never over a
working checkout. The final service installation expects the standard project
location above; an isolated restore can be inspected or built elsewhere:

```bash
BASELINE=/home/jd/px4-nav-baselines/baseline-20261026
RESTORE=/home/jd/px4-nav-baselines/restore-check
mkdir "$RESTORE"
tar -xzf "$BASELINE/source.tar.gz" -C "$RESTORE"
tar -xzf "$BASELINE/dependencies.tar.gz" -C "$RESTORE"
docker load --input "$BASELINE/dev-image.tar.gz"
git -C "$RESTORE" rev-parse HEAD
```

The restored commit must equal `baseline.json.repository.commit`; the installed
image ID must equal `container_image`. Rebuild PX4 and the workspace **inside the
saved container**, using `scripts/container_run.sh`; build/install trees are not
portable between machines. Install host-specific units/adapters from tracked
definitions, and provision private viewer credentials separately. The baseline
does not archive SSH keys, VNC credentials or a whole desktop session.

Online APT repositories and upstream branches remain mutable. The reproducibility
claim is restoration from the saved image and dependency archives, not a
bit-identical new Docker build from the Internet. Keep an independent copy of the
snapshot; a checksum verifies integrity, not the trustworthiness of its author.


## Commands Inside The Dev Shell

Inside `./scripts/dev_shell.sh`, use:

```bash
make build
make test
make test-scripts
make quality
make format
make sim-urban-point-to-point-gui
make sim-urban-point-to-point-headless
make sim-cooperative-traffic-urban-gui
make sim-cooperative-traffic-urban-headless
```

`make build` runs `colcon build` for `drone_city_nav` with build, install, and
log directories rooted at `build/`, `install/`, and `log/`.

`make test` builds and then runs:

```bash
ctest --test-dir build/drone_city_nav --output-on-failure
```

`make test-scripts` runs Python script-level tests:

```bash
python3 -m unittest discover scripts/tests
```

`make quality` runs the repository quality gate, including dry-run formatting,
build, C++ tests, scoped `clang-tidy`, and scoped `cppcheck` when the required
inputs are available.

## Simulation Commands

Point-to-point mission:

```bash
./scripts/sim_urban_point_to_point_gui.sh
./scripts/sim_urban_point_to_point_headless.sh
```

Both fly the default sensor set, the forward stereo pair and the two
time-of-flight sensors, with the lidar absent from the vehicle, and the
`visual_inertial` localization profile: no GNSS and no magnetometer either.
The 3D lidar profile, which defaults to `LOCALIZATION_PROFILE=lidar_inertial`,
is a request, as is `LOCALIZATION_PROFILE=gnss` on either sensor set:

```bash
./scripts/sim_urban_point_to_point_lidar_headless.sh
```

A headless stereo flight of the mission takes 245 to 459 s of wall time on the
reference workstation (real-time factor 0.8 to 1.0); the target's flight
window is 600 s.

Finite cooperative traffic mission:

```bash
./scripts/sim_cooperative_traffic_urban_gui.sh
./scripts/sim_cooperative_traffic_urban_headless.sh
```

The cooperative launch carries the same sensor defaults per vehicle and has
not been flown on them; `CAMERA_PROFILE=none NAVIGATION_SENSOR_PROFILE=lidar`
is the configuration it was accepted with.

The complete finite scenario is defined in
`drone_city_nav/config/cooperative_traffic_urban_scenario.json`. The shell
runner and ROS launch both load this file, and each Gazebo spawn is derived from
the world's `map_to_sdf` transform. Set `MULTI_VEHICLE_SCENARIO_PATH` to run
another validated scenario; do not add independent shell spawn overrides.

Every vehicle plans its own route and publishes its own flight intent. Conflicts
are resolved between peers through those intents; no node plans for another
vehicle and no vehicle receives the physical truth of another.

The mission start barrier waits for every planner world and for confirmed
agreement between each PX4 navigation pose and its physical Gazebo model pose. A
persistent mismatch blocks mission start and commands airborne vehicles to hold.
Static planner readiness comes from the resident Occupancy3D ESDF and does not
wait for a lidar snapshot.

The headless command validates every PX4 log. The mission result requires all
vehicles to reach their goals with their separation contract satisfied, or it
records a typed failure. The GUI command keeps Gazebo and RViz open after either
outcome; stop it explicitly when inspection is complete.

A mission error never requests disarm. Force-disarm occurs only after a typed
physical-collision or proximity-collision destruction event, and a physical
crash is a failed technical run settled only after that vehicle's disarm and the
confirmed holds of the survivors.

Stop simulator leftovers:

```bash
./scripts/stop_sim.sh
```

Preview cleanup without terminating processes:

```bash
./scripts/stop_sim.sh --dry-run
```

The GUI workflow logs Gazebo server output to `log/gz_drone_nav.log` and Gazebo
GUI output to `log/gz_gui_drone_nav.log`.

## Logs And Artifacts

Important runtime artifacts:

- `log/mppi/` - production MPPI JSONL diagnostics;
- `log/lidar_memory_hits/` - accepted and classified lidar-memory hits;
- `log/gazebo_scene_debug/` - bounded Gazebo scene diagnostics;
- `build/` - colcon build tree;
- `install/` - colcon install tree.

Generated build outputs and logs are not project source and should not be
committed.

## Debug Bags

Record a debug ROS bag while simulation is running:

```bash
./scripts/record_debug_bag.sh
```

The script records the main planning, obstacle, and debug topics used for
post-run analysis.
