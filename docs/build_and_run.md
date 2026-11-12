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
`./scripts/bootstrap.sh` prepares a fresh clone end to end (dev image, PX4
checkout and SITL build, workspace build, urban assets) and starts the urban
point-to-point simulation; see `installation.md`.

All host scripts go through `scripts/container_run.sh`. The container runner
sources the supported ROS 2 setup and the container-built `px4_msgs` setup
before running the requested command. Normal builds do not require manual
`source /opt/ros/...` or `source /opt/px4_msgs_ws/...` steps.

If you intentionally use a custom image or external dependency install, set:

```bash
ROS_SETUP_FILE=/path/to/ros/setup.bash
PX4_MSGS_SETUP_FILE=/path/to/px4_msgs/setup.bash
```

## The Onboard Build (Roadmap Item 27 Block 2)

What a flight computer carries is built without the simulator:
`drone_city_nav/CMakeLists.txt` puts everything that needs Gazebo or SDFormat
(the contact system, the truth adapter, the smoke relay, the Gazebo stereo
process, the heading source, the crash node, the SDF voxelizer) behind
`DRONE_CITY_NAV_ENABLE_SIMULATION` (ON in the development image), and the
Dockerfile's `onboard` stage carries ROS 2 base, the messages, Eigen, OpenCV,
zstd, `px4_msgs` and CUDA 13 (on amd64; a Jetson brings CUDA with JetPack)
and no Gazebo. `tools/onboard_image.sh` builds the stage as
`drone-gazebo-onboard:amd64`, then the package in it from the mounted
workspace with the option off (into `build-onboard-amd64/`) and runs the
unit tests, the CUDA ones on the host's GPU; `--arm64` builds the arm64
stage under emulation (`qemu-user-static`, `binfmt-support`) without CUDA
and tests it there, slowly. The development image is the Dockerfile's `dev`
stage (`scripts/build_dev_image.sh`). Measured on 2026-10-25: amd64: the image 2.35 GB with 0 Gazebo packages and CUDA 13.0; the package with the option off built in 6 min and its 154 unit tests passed (the seven CUDA ones on the host's GPU). arm64 under emulation: the stage 1.75 GB (77 min to build, px4_msgs's generation the long part), the package in 1 h 2 min on 16 threads, its warnings a subset of the amd64 build's, 140 of 142 unit tests passed (139 at first; the estimator test's ctest timeout raised from 60 to 600 s, its 4.7 s running past a minute at the emulation's speed); the two left, the D* Lite repair's share of a 20 ms budget and the initial search's certification within its budget, assume the development host's speed and are not x86 dependencies, of which none was found.

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

## The MAVLink Gate On A Shared Host

PX4 SITL sends its GCS stream to `127.0.0.1:14550` and its offboard stream to
`14540`, so any process that binds either port receives that stream and can
command the vehicle over MAVLink. On 2026-11-12 a foreign `pymavlink` client on
the simulation host did exactly that, sending `DO_SET_MODE`, `ARM_DISARM` and
`NAV_TAKEOFF` into a running flight; PX4 entered `AUTO_TAKEOFF` and four
flights were destroyed while the stack's own logs looked like a stop-arbitration
defect. The record is `log/runs/nav_state17_root_cause_20261112.txt`.

Every `sim_*.sh` wrapper therefore runs `scripts/sitl_mavlink_guard.py` before
the launch:

- it probes both ports without `SO_REUSEADDR`, so a port another process holds
  is reported and the run is refused (`exit 3`) before anything is spawned;
- it then holds both ports for the flight and logs every mode, command or
  setpoint frame whose source system is not PX4's own to
  `log/mavlink_guard.log`; plain telemetry from another simulator on this host
  (its own `MAV_SYS_ID`) is not counted, since only a frame that can take the
  vehicle away from the stack is an intrusion;
- it frees them by itself when the wrapper exits, however it exits.

The navigation stack drives PX4 over uXRCE/DDS, so holding these ports costs the
flight nothing: the guard never sends, PX4 sees no ground station, and a flight
under the guard behaves exactly as one with nobody listening. `SITL_MAVLINK_GUARD=false`
flies without the gate.

The probe also covers the ports PX4 SITL itself listens on - `18570`,
`14580`, `14280` and `13030` - because our PX4 binds them moments after the
launch. Another PX4 on the same host (a second simulator, as on 2026-11-12)
holds those sockets, and two instances fighting over one set of ports produce
flights whose evidence describes neither vehicle; the run refuses instead
(`--listen-ports none` skips that part of the probe).

That gate covers what a local process can bind. It does not cover a peer on the
network: PX4's own UDP sockets are bound to `0.0.0.0` and its `-x` flag is the
FTP server, not a bind restriction, so a remote host can address `18570` or
`14580` directly - a flight under the guard logged `partner IP:
192.168.110.76`. `scripts/host_mavlink_firewall.sh` closes that on the host
(`sudo bash scripts/host_mavlink_firewall.sh apply`, `remove` to undo): a chain
that accepts those four UDP ports on `lo` only and drops them elsewhere, with a
`PX4_MAVLINK_DROP` log prefix and packet counters. It is not persisted across a
reboot, and it blocks a ground station in another machine by design
(`MAVLINK_FIREWALL_ALLOW_INTERFACES` keeps one).

Two simulators on one host cannot share these ports, and PX4 makes that worse
than it looks: its GCS stream goes to a hardcoded `127.0.0.1:14550` whatever the
instance number (`mavlink_main.cpp`, `-o 14550`), so every PX4 on the host sends
its ground-station stream to the same place. A guard on `14550` therefore holds
the port another simulator's operator expects, and an operator script that opens
`14550` with `source_system=255` - the pattern in this host's `/tmp/ap_*.py` -
sends its commands to whichever PX4 answers. Run one simulator at a time, or
point that operator at its own simulator's instance ports, and use
`SITL_MAVLINK_GUARD_PORTS` to narrow which ports a flight holds.

Even with both, a flight on a shared network should be read from PX4's own ulog
(`vehicle_command` source ids, `telemetry_status`, the logged
`partner IP`) before its result is believed.

## Logs And Artifacts

Important runtime artifacts:

- `log/mppi/` - production MPPI JSONL diagnostics;
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
