# Installation And Host Requirements

The supported workflow is container-based. The host provides Docker, display
access for GUI runs, and access to the repository files. ROS 2, Gazebo, PX4
messages, and build tools are provided by the project container image.

## Supported Host

The repository is developed for a Linux host. The scripts assume:

- a POSIX shell environment;
- Docker with permission to run containers;
- X11 or Wayland/XWayland support for GUI runs;
- a local checkout with normal user ownership.

The project may work on other host distributions, but the documented path is
the repository Docker workflow.

On WSL2, systemd services alone do not keep the distribution alive
([Microsoft's lifecycle note](https://learn.microsoft.com/en-us/windows/wsl/systemd#how-does-enabling-systemd-affect-wsl-architecture)).
If a local desktop launcher starts the container workflow through a systemd
service, keep its `wsl.exe` client attached until the service ends, for example
with `systemctl start --wait` after starting or restarting the service. Keep
that launcher window open and use a separate stop command to end the simulation.

## Required Host Packages

Install at least:

- `docker` or Docker Engine from your distribution/vendor;
- `git`;
- a shell with standard GNU/Linux command-line tools.

For GUI simulation, the host must allow containerized GUI applications to reach
the display server. The scripts handle the common X11 case; Wayland setups
usually need XWayland compatibility.

## WSL2 Graphics Acceleration

CUDA access and OpenGL rendering are separate: `--gpus all` and a successful
`CUDA_PREFLIGHT` do not prove that Gazebo or RViz renders on the GPU.

When `/dev/dxg` and `/usr/lib/wsl/lib/libd3d12.so` are present,
`scripts/container_run.sh` mounts `/usr/lib/wsl` read-only, including the WSL
bridge libraries and Windows driver store. It adds the bridge library path and
selects Mesa's `d3d12` Gallium driver. The adapter selector
`MESA_D3D12_DEFAULT_ADAPTER_NAME` defaults to `NVIDIA`; set that environment
variable before running the wrapper to select another adapter. Native Linux
hosts do not receive these WSL-specific mounts or settings.

In the running container, Gazebo records its actual renderer in
`${HOME}/.gz/rendering/ogre2.log`. On this WSL path, expect
`GL_RENDERER = D3D12 (NVIDIA ...)`; `llvmpipe` means CPU software rendering.
Check the actual GUI and simulation timing after restarting the container:
an OpenGL probe alone does not establish the simulator's frame rate.

## Docker Access

The user running the scripts must be able to run Docker. Usually this means the
user is in the `docker` group:

```bash
docker ps
```

If this fails with a permission error, fix host Docker permissions before using
the project scripts.

On native NVIDIA Linux hosts, the container runner requests the `compute`,
`utility`, `graphics`, and `display` driver capabilities. The graphics/display
libraries are required in addition to CUDA access; without them a container can
pass its CUDA check while OpenGL falls back to `llvmpipe`.

## What Not To Install On The Host

Do not install or depend on host ROS 2, host Gazebo, or host PX4 packages for
normal development. The dev image provides the supported environment.

Avoid mixing host and container build artifacts. The repository scripts run the
container with the current UID/GID so generated files remain owned by the host
user.

## Restore Or Deliberately Rebuild The Dev Image

The default is the immutable local image ID in `docker/dev-image.lock`.
Restore the archived image before running bootstrap or a container wrapper:

```bash
docker load --input /path/to/baseline/dev-image.tar.gz
```

The wrapper resolves even an explicit image tag to its actual content ID before
running it. Both `manifest.json` and `resources_host.json` record that ID.
Missing baseline images fail; no automatic build or pull can substitute `latest`.

To deliberately build and use a non-baseline image:

```bash
./scripts/build_dev_image.sh
export DRONE_GAZEBO_DEV_IMAGE=drone-gazebo-dev:latest
```

This does not update `docker/dev-image.lock`. Online APT repositories and upstream
branches can change, so rebuilding the Dockerfile is not a byte-reproducible
restoration. Save and verify the existing image archive to reproduce a baseline;
change the lock deliberately only when adopting a new dependency baseline.

The ROS 2/Gazebo package installation uses locked BuildKit APT caches and
retries transient downloads ten times. If a build still fails on a network
error, rerun it with the same Docker builder: completed package downloads are
retained in its cache. Repository signature verification remains enabled.

On networks where the upstream Ubuntu/ROS servers are unreliable, the optional
`APT_MIRROR_ROOT` build argument changes Ubuntu/ROS download URLs without changing
signing keys or disabling verification. Mirror mode configures ROS binary
repositories (`Types: deb`), as documented by the
[Tsinghua mirror](https://mirrors.tuna.tsinghua.edu.cn/help/ros2/); its source
package indexes are not mirrored. Ubuntu source types remain unchanged.

```bash
docker build --build-arg APT_MIRROR_ROOT=https://mirrors.tuna.tsinghua.edu.cn \
  -f docker/Dockerfile -t drone-gazebo-dev:latest .
```

The container entrypoint sources the supported ROS and `px4_msgs` setup files
automatically:

```text
/opt/ros/${ROS_DISTRO}/setup.bash
/opt/px4_msgs_ws/install/setup.bash
```

You should not need to source these files manually. Use `ROS_SETUP_FILE` or
`PX4_MSGS_SETUP_FILE` only when deliberately testing a non-standard container
or dependency install.

## First Sanity Check

From the repository root, one script prepares everything and runs the urban
point-to-point flight:

```bash
./scripts/bootstrap.sh
```

It checks the host tools, NVIDIA runtime and locally restored immutable image,
clones PX4 Autopilot v1.17.0 if absent, and builds
`px4_sitl` there inside the container, builds the workspace, fetches the
versioned `urban_circuit_practice_01` assets, then launches the GUI flight
(`--headless` for the headless mission check, `--no-run` to stop after the
preparation). Each step is skipped when its result is already in place.

The same steps by hand:

```bash
docker load --input /path/to/baseline/dev-image.tar.gz
./scripts/setup_px4_autopilot.sh
./scripts/dev_shell.sh make -C external/PX4-Autopilot px4_sitl
./scripts/build.sh
./scripts/test.sh
```

For a simulator sanity check:

```bash
./scripts/sim_urban_point_to_point_headless.sh
```

For a GUI sanity check:

```bash
./scripts/sim_urban_point_to_point_gui.sh
```

If GUI startup fails, first check Docker display permissions and see
`troubleshooting.md`.
