#!/usr/bin/env python3
"""Static tests for container entrypoint scripts."""

from __future__ import annotations

import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1]


class ContainerEntrypointTest(unittest.TestCase):
    def read_script(self, name: str) -> str:
        return (SCRIPTS_DIR / name).read_text(encoding="utf-8")

    def test_dev_shell_uses_shared_container_runner(self) -> None:
        text = self.read_script("dev_shell.sh")

        self.assertIn('exec "${repo_root}/scripts/container_run.sh" "$@"', text)
        self.assertNotIn("docker run", text)

    def test_common_wrappers_use_shared_container_runner(self) -> None:
        expected_targets = {
            "build.sh": "make build",
            "test.sh": "make test",
        }

        for script_name, make_target in expected_targets.items():
            with self.subTest(script_name=script_name):
                text = self.read_script(script_name)
                self.assertIn(
                    f'exec "${{repo_root}}/scripts/container_run.sh" {make_target}',
                    text,
                )
                self.assertNotIn("docker run", text)

    def test_every_sim_wrapper_runs_its_target_between_cleanups(self) -> None:
        # Every sim_*.sh wrapper hands its make target to the wrapped runner,
        # which stops stale simulation processes before the run and again once
        # the run ends, however it ends.
        wrappers = sorted(SCRIPTS_DIR.glob("sim_*.sh"))
        self.assertGreaterEqual(len(wrappers), 5)
        for wrapper in wrappers:
            with self.subTest(script_name=wrapper.name):
                text = wrapper.read_text(encoding="utf-8")
                self.assertRegex(
                    text,
                    r'exec "\$\{repo_root\}/scripts/run_sim_wrapped\.sh" make sim-[a-z0-9-]+\n',
                )
                self.assertNotIn("docker run", text)
                self.assertNotIn("container_run.sh", text)

        runner = self.read_script("run_sim_wrapped.sh")
        prune_index = runner.index('"${repo_root}/scripts/prune_sim_logs.sh"')
        cleanup_index = runner.index('"${repo_root}/scripts/cleanup_sim_processes.sh"')
        self.assertLess(prune_index, cleanup_index)
        trap_index = runner.index(
            "trap '\"${repo_root}/scripts/cleanup_sim_processes.sh\" || true' EXIT"
        )
        container_index = runner.index('"${repo_root}/scripts/container_run.sh" "$@"')
        self.assertLess(cleanup_index, trap_index)
        self.assertLess(trap_index, container_index)
        self.assertNotIn("exec ", runner.split("container_run.sh")[0].split("trap")[1])


    def test_stop_sim_uses_shared_cleanup(self) -> None:
        text = self.read_script("stop_sim.sh")

        self.assertIn('exec "${repo_root}/scripts/cleanup_sim_processes.sh" "$@"', text)
        self.assertNotIn("docker run", text)

    def test_cleanup_stops_related_simulation_containers(self) -> None:
        text = self.read_script("cleanup_sim_processes.sh")

        self.assertIn("docker ps --format", text)
        self.assertNotIn("docker ps --filter", text)
        self.assertIn("container_has_simulation_processes", text)
        self.assertIn('docker stop -t "${container_stop_timeout_s}"', text)
        self.assertIn("docker kill", text)
        self.assertIn("DRONE_GAZEBO_DOCKER_COMMAND_TIMEOUT_S", text)
        self.assertIn("gz[[:space:]]+sim", text)
        self.assertIn("PX4-Autopilot", text)
        self.assertIn("MicroXRCEAgent", text)

    def test_container_runner_owns_docker_invocation(self) -> None:
        text = self.read_script("container_run.sh")

        self.assertIn("docker run", text)
        self.assertIn("--volume \"${repo_root}:/workspace:rw\"", text)
        self.assertIn('bash -c "${container_command}" bash "$@"', text)

    def test_container_gpu_access_is_required_for_all_entrypoints(self) -> None:
        runner = self.read_script("container_run.sh")
        build = self.read_script("build.sh")

        self.assertIn('gpu_args=(--gpus all)', runner)
        self.assertIn("NVIDIA container runtime is required but unavailable.", runner)
        self.assertNotIn("DRONE_GAZEBO_CONTAINER_GPU", runner)
        self.assertNotIn("DRONE_GAZEBO_CONTAINER_GPU", build)

    def test_container_runner_sources_ros_and_px4_msgs_setups(self) -> None:
        text = self.read_script("container_run.sh")

        self.assertIn("ROS_SETUP_FILE", text)
        self.assertIn("PX4_MSGS_SETUP_FILE", text)
        self.assertIn(
            'source_setup_file "${ROS_SETUP_FILE:-/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash}"',
            text,
        )
        self.assertIn(
            'source_setup_file "${PX4_MSGS_SETUP_FILE:-/opt/px4_msgs_ws/install/setup.bash}"',
            text,
        )

    def test_container_runner_forwards_navigation_validation_contract(self) -> None:
        text = self.read_script("container_run.sh")

        for variable in (
            "POINT_TO_POINT_SCENARIO_PATH",
            "REQUIRE_OBSERVED_3D_ROUTE_VOLUME_CROSSING",
            "OBSERVED_3D_ROUTE_VOLUME_BOUNDS_M",
            "ENABLE_LIVENESS_RECOVERY",
            "ENABLE_ROUTE_STALL_RECOVERY",
        ):
            with self.subTest(variable=variable):
                self.assertRegex(text, rf"(?m)^  {variable}$")
        for legacy_variable in (
            "REQUIRE_INCREMENTAL_TOPOLOGY_EVIDENCE",
            "MAXIMUM_NO_EXECUTABLE_ROUTE_AGE_MS",
            "ENABLE_TOPOLOGICAL_BACKTRACKING",
            "ENABLE_NO_STATIC_CYCLE_RECOVERY",
            "STATIC_GLOBAL_LATTICE_DEADLINE_MS",
        ):
            self.assertNotIn(legacy_variable, text)


if __name__ == "__main__":
    unittest.main()
