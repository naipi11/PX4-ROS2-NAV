#!/usr/bin/env python3
"""Exercise image selection without a Docker daemon or simulation processes."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1]
BASELINE = "sha256:" + "1" * 64
OVERRIDE = "sha256:" + "2" * 64


class DevImageTest(unittest.TestCase):
    def setUp(self) -> None:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        (self.root / "scripts").mkdir()
        (self.root / "docker").mkdir()
        self.bin = self.root / "bin"
        self.bin.mkdir()
        for name in ("container_run.sh", "bootstrap.sh", "cleanup_sim_processes.sh",
                     "dev_image.sh"):
            shutil.copy2(SCRIPTS / name, self.root / "scripts" / name)
        (self.root / "scripts/gazebo_process_cleanup.py").write_text(
            "raise SystemExit(0)\n", encoding="utf-8")
        (self.root / "docker/dev-image.lock").write_text(BASELINE + "\n", encoding="utf-8")
        docker = self.bin / "docker"
        docker.write_text(f"#!{sys.executable}\n" + '''import json
import os
import sys
from pathlib import Path
args = sys.argv[1:]
state = json.loads(Path(os.environ["FAKE_DOCKER_STATE"]).read_text())
with Path(os.environ["FAKE_DOCKER_CALLS"]).open("a") as stream:
    stream.write(json.dumps(args) + "\\n")
if args[:2] == ["image", "inspect"]:
    image = state["images"].get(args[-1])
    if image is None:
        sys.exit(1)
    print(image)
elif args[0] == "info":
    print('{"nvidia": {}}')
elif args[0] == "ps":
    print("\\n".join(state.get("containers", {})))
elif args[0] == "inspect":
    container = state["containers"][args[-1]]
    if ".Mounts" in args[2]:
        print(container["image"] + " " + container.get("mount", "") + " ")
    else:
        print("make sim-urban-point-to-point-gui")
elif args[0] == "top":
    print("make sim-urban-point-to-point-gui")
elif args[0] != "run":
    sys.exit("Unexpected Docker command: " + repr(args))
''', encoding="utf-8")
        docker.chmod(0o755)
        nvidia = self.bin / "nvidia-smi"
        nvidia.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
        nvidia.chmod(0o755)
        self.state = {"images": {BASELINE: BASELINE, "drone-gazebo-dev:latest": OVERRIDE}}
        self.calls_path = self.root / "docker-calls.jsonl"
        self.environment = dict(os.environ)
        for name in ("DRONE_GAZEBO_DEV_IMAGE", "DRONE_GAZEBO_DEV_IMAGE_ID",
                     "DRONE_GAZEBO_CLEAN_STALE_PROCESSES", "DRONE_GAZEBO_CLEAN_STALE_DRY_RUN"):
            self.environment.pop(name, None)
        self.environment.update({
            "PATH": str(self.bin) + os.pathsep + os.environ["PATH"],
            "FAKE_DOCKER_STATE": str(self.root / "docker-state.json"),
            "FAKE_DOCKER_CALLS": str(self.calls_path),
            "DISPLAY": "",
        })

    def invoke(self, script: str, *arguments: str) -> subprocess.CompletedProcess[str]:
        (self.root / "docker-state.json").write_text(json.dumps(self.state), encoding="utf-8")
        return subprocess.run(
            ["bash", str(self.root / "scripts" / script), *arguments],
            env=self.environment, capture_output=True, text=True, check=False, timeout=15)

    def calls(self) -> list[list[str]]:
        return [json.loads(line) for line in self.calls_path.read_text(encoding="utf-8").splitlines()]

    def assert_run_image(self, image: str) -> None:
        run, = [call for call in self.calls() if call[0] == "run"]
        self.assertEqual(run[run.index("--workdir") + 2], image)
        self.assertIn("DRONE_GAZEBO_DEV_IMAGE=" + image, run)
        self.assertIn("DRONE_GAZEBO_DEV_IMAGE_ID=" + image, run)

    def test_default_ignores_drifted_latest(self) -> None:
        result = self.invoke("container_run.sh", "true")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_run_image(BASELINE)

    def test_explicit_tag_runs_and_records_resolved_id(self) -> None:
        self.environment["DRONE_GAZEBO_DEV_IMAGE"] = "drone-gazebo-dev:latest"
        result = self.invoke("container_run.sh", "true")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assert_run_image(OVERRIDE)

    def test_missing_baseline_never_runs_or_builds_latest(self) -> None:
        del self.state["images"][BASELINE]
        for script, arguments in (("container_run.sh", ("true",)),
                                  ("bootstrap.sh", ("--no-run",))):
            with self.subTest(script=script):
                result = self.invoke(script, *arguments)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("docker load", result.stderr)
                self.assertIn("DRONE_GAZEBO_DEV_IMAGE", result.stderr)
        self.assertFalse(any(call[0] in ("run", "build", "pull") for call in self.calls()))

    def test_cleanup_matches_actual_image_and_repository_mount(self) -> None:
        self.state["containers"] = {
            "baseline": {"image": BASELINE},
            "other": {"image": OVERRIDE},
            "mounted": {"image": OVERRIDE, "mount": str(self.root)},
        }
        for override, expected, excluded in ((None, "baseline", "other"),
                                             ("drone-gazebo-dev:latest", "other", "baseline")):
            with self.subTest(override=override):
                if override is not None:
                    self.environment["DRONE_GAZEBO_DEV_IMAGE"] = override
                result = self.invoke("cleanup_sim_processes.sh", "--dry-run")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("candidate: container=" + expected, result.stdout)
                self.assertIn("candidate: container=mounted", result.stdout)
                self.assertNotIn("candidate: container=" + excluded, result.stdout)


if __name__ == "__main__":
    unittest.main()
