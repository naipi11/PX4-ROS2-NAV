#!/usr/bin/env python3
"""Archive and verify a stopped deployment without rebuilding its dependencies."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from runtime_manifest import atomic_write_json, file_record, read_manifest, sha256_file

REPOSITORY = Path(__file__).resolve().parents[1]
SCHEMA = "drone_city_nav_deployment_baseline_v1"
PX4 = Path("external/PX4-Autopilot")
ENVIRONMENTS = Path("external/environment-artifacts")


def command(*arguments: str, cwd: Path = REPOSITORY) -> str:
    return subprocess.check_output(arguments, cwd=cwd, text=True).strip()


def repository_record(path: Path) -> dict[str, str]:
    return {
        "commit": command("git", "rev-parse", "HEAD", cwd=path),
        "status": command("git", "status", "--porcelain=v1", "--untracked-files=all", cwd=path),
    }


def save_image(image_id: str, destination: Path) -> None:
    # Pipe to gzip instead of temporarily storing an uncompressed image too.
    with destination.open("xb") as output:
        with subprocess.Popen(["docker", "image", "save", image_id], stdout=subprocess.PIPE) as save:
            assert save.stdout is not None
            try:
                compressed = subprocess.run(["gzip", "-1"], stdin=save.stdout, stdout=output)
            finally:
                save.stdout.close()
            save_code = save.wait()
    if save_code != 0 or compressed.returncode != 0:
        raise RuntimeError("image export failed; incomplete directory is not a baseline")


def capture(destination: Path, run_directory: Path | None) -> None:
    source = repository_record(REPOSITORY)
    if source["status"]:
        raise RuntimeError("commit source changes before capturing a baseline")
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite an existing snapshot: {destination}")
    # Avoid copying a changing .git tree, source tree, or live flight artifacts.
    active = subprocess.run(
        ["systemctl", "--user", "is-active", "--quiet", "px4-nav-simulation.service"]
    )
    if active.returncode != 3:
        raise RuntimeError("stop the simulation service before capturing (service state must be inactive)")
    image_id = (REPOSITORY / "docker/dev-image.lock").read_text().strip()
    actual_image = command("docker", "image", "inspect", "--format", "{{.Id}}", image_id)
    if actual_image != image_id:
        raise RuntimeError("docker/dev-image.lock must identify the installed immutable image")
    px4 = repository_record(REPOSITORY / PX4)
    submodules = command("git", "submodule", "status", "--recursive", cwd=REPOSITORY / PX4)
    run = None
    if run_directory is not None:
        run = read_manifest(run_directory / "manifest.json")
        if run["repository"]["commit"] != source["commit"] or run["repository"]["dirty"]:
            raise RuntimeError("flight evidence does not belong to this clean source commit")
        if run.get("container_image") != image_id:
            raise RuntimeError("flight evidence does not identify the locked container image")

    destination.mkdir(parents=True)
    (destination / ".keep").write_text("Frozen deployment baseline; retain archives and evidence.\n")
    artifacts: list[dict] = []

    print("Archiving committed source and its Git history", flush=True)
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=REPOSITORY)
    subprocess.run(
        ["tar", "-czf", str(destination / "source.tar.gz"), "--null", "-T", "-"],
        cwd=REPOSITORY,
        input=tracked + b".git\0",
        check=True,
    )
    artifacts.append(file_record(destination / "source.tar.gz", destination))

    print("Archiving PX4 source/submodules and installed/derived scene assets (no build outputs)", flush=True)
    subprocess.run(
        ["tar", "--exclude=external/PX4-Autopilot/build", "--exclude=__pycache__",
         "-czf", str(destination / "dependencies.tar.gz"), str(PX4), str(ENVIRONMENTS)],
        cwd=REPOSITORY,
        check=True,
    )
    artifacts.append(file_record(destination / "dependencies.tar.gz", destination))

    print(f"Exporting immutable container {image_id}", flush=True)
    save_image(image_id, destination / "dev-image.tar.gz")
    artifacts.append(file_record(destination / "dev-image.tar.gz", destination))

    packages = subprocess.check_output(
        ["docker", "run", "--rm", "--network", "none", "--entrypoint", "dpkg-query",
         image_id, "-W", "-f=${Package}\t${Version}\n"], text=True
    )
    (destination / "packages.tsv").write_text(packages)
    artifacts.append(file_record(destination / "packages.tsv", destination))
    (destination / "px4-submodules.txt").write_text(submodules + "\n")
    artifacts.append(file_record(destination / "px4-submodules.txt", destination))

    # Per-host colcon settings are not source; retain them separately if present.
    build_configuration = []
    configured_defaults = os.environ.get("COLCON_DEFAULTS_FILE")
    default_paths = [Path(configured_defaults)] if configured_defaults else [
        Path.home() / ".colcon/defaults.yaml", REPOSITORY / ".colcon/defaults.yaml"
    ]
    for index, path in enumerate(default_paths):
        if path.is_file():
            saved = destination / f"colcon-defaults-{index}.yaml"
            saved.write_bytes(path.read_bytes())
            artifacts.append(file_record(saved, destination))
            build_configuration.append({"original_path": str(path), "saved_path": saved.name})

    if run_directory is not None:
        evidence = destination / "evidence"
        evidence.mkdir()
        for name in ("manifest.json", "resources_host.json"):
            original = run_directory / name
            if original.is_file():
                saved = evidence / name
                saved.write_bytes(original.read_bytes())
                artifacts.append(file_record(saved, destination))
        result_path = evidence / "mission_result.txt"
        with (run_directory / "ros_drone_nav.log").open(errors="replace") as source_log:
            results = [line for line in source_log if "MISSION_RESULT " in line]
        result_path.write_text("".join(results) if results else "No MISSION_RESULT observed.\n")
        artifacts.append(file_record(result_path, destination))
        (run_directory / ".keep").write_text("Retained flight evidence for " + str(destination) + "\n")

    if repository_record(REPOSITORY) != source or repository_record(REPOSITORY / PX4) != px4:
        raise RuntimeError("source changed during capture; incomplete directory is not a baseline")
    if command("git", "submodule", "status", "--recursive", cwd=REPOSITORY / PX4) != submodules:
        raise RuntimeError("PX4 submodule checkouts changed during capture")
    document = {
        "schema": SCHEMA,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "repository": source,
        "container_image": image_id,
        "px4": px4,
        "host": {
            "kernel": command("uname", "-srmo"),
            "docker": command("docker", "version", "--format", "{{.Server.Version}}"),
            "gpu": command("nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"),
        },
        "build_configuration": build_configuration,
        "flight_directory": str(run_directory) if run_directory else None,
        "flight": run,
        "artifacts": artifacts,
        "restore_contract": "Restore the saved image and dependencies; a fresh online Docker build is not byte-reproducible.",
    }
    atomic_write_json(destination / "baseline.json", document)
    print(f"BASELINE_CAPTURED commit={source['commit']} manifest={destination / 'baseline.json'}", flush=True)


def verify(destination: Path) -> None:
    document = json.loads((destination / "baseline.json").read_text())
    if document.get("schema") != SCHEMA or not document.get("artifacts"):
        raise ValueError("unsupported or empty deployment baseline")
    for record in document["artifacts"]:
        path = (destination / record["path"]).resolve()
        if not path.is_relative_to(destination):
            raise ValueError("baseline artifact must be inside the snapshot directory")
        if path.stat().st_size != record["size_bytes"] or sha256_file(path) != record["sha256"]:
            raise ValueError(f"baseline artifact changed: {record['path']}")
        print(f"VERIFIED {record['path']}", flush=True)
    print(f"BASELINE_VERIFIED commit={document['repository']['commit']}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("capture", "verify"))
    parser.add_argument("directory", type=Path)
    parser.add_argument("--run-directory", type=Path)
    args = parser.parse_args()
    destination = args.directory.resolve()
    if args.action == "capture":
        capture(destination, args.run_directory.resolve() if args.run_directory else None)
    else:
        if args.run_directory:
            parser.error("--run-directory is only valid for capture")
        verify(destination)


if __name__ == "__main__":
    main()
