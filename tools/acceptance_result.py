#!/usr/bin/env python3
"""Retain one series flight's checker result, host verdict and source identity."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from runtime_manifest import atomic_write_json, file_record, read_manifest  # noqa: E402
from host_verdict import inspect as inspect_host  # noqa: E402

EXIT_CODES = {"passed": 0, "failed": 1, "void": 2, "error": 3}


def record_result(run: str, commit: str, profile: str, command_exit: int,
                  cleanup_exit: int) -> dict:
    """No second mission criterion: combine the existing checker and A7 verdict."""
    directory = Path("log/runs") / run
    result = {
        "schema": "drone_city_nav_acceptance_result_v1",
        "run_id": run,
        "expected_commit": commit,
        "profile": profile,
        "command_exit_code": command_exit,
        "cleanup_exit_code": cleanup_exit,
        "status": "error",
        "host_valid": None,
        "host_reasons": [],
        "evidence_errors": [],
    }
    try:
        manifest_path = directory / "manifest.json"
        manifest = read_manifest(manifest_path)
        current_commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True).strip()
        dirty = subprocess.check_output(
            ["git", "status", "--porcelain=v1", "--untracked-files=all"], text=True).strip()
        if (current_commit != commit or dirty or manifest["run_id"] != run
                or manifest["repository"]["commit"] != commit
                or manifest["repository"]["dirty"] is not False):
            raise ValueError("flight and current clean source must match the series commit")
        overrides = manifest["effective_overrides"]
        expected_sensor, expected_localization = (
            ("lidar", "lidar_inertial") if profile == "lidar"
            else ("stereo_tof", "visual_inertial"))
        if (overrides.get("NAVIGATION_SENSOR_PROFILE") != expected_sensor
                or overrides.get("LOCALIZATION_PROFILE") != expected_localization):
            raise ValueError("flight does not match the requested GNSS-denied sensor profile")
        result["runtime_manifest"] = file_record(manifest_path)
        check_path = directory / "mission_check.json"
        check = json.loads(check_path.read_text(encoding="utf-8"))
        if (check["schema"] != "drone_city_nav_headless_result_v1"
                or check["runtime_manifest"]["sha256"] != result["runtime_manifest"]["sha256"]
                or check["mission_check"] is not True
                or check["allow_mission_failure"] is not False
                or type(check["passed"]) is not bool
                or not isinstance(check["errors"], list)
                or check["passed"] != (not check["errors"])):
            raise ValueError("missing, inconsistent or differently bound mission check")
        result["mission_check"] = file_record(check_path)
        result["check_passed"] = check["passed"]
        result["check_errors"] = check["errors"]
        result["acceptance_log"] = file_record(directory / "acceptance.log")
        # An absent ROS log is not evidence of young estimator poses.
        if not (directory / "ros_drone_nav.log").is_file():
            raise FileNotFoundError(directory / "ros_drone_nav.log")
        result["host_reasons"] = inspect_host(run)
        result["host_valid"] = not result["host_reasons"]
        if cleanup_exit:
            raise ValueError(f"simulation cleanup failed with exit {cleanup_exit}")
        if result["host_reasons"]:
            result["status"] = "void"
        elif command_exit or not check["passed"]:
            result["status"] = "failed"
        else:
            result["status"] = "passed"
    except (OSError, ValueError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        result["evidence_errors"].append(str(error))
    if command_exit in (130, 143):
        result["status"] = "interrupted"
    result["exit_code"] = (command_exit if result["status"] == "interrupted"
                           else EXIT_CODES[result["status"]])
    atomic_write_json(directory / "acceptance.json", result)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run")
    parser.add_argument("--commit", required=True)
    parser.add_argument("--profile", choices=("cameras", "lidar"), required=True)
    parser.add_argument("--command-exit", type=int, required=True)
    parser.add_argument("--cleanup-exit", type=int, required=True)
    args = parser.parse_args()
    result = record_result(args.run, args.commit, args.profile,
                           args.command_exit, args.cleanup_exit)
    if result["host_valid"] is not None:
        reasons = result["host_reasons"]
        print(f"HOST {'VOID' if reasons else 'valid'} {args.run}"
              + (": " + "; ".join(reasons) if reasons else ""))
    for error in result["evidence_errors"]:
        print(f"ACCEPTANCE ERROR {args.run}: {error}", file=sys.stderr)
    print(f"ACCEPTANCE {result['status'].upper()} {args.run} "
          f"record=log/runs/{args.run}/acceptance.json")
    return result["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
