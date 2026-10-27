#!/usr/bin/env python3
"""Index one recorded flight using its retained acceptance verdict.

Exit 0: accepted flight and recording; 2: void flight or bad recording;
1/3/130/143: stop, never retry a failed, unproved or interrupted flight.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

from acceptance_result import EXIT_CODES

HEADER = ("| Scenario | Run | Acceptance | Speed, m/s | True position, m | Gave the goal up | Host (A7) "
          "| Recording | Files |\n|---|---|---|---|---|---|---|---|---|\n")


def row(scenario: str, run: str, directory: Path) -> tuple[str, int]:
    run_directory = Path("log/runs") / run
    result = json.loads((run_directory / "acceptance.json").read_text())
    status, exit_code = result["status"], result["exit_code"]
    if type(exit_code) is not int or (exit_code != EXIT_CODES.get(status) and not (
            status == "interrupted" and exit_code in (130, 143))):
        raise ValueError("inconsistent acceptance status and exit code")
    check_path = run_directory / "acceptance.log"
    check = check_path.read_text(errors="ignore") if check_path.exists() else ""
    record = Path(f"log/tools/record_{run}.log")
    recording = record.read_text(errors="ignore") if record.exists() else ""
    speed = re.search(r"mean flight speed is ([\d.]+) m/s", check)
    truths = re.findall(r"the true position is ([\d.]+) m from the goal", check)
    trigger = re.search(r"given up by the (\w+) trigger", check)
    whole = "vehicle is whole" in check
    host = ("valid" if result["host_valid"] else
            "VOID: " + "; ".join(result["host_reasons"])
            if result["host_valid"] is False else "unavailable")
    files = sorted(path.name for path in directory.glob(f"*_{run}_*.mp4"))
    good = recording.count("RECORDING ok") == 2
    errors = result.get("check_errors", []) + result["evidence_errors"]
    verdict = status.upper() + (": " + "; ".join(errors)[:120] if errors else "")
    if whole:
        verdict += " (whole)"
    text = (f"| {scenario} | {run} | {verdict} | {speed.group(1) if speed else '-'} | "
            f"{' / '.join(truths) or '-'} | {trigger.group(1) if trigger else '-'} | "
            f"{host} | "
            f"{'ok' if good else 'BAD'} | {', '.join(files) or '-'} |\n")
    return text, 2 if exit_code == 0 and not good else exit_code


def main() -> int:
    scenario, run, directory = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    index = directory / "index.md"
    try:
        text, status = row(scenario, run, directory)
        if not index.exists():
            index.write_text("# Recorded flights\n\nA recorded flight is flown slowed against the wall clock and judged like "
                             "any other (specification A7, A11): the host's verdict is in its row.\n\n" + HEADER)
        with index.open("a") as stream:
            stream.write(text)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"Recording acceptance evidence error: {error}", file=sys.stderr)
        return 3
    print("OK" if status == 0 else "RETRY" if status == 2 else "STOP")
    return status


if __name__ == "__main__":
    sys.exit(main())
