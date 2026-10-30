#!/usr/bin/env python3
"""Pair an estimator trajectory with an independent pose reference by time.

Both inputs must use the same physical body origin and coordinate convention.
No extrapolation, spatial alignment or scale fitting is performed here.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path
import sys


OUTPUT_COLUMNS = ("stamp_s", "px", "py", "pz", "qw", "qx", "qy", "qz",
                  "tx", "ty", "tz", "tqw", "tqx", "tqy", "tqz")


def read_poses(path: Path, position_names: tuple[str, str, str]) -> tuple[list, bool]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = ("stamp_s", *position_names, "qw", "qx", "qy", "qz")
        if reader.fieldnames is None or not set(required).issubset(reader.fieldnames):
            raise ValueError(f"missing pose columns in {path}")
        has_health = "healthy" in reader.fieldnames
        result = []
        for line, row in enumerate(reader, 2):
            try:
                values = [float(row[name]) for name in required]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("nonfinite pose")
                if result and values[0] <= result[-1][0]:
                    raise ValueError("timestamps must be strictly increasing")
                qnorm = math.hypot(*values[4:8])
                if abs(qnorm - 1.0) > 1e-3:
                    raise ValueError("quaternion is not unit length")
                values[4:8] = [value / qnorm for value in values[4:8]]
                health = row["healthy"] if has_health else None
                if has_health and health not in ("0", "1"):
                    raise ValueError("healthy must be 0 or 1")
                result.append((values[0], values[1:4], values[4:8], health))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{path} row {line}: {exc}") from exc
    if len(result) < 2:
        raise ValueError(f"at least two poses are required: {path}")
    return result, has_health


def slerp(a: list[float], b: list[float], fraction: float) -> list[float]:
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b = [-value for value in b]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        q = [x + fraction * (y - x) for x, y in zip(a, b)]
    else:
        angle = math.acos(dot)
        scale = math.sin(angle)
        left = math.sin((1.0 - fraction) * angle) / scale
        right = math.sin(fraction * angle) / scale
        q = [left * x + right * y for x, y in zip(a, b)]
    norm = math.hypot(*q)
    return [value / norm for value in q]


def associate(estimates: list, reference: list, start: float, end: float,
              max_gap: float) -> tuple[list, dict]:
    stamps = [pose[0] for pose in reference]
    paired = []
    outside = 0
    gaps = 0
    selected = 0
    for stamp, position, quaternion, health in estimates:
        if stamp < start or stamp > end:
            continue
        selected += 1
        i = bisect_left(stamps, stamp)
        if i < len(stamps) and stamps[i] == stamp:
            truth_position, truth_quaternion = reference[i][1:3]
        elif i == 0 or i == len(stamps):
            outside += 1
            continue
        else:
            a, b = reference[i - 1], reference[i]
            width = b[0] - a[0]
            if width > max_gap:
                gaps += 1
                continue
            fraction = (stamp - a[0]) / width
            truth_position = [x + fraction * (y - x) for x, y in zip(a[1], b[1])]
            truth_quaternion = slerp(a[2], b[2], fraction)
        paired.append([stamp, *position, *quaternion, *truth_position, *truth_quaternion, health])
    if len(paired) < 2:
        raise ValueError("fewer than two estimates have reference coverage")
    return paired, {"selected_estimates": selected, "paired_estimates": len(paired),
                    "outside_reference": outside, "reference_gaps": gaps,
                    "maximum_reference_gap_s": max_gap,
                    "first_stamp_s": paired[0][0], "last_stamp_s": paired[-1][0]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("estimate", type=Path, help="CSV: stamp_s,px,py,pz,qw,qx,qy,qz")
    parser.add_argument("reference", type=Path, help="CSV: stamp_s,x,y,z,qw,qx,qy,qz")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--start-s", type=float, default=-math.inf)
    parser.add_argument("--end-s", type=float, default=math.inf)
    parser.add_argument("--max-reference-gap-s", type=float, required=True)
    args = parser.parse_args()
    try:
        if not math.isfinite(args.max_reference_gap_s) or args.max_reference_gap_s <= 0 or args.start_s > args.end_s:
            raise ValueError("invalid time interval or reference gap")
        estimates, health = read_poses(args.estimate, ("px", "py", "pz"))
        reference, _ = read_poses(args.reference, ("x", "y", "z"))
        pairs, summary = associate(estimates, reference, args.start_s, args.end_s, args.max_reference_gap_s)
        with args.output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow([*OUTPUT_COLUMNS, *(["healthy"] if health else [])])
            for row in pairs:
                writer.writerow(row if health else row[:-1])
        args.summary.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (OSError, ValueError) as exc:
        print(f"reference association: {exc}", file=sys.stderr)
        return 1
    print(f"Paired {summary['paired_estimates']}/{summary['selected_estimates']} estimates; "
          f"outside={summary['outside_reference']} gaps={summary['reference_gaps']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
