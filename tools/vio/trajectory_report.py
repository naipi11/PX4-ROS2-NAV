#!/usr/bin/env python3
"""Summarize paired replay positions; this is not a flight-safety assessment.

The optional alignment is fixed by the first pair only: rotate the estimate
about the shared vertical axis by the difference in initial yaw, then translate
its initial position onto the reference. No scale is fitted. Percentiles use
linear interpolation between adjacent sorted samples.
"""

import argparse
import csv
import json
import math
from pathlib import Path
import sys


COLUMNS = ("stamp_s", "px", "py", "pz", "qw", "qx", "qy", "qz",
           "tx", "ty", "tz", "tqw", "tqx", "tqy", "tqz")


def yaw(quaternion, require_defined=False):
    w, x, y, z = quaternion
    norm = math.hypot(w, x, y, z)
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise ValueError("quaternion must have unit norm within 0.001")
    w, x, y, z = (component / norm for component in quaternion)
    # The yaw of a vertical body x-axis is undefined.
    forward_x = 1.0 - 2.0 * (y * y + z * z)
    forward_y = 2.0 * (x * y + w * z)
    if require_defined and math.hypot(forward_x, forward_y) < 1e-9:
        raise ValueError("initial yaw is undefined for a vertical forward axis")
    return math.atan2(forward_y, forward_x)


def read_samples(path):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise ValueError("missing or duplicate CSV headers")
        missing = set(COLUMNS) - set(reader.fieldnames)
        if missing:
            raise ValueError("missing CSV columns: " + ", ".join(sorted(missing)))
        has_health = "healthy" in reader.fieldnames
        samples = []
        for line, row in enumerate(reader, 2):
            try:
                values = [float(row[name]) for name in COLUMNS]
                if not all(math.isfinite(value) for value in values):
                    raise ValueError("nonfinite value")
                if samples and values[0] <= samples[-1][0][0]:
                    raise ValueError("timestamps must be strictly increasing")
                yaw(values[4:8])
                yaw(values[11:15])
                health = None
                if has_health:
                    if row["healthy"] not in ("0", "1"):
                        raise ValueError("healthy must be 0 or 1")
                    health = int(row["healthy"])
                samples.append((values, health))
            except (TypeError, ValueError) as exc:
                raise ValueError(f"CSV row {line}: {exc}") from exc
    if len(samples) < 2:
        raise ValueError("at least two paired samples are required")
    return samples, has_health


def percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low = math.floor(index)
    high = math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def report(samples, has_health, align_initial_yaw=False):
    first = samples[0][0]
    angle = (yaw(first[11:15], True) - yaw(first[4:8], True)) if align_initial_yaw else 0.0
    cosine, sine = math.cos(angle), math.sin(angle)
    errors = []
    horizontal_errors = []
    path_3d = 0.0
    path_horizontal = 0.0
    previous = None
    for values, _ in samples:
        estimate = values[1:4]
        reference = values[8:11]
        if align_initial_yaw:
            dx, dy, dz = (estimate[i] - first[1 + i] for i in range(3))
            estimate = (first[8] + cosine * dx - sine * dy,
                        first[9] + sine * dx + cosine * dy, first[10] + dz)
        delta = [estimate[i] - reference[i] for i in range(3)]
        horizontal_errors.append(math.hypot(delta[0], delta[1]))
        errors.append(math.hypot(*delta))
        if previous is not None:
            step = [reference[i] - previous[i] for i in range(3)]
            path_3d += math.hypot(*step)
            path_horizontal += math.hypot(step[0], step[1])
        previous = reference
    result = {
        "samples": len(samples),
        "start_stamp_s": first[0],
        "end_stamp_s": samples[-1][0][0],
        "duration_s": samples[-1][0][0] - first[0],
        "alignment": "initial_position_yaw" if align_initial_yaw else "none",
        "reference_path_length_3d_m": path_3d,
        "reference_path_length_horizontal_m": path_horizontal,
        "final_error_3d_m": errors[-1],
        "final_error_horizontal_m": horizontal_errors[-1],
        "final_error_3d_percent_of_path": 100.0 * errors[-1] / path_3d if path_3d > 0 else None,
        "final_error_horizontal_percent_of_path": (100.0 * horizontal_errors[-1] / path_horizontal
                                                   if path_horizontal > 0 else None),
        "absolute_error_rmse_m": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "absolute_error_p95_m": percentile(errors, 0.95),
        "absolute_error_max_m": max(errors),
        "health_coverage": len(samples) if has_health else 0,
        "healthy_fraction": sum(health for _, health in samples) / len(samples) if has_health else None,
    }
    if any(isinstance(value, float) and not math.isfinite(value) for value in result.values()):
        raise ValueError("computed metrics are nonfinite")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="paired replay CSV")
    parser.add_argument("--output", type=Path, required=True, help="report JSON path")
    parser.add_argument("--align-initial-yaw", action="store_true",
                        help="align initial position and yaw only; never fit scale")
    args = parser.parse_args()
    try:
        samples, has_health = read_samples(args.csv)
        result = report(samples, has_health, args.align_initial_yaw)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (OSError, ValueError, OverflowError) as exc:
        print(f"trajectory report: {exc}", file=sys.stderr)
        return 1
    health = (f"{result['healthy_fraction']:.1%}" if has_health else "unavailable")
    print(f"{result['samples']} samples over {result['duration_s']:.3f} s; "
          f"alignment={result['alignment']}; final error={result['final_error_3d_m']:.3f} m; "
          f"RMSE={result['absolute_error_rmse_m']:.3f} m; healthy={health}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
