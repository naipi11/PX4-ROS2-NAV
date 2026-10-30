#!/usr/bin/env python3
"""Compare LIO's estimated prism position with independent Leica positions.

Leica measures position only. The lever arm is rotated by each estimated
attitude; the reference's placeholder quaternion is never used. Rigid
alignment is reported both on the entire overlap and, where observable, on a
prefix with the remainder held out. Scale is never fitted.
"""
from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        result = list(csv.DictReader(stream))
    if len(result) < 2:
        raise ValueError(f"not enough rows in {path}")
    return result


def rotate(q: np.ndarray, point: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise ValueError("estimated orientation is not a unit quaternion")
    w, x, y, z = q / norm
    cross = 2.0 * np.cross([x, y, z], point)
    return point + w * cross + np.cross([x, y, z], cross)


def rigid_alignment(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, list[float]]:
    if len(source) < 3 or source.shape != target.shape:
        raise ValueError("rigid alignment needs at least three paired positions")
    a = source.mean(axis=0)
    b = target.mean(axis=0)
    u, singular, vt = np.linalg.svd((source - a).T @ (target - b))
    if singular[0] <= 1e-8 or singular[1] <= singular[0] * 1e-6:
        raise ValueError("alignment positions do not span two independent directions")
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    return rotation, b - rotation @ a, singular.tolist()


def summarize(errors: np.ndarray) -> dict:
    return {"samples": len(errors), "rmse_m": float(np.sqrt(np.mean(errors ** 2))),
            "median_m": float(np.median(errors)), "p95_m": float(np.percentile(errors, 95)),
            "maximum_m": float(np.max(errors)), "final_m": float(errors[-1])}


def compare(estimate_path: Path, reference_path: Path, metadata_path: Path,
            max_gap_s: float, fit_duration_s: float) -> tuple[dict, list]:
    if max_gap_s <= 0 or fit_duration_s <= 0 or not math.isfinite(max_gap_s + fit_duration_s):
        raise ValueError("reference gap and alignment duration must be finite and positive")
    estimates = rows(estimate_path)
    references = rows(reference_path)
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    lever = np.asarray(metadata["body_to_prism_translation_m"], dtype=float)
    if lever.shape != (3,) or not np.isfinite(lever).all():
        raise ValueError("invalid body-to-prism lever arm")
    ref_ns = [int(row["stamp_ns"]) for row in references]
    if any(a >= b for a, b in zip(ref_ns, ref_ns[1:])):
        raise ValueError("reference timestamps are not strictly increasing")
    ref_positions = np.asarray([[float(row[k]) for k in ("x", "y", "z")] for row in references])
    if not np.isfinite(ref_positions).all():
        raise ValueError("nonfinite Leica position")
    matched = []
    outside = gaps = 0
    previous_ns = -1
    for row in estimates:
        stamp_ns = int(round(float(row["stamp_s"]) * 1e9))
        if stamp_ns <= previous_ns:
            raise ValueError("estimate timestamps are not strictly increasing")
        previous_ns = stamp_ns
        position = np.asarray([float(row[k]) for k in ("px", "py", "pz")])
        q = np.asarray([float(row[k]) for k in ("qw", "qx", "qy", "qz")])
        if not np.isfinite(position).all() or row["healthy"] not in ("0", "1"):
            raise ValueError("invalid LIO pose or health")
        prism = position + rotate(q, lever)
        i = bisect_left(ref_ns, stamp_ns)
        if i < len(ref_ns) and ref_ns[i] == stamp_ns:
            observed = ref_positions[i]
        elif i == 0 or i == len(ref_ns):
            outside += 1
            continue
        else:
            width = ref_ns[i] - ref_ns[i - 1]
            if width > max_gap_s * 1e9:
                gaps += 1
                continue
            fraction = (stamp_ns - ref_ns[i - 1]) / width
            observed = ref_positions[i - 1] + fraction * (ref_positions[i] - ref_positions[i - 1])
        matched.append((stamp_ns, prism, observed, row["healthy"] == "1", row.get("initial_scan") == "1"))
    if len(matched) < 3:
        raise ValueError("insufficient Leica reference coverage")
    source = np.asarray([row[1] for row in matched])
    target = np.asarray([row[2] for row in matched])
    rotation, translation, singular = rigid_alignment(source, target)
    aligned = source @ rotation.T + translation
    errors = np.linalg.norm(aligned - target, axis=1)
    times = np.asarray([row[0] for row in matched], dtype=np.int64)
    prefix = times <= times[0] + fit_duration_s * 1e9
    held_out = None
    if np.count_nonzero(prefix) >= 3 and np.count_nonzero(~prefix) >= 1:
        try:
            r0, t0, s0 = rigid_alignment(source[prefix], target[prefix])
            held_errors = np.linalg.norm(source[~prefix] @ r0.T + t0 - target[~prefix], axis=1)
            held_out = {"fit_duration_s": fit_duration_s, "fit_samples": int(np.count_nonzero(prefix)),
                        "rotation": r0.tolist(), "translation_m": t0.tolist(),
                        "singular_values": s0, "held_out": summarize(held_errors)}
        except ValueError:
            held_out = None
    report = {"reference": "Leica position of prism; no reference orientation",
              "scan_timing": metadata["scan_timing"], "prism_lever_body_m": lever.tolist(),
              "estimate_samples": len(estimates), "paired_samples": len(matched),
              "outside_reference": outside, "reference_gaps": gaps,
              "max_reference_gap_s": max_gap_s, "first_stamp_s": float(times[0] * 1e-9),
              "last_stamp_s": float(times[-1] * 1e-9),
              "healthy_fraction_all_estimates": sum(r["healthy"] == "1" for r in estimates) / len(estimates),
              "healthy_fraction_paired": sum(row[3] for row in matched) / len(matched),
              "initial_scan_included": any(row[4] for row in matched),
              "reference_path_length_m": float(np.linalg.norm(np.diff(target, axis=0), axis=1).sum()),
              "alignment": "rigid SE(3) fit to all paired prism positions, no scale",
              "rotation": rotation.tolist(), "translation_m": translation.tolist(),
              "alignment_singular_values": singular, "all_overlap": summarize(errors),
              "prefix_fit_holdout": held_out}
    data = [(row[0], *aligned[i], *row[2], float(errors[i]), int(row[3]), int(row[4]))
            for i, row in enumerate(matched)]
    return report, data


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("estimate", type=Path)
    parser.add_argument("reference", type=Path)
    parser.add_argument("metadata", type=Path)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--paired", type=Path, required=True)
    parser.add_argument("--max-reference-gap-s", type=float, default=0.1)
    parser.add_argument("--fit-duration-s", type=float, default=20.0)
    args = parser.parse_args()
    try:
        report, data = compare(args.estimate, args.reference, args.metadata,
                               args.max_reference_gap_s, args.fit_duration_s)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        with args.paired.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp_ns", "estimate_x", "estimate_y", "estimate_z",
                             "reference_x", "reference_y", "reference_z", "error_m", "healthy", "initial_scan"])
            writer.writerows(data)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"prism comparison: {exc}", file=sys.stderr)
        return 1
    print(f"Paired {report['paired_samples']}/{report['estimate_samples']} LIO scans; "
          f"rigid-fit RMSE={report['all_overlap']['rmse_m']:.3f} m, "
          f"healthy={report['healthy_fraction_paired']:.1%}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
