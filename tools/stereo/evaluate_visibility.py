#!/usr/bin/env python3
"""Measure sampled stereo outputs against independent optical-axis reference depth.

Class 3 is sparse lidar with unverified stereo visibility. Unknown reference
pixels are coverage only, not free space or accuracy evidence. These descriptive
statistics are not an empirical safety guarantee.
"""

import argparse
import json
from pathlib import Path

import numpy as np


BAND_EDGES = (0.3, 1.0, 2.0, 4.0, 6.0, 8.0, float("inf"))


def _quantiles(values):
    if values.size == 0:
        return {key: None for key in ("median", "p90", "p95", "p99")}
    result = np.quantile(values, (0.5, 0.9, 0.95, 0.99))
    return dict(zip(("median", "p90", "p95", "p99"), map(float, result)))


def _scalar(data, name, positive=False):
    value = np.asarray(data[name])
    if value.shape != () or value.dtype.kind not in "iuf":
        raise ValueError(f"{name} must be a numeric scalar")
    value = float(value)
    if not np.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"invalid {name}")
    return value


def _metrics(mask, reference, kind, prediction, fb, threshold):
    hits = mask & (kind == 1)
    free = mask & (kind == 2)
    ref_hit = reference[hits].astype(np.float64)
    pred_hit = prediction[hits].astype(np.float64)
    return {
        "references": int(np.count_nonzero(mask)),
        "hits": int(np.count_nonzero(hits)),
        "free_only": int(np.count_nonzero(free)),
        "unobservable": int(np.count_nonzero(mask & (kind == 3))),
        "no_return": int(np.count_nonzero(mask & (kind == 0))),
        "hit_overreach": int(np.count_nonzero(pred_hit > ref_hit + threshold)),
        "signed_depth_error_m": _quantiles(pred_hit - ref_hit),
        "absolute_depth_error_m": _quantiles(np.abs(pred_hit - ref_hit)),
        "signed_disparity_error_px": _quantiles(fb / pred_hit - fb / ref_hit),
        "absolute_disparity_error_px": _quantiles(np.abs(fb / pred_hit - fb / ref_hit)),
        "free_only_overreach": int(np.count_nonzero(
            prediction[free].astype(np.float64) > reference[free].astype(np.float64) + threshold)),
    }


def evaluate(data, threshold_m=0.25):
    """Return JSON-compatible descriptive metrics for the shared NPZ contract."""
    if not np.isfinite(threshold_m) or threshold_m < 0:
        raise ValueError("threshold_m must be finite and nonnegative")
    names = ("left", "right", "reference_depth_m", "reference_class",
             "pred_kind", "pred_depth_m", "sampled")
    try:
        arrays = {name: np.asarray(data[name]) for name in names}
        focal = _scalar(data, "focal_px", positive=True)
        baseline = _scalar(data, "baseline_m", positive=True)
        _scalar(data, "principal_x_px")
        _scalar(data, "principal_y_px")
    except KeyError as exc:
        raise ValueError(f"missing field: {exc.args[0]}") from exc
    shape = arrays["left"].shape
    if len(shape) != 2 or 0 in shape or any(a.shape != shape for a in arrays.values()):
        raise ValueError("all image arrays must share a nonempty two-dimensional shape")
    for name in ("left", "right", "reference_class", "pred_kind"):
        if arrays[name].dtype != np.uint8:
            raise ValueError(f"{name} must be uint8")
    for name in ("reference_depth_m", "pred_depth_m"):
        if arrays[name].dtype != np.float32:
            raise ValueError(f"{name} must be float32")
    if arrays["sampled"].dtype != np.bool_:
        raise ValueError("sampled must be bool")
    ref, classes = arrays["reference_depth_m"], arrays["reference_class"]
    pred, kind, sampled = arrays["pred_depth_m"], arrays["pred_kind"], arrays["sampled"]
    if np.any(classes > 3) or np.any(kind > 3):
        raise ValueError("invalid reference class or prediction kind")
    known = classes != 0
    endpoints = (kind == 1) | (kind == 2)
    if (np.any(~np.isfinite(ref[known])) or np.any(ref[known] <= 0)
            or np.any(~np.isnan(ref[~known]))):
        raise ValueError("reference depth must be positive for known pixels and NaN otherwise")
    if (np.any(~np.isfinite(pred[endpoints])) or np.any(pred[endpoints] <= 0)
            or np.any(~np.isnan(pred[~endpoints]))):
        raise ValueError("prediction depth must be positive for endpoints and NaN otherwise")
    fb = focal * baseline
    if not np.isfinite(fb) or fb <= 0:
        raise ValueError("invalid focal-baseline product")
    result = {
        "threshold_m": float(threshold_m),
        "class_descriptions": {"1": "visible in both stereo views",
                               "2": "occluded or outside the aligned right image",
                               "3": "sparse synchronized lidar; stereo visibility unverified"},
        "coverage": {
            "pixels": int(sampled.size),
            "sampled": int(np.count_nonzero(sampled)),
            "sampled_without_reference": int(np.count_nonzero(sampled & ~known)),
            "sampled_with_reference": int(np.count_nonzero(sampled & known)),
        },
        "classes": {},
    }
    for class_id in (1, 2, 3):
        mask = sampled & (classes == class_id)
        metrics = _metrics(mask, ref, kind, pred, fb, threshold_m)
        metrics["below_0_3_m"] = _metrics(mask & (ref < BAND_EDGES[0]), ref, kind, pred, fb, threshold_m)
        metrics["depth_bands"] = [
            {"min_m": lower, "max_m": None if np.isinf(upper) else upper,
             **_metrics(mask & (ref >= lower) & (ref < upper), ref, kind, pred, fb, threshold_m)}
            for lower, upper in zip(BAND_EDGES, BAND_EDGES[1:])
        ]
        result["classes"][str(class_id)] = metrics
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--threshold-m", type=float, default=0.25)
    args = parser.parse_args()
    with np.load(args.input, allow_pickle=False) as data:
        result = evaluate(data, args.threshold_m)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
