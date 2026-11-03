#!/usr/bin/env python3
"""Prepare Middlebury Eval3 rectified images and disparity for stereo evaluation."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def read_pfm(path):
    with Path(path).open("rb") as stream:
        if stream.readline().strip() != b"Pf":
            raise ValueError("expected a single-channel PFM")
        dimensions = stream.readline().split()
        if len(dimensions) != 2:
            raise ValueError("invalid PFM dimensions")
        width, height = map(int, dimensions)
        scale = float(stream.readline())
        if width <= 0 or height <= 0 or not np.isfinite(scale) or scale == 0:
            raise ValueError("invalid PFM dimensions or scale")
        dtype = np.dtype("<f4" if scale < 0 else ">f4")
        values = np.frombuffer(stream.read(), dtype=dtype)
        if values.size != width * height:
            raise ValueError("PFM payload length does not match dimensions")
        # Middlebury's SDK uses the header sign for byte order and reads the
        # stored float values directly as disparity pixels; it does not scale them.
        return np.flipud(values.reshape(height, width)).astype(np.float32)


def read_calibration(path):
    fields = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            fields[key.strip()] = value.strip()
    def matrix(key):
        rows = fields[key].strip("[]").split(";")
        result = np.array([[float(v) for v in row.split()] for row in rows])
        if result.shape != (3, 3) or not np.isfinite(result).all():
            raise ValueError(f"invalid {key} matrix")
        return result
    cam0, cam1 = matrix("cam0"), matrix("cam1")
    doffs, baseline = float(fields["doffs"]), float(fields["baseline"])
    width, height = int(fields["width"]), int(fields["height"])
    if (not np.isfinite([doffs, baseline]).all() or baseline <= 0 or
            width <= 0 or height <= 0 or cam0[0, 0] <= 0 or cam0[1, 1] <= 0):
        raise ValueError("invalid calibration")
    if (not np.allclose([cam0[1, 1], cam1[0, 0], cam1[1, 1]], cam0[0, 0], atol=1e-3, rtol=0)
            or not np.isclose(cam0[1, 2], cam1[1, 2], atol=1e-3, rtol=0)
            or not np.isclose(cam1[0, 2] - cam0[0, 2], doffs, atol=2e-3, rtol=0)):
        raise ValueError("calibration is incompatible with a common rectified projection")
    return cam0, cam1, doffs, baseline, width, height


def prepare(scene, left_path, right_path, calibration_path, disparity_path, mask_path):
    cam0, cam1, doffs, baseline_mm, width, height = read_calibration(calibration_path)
    left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE)
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    disparity = read_pfm(disparity_path)
    shape = (height, width)
    if any(image is None or image.shape != shape for image in (left, right, mask, disparity)):
        raise ValueError("image, mask, disparity, and calibration dimensions must agree")
    if np.any((mask != 0) & (mask != 128) & (mask != 255)):
        raise ValueError("invalid Middlebury nonocclusion mask values")

    # The destination right pixel x samples the original right image at x+doffs.
    x = np.arange(width, dtype=np.float32)
    map_x = np.broadcast_to(x + doffs, shape).copy()
    map_y = np.broadcast_to(np.arange(height, dtype=np.float32)[:, None], shape).copy()
    aligned_right = cv2.remap(right, map_x, map_y, cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    effective_disparity = disparity + doffs
    # Middlebury marks unknown correspondence as 0, occluded as 128, and
    # nonoccluded as 255. A mask-unknown pixel is not an evaluation reference.
    valid = (np.isfinite(disparity) & (disparity >= 0) & np.isfinite(effective_disparity)
             & (effective_disparity > 0) & (mask != 0))
    depth = np.full(shape, np.nan, dtype=np.float32)
    depth[valid] = (baseline_mm / 1000.0 * cam0[0, 0] / effective_disparity[valid]).astype(np.float32)
    # A match must lie in the aligned destination and have source-image support.
    match_x = x[None, :] - effective_disparity
    source_x = match_x + doffs
    in_view = (match_x >= 0) & (match_x <= width - 1) & (source_x >= 0) & (source_x <= width - 1)
    reference_class = np.zeros(shape, dtype=np.uint8)
    reference_class[valid] = 2
    reference_class[valid & (mask == 255) & in_view] = 1
    arrays = dict(left=left, right=aligned_right, reference_depth_m=depth,
                  reference_class=reference_class, focal_px=np.float64(cam0[0, 0]),
                  principal_x_px=np.float64(cam0[0, 2]), principal_y_px=np.float64(cam0[1, 2]),
                  baseline_m=np.float64(baseline_mm / 1000.0))
    metadata = {
        "dataset": "Middlebury Stereo Evaluation Version 3 (Eval3), half resolution",
        "attribution": "Middlebury Stereo Datasets, https://vision.middlebury.edu/stereo/data/",
        "scene": scene,
        "inputs": {"left": str(left_path), "right": str(right_path), "calibration": str(calibration_path),
                   "disparity": str(disparity_path), "nonocclusion_mask": str(mask_path)},
        "cam0": cam0.tolist(), "cam1": cam1.tolist(), "doffs_px": doffs,
        "right_image_translation_px": -doffs, "baseline_mm": baseline_mm,
        "width": width, "height": height,
        "pfm_disparity_units": "raw floating-point pixel disparity; scale sign specifies byte order in the Middlebury SDK",
        "mask_values": "0 unknown, 128 occluded, 255 nonoccluded, as defined by the Middlebury SDK",
        "principal_point_offset_residual_px": float(cam1[0, 2] - doffs - cam0[0, 2]),
        "vertical_principal_point_difference_px": float(cam1[1, 2] - cam0[1, 2]),
        "focal_difference_px": [float(cam1[0, 0] - cam0[0, 0]), float(cam1[1, 1] - cam0[1, 1])],
        "limitations": "Rectification and synchronization are not assumed perfect or independently verified; the right image is shifted horizontally only. Reference gaps are unknown, not free space."
    }
    return arrays, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--left", type=Path, required=True)
    parser.add_argument("--right", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--disparity", type=Path, required=True)
    parser.add_argument("--mask", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    args = parser.parse_args()
    inputs = [args.left, args.right, args.calibration, args.disparity, args.mask]
    destinations = [args.output.resolve(), args.metadata.resolve()]
    if len(set(destinations)) != 2 or set(destinations) & {p.resolve() for p in inputs}:
        parser.error("output and metadata must be distinct from each other and all inputs")
    if args.output.exists() or args.metadata.exists():
        parser.error("output and metadata must not already exist")
    arrays, metadata = prepare(args.scene, *inputs)
    with args.output.open("xb") as stream:
        np.savez_compressed(stream, **arrays)
    try:
        with args.metadata.open("x", encoding="utf-8") as stream:
            json.dump(metadata, stream, indent=2, allow_nan=False)
            stream.write("\n")
    except Exception:
        args.output.unlink()
        raise


if __name__ == "__main__":
    main()
