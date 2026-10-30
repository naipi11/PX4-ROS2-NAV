#!/usr/bin/env python3
"""Convert an ASL-format EuRoC sequence into a calibrated offline VIO record.

The record uses a common integer-nanosecond epoch, rectified mono images,
IMU samples in FRD and Vicon poses transformed to the IMU origin. The Vicon
poses are evaluation data, not measurements supplied to the filter. The replay
uses a declared initial position and heading from that reference; it never
feeds subsequent reference poses into the filter.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml


SCHEMA = "drone_city_nav_euroc_calibration_v1"
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def read_csv(path: Path) -> list[list[str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.reader(stream) if row and not row[0].startswith("#")]
    if not rows:
        raise ValueError(f"empty sensor data: {path}")
    stamps = [int(row[0]) for row in rows]
    if any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError(f"sensor timestamps are not strictly increasing: {path}")
    return rows


def seconds(ns: int) -> str:
    sign = "-" if ns < 0 else ""
    whole, fraction = divmod(abs(ns), 1_000_000_000)
    return f"{sign}{whole}.{fraction:09d}"


def calibration(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"invalid calibration: {path}")
    return value


def transform(sensor: dict) -> tuple[np.ndarray, np.ndarray]:
    data = sensor["T_BS"]
    if data["rows"] != 4 or data["cols"] != 4:
        raise ValueError("sensor T_BS must be 4x4")
    matrix = np.asarray(data["data"], dtype=float).reshape(4, 4)
    if not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("invalid sensor T_BS")
    rotation = matrix[:3, :3]
    if np.linalg.det(rotation) <= 0 or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        raise ValueError("sensor T_BS rotation is not orthonormal")
    # Some ASL sensor transforms are rounded to five decimal places.
    return Rotation.from_matrix(rotation).as_matrix(), matrix[:3, 3]


def camera_intrinsics(sensor: dict) -> tuple[np.ndarray, np.ndarray]:
    if sensor.get("camera_model") != "pinhole" or sensor.get("distortion_model") != "radial-tangential":
        raise ValueError("only EuRoC pinhole radial-tangential cameras are supported")
    fx, fy, cx, cy = map(float, sensor["intrinsics"])
    distortion = np.asarray(sensor["distortion_coefficients"], dtype=float)
    if fx <= 0 or fy <= 0 or len(distortion) != 4 or not np.isfinite([fx, fy, cx, cy, *distortion]).all():
        raise ValueError("invalid camera intrinsics")
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float), distortion


def rectification(left: dict, right: dict, imu: dict) -> tuple[dict, tuple[np.ndarray, ...]]:
    size = tuple(map(int, left["resolution"]))
    if len(size) != 2 or min(size) <= 0 or size != tuple(map(int, right["resolution"])):
        raise ValueError("stereo camera resolutions must match")
    k0, d0 = camera_intrinsics(left)
    k1, d1 = camera_intrinsics(right)
    r_b0, p_b0 = transform(left)
    r_b1, p_b1 = transform(right)
    r_bi, p_bi = transform(imu)
    # OpenCV's stereoRectify takes the transform from raw cam0 to raw cam1.
    r_10 = r_b1.T @ r_b0
    p_10 = r_b1.T @ (p_b0 - p_b1)
    r0, r1, p0, p1, _, roi0, roi1 = cv2.stereoRectify(
        k0, d0, k1, d1, size, r_10, p_10, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    focal = float(p0[0, 0])
    baseline = float(-p1[0, 3] / p1[0, 0])
    if (focal <= 0 or baseline <= 0 or not np.isfinite(p0).all() or not np.isfinite(p1).all()
            or not np.allclose([p0[1, 1], p1[0, 0], p1[1, 1]], focal, atol=1e-8)
            or not np.allclose(p0[:2, 2], p1[:2, 2], atol=1e-8)):
        raise ValueError("rectification did not produce a positive horizontal square-pixel pair")
    r_body_rect = FLU_TO_FRD @ r_bi.T @ r_b0 @ r0.T
    p_body_left = FLU_TO_FRD @ r_bi.T @ (p_b0 - p_bi)
    p_body_right = FLU_TO_FRD @ r_bi.T @ (p_b1 - p_bi)
    if not np.allclose(p_body_right - p_body_left, r_body_rect @ [baseline, 0, 0], atol=1e-6):
        raise ValueError("rectified baseline disagrees with camera-to-IMU extrinsics")
    quaternion = Rotation.from_matrix(r_body_rect).as_quat()  # x,y,z,w
    maps = (*cv2.initUndistortRectifyMap(k0, d0, r0, p0, size, cv2.CV_32FC1),
            *cv2.initUndistortRectifyMap(k1, d1, r1, p1, size, cv2.CV_32FC1))
    result = {
        "width": size[0], "height": size[1], "focal_px": focal,
        "principal_x_px": float(p0[0, 2]), "principal_y_px": float(p0[1, 2]),
        "baseline_m": baseline,
        "left_camera_to_body_frd_wxyz": quaternion[[3, 0, 1, 2]].tolist(),
        "left_camera_position_body_frd_m": p_body_left.tolist(),
        "right_camera_position_body_frd_m": p_body_right.tolist(),
        "left_rectification_rotation": r0.tolist(), "right_rectification_rotation": r1.tolist(),
        "left_valid_roi": list(roi0), "right_valid_roi": list(roi1),
    }
    return result, maps


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def convert(mav: Path, out: Path) -> dict:
    if out.exists():
        raise ValueError(f"output already exists: {out}")
    paths = {name: mav / name / "sensor.yaml" for name in ("cam0", "cam1", "imu0", "vicon0")}
    sensors = {name: calibration(path) for name, path in paths.items()}
    left, right = read_csv(mav / "cam0/data.csv"), read_csv(mav / "cam1/data.csv")
    imu, vicon = read_csv(mav / "imu0/data.csv"), read_csv(mav / "vicon0/data.csv")
    if [row[0] for row in left] != [row[0] for row in right]:
        raise ValueError("left and right camera timestamps do not match")
    geometry, maps = rectification(sensors["cam0"], sensors["cam1"], sensors["imu0"])
    epoch = int(imu[0][0])
    paired = [(a, b) for a, b in zip(left, right) if int(vicon[0][0]) <= int(a[0]) <= int(vicon[-1][0])]
    if len(paired) < 2:
        raise ValueError("no stereo frames inside the Vicon reference interval")
    r_bv, p_bv = transform(sensors["vicon0"])
    r_bi, p_bi = transform(sensors["imu0"])
    r_bv_q = Rotation.from_matrix(r_bv)
    r_bi_q = Rotation.from_matrix(r_bi)
    out.mkdir(parents=True)
    with (out / "imu_px4.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp_s", "gx", "gy", "gz", "ax", "ay", "az"])
        for row in imu:
            gyro = FLU_TO_FRD @ np.asarray(row[1:4], dtype=float)
            accel = FLU_TO_FRD @ np.asarray(row[4:7], dtype=float)
            writer.writerow([seconds(int(row[0]) - epoch), *gyro, *accel])
    with (out / "truth.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["stamp_s", "x", "y", "z", "qw", "qx", "qy", "qz"])
        for row in vicon:
            p_rv = np.asarray(row[1:4], dtype=float)
            q = np.asarray(row[4:8], dtype=float)
            r_rv = Rotation.from_quat(q[[1, 2, 3, 0]])
            r_rb = r_rv * r_bv_q.inv()
            p_ri = p_rv + r_rb.apply(p_bi - p_bv)
            q_ri = (r_rb * r_bi_q).as_quat()
            writer.writerow([seconds(int(row[0]) - epoch), *p_ri, *q_ri[[3, 0, 1, 2]]])
    with ((out / "left.bin").open("wb") as left_out,
          (out / "right.bin").open("wb") as right_out,
          (out / "frames.csv").open("w", newline="", encoding="utf-8") as index):
        writer = csv.writer(index)
        writer.writerow(["side", "stamp_s", "wall_s", "width", "height", "byte_offset"])
        for left_row, right_row in paired:
            stamp = seconds(int(left_row[0]) - epoch)
            for side, row, dest, mx, my in (("left", left_row, left_out, maps[0], maps[1]),
                                             ("right", right_row, right_out, maps[2], maps[3])):
                camera = "cam0" if side == "left" else "cam1"
                image = cv2.imread(str(mav / camera / "data" / row[1]), cv2.IMREAD_GRAYSCALE)
                if image is None or image.shape != (geometry["height"], geometry["width"]):
                    raise ValueError(f"missing or wrongly sized {camera} image: {row[1]}")
                rectified = cv2.remap(image, mx, my, cv2.INTER_LINEAR)
                writer.writerow([side, stamp, "", geometry["width"], geometry["height"], dest.tell()])
                dest.write(rectified.tobytes())
    imu_sensor = sensors["imu0"]
    geometry.update({
        "schema": SCHEMA, "time_origin_ns": str(epoch), "reference": "raw_vicon0_to_imu",
        "reference_note": "Vicon pose transformed using its T_BS and IMU T_BS; camera/IMU stamps are shutter-aligned by EuRoC; Vicon synchronization has the dataset's documented limits.",
        "frame_stamp_offset_s": 0.0,
        "gyroscope_noise_density": float(imu_sensor["gyroscope_noise_density"]),
        "accelerometer_noise_density": float(imu_sensor["accelerometer_noise_density"]),
        "gyroscope_random_walk": float(imu_sensor["gyroscope_random_walk"]),
        "accelerometer_random_walk": float(imu_sensor["accelerometer_random_walk"]),
        "stereo_pairs": len(paired), "imu_samples": len(imu), "reference_samples": len(vicon),
        "calibration_sha256": {name: sha256(path) for name, path in paths.items()},
    })
    (out / "calibration.json").write_text(json.dumps(geometry, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return geometry


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mav0", type=Path, help="ASL-format mav0 directory")
    parser.add_argument("output", type=Path, help="new output directory")
    args = parser.parse_args()
    try:
        result = convert(args.mav0, args.output)
    except (OSError, ValueError, KeyError, TypeError, cv2.error) as exc:
        print(f"EuRoC conversion: {exc}", file=sys.stderr)
        return 1
    print(f"Converted {result['stereo_pairs']} calibrated stereo pairs, "
          f"{result['imu_samples']} IMU samples and {result['reference_samples']} Vicon poses")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
