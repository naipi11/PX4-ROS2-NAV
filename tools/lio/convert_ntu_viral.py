#!/usr/bin/env python3
"""Convert a real NTU VIRAL ROS1 bag to the offline LIO record format.

The original Ouster header stamps and measured points are retained; each scan
is treated as a rigid cloud at its header stamp, without deskew or restamping.
The independent Leica prism positions are exported separately for evaluation.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import yaml


IMU_TOPIC = "/imu/imu"
CLOUD_TOPIC = "/os1_cloud_node1/points"
REFERENCE_TOPIC = "/leica/pose/relative"


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def matrix_from_opencv_yaml(path: Path, key: str) -> np.ndarray:
    text = path.read_text(encoding="utf-8").replace("%YAML:1.0", "").replace("!!opencv-matrix", "")
    value = yaml.safe_load(text)[key]
    if value["rows"] != 4 or value["cols"] != 4:
        raise ValueError(f"{key} is not 4x4")
    matrix = np.asarray(value["data"], dtype=float).reshape(4, 4)
    rotation = matrix[:3, :3]
    if (not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8)
            or np.linalg.det(rotation) <= 0 or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)):
        raise ValueError(f"invalid rigid transform: {key}")
    return matrix


def decode_cloud(message, transform: np.ndarray) -> tuple[np.ndarray, int, tuple[int, int]]:
    fields = {field.name: field for field in message.fields}
    types = {"x": 7, "y": 7, "z": 7, "t": 6, "range": 6}  # ROS FLOAT32 and UINT32
    for name, datatype in types.items():
        field = fields.get(name)
        if field is None or field.datatype != datatype or field.count != 1:
            raise ValueError(f"invalid or absent Ouster field: {name}")
        if field.offset < 0 or field.offset + 4 > message.point_step:
            raise ValueError(f"Ouster field outside point_step: {name}")
    if (message.width <= 0 or message.height <= 0 or message.point_step <= 0
            or message.row_step < message.width * message.point_step
            or len(message.data) < message.row_step * message.height):
        raise ValueError("invalid PointCloud2 layout")
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype({"names": list(types), "formats": [endian + "f4"] * 3 + [endian + "u4"] * 2,
                      "offsets": [fields[name].offset for name in types], "itemsize": message.point_step})
    points = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                        strides=(message.row_step, message.point_step)).reshape(-1)
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    valid = np.isfinite(xyz).all(axis=1) & (points["range"] > 0) & (np.linalg.norm(xyz, axis=1) > 0)
    invalid = int(len(xyz) - np.count_nonzero(valid))
    if not np.any(valid):
        raise ValueError("Ouster cloud has no valid physical returns")
    body = xyz[valid] @ transform[:3, :3].T + transform[:3, 3]
    times = points["t"][valid]
    return np.asarray(body, dtype="<f4", order="C"), invalid, (int(times.min()), int(times.max()))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def convert(bag: Path, calibration_dir: Path, output: Path) -> dict:
    from rosbags.highlevel import AnyReader  # optional, isolated offline tool dependency

    if output.exists():
        raise ValueError(f"output already exists: {output}")
    lidar_path = calibration_dir / "lidar_horz.yaml"
    imu_path = calibration_dir / "imu_v100.yaml"
    prism_path = calibration_dir / "leica_prism.yaml"
    lidar = matrix_from_opencv_yaml(lidar_path, "T_Body_Lidar")
    imu_transform = matrix_from_opencv_yaml(imu_path, "T_Body_Imu")
    prism = matrix_from_opencv_yaml(prism_path, "T_Body_Prism")
    if not np.allclose(imu_transform, np.eye(4), atol=1e-8):
        raise ValueError("this converter requires the IMU at the declared body origin")
    with AnyReader([bag]) as reader:
        connections = [c for c in reader.connections if c.topic in (IMU_TOPIC, CLOUD_TOPIC, REFERENCE_TOPIC)]
        if set(c.topic for c in connections) != {IMU_TOPIC, CLOUD_TOPIC, REFERENCE_TOPIC}:
            raise ValueError("bag lacks IMU, horizontal lidar or Leica reference")
        imu_connection = [c for c in connections if c.topic == IMU_TOPIC]
        first = next(reader.messages(connections=imu_connection), None)
        if first is None:
            raise ValueError("bag has no IMU messages")
        epoch = stamp_ns(reader.deserialize(first[2], first[0].msgtype))
        output.mkdir(parents=True)
        counts = {topic: 0 for topic in (IMU_TOPIC, CLOUD_TOPIC, REFERENCE_TOPIC)}
        previous = {topic: -1 for topic in counts}
        invalid = 0
        point_time_min = None
        point_time_max = None
        cloud_frame = None
        with ((output / "imu.csv").open("w", newline="", encoding="utf-8") as imu_file,
              (output / "scans.csv").open("w", newline="", encoding="utf-8") as scan_file,
              (output / "prism_reference.csv").open("w", newline="", encoding="utf-8") as truth_file,
              (output / "scans.bin").open("wb") as data_file):
            wi, ws, wt = csv.writer(imu_file), csv.writer(scan_file), csv.writer(truth_file)
            wi.writerow(["stamp_ns", "gx", "gy", "gz", "ax", "ay", "az"])
            ws.writerow(["stamp_ns", "offset_bytes", "point_count"])
            wt.writerow(["stamp_ns", "x", "y", "z"])
            for connection, bag_stamp, raw in reader.messages(connections=connections):
                message = reader.deserialize(raw, connection.msgtype)
                stamp = stamp_ns(message) - epoch
                topic = connection.topic
                if stamp < 0 or stamp <= previous[topic]:
                    raise ValueError(f"nonmonotonic or pre-epoch {topic} timestamp")
                previous[topic] = stamp
                counts[topic] += 1
                if topic == IMU_TOPIC:
                    gyro, accel = message.angular_velocity, message.linear_acceleration
                    values = (gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z)
                    if not all(np.isfinite(values)):
                        raise ValueError("nonfinite IMU measurement")
                    wi.writerow([stamp, *values])
                elif topic == CLOUD_TOPIC:
                    if cloud_frame is None:
                        cloud_frame = message.header.frame_id
                    if message.header.frame_id != cloud_frame:
                        raise ValueError("lidar frame changed within the bag")
                    points, rejected, (minimum, maximum) = decode_cloud(message, lidar)
                    invalid += rejected
                    point_time_min = minimum if point_time_min is None else min(point_time_min, minimum)
                    point_time_max = maximum if point_time_max is None else max(point_time_max, maximum)
                    ws.writerow([stamp, data_file.tell(), len(points)])
                    data_file.write(points.tobytes())
                else:
                    position = message.pose.position
                    values = (position.x, position.y, position.z)
                    if not all(np.isfinite(values)):
                        raise ValueError("nonfinite Leica position")
                    wt.writerow([stamp, *values])
    if any(count == 0 for count in counts.values()):
        raise ValueError("one or more selected streams are empty")
    metadata = {"schema": "drone_city_nav_ntu_lio_record_v1", "epoch_ns": str(epoch),
                "source_bag_sha256": sha256(bag), "calibration_sha256": {
                    "lidar": sha256(lidar_path), "imu": sha256(imu_path), "prism": sha256(prism_path)},
                "sensor_topics": counts, "cloud_frame_id": cloud_frame,
                "body_frame": "native VectorNav IMU axes treated as FRD; recorded rest acceleration points approximately along negative z",
                "lidar_to_body": lidar.tolist(), "imu_to_body": imu_transform.tolist(),
                "body_to_prism_translation_m": prism[:3, 3].tolist(),
                "invalid_lidar_returns": invalid, "point_time_min_ns": point_time_min,
                "point_time_max_ns": point_time_max,
                "scan_timing": "Original header stamps; individual point times retained only as bounds, no deskew or restamping.",
                "reference": "Leica prism position only; orientation is not measured and is not exported."}
    (output / "metadata.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return metadata


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("calibration_dir", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        result = convert(args.bag, args.calibration_dir, args.output)
    except (OSError, ValueError, KeyError, TypeError, ImportError, StopIteration) as exc:
        print(f"NTU conversion: {exc}", file=sys.stderr)
        return 1
    print(json.dumps({"epoch_ns": result["epoch_ns"], "counts": result["sensor_topics"],
                      "invalid_lidar_returns": result["invalid_lidar_returns"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
