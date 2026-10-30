#!/usr/bin/env python3
"""Replay recorded stereo and project VIO estimates through obstacle memory."""

from __future__ import annotations

import argparse
import copy
import csv
from decimal import Decimal, ROUND_HALF_UP
import json
import math
import os
from pathlib import Path
import signal
import random
import subprocess
import sys
import time


REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / "scripts"))
from capture_raw_snapshot_3d import snapshot_document  # noqa: E402

OFFSET_US = 10_000_000
RANGE_M = 35.0
RESOLUTION_M = 0.25
TIMEOUT_S = 30.0
ESTIMATE_FIELDS = ("px", "py", "pz", "qw", "qx", "qy", "qz", "vx", "vy", "vz", "heading_variance_rad2")


def stamp_us(text: str) -> int:
    return int((Decimal(text) * 1_000_000).to_integral_value(rounding=ROUND_HALF_UP)) + OFFSET_US


def read_inputs(record: Path, estimate: Path, limit: int | None):
    calibration = json.loads((record / "calibration.json").read_text(encoding="utf-8"))
    frames = {}
    with (record / "frames.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamp = stamp_us(row["stamp_s"])
            side = row["side"]
            if side not in ("left", "right") or side in frames.setdefault(stamp, {}):
                raise ValueError(f"invalid or duplicate frame at {stamp}: {side}")
            frames[stamp][side] = (int(row["byte_offset"]), int(row["width"]), int(row["height"]))
    estimates = {}
    health = {"rows": 0, "healthy": 0, "unhealthy": 0, "dead_reckoning": 0, "invalid": 0}
    with estimate.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"stamp_s", "healthy", "dead_reckoning", *ESTIMATE_FIELDS}
        if not required.issubset(reader.fieldnames or ()):
            raise ValueError(f"estimate missing fields: {sorted(required - set(reader.fieldnames or ()))}")
        for row in reader:
            health["rows"] += 1
            healthy = row["healthy"].strip().lower() in ("1", "true")
            dead = row["dead_reckoning"].strip().lower() in ("1", "true")
            health["healthy" if healthy else "unhealthy"] += 1
            health["dead_reckoning"] += int(dead)
            # Deliberately access only the estimator whitelist, never reference columns.
            values = {name: float(row[name]) for name in ESTIMATE_FIELDS}
            norm = math.sqrt(sum(values[k] ** 2 for k in ("qw", "qx", "qy", "qz")))
            if not healthy or not all(math.isfinite(v) for v in values.values()) or norm < 1e-9 or values["heading_variance_rad2"] < 0:
                health["invalid"] += 1
                continue
            stamp = stamp_us(row["stamp_s"])
            if stamp in estimates:
                raise ValueError(f"duplicate estimate timestamp after quantization: {stamp}")
            for key in ("qw", "qx", "qy", "qz"):
                values[key] /= norm
            values["dead_reckoning"] = dead
            estimates[stamp] = values
    paired = []
    for stamp in sorted(frames):
        pair = frames[stamp]
        if set(pair) != {"left", "right"}:
            raise ValueError(f"unpaired stereo frame at {stamp}")
        if any(item[1:] != (int(calibration["width"]), int(calibration["height"])) for item in pair.values()):
            raise ValueError(f"frame dimensions disagree with calibration at {stamp}")
        if stamp in estimates:
            paired.append((stamp, pair, estimates[stamp]))
    available = len(paired)
    if limit is not None:
        paired = paired[:limit]
    if not paired or paired[0][0] <= 0:
        raise ValueError("no usable positive-timestamp frame/estimate pairs")
    return calibration, paired, health, len(frames), available


def quaternion_product(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def configuration(cal, pairs):
    # NED to map: east, north, up, with a constant positive altitude offset.
    altitude_offset = max(10.0, max(p[2]["pz"] for p in pairs) + 1.0)
    positions = [(p[2]["py"], p[2]["px"], altitude_offset - p[2]["pz"]) for p in pairs]
    translation = [float(v) for v in cal["left_camera_position_body_frd_m"]]
    margin = RANGE_M + math.sqrt(sum(v*v for v in translation)) + RESOLUTION_M
    lower = [math.floor((min(p[i] for p in positions) - margin) / RESOLUTION_M) * RESOLUTION_M for i in range(3)]
    upper = [math.ceil((max(p[i] for p in positions) + margin) / RESOLUTION_M) * RESOLUTION_M for i in range(3)]
    q = quaternion_product(cal["left_camera_to_body_frd_wxyz"], [0.5, 0.5, -0.5, 0.5])
    depth = dict(use_sim_time=True, image_width=int(cal["width"]), image_height=int(cal["height"]),
                 focal_px=float(cal["focal_px"]), principal_x_px=float(cal["principal_x_px"]),
                 principal_y_px=float(cal["principal_y_px"]), baseline_m=float(cal["baseline_m"]),
                 left_image_topic="/stereo/left/image", right_image_topic="/stereo/right/image",
                 returns_topic="/stereo_depth/points", frame_id="stereo_left",
                 pixel_stride=4, minimum_range_m=0.3, disparity_error_px=0.45, allowed_depth_error_m=0.25)
    memory = dict(use_sim_time=True, px4_clock_is_ros_clock=True, frame_id="map", use_static_map=False,
                  persistent_memory_enabled=True, persistent_memory_diagnostics_enabled=True,
                  persistent_memory_spectator_vehicle_id="", lidar_3d_topic="/euroc/replay/stereo_depth/points",
                  px4_local_position_topic="/euroc/vio/local_position", px4_vehicle_attitude_topic="/euroc/vio/attitude",
                  lidar_3d_hit_only_returns=True, lidar_surface_interpolation_enabled=False,
                  lidar_3d_minimum_range_m=0.3, max_lidar_range_m=RANGE_M,
                  use_full_lidar_extrinsic=True, compensate_lidar_attitude=True,
                  lidar_extrinsic_translation_body_frd_m=translation,
                  lidar_extrinsic_quaternion_lidar_flu_to_body_frd=q,
                  px4_to_map_m00=0.0, px4_to_map_m01=1.0, px4_to_map_m10=1.0, px4_to_map_m11=0.0,
                  px4_local_origin_x_m=0.0, px4_local_origin_y_m=0.0, px4_local_origin_z_m=altitude_offset,
                  use_initial_pose_until_px4=False, use_px4_heading_for_scan=True,
                  startup_heading_stable_sample_count=1,
                  maximum_heading_variance_rad2=0.05,
                  motion_compensate_lidar_pose=True, lidar_position_source_time_offset_s=0.0,
                  lidar_attitude_source_time_offset_s=0.0, lidar_scan_alignment_maximum_wait_s=0.35,
                  lidar_pose_source_maximum_receive_delay_s=1.0, lidar_pose_source_maximum_future_skew_s=0.1,
                  min_mapping_altitude_m=0.0, grid_resolution_m=RESOLUTION_M,
                  grid_origin_x=lower[0], grid_origin_y=lower[1], grid_origin_z=lower[2],
                  grid_width_m=upper[0]-lower[0], grid_height_m=upper[1]-lower[1], grid_depth_m=upper[2]-lower[2],
                  scan_stride=1, hit_weight=2, miss_weight=1, min_score=-8, max_score=12,
                  occupied_score=3, free_score=-1, decay_seconds_per_confirmation=0.0,
                  lidar_pose_heading_uncertainty_rad=0.0, lidar_memory_scan_queue_capacity=1,
                  obstacle_memory_3d_transport_rate_hz=20.0, gazebo_aligned_rviz_axes_swapped=False,
                  lidar_self_filter_radius_m=0.9, lidar_self_filter_upward_extent_m=0.5,
                  lidar_self_filter_downward_extent_m=0.5)
    return depth, memory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", required=True, type=Path)
    parser.add_argument("--estimate", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--max-frames", type=int)
    args = parser.parse_args()
    if args.max_frames is not None and args.max_frames <= 0:
        parser.error("--max-frames must be positive")
    cal, pairs, health, frame_count, available = read_inputs(args.record, args.estimate, args.max_frames)
    depth, memory = configuration(cal, pairs)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    params = args.output_dir / "parameters.json"
    params.write_text(json.dumps({"stereo_depth_node": {"ros__parameters": depth},
                                  "obstacle_memory_3d_node": {"ros__parameters": memory}}, indent=2) + "\n", encoding="utf-8")
    # Isolate replay topics and clock from unrelated graphs, including local Gazebo.
    os.environ["ROS_DOMAIN_ID"] = str(random.SystemRandom().randrange(100, 230))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, PointCloud2
    from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
    from drone_city_nav.msg import LatestSensorObstacleScan, ObstacleMemoryStatus, RawObstacleSnapshot3D, RawObstacleDelta3D

    class Replay(Node):
        def __init__(self):
            super().__init__("real_recording_memory_replay")
            self.clock_pub = self.create_publisher(Clock, "/clock", 10)
            self.left = self.create_publisher(Image, "/stereo/left/image", qos_profile_sensor_data)
            self.right = self.create_publisher(Image, "/stereo/right/image", qos_profile_sensor_data)
            self.position = self.create_publisher(VehicleLocalPosition, "/euroc/vio/local_position", qos_profile_sensor_data)
            self.attitude = self.create_publisher(VehicleAttitude, "/euroc/vio/attitude", qos_profile_sensor_data)
            self.cloud_relay = self.create_publisher(PointCloud2, "/euroc/replay/stereo_depth/points", qos_profile_sensor_data)
            transient = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            delta_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.clouds, self.scans, self.statuses = {}, {}, {}
            self.snapshots, self.deltas = {}, {}
            self.pose_pair_retransmissions = 0
            self.cloud_count = self.nonempty_cloud_count = 0
            self.scan_count = self.scan_source_beams = self.scan_invalid_beams = 0
            self.snapshot_count = self.delta_count = 0
            self.forwarded_stamps = set()
            self.image_pair_retransmissions = self.duplicate_clouds = 0
            self.create_subscription(PointCloud2, "/stereo_depth/points", self.on_cloud, qos_profile_sensor_data)
            self.create_subscription(LatestSensorObstacleScan, "/drone_city_nav/latest_sensor_obstacle_scan", lambda m: self.scans.__setitem__(self.key(m), m), qos_profile_sensor_data)
            self.create_subscription(ObstacleMemoryStatus, "/drone_city_nav/obstacle_memory_status", lambda m: self.statuses.__setitem__(self.key(m), m), 10)
            self.create_subscription(RawObstacleSnapshot3D, "/drone_city_nav/raw_obstacle_snapshot_3d", self.on_snapshot, transient)
            self.create_subscription(RawObstacleDelta3D, "/drone_city_nav/raw_obstacle_delta_3d", self.on_delta, delta_qos)

        @staticmethod
        def key(message):
            return message.header.stamp.sec * 1_000_000 + message.header.stamp.nanosec // 1000

        def on_cloud(self, message):
            stamp = self.key(message)
            if stamp in self.forwarded_stamps or stamp in self.clouds:
                self.duplicate_clouds += 1
                return
            self.clouds[stamp] = message
            self.cloud_count += 1
            self.nonempty_cloud_count += int(message.width * message.height > 0)

        def on_snapshot(self, message):
            self.snapshot_count += 1
            self.snapshots[(message.producer_instance_id, message.obstacle_snapshot_revision)] = message
            keys = sorted((key for key in self.snapshots if key[0] == message.producer_instance_id), key=lambda key: key[1])
            for key in keys[:-2]:
                del self.snapshots[key]

        def on_delta(self, message):
            self.delta_count += 1
            key = (message.producer_instance_id, message.base_snapshot_revision)
            previous = self.deltas.get(key)
            if previous is None or message.obstacle_snapshot_revision > previous.obstacle_snapshot_revision:
                self.deltas[key] = message
            keys = sorted((key for key in self.deltas if key[0] == message.producer_instance_id), key=lambda key: key[1])
            for old in keys[:-2]:
                del self.deltas[old]

        def wait_for(self, predicate, children, what, heartbeat=None, heartbeat_interval_s=0.05):
            deadline = time.monotonic() + TIMEOUT_S
            next_heartbeat = time.monotonic() + heartbeat_interval_s
            while not predicate():
                for child in children:
                    if child.poll() is not None:
                        raise RuntimeError(f"node exited while waiting for {what}: {child.args}")
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"timeout waiting for {what}; clouds={list(self.clouds)} scans={list(self.scans)}")
                if heartbeat is not None and time.monotonic() >= next_heartbeat:
                    heartbeat()
                    next_heartbeat = time.monotonic() + heartbeat_interval_s
                rclpy.spin_once(self, timeout_sec=0.05)

        def set_clock(self, stamp):
            msg = Clock()
            msg.clock.sec, us = divmod(stamp, 1_000_000)
            msg.clock.nanosec = us * 1000
            self.clock_pub.publish(msg)

        def publish_pose(self, stamp, p):
            pos = VehicleLocalPosition()
            pos.timestamp = pos.timestamp_sample = stamp
            pos.x, pos.y, pos.z = p["px"], p["py"], p["pz"]
            pos.vx, pos.vy, pos.vz = p["vx"], p["vy"], p["vz"]
            w, x, y, z = (p[k] for k in ("qw", "qx", "qy", "qz"))
            pos.heading = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
            pos.heading_var = p["heading_variance_rad2"]
            pos.xy_valid = pos.z_valid = pos.heading_good_for_control = True
            attitude = VehicleAttitude()
            attitude.timestamp = attitude.timestamp_sample = stamp
            attitude.q = [w, x, y, z]
            self.position.publish(pos)
            self.attitude.publish(attitude)
            return pos, attitude

        def publish_image(self, pub, stamp, item, stream, side):
            offset, width, height = item
            stream.seek(offset)
            data = stream.read(width * height)
            if len(data) != width * height:
                raise ValueError(f"short {side} image at {stamp}")
            image = Image()
            image.header.stamp.sec, us = divmod(stamp, 1_000_000)
            image.header.stamp.nanosec = us * 1000
            image.header.frame_id = "stereo_" + side
            image.width, image.height, image.step = width, height, width
            image.encoding = "mono8"
            image.data = data
            pub.publish(image)

    children = []
    logs = []
    node = None
    rclpy.init()
    try:
        node = Replay()
        for executable in ("stereo_depth_node", "obstacle_memory_3d_node"):
            log = (args.output_dir / (executable + ".log")).open("w", encoding="utf-8")
            logs.append(log)
            children.append(subprocess.Popen(["ros2", "run", "drone_city_nav", executable,
                                               "--ros-args", "--params-file", str(params.resolve())],
                                              stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy(), start_new_session=True))
        node.wait_for(lambda: node.left.get_subscription_count() >= 1 and node.right.get_subscription_count() >= 1
                      and node.position.get_subscription_count() >= 1 and node.attitude.get_subscription_count() >= 1
                      and node.clock_pub.get_subscription_count() >= 2
                      and node.cloud_relay.get_subscription_count() >= 1
                      and node.count_publishers("/stereo_depth/points") >= 1
                      and node.count_publishers("/drone_city_nav/latest_sensor_obstacle_scan") >= 1
                      and node.count_publishers("/drone_city_nav/obstacle_memory_status") >= 1,
                      children, "node subscriptions and publishers")
        with (args.record / "left.bin").open("rb") as left, (args.record / "right.bin").open("rb") as right:
            for stamp, pair, pose in pairs:
                # The consumers' simulated clocks must receive this stamp before
                # a cloud: a receive time of zero makes the memory discard it.
                end = time.monotonic() + 0.1
                while time.monotonic() < end:
                    node.set_clock(stamp)
                    rclpy.spin_once(node, timeout_sec=0.01)
                position, attitude = node.publish_pose(stamp, pose)
                end = time.monotonic() + 0.05
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.01)
                node.publish_image(node.left, stamp, pair["left"], left, "left")
                node.publish_image(node.right, stamp, pair["right"], right, "right")
                def resend_pair():
                    node.publish_image(node.left, stamp, pair["left"], left, "left")
                    node.publish_image(node.right, stamp, pair["right"], right, "right")
                    node.image_pair_retransmissions += 1
                node.wait_for(lambda: stamp in node.clouds, children, f"cloud for {stamp}",
                              resend_pair, heartbeat_interval_s=1.0)
                # Relay the actual depth output only once after its memory subscriber
                # is matched; graph discovery alone did not guarantee DDS delivery.
                node.forwarded_stamps.add(stamp)
                node.cloud_relay.publish(node.clouds.pop(stamp))
                def resend_pose():
                    # The first heading transition may clear an earlier attitude;
                    # either best-effort pose sample may also be missed. Repeat
                    # the same recorded pose, but never relay the cloud twice.
                    node.set_clock(stamp)
                    node.position.publish(position)
                    node.attitude.publish(attitude)
                    node.pose_pair_retransmissions += 1
                node.wait_for(lambda: stamp in node.scans, children, f"aligned scan for {stamp}",
                              resend_pose, heartbeat_interval_s=0.25)
                scan = node.scans.pop(stamp)
                node.scan_count += 1
                node.scan_source_beams += scan.source_beam_count
                node.scan_invalid_beams += scan.invalid_beam_count
        final_stamp = pairs[-1][0]
        node.set_clock(final_stamp + 100_000)
        node.wait_for(lambda: final_stamp in node.statuses, children, "final memory status")
        status = node.statuses[final_stamp]
        def final_message():
            candidates = [m for (producer, _), m in node.snapshots.items() if producer == status.producer_instance_id]
            candidates += [m for (producer, _), m in node.deltas.items() if producer == status.producer_instance_id]
            candidates = [m for m in candidates if node.key(m) <= final_stamp]
            if status.raw_snapshot_published or status.raw_delta_published:
                candidates = [m for m in candidates if node.key(m) == final_stamp]
            return max(candidates, key=lambda m: m.obstacle_snapshot_revision, default=None)
        node.wait_for(lambda: final_message() is not None, children, "final raw snapshot or delta")
        latest = final_message()
        if isinstance(latest, RawObstacleDelta3D):
            base_key = (latest.producer_instance_id, latest.base_snapshot_revision)
            node.wait_for(lambda: base_key in node.snapshots, children, "compatible raw base snapshot")
            base = copy.deepcopy(node.snapshots[base_key])
            geometry = ("origin_x_m", "origin_y_m", "origin_z_m", "resolution_m", "width_cells", "height_cells", "depth_cells", "chunk_size_cells")
            if any(getattr(base, k) != getattr(latest, k) for k in geometry):
                raise RuntimeError("delta and snapshot geometry mismatch")
            chunks = {(c.x, c.y, c.z): c for c in base.chunks}
            for chunk in latest.chunks:
                chunks[(chunk.x, chunk.y, chunk.z)] = chunk
            base.chunks = list(chunks.values())
            base.header = latest.header
            base.obstacle_snapshot_revision = latest.obstacle_snapshot_revision
            latest = base
        document = snapshot_document(latest, None)
        if node.scan_count == 0 or node.scan_source_beams == 0 or document is None:
            raise RuntimeError("no actual scan or occupied raw snapshot observed")
        if document["selection"]["occupied_voxel_count"] != status.occupied_cell_count:
            raise RuntimeError("final raw snapshot occupied count disagrees with memory status")
        (args.output_dir / "raw_snapshot_3d.json").write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
        report = {"record": str(args.record), "estimate": str(args.estimate),
                  "initialization": "VIO initialized once from raw Vicon position and heading; no reference measurements are published by this replay",
                  "scope": "Open-loop replay; raw stereo is not map ground truth and does not demonstrate closed-loop safety.",
                  "clock": "record-relative seconds rounded to microseconds plus 10 seconds; /clock and PX4 timestamps share this timeline",
                  "frames_indexed": frame_count, "usable_pairs_before_limit": available, "pairs_published": len(pairs),
                  "clouds_received": node.cloud_count, "nonempty_clouds": node.nonempty_cloud_count,
                  "scans_received": node.scan_count, "scan_source_beams": node.scan_source_beams,
                  "pose_pair_retransmissions": node.pose_pair_retransmissions,
                  "image_pair_retransmissions": node.image_pair_retransmissions,
                  "duplicate_clouds_discarded": node.duplicate_clouds,
                  "transport_note": "Frames are paced by real aligned scan acknowledgements; 20 Hz transport and the single-slot integration queue can coalesce updates. Status coverage is reported separately.",
                  "scan_invalid_beams": node.scan_invalid_beams,
                  "statuses_received": len(node.statuses), "snapshot_messages": node.snapshot_count, "delta_messages": node.delta_count,
                  "health": health, "published_dead_reckoning": sum(p[2]["dead_reckoning"] for p in pairs),
                  "final_status": {"producer_instance_id": int(status.producer_instance_id), "sequence": int(status.sequence),
                                   "occupied_cell_count": int(status.occupied_cell_count), "raw_snapshot_published": status.raw_snapshot_published,
                                   "raw_delta_published": status.raw_delta_published},
                  "snapshot": {"producer_instance_id": document["producer_instance_id"], "revision": document["revision"], **document["selection"]},
                  "assumptions": "Simulation stereo disparity error, range and vehicle self-filter are not calibrated guarantees for this recording. Heading integration uncertainty bound is disabled, not measured as zero. Grid covers estimated positions plus configured maximum range and camera offset; this is not a ground-truth clipping assessment. No static map or ToF input.",
                  "parameters": {"stereo": depth, "memory": memory}}
        (args.output_dir / "metadata.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    finally:
        for child in children:
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()
        for log in logs:
            log.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
