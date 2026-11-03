#!/usr/bin/env python3
"""Replay prepared stereo frames through the production ROS depth node.

Reference arrays are copied to the output for offline scoring only; they are
never published. Output cloud endpoints are projected back to sampled pixels.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import time

import numpy as np


def cloud_to_samples(cloud, width, height, focal, cx, cy, stride):
    """Return sampled hit/free endpoints; coarse unobservable rays stay separate."""
    fields = {field.name: field for field in cloud.fields}
    if any(name not in fields or fields[name].datatype != 7 or fields[name].count != 1
           for name in ('x', 'y', 'z', 'intensity')):
        raise ValueError('depth cloud lacks scalar float32 fields')
    if (cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step
            or len(cloud.data) < cloud.row_step * cloud.height):
        raise ValueError('invalid depth cloud layout')
    offsets = [fields[name].offset for name in ('x', 'y', 'z', 'intensity')]
    if any(offset < 0 or offset + 4 > cloud.point_step for offset in offsets):
        raise ValueError('cloud field lies outside point step')
    endian = '>' if cloud.is_bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity'],
                      'formats': [endian + 'f4'] * 4, 'offsets': offsets,
                      'itemsize': cloud.point_step})
    points = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                        strides=(cloud.row_step, cloud.point_step)).reshape(-1)
    xyz = np.column_stack((points['x'], points['y'], points['z'])).astype(np.float64)
    flags = points['intensity'].astype(np.float64)
    if np.any(~np.isfinite(xyz)) or np.any(~np.isfinite(flags)):
        raise ValueError('nonfinite depth cloud point')
    kinds = np.zeros(flags.shape, dtype=np.uint8)
    kinds[(flags >= -0.5) & (flags < 0.5)] = 2
    kinds[(flags >= 0.5) & (flags < 1.5)] = 1
    kinds[(flags >= 1.5) & (flags < 2.5)] = 3
    if np.any(kinds == 0) or np.any(xyz[:, 0] <= 0):
        raise ValueError('invalid depth cloud evidence or forward endpoint')
    u = cx - focal * xyz[:, 1] / xyz[:, 0]
    v = cy - focal * xyz[:, 2] / xyz[:, 0]
    col = np.rint(u).astype(np.int64)
    row = np.rint(v).astype(np.int64)
    inside = ((col >= 0) & (col < width) & (row >= 0) & (row < height)
              & (np.abs(u - col) <= 0.05) & (np.abs(v - row) <= 0.05))
    grid = inside & (col % stride == 0) & (row % stride == 0)
    if np.any((kinds != 3) & ~grid):
        raise ValueError('measured stereo return does not map to a sample pixel')
    measured = grid & (kinds != 3)
    index = row[measured] * width + col[measured]
    if len(np.unique(index)) != len(index):
        raise ValueError('multiple measured returns map to the same pixel')
    pred_kind = np.zeros((height, width), dtype=np.uint8)
    pred_depth = np.full((height, width), np.nan, dtype=np.float32)
    pred_kind.flat[index] = kinds[measured]
    pred_depth.flat[index] = xyz[measured, 0].astype(np.float32)
    # Frustum samples are coarser than the stereo pixel grid and may repeat at
    # several ranges. Never expand those sparse markers into synthetic pixels.
    unobservable = grid & (kinds == 3)
    unobservable_indices = np.unique(row[unobservable] * width + col[unobservable])
    unobservable_indices = unobservable_indices[pred_kind.flat[unobservable_indices] == 0]
    pred_kind.flat[unobservable_indices] = 3
    sampled = np.zeros((height, width), dtype=bool)
    sampled[::stride, ::stride] = True
    stats = {'surface_hits': int(np.count_nonzero(kinds == 1)),
             'free_only_rays': int(np.count_nonzero(kinds == 2)),
             'coarse_unobservable_points': int(np.count_nonzero(kinds == 3)),
             'unobservable_on_sample_pixels': int(len(unobservable_indices))}
    return pred_kind, pred_depth, sampled, stats


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--stride', type=int, default=4)
    parser.add_argument('--disparity-error-px', type=float, default=0.45)
    parser.add_argument('--allowed-depth-error-m', type=float, default=0.25)
    parser.add_argument('--continuous', action='store_true',
                        help='retain node state between frames; only use with a continuous recording')
    parser.add_argument('--timeout-s', type=float, default=30.0)
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('--ros-domain-id', type=int)
    args = parser.parse_args()
    if (args.stride <= 0 or not math.isfinite(args.timeout_s) or args.timeout_s <= 0
            or not math.isfinite(args.disparity_error_px) or args.disparity_error_px <= 0
            or not math.isfinite(args.allowed_depth_error_m) or args.allowed_depth_error_m <= 0
            or (args.max_frames is not None and args.max_frames <= 0)
            or (args.ros_domain_id is not None and not 0 <= args.ros_domain_id <= 232)):
        parser.error('invalid stride, timeout, confidence, frame limit or ROS domain')
    files = sorted(args.input_dir.glob('*.npz'))
    if args.max_frames is not None:
        files = files[:args.max_frames]
    if not files:
        parser.error('input directory contains no prepared frames')
    if args.input_dir.resolve() == args.output_dir.resolve():
        parser.error('input and output directories must differ')
    with np.load(files[0], allow_pickle=False) as first:
        shape = first['left'].shape
        calibration = tuple(float(first[key]) for key in
                            ('focal_px', 'principal_x_px', 'principal_y_px', 'baseline_m'))
    if len(shape) != 2 or min(shape) <= 0 or not all(map(math.isfinite, calibration)):
        parser.error('invalid prepared frame geometry')
    height, width = shape
    focal, cx, cy, baseline = calibration
    if focal <= 0 or baseline <= 0:
        parser.error('focal length and baseline must be positive')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id if args.ros_domain_id is not None
                                      else random.SystemRandom().randrange(100, 230))
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2

    class Replay(Node):
        def __init__(self):
            super().__init__('stereo_visibility_replay')
            image_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
            self.left = self.create_publisher(Image, '/stereo/left/image', image_qos)
            self.right = self.create_publisher(Image, '/stereo/right/image', image_qos)
            self.clouds = {}
            self.create_subscription(PointCloud2, '/stereo_depth/points', self.receive, qos_profile_sensor_data)

        def receive(self, cloud):
            key = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
            self.clouds[key] = cloud

        def wait(self, predicate, child, label):
            deadline = time.monotonic() + args.timeout_s
            while not predicate():
                if child.poll() is not None:
                    raise RuntimeError(f'depth node exited while waiting for {label}')
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'timeout waiting for {label}')
                rclpy.spin_once(self, timeout_sec=0.05)

    command = ['ros2', 'run', 'drone_city_nav', 'stereo_depth_node', '--ros-args']
    parameters = {'image_width': width, 'image_height': height, 'focal_px': focal,
                  'principal_x_px': cx, 'principal_y_px': cy, 'baseline_m': baseline,
                  'pixel_stride': args.stride, 'disparity_error_px': args.disparity_error_px,
                  'allowed_depth_error_m': args.allowed_depth_error_m}
    for name, value in parameters.items():
        command.extend(['-p', f'{name}:={value}'])
    records = []
    rclpy.init()
    node = Replay()
    def stop_child(child):
        if child is None:
            return
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            child.wait()

    with (args.output_dir / 'stereo_depth_node.log').open('w', encoding='utf-8') as log:
        child = None
        try:
            previous_stamp = 0
            for sequence, path in enumerate(files):
                if child is None or not args.continuous:
                    if child is not None:
                        stop_child(child)
                        child = None
                        deadline = time.monotonic() + args.timeout_s
                        while (node.left.get_subscription_count() or node.right.get_subscription_count()
                               or node.count_publishers('/stereo_depth/points')):
                            if time.monotonic() >= deadline:
                                raise RuntimeError('previous stereo node did not leave the graph')
                            rclpy.spin_once(node, timeout_sec=0.05)
                    child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                             env=os.environ.copy(), start_new_session=True)
                    node.wait(lambda: node.left.get_subscription_count() >= 1
                              and node.right.get_subscription_count() >= 1
                              and node.count_publishers('/stereo_depth/points') >= 1,
                              child, 'stereo graph discovery')
                with np.load(path, allow_pickle=False) as source:
                    frame = {key: source[key] for key in source.files}
                if (frame['left'].shape != shape or frame['right'].shape != shape
                        or frame['left'].dtype != np.uint8 or frame['right'].dtype != np.uint8
                        or tuple(float(frame[key]) for key in
                                 ('focal_px', 'principal_x_px', 'principal_y_px', 'baseline_m')) != calibration):
                    raise ValueError(f'{path.name}: inconsistent frame geometry')
                if 'timestamp_s' in frame:
                    stamp = int(round(float(frame['timestamp_s']) * 1_000_000_000))
                else:
                    stamp = 10_000_000_000 + sequence * 100_000_000
                if stamp <= previous_stamp or stamp // 1_000_000_000 > 2_147_483_647:
                    raise ValueError(f'{path.name}: invalid or nonmonotonic frame timestamp')
                previous_stamp = stamp
                images = []
                for side in ('left', 'right'):
                    image = Image()
                    image.header.stamp.sec, image.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                    image.header.frame_id = 'stereo_' + side
                    image.width, image.height, image.step = width, height, width
                    image.encoding = 'mono8'
                    image.data = frame[side].tobytes()
                    images.append(image)
                start = time.monotonic()
                deadline = start + args.timeout_s
                retransmissions = 0
                next_send = start
                while stamp not in node.clouds:
                    now = time.monotonic()
                    if now >= next_send:
                        node.left.publish(images[0])
                        node.right.publish(images[1])
                        retransmissions += 1
                        next_send = now + 1.0
                    if child.poll() is not None:
                        raise RuntimeError(f'{path.name}: depth node exited')
                    if now >= deadline:
                        raise RuntimeError(f'{path.name}: depth cloud timed out')
                    rclpy.spin_once(node, timeout_sec=0.05)
                cloud = node.clouds.pop(stamp)
                elapsed_ms = (time.monotonic() - start) * 1000.0
                pred_kind, pred_depth, sampled, stats = cloud_to_samples(
                    cloud, width, height, focal, cx, cy, args.stride)
                frame.update(pred_kind=pred_kind, pred_depth_m=pred_depth, sampled=sampled)
                target = args.output_dir / path.name
                with target.open('xb') as stream:
                    np.savez_compressed(stream, **frame)
                records.append({'input': path.name, 'input_sha256': sha256(path), 'output': target.name,
                                'timestamp_ns': stamp, 'wall_delivery_ms': elapsed_ms,
                                'image_pair_retransmissions': retransmissions - 1, **stats})
                print(f'{path.name}: {stats["surface_hits"]} hits, {stats["free_only_rays"]} free rays', flush=True)
        finally:
            stop_child(child)
            node.destroy_node()
            rclpy.shutdown()
    report = {'node': 'drone_city_nav/stereo_depth_node', 'parameters': parameters,
              'temporal_mode': 'continuous' if args.continuous else 'independent_frames',
              'scope': 'Open-loop replay; reference is not published. Independent frames reset node history and do not evaluate temporal blindness. Wall delivery is not onboard control latency. Coarse unobservable markers are not expanded to pixels.',
              'frames': records}
    (args.output_dir / 'metadata.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
