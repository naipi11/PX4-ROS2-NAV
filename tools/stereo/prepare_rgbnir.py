#!/usr/bin/env python3
"""Prepare synchronized RGB-NIR stereo and sparse lidar for offline evaluation.

The lidar points do not prove mutual stereo visibility: unlabelled pixels stay
unknown. The frame-to-scan timestamp difference does not deskew the scan.
"""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def rectification(calibration):
    """Return camera rectification and maps; the source calibration uses mm."""
    keys = ('mtx_left', 'dist_left', 'mtx_right', 'dist_right', 'R', 'T', 'image_size')
    try:
        k0, d0, k1, d1, rotation, translation, size = (np.asarray(calibration[k]) for k in keys)
    except KeyError as exc:
        raise ValueError(f'missing stereo calibration: {exc.args[0]}') from exc
    if (k0.shape != (3, 3) or k1.shape != (3, 3) or rotation.shape != (3, 3)
            or translation.size != 3 or size.size != 2):
        raise ValueError('invalid stereo calibration dimensions')
    if not all(np.isfinite(v).all() for v in (k0, d0, k1, d1, rotation, translation, size)):
        raise ValueError('nonfinite stereo calibration')
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)):
        raise ValueError('invalid stereo rotation')
    width, height = (int(v) for v in size.reshape(-1))
    if width <= 0 or height <= 0 or (width, height) != tuple(size.reshape(-1)):
        raise ValueError('invalid image size')
    r0, r1, p0, p1, _, _, _ = cv2.stereoRectify(
        k0, d0, k1, d1, (width, height), rotation, translation.reshape(3, 1),
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    focal = float(p0[0, 0])
    baseline_m = float(-p1[0, 3] / p1[0, 0] / 1000.0)
    if (focal <= 0 or baseline_m <= 0 or not np.isfinite(p0).all() or not np.isfinite(p1).all()
            or not np.allclose([p0[1, 1], p1[0, 0], p1[1, 1]], focal, atol=1e-8)
            or not np.allclose(p0[:2, 2], p1[:2, 2], atol=1e-8)):
        raise ValueError('rectification does not produce a positive horizontal pair')
    maps0 = cv2.initUndistortRectifyMap(k0, d0, r0, p0, (width, height), cv2.CV_32FC1)
    maps1 = cv2.initUndistortRectifyMap(k1, d1, r1, p1, (width, height), cv2.CV_32FC1)
    return (width, height), r0, p0, baseline_m, maps0, maps1


def sampled_reference(points_lidar_m, camera_to_lidar_mm, left_rectification,
                      focal_px, principal_x_px, principal_y_px, baseline_m,
                      left_maps, right_maps, stride=4, tolerance_px=0.5):
    """Project only lidar measurements close to actual stereo sample centers.

    Class 3 means the lidar covers the pixel but camera-to-camera visibility is
    unverified. Class 2 is reserved for a right projection outside the rectified
    image. Conflicting nearby lidar depths are excluded rather than averaged.
    """
    transform = np.asarray(camera_to_lidar_mm, dtype=np.float64)
    rotation = np.asarray(left_rectification, dtype=np.float64)
    points = np.asarray(points_lidar_m, dtype=np.float64).reshape(-1, 3)
    if (transform.shape != (4, 4) or rotation.shape != (3, 3)
            or not np.isfinite(transform).all() or not np.isfinite(rotation).all()
            or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(transform[:3, :3]), 1.0, atol=1e-5)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)):
        raise ValueError('invalid camera-to-lidar or rectification transform')
    if (not np.isfinite([focal_px, principal_x_px, principal_y_px, baseline_m, tolerance_px]).all()
            or focal_px <= 0 or baseline_m <= 0 or not isinstance(stride, int) or stride <= 0
            or not 0 <= tolerance_px <= 0.5):
        raise ValueError('invalid projection parameters')
    if len(left_maps) != 2 or len(right_maps) != 2:
        raise ValueError('two rectification maps per camera are required')
    shape = left_maps[0].shape
    if len(shape) != 2 or any(m.shape != shape for m in (*left_maps, *right_maps)):
        raise ValueError('rectification map dimensions differ')
    height, width = shape
    valid = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 1e-6)
    points = points[valid]
    # cam2lidar maps camera millimetres into lidar millimetres.
    camera_mm = (points * 1000.0 - transform[:3, 3]) @ transform[:3, :3]
    rect_mm = camera_mm @ rotation.T
    valid = np.isfinite(rect_mm).all(axis=1) & (rect_mm[:, 2] > 0)
    rect_mm = rect_mm[valid]
    z = rect_mm[:, 2] / 1000.0
    u = focal_px * rect_mm[:, 0] / rect_mm[:, 2] + principal_x_px
    v = focal_px * rect_mm[:, 1] / rect_mm[:, 2] + principal_y_px
    col = np.rint(u / stride).astype(np.int64) * stride
    row = np.rint(v / stride).astype(np.int64) * stride
    valid = ((col >= 0) & (col < width) & (row >= 0) & (row < height)
             & (np.abs(u - col) <= tolerance_px) & (np.abs(v - row) <= tolerance_px))
    col, row, z, u, v = col[valid], row[valid], z[valid], u[valid], v[valid]
    # A rectified left pixel with no raw-image support cannot be assessed.
    left_x, left_y = left_maps
    valid = ((left_x[row, col] >= 0) & (left_x[row, col] <= width - 1)
             & (left_y[row, col] >= 0) & (left_y[row, col] <= height - 1))
    col, row, z, u, v = col[valid], row[valid], z[valid], u[valid], v[valid]
    indexes = row * width + col
    nearest = np.full(width * height, np.inf, dtype=np.float64)
    farthest = np.full(width * height, -np.inf, dtype=np.float64)
    np.minimum.at(nearest, indexes, z)
    np.maximum.at(farthest, indexes, z)
    populated = np.isfinite(nearest)
    conflict = populated & ((farthest - nearest) > np.maximum(0.15, 0.03 * nearest))
    nearest[conflict] = np.inf
    depth = np.full(width * height, np.nan, dtype=np.float32)
    classes = np.zeros(width * height, dtype=np.uint8)
    accepted = np.isfinite(nearest)
    depth[accepted] = nearest[accepted].astype(np.float32)
    classes[accepted] = 3
    # The common-principal rectification has right x = left x - f*b/z.
    chosen = np.flatnonzero(accepted)
    chosen_row, chosen_col = np.divmod(chosen, width)
    right_u = chosen_col - focal_px * baseline_m / nearest[chosen]
    right_col = np.rint(right_u).astype(np.int64)
    in_right = (right_col >= 0) & (right_col < width)
    supported = np.zeros(len(chosen), dtype=bool)
    right_x, right_y = right_maps
    rr, rc = chosen_row[in_right], right_col[in_right]
    supported[in_right] = ((right_x[rr, rc] >= 0) & (right_x[rr, rc] <= width - 1)
                           & (right_y[rr, rc] >= 0) & (right_y[rr, rc] <= height - 1))
    classes[chosen[~supported]] = 2
    stats = {'lidar_points_finite_nonzero': int(len(points)),
             'projected_near_sample_points': int(len(indexes)),
             'conflicting_sample_pixels': int(np.count_nonzero(conflict)),
             'sparse_visibility_unverified_pixels': int(np.count_nonzero(classes == 3)),
             'outside_right_view_pixels': int(np.count_nonzero(classes == 2))}
    return depth.reshape(shape), classes.reshape(shape), stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--max-lag-ms', type=float, default=20.0)
    parser.add_argument('--stride', type=int, default=4)
    parser.add_argument('--pixel-tolerance-px', type=float, default=0.5)
    args = parser.parse_args()
    if not np.isfinite(args.max_lag_ms) or args.max_lag_ms < 0:
        parser.error('max-lag-ms must be finite and nonnegative')
    # h5py is optional and installed only in the recording-evaluation environment.
    import h5py
    scene = args.scene_dir
    calibration_path = scene / 'calibration.npz'
    extrinsic_path = scene / 'cam2lidar.npy'
    with np.load(calibration_path, allow_pickle=False) as cal:
        size, r0, p0, baseline, maps0, maps1 = rectification(cal)
    extrinsic = np.load(extrinsic_path, allow_pickle=False)
    width, height = size
    args.output_dir.mkdir(parents=True, exist_ok=False)
    records = []
    rejected_lag = 0
    with h5py.File(scene / 'storage.hdf5', 'r') as archive:
        for frame_id in sorted(archive['frame'].keys()):
            group = archive['frame'][frame_id]
            if 'image' not in group or 'lidar' not in group or 'points' not in group['lidar']:
                raise ValueError(f'{frame_id}: missing recorded image or lidar data')
            image = group['image']
            lidar = group['lidar']
            stamp_s = float(image.attrs['timestamp'])
            lidar_stamp_ns = float(lidar.attrs['lidar_timestamp_ns'])
            lag_ms = (lidar_stamp_ns * 1e-9 - stamp_s) * 1000.0
            if not np.isfinite([stamp_s, lidar_stamp_ns, lag_ms]).all():
                raise ValueError(f'{frame_id}: nonfinite timestamps')
            if abs(lag_ms) > args.max_lag_ms:
                rejected_lag += 1
                continue
            images = []
            for side, maps in (('left', maps0), ('right', maps1)):
                path = scene / frame_id / 'nir' / (side + '_distorted.png')
                image_data = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
                if image_data is None or image_data.shape != (height, width):
                    raise ValueError(f'{frame_id}: missing or invalid {side} NIR image')
                images.append(cv2.remap(image_data, *maps, cv2.INTER_LINEAR,
                                        borderMode=cv2.BORDER_CONSTANT, borderValue=0))
            ref, classes, stats = sampled_reference(
                lidar['points'][:], extrinsic, r0, float(p0[0, 0]), float(p0[0, 2]),
                float(p0[1, 2]), baseline, maps0, maps1, args.stride, args.pixel_tolerance_px)
            target = args.output_dir / (frame_id + '.npz')
            with target.open('xb') as stream:
                np.savez_compressed(stream, left=images[0], right=images[1],
                                    reference_depth_m=ref, reference_class=classes,
                                    focal_px=np.float64(p0[0, 0]), principal_x_px=np.float64(p0[0, 2]),
                                    principal_y_px=np.float64(p0[1, 2]), baseline_m=np.float64(baseline),
                                    timestamp_s=np.float64(stamp_s), frame_id=np.array(frame_id))
            records.append({'frame_id': frame_id, 'timestamp_s': stamp_s,
                            'lidar_timestamp_ns': lidar_stamp_ns, 'lidar_minus_image_ms': lag_ms,
                            'nir_exposure_left_recorded': float(image.attrs['nir_exposure_left']),
                            'nir_exposure_right_recorded': float(image.attrs['nir_exposure_right']), **stats})
    if not records:
        raise ValueError('no image/lidar pairs meet the timestamp limit')
    report = {'dataset': 'Pixel-aligned RGB-NIR Stereo Imaging and Dataset for Robot Vision, CVPR 2025',
              'source': 'https://huggingface.co/datasets/DivisonOfficer/Pixel-aligned_RGB-NIR_stereo_dataset',
              'scene': scene.name, 'max_lag_ms': args.max_lag_ms, 'stride': args.stride,
              'pixel_tolerance_px': args.pixel_tolerance_px, 'image_size': [width, height],
              'rectified_focal_px': float(p0[0, 0]), 'rectified_baseline_m': baseline,
              'rectified_principal_px': [float(p0[0, 2]), float(p0[1, 2])],
              'calibration_sha256': file_sha256(calibration_path),
              'camera_to_lidar_sha256': file_sha256(extrinsic_path),
              'frames_selected': len(records), 'frames_rejected_timestamp': rejected_lag,
              'limitations': 'Sparse lidar projection does not establish stereo visibility or full scene coverage. Camera and lidar scan stamps are compared, but individual lidar point times and motion deskew are unavailable. Recorded exposure values are reported in source units.',
              'frames': records}
    (args.output_dir / 'metadata.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(f"{scene.name}: {len(records)} frames selected, {rejected_lag} outside timestamp limit")


if __name__ == '__main__':
    main()
