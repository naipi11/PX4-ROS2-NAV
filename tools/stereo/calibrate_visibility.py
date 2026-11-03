#!/usr/bin/env python3
"""Fit an empirical disparity-error candidate and evaluate held-out scenes.

This is descriptive offline analysis, not a guaranteed camera range. Sparse
lidar coverage and stereo mutual visibility are not established by this tool.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


BANDS = (0.0, 0.3, 1.0, 2.0, 4.0, 6.0, 8.0, 12.0, float('inf'))


def quantiles(values):
    values = np.asarray(values, dtype=np.float64)
    if not values.size:
        return {'median': None, 'p95': None, 'p99': None, 'max': None}
    if not np.isfinite(values).all():
        raise ValueError('nonfinite measurement')
    q = np.quantile(values, (0.5, 0.95, 0.99))
    return {'median': float(q[0]), 'p95': float(q[1]), 'p99': float(q[2]),
            'max': float(np.max(values))}


def frame_samples(frame, threshold_m=0.25):
    ref = np.asarray(frame['reference_depth_m'])
    classes = np.asarray(frame['reference_class'])
    pred = np.asarray(frame['pred_depth_m'])
    kind = np.asarray(frame['pred_kind'])
    sampled = np.asarray(frame['sampled'])
    if (ref.ndim != 2 or any(x.shape != ref.shape for x in (classes, pred, kind, sampled))
            or sampled.dtype != np.bool_ or classes.dtype != np.uint8 or kind.dtype != np.uint8
            or np.any(classes > 3) or np.any(kind > 3)
            or not math.isfinite(threshold_m) or threshold_m < 0):
        raise ValueError('invalid reference or prediction arrays')
    focal, baseline = float(frame['focal_px']), float(frame['baseline_m'])
    if not math.isfinite(focal * baseline) or focal <= 0 or baseline <= 0:
        raise ValueError('invalid camera geometry')
    known = sampled & (classes == 3)
    if np.any(~np.isfinite(ref[known])) or np.any(ref[known] <= 0):
        raise ValueError('invalid sparse reference depth')
    endpoints = known & ((kind == 1) | (kind == 2))
    if np.any(~np.isfinite(pred[endpoints])) or np.any(pred[endpoints] <= 0):
        raise ValueError('invalid predicted endpoint')
    hits = known & (kind == 1)
    free = known & (kind == 2)
    ref_hit = ref[hits].astype(np.float64)
    pred_hit = pred[hits].astype(np.float64)
    signed = focal * baseline * (1.0 / pred_hit - 1.0 / ref_hit)
    overreach = np.zeros(ref.shape, dtype=bool)
    overreach[endpoints] = pred[endpoints].astype(np.float64) > ref[endpoints].astype(np.float64) + threshold_m
    band_rows = []
    for low, high in zip(BANDS, BANDS[1:]):
        mask = known & (ref >= low) & (ref < high)
        band_rows.append({'min_m': low, 'max_m': None if math.isinf(high) else high,
                          'references': int(np.count_nonzero(mask)),
                          'hits': int(np.count_nonzero(mask & (kind == 1))),
                          'free_only': int(np.count_nonzero(mask & (kind == 2))),
                          'no_return': int(np.count_nonzero(mask & (kind == 0))),
                          'unobservable': int(np.count_nonzero(mask & (kind == 3))),
                          'overreach': int(np.count_nonzero(mask & overreach))})
    counts = {'sampled': int(np.count_nonzero(sampled)),
              'sampled_without_reference': int(np.count_nonzero(sampled & (classes == 0))),
              'sparse_reference': int(np.count_nonzero(known)),
              'outside_right_view_reference': int(np.count_nonzero(sampled & (classes == 2))),
              'hits': int(np.count_nonzero(hits)), 'free_only': int(np.count_nonzero(free)),
              'no_return': int(np.count_nonzero(known & (kind == 0))),
              'unobservable': int(np.count_nonzero(known & (kind == 3))),
              'hit_overreach': int(np.count_nonzero(hits & overreach)),
              'free_only_overreach': int(np.count_nonzero(free & overreach))}
    return counts, band_rows, signed, pred_hit - ref_hit


def fit_candidate(training_errors, focal_px, baseline_m, allowed_depth_error_m=0.25, percentile=0.99):
    if (not training_errors or not all(map(math.isfinite, (focal_px, baseline_m, allowed_depth_error_m, percentile)))
            or focal_px <= 0 or baseline_m <= 0 or allowed_depth_error_m <= 0
            or not 0 < percentile < 1):
        raise ValueError('invalid calibration inputs')
    per_scene = {}
    for name, values in training_errors.items():
        absolute = np.abs(np.asarray(values, dtype=np.float64))
        if absolute.size == 0 or not np.isfinite(absolute).all():
            raise ValueError(f'{name}: no finite calibration hits')
        per_scene[name] = float(np.quantile(absolute, percentile))
    error = max(per_scene.values())
    fb = focal_px * baseline_m
    if error <= 0:
        linearized = exact = None
    else:
        linearized = math.sqrt(allowed_depth_error_m * fb / error)
        # Exact two-sided depth bound for |disparity error| <= error, before
        # considering detection coverage or lidar-reference uncertainty.
        exact = (math.sqrt(allowed_depth_error_m ** 2 + 4 * allowed_depth_error_m * fb / error)
                 - allowed_depth_error_m) / 2.0
    return {'calibration_percentile': float(percentile),
            'per_scene_absolute_disparity_quantile_px': per_scene,
            'candidate_disparity_error_px': error,
            'allowed_depth_error_m': allowed_depth_error_m,
            'node_linearized_confident_depth_m': linearized,
            'exact_two_sided_depth_bound_m': exact,
            'status': 'conditional empirical candidate, not a detection or safety guarantee'}


def collect_scene(directory):
    metadata = json.loads((directory / 'metadata.json').read_text(encoding='utf-8'))
    if metadata['temporal_mode'] != 'independent_frames':
        raise ValueError(f'{directory}: expected independently replayed frames')
    records = metadata['frames']
    if not records:
        raise ValueError(f'{directory}: no replayed frames')
    totals = None
    bands = None
    disparity = []
    depth_error = []
    blind_frames = 0
    for row in records:
        path = directory / row['output']
        with np.load(path, allow_pickle=False) as frame:
            counts, frame_bands, d, z = frame_samples(frame)
        if totals is None:
            totals = {key: 0 for key in counts}
            bands = [{**entry, **{k: 0 for k in ('references', 'hits', 'free_only', 'no_return', 'unobservable', 'overreach')}}
                     for entry in frame_bands]
        for key, value in counts.items():
            totals[key] += value
        for target, current in zip(bands, frame_bands):
            for key in ('references', 'hits', 'free_only', 'no_return', 'unobservable', 'overreach'):
                target[key] += current[key]
        disparity.append(d)
        depth_error.append(z)
        blind_frames += int(row['surface_hits'] == 0 and row['free_only_rays'] == 0)
    all_disparities = np.concatenate(disparity)
    all_depth_errors = np.concatenate(depth_error)
    result = {'frames': len(records), 'zero_stereo_return_frames': blind_frames, **totals,
              'signed_disparity_error_px': quantiles(all_disparities),
              'absolute_disparity_error_px': quantiles(np.abs(all_disparities)),
              'signed_depth_error_m': quantiles(all_depth_errors),
              'absolute_depth_error_m': quantiles(np.abs(all_depth_errors)),
              'depth_bands': bands}
    return metadata, result, all_disparities


def analyze(manifest):
    scenes = manifest['scenes']
    names = [scene['name'] for scene in scenes]
    if len(set(names)) != len(names):
        raise ValueError('scene IDs must be unique across calibration and holdout')
    if {scene['split'] for scene in scenes} != {'calibration', 'holdout'}:
        raise ValueError('both calibration and holdout scenes are required')
    report, training, holdout = {}, {}, {}
    geometry = None
    for scene in scenes:
        name = scene['name']
        baseline_meta, baseline_summary, _ = collect_scene(Path(scene['baseline_dir']))
        measurement_meta, measurement_summary, errors = collect_scene(Path(scene['measurement_dir']))
        if baseline_meta['parameters']['allowed_depth_error_m'] != 0.25:
            raise ValueError(f'{name}: baseline confidence is not the configured 0.25 m')
        if measurement_meta['parameters']['allowed_depth_error_m'] <= 0.25:
            raise ValueError(f'{name}: measurement replay was not relaxed')
        keys = ('image_width', 'image_height', 'focal_px', 'principal_x_px', 'principal_y_px',
                'baseline_m', 'pixel_stride', 'disparity_error_px')
        baseline_config = tuple(baseline_meta['parameters'][k] for k in keys)
        measurement_config = tuple(measurement_meta['parameters'][k] for k in keys)
        if baseline_config != measurement_config or (geometry is not None and baseline_config != geometry):
            raise ValueError('camera configuration differs between scenes or replay modes')
        geometry = baseline_config
        base_inputs = [(r['input'], r['input_sha256']) for r in baseline_meta['frames']]
        measure_inputs = [(r['input'], r['input_sha256']) for r in measurement_meta['frames']]
        if base_inputs != measure_inputs:
            raise ValueError(f'{name}: baseline and measurement frames differ')
        report[name] = {'split': scene['split'], 'baseline': baseline_summary,
                        'measurement_only_relaxed_confidence': measurement_summary}
        if scene['split'] == 'calibration':
            training[name] = errors
        else:
            holdout[name] = errors
    fitted = fit_candidate(training, geometry[2], geometry[5])
    candidate = fitted['candidate_disparity_error_px']
    heldout = {}
    for name, values in holdout.items():
        absolute = np.abs(values)
        heldout[name] = {'matched_hits': int(absolute.size),
                         'exceeds_candidate': int(np.count_nonzero(absolute > candidate)),
                         'exceeds_candidate_fraction': float(np.mean(absolute > candidate)) if absolute.size else None}
    return {'scope': 'Descriptive, conditional sparse-lidar calibration; class 3 mutual visibility is unverified. Pixels without a reference or return are not evidence of safety. Relaxed confidence is measurement-only. Neither range nor simulation braking assumptions are a real-vehicle guarantee.',
            'camera': {'width': geometry[0], 'height': geometry[1], 'focal_px': geometry[2],
                       'principal_x_px': geometry[3], 'principal_y_px': geometry[4],
                       'baseline_m': geometry[5], 'stride': geometry[6]},
            'calibration': fitted, 'heldout_exceedances': heldout, 'scenes': report}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding='utf-8'))
    result = analyze(manifest)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'candidate': result['calibration'], 'heldout': result['heldout_exceedances']}, indent=2))


if __name__ == '__main__':
    main()
