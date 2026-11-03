"""Behavioral tests for conditional calibration and sparse-reference safety."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[2] / 'tools/stereo/calibrate_visibility.py'
spec = importlib.util.spec_from_file_location('calibrate_visibility', SCRIPT)
calibrate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(calibrate)


class StereoCalibrationTest(unittest.TestCase):
    def test_scene_worst_quantile_and_exact_bound(self):
        result = calibrate.fit_candidate({'first': np.array([1., -1., 1.]),
                                          'second': np.array([2., -2., 2.])}, 1000, 0.2)
        self.assertEqual(result['candidate_disparity_error_px'], 2)
        bound = result['exact_two_sided_depth_bound_m']
        self.assertLess(bound, result['node_linearized_confident_depth_m'])
        self.assertAlmostEqual(bound * bound * 2 / (200 - bound * 2), 0.25)

    def test_sparse_coverage_and_free_through(self):
        frame = {'reference_depth_m': np.array([[2., 2., np.nan, 1., 2.]], dtype=np.float32),
                 'reference_class': np.array([[3, 3, 0, 2, 3]], dtype=np.uint8),
                 'pred_depth_m': np.array([[3., 2.3, 10., 3., np.nan]], dtype=np.float32),
                 'pred_kind': np.array([[1, 2, 1, 1, 0]], dtype=np.uint8),
                 'sampled': np.array([[True, True, True, True, True]]),
                 'focal_px': np.float64(100), 'baseline_m': np.float64(0.1)}
        counts, _, disparity, depth = calibrate.frame_samples(frame)
        self.assertEqual(counts['sparse_reference'], 3)
        self.assertEqual(counts['sampled_without_reference'], 1)
        self.assertEqual(counts['outside_right_view_reference'], 1)
        self.assertEqual((counts['hits'], counts['free_only'], counts['no_return']), (1, 1, 1))
        self.assertEqual((counts['hit_overreach'], counts['free_only_overreach']), (1, 1))
        self.assertAlmostEqual(float(depth[0]), 1.)
        self.assertAlmostEqual(float(disparity[0]), 10.0 * (1/3 - 1/2))

    def test_fit_requires_real_calibration_samples(self):
        with self.assertRaises(ValueError):
            calibrate.fit_candidate({'empty': np.array([])}, 100, 0.1)
        with self.assertRaises(ValueError):
            calibrate.fit_candidate({'bad': np.array([np.nan])}, 100, 0.1)
        with self.assertRaises(ValueError):
            calibrate.fit_candidate({'scene': np.array([1.])}, 100, 0)

    def test_holdout_never_changes_the_fitted_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            scenes = []
            for name, split, predicted in (('train', 'calibration', 2.5), ('test', 'holdout', 4.0)):
                paths = []
                for mode, allowed in (('baseline', 0.25), ('measurement', 1000.0)):
                    path = root / name / mode
                    path.mkdir(parents=True)
                    frame = {'reference_depth_m': np.array([[2.]], dtype=np.float32),
                             'reference_class': np.array([[3]], dtype=np.uint8),
                             'pred_depth_m': np.array([[predicted]], dtype=np.float32),
                             'pred_kind': np.array([[1]], dtype=np.uint8),
                             'sampled': np.array([[True]]),
                             'focal_px': np.float64(100), 'baseline_m': np.float64(0.1)}
                    np.savez(path / 'frame.npz', **frame)
                    parameters = {'image_width': 1, 'image_height': 1, 'focal_px': 100.,
                                  'principal_x_px': 0., 'principal_y_px': 0., 'baseline_m': 0.1,
                                  'pixel_stride': 4, 'disparity_error_px': 0.45,
                                  'allowed_depth_error_m': allowed}
                    metadata = {'temporal_mode': 'independent_frames', 'parameters': parameters,
                                'frames': [{'input': 'frame.npz', 'input_sha256': 'same',
                                            'output': 'frame.npz', 'surface_hits': 1, 'free_only_rays': 0}]}
                    (path / 'metadata.json').write_text(json.dumps(metadata), encoding='utf-8')
                    paths.append(str(path))
                scenes.append({'name': name, 'split': split,
                               'baseline_dir': paths[0], 'measurement_dir': paths[1]})
            report = calibrate.analyze({'scenes': scenes})
            self.assertAlmostEqual(report['calibration']['candidate_disparity_error_px'], 1.0)
            self.assertEqual(report['heldout_exceedances']['test']['exceeds_candidate'], 1)


if __name__ == '__main__':
    unittest.main()
