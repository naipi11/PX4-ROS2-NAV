"""Behavioral tests for sparse synchronized stereo reference geometry."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[2] / 'tools/stereo/prepare_rgbnir.py'
spec = importlib.util.spec_from_file_location('prepare_rgbnir', SCRIPT)
prepare = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare)


def maps(size=12):
    x, y = np.meshgrid(np.arange(size, dtype=np.float32), np.arange(size, dtype=np.float32))
    return x, y


def transform():
    result = np.eye(4)
    result[0, 3] = 1000.0  # camera origin is one metre along the lidar x axis
    return result


class SynchronizedProjectionTest(unittest.TestCase):
    def project(self, points, left=None, right=None):
        return prepare.sampled_reference(np.asarray(points), transform(), np.eye(3),
                                         8.0, 4.0, 4.0, 0.5,
                                         left or maps(), right or maps(), 4, 0.5)

    def test_metric_extrinsic_and_right_view(self):
        # The first point becomes camera (0,0,2); the second camera (-0.5,0,1).
        depth, classes, stats = self.project([[1, 0, 2], [0.5, 0, 1], [1.2, 0, 2],
                                               [np.nan, 0, 1], [0, 0, 0]])
        self.assertEqual(classes[4, 4], 3)  # sparse lidar, visibility not established
        self.assertAlmostEqual(float(depth[4, 4]), 2.0)
        self.assertEqual(classes[4, 0], 2)  # correspondence lies outside right image
        self.assertAlmostEqual(float(depth[4, 0]), 1.0)
        self.assertEqual(stats['sparse_visibility_unverified_pixels'], 1)
        self.assertEqual(stats['outside_right_view_pixels'], 1)
        self.assertEqual(np.count_nonzero(classes), 2)  # the 0.8-pixel offset is not sampled

    def test_conflicting_depths_are_unknown(self):
        depth, classes, stats = self.project([[1, 0, 2], [1, 0, 3]])
        self.assertEqual(classes[4, 4], 0)
        self.assertTrue(np.isnan(depth[4, 4]))
        self.assertEqual(stats['conflicting_sample_pixels'], 1)

    def test_missing_rectification_support_is_not_reference(self):
        left = list(maps())
        left[0][4, 4] = -1
        depth, classes, _ = self.project([[1, 0, 2]], left=tuple(left))
        self.assertEqual(classes[4, 4], 0)
        self.assertTrue(np.isnan(depth[4, 4]))
        right = list(maps())
        right[0][4, 2] = -1
        depth, classes, _ = self.project([[1, 0, 2]], right=tuple(right))
        self.assertEqual(classes[4, 4], 2)
        self.assertAlmostEqual(float(depth[4, 4]), 2.0)

    def test_invalid_transform_and_projection_parameters_are_rejected(self):
        reflection = transform()
        reflection[0, 0] = -1
        with self.assertRaises(ValueError):
            prepare.sampled_reference(np.array([[1, 0, 2]]), reflection, np.eye(3),
                                      8, 4, 4, 0.5, maps(), maps())
        with self.assertRaises(ValueError):
            prepare.sampled_reference(np.array([[1, 0, 2]]), transform(), np.eye(3),
                                      8, 4, 4, 0.5, maps(), maps(), 4, 0.6)

    def test_rectification_uses_metric_baseline(self):
        k = np.array([[100., 0, 20.], [0, 100., 10.], [0, 0, 1.]])
        cal = {'mtx_left': k, 'mtx_right': k, 'dist_left': np.zeros(5),
               'dist_right': np.zeros(5), 'R': np.eye(3), 'T': np.array([-120., 0, 0]),
               'image_size': np.array([40, 20])}
        size, _, p0, baseline, _, _ = prepare.rectification(cal)
        self.assertEqual(size, (40, 20))
        self.assertAlmostEqual(float(p0[0, 0]), 100.)
        self.assertAlmostEqual(baseline, 0.12)


if __name__ == '__main__':
    unittest.main()
