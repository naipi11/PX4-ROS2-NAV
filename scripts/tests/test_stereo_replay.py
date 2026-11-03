"""Behavioral tests for recovering sampled pixels from stereo-node clouds."""
import importlib.util
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPT = Path(__file__).resolve().parents[2] / 'tools/stereo/replay_depth.py'
spec = importlib.util.spec_from_file_location('replay_depth', SCRIPT)
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def cloud(rows, big=False):
    fields = [SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
              for name, offset in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
    data = bytearray(len(rows) * 20)
    for index, point in enumerate(rows):
        struct.pack_into(('>' if big else '<') + 'ffff', data, index * 20, *point)
    return SimpleNamespace(fields=fields, point_step=20, width=len(rows), height=1,
                           row_step=len(data), data=data, is_bigendian=big)


class StereoReplayTest(unittest.TestCase):
    def test_hit_free_and_coarse_unobservable_are_not_conflated(self):
        for big in (False, True):
            with self.subTest(big_endian=big):
                # With f=8, c=(4,4), these project to (4,4), (0,4), (3,3).
                message = cloud([(2, 0, 0, 1), (2, 1, 0, 0), (2, 0.25, 0.25, 2)], big)
                kind, depth, sampled, stats = replay.cloud_to_samples(message, 12, 12, 8, 4, 4, 4)
                self.assertEqual(kind[4, 4], 1)
                self.assertEqual(kind[4, 0], 2)
                self.assertEqual(float(depth[4, 4]), 2.0)
                self.assertEqual(float(depth[4, 0]), 2.0)
                self.assertEqual(kind[3, 3], 0)
                self.assertFalse(sampled[3, 3])
                self.assertEqual(stats['coarse_unobservable_points'], 1)
                self.assertEqual(stats['unobservable_on_sample_pixels'], 0)

    def test_duplicate_and_off_grid_hits_are_rejected(self):
        duplicate = cloud([(2, 0, 0, 1), (3, 0, 0, 1)])
        with self.assertRaises(ValueError):
            replay.cloud_to_samples(duplicate, 12, 12, 8, 4, 4, 4)
        off_grid = cloud([(2, 0.25, 0, 1)])
        with self.assertRaises(ValueError):
            replay.cloud_to_samples(off_grid, 12, 12, 8, 4, 4, 4)

    def test_nonfinite_and_unknown_evidence_are_rejected(self):
        for point in ((float('nan'), 0, 0, 1), (2, 0, 0, 4), (0, 0, 0, 1)):
            with self.subTest(point=point), self.assertRaises(ValueError):
                replay.cloud_to_samples(cloud([point]), 12, 12, 8, 4, 4, 4)


if __name__ == '__main__':
    unittest.main()
