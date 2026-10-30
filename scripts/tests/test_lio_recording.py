"""Behavioral tests for real lidar record conversion and evaluation."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[2]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


CONVERTER = module("convert_ntu_viral", ROOT / "tools/lio/convert_ntu_viral.py")
COMPARISON = module("compare_prism", ROOT / "tools/lio/compare_prism.py")


def write_csv(path, names, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(names)
        writer.writerows(rows)


class LidarRecordingTest(unittest.TestCase):
    def test_physical_returns_respect_layout_endianness_and_mount(self):
        fields = [SimpleNamespace(name=n, offset=o, datatype=t, count=1)
                  for n, o, t in (("x", 0, 7), ("y", 4, 7), ("z", 8, 7),
                                  ("t", 12, 6), ("range", 16, 6))]
        transform = np.eye(4)
        transform[:3, 3] = [1, -2, 3]
        for big in (False, True):
            with self.subTest(big_endian=big):
                data = bytearray(2 * 52)
                for offset, point in zip((0, 24, 52, 76),
                                         ((1, 2, 3, 10, 100), (0, 0, 0, 20, 0),
                                          (-2, 1, 4, 30, 200), (math.nan, 0, 1, 40, 100))):
                    struct.pack_into((">" if big else "<") + "fffII", data, offset, *point)
                cloud = SimpleNamespace(fields=fields, width=2, height=2, point_step=24,
                                        row_step=52, data=data, is_bigendian=big)
                points, invalid, times = CONVERTER.decode_cloud(cloud, transform)
                np.testing.assert_allclose(points, [[2, 0, 6], [-1, -1, 7]])
                self.assertEqual(invalid, 2)
                self.assertEqual(times, (10, 30))

    def test_prism_lever_arm_and_rigid_alignment_use_no_reference_attitude(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            estimates, reference, metadata = root / "estimate.csv", root / "prism.csv", root / "meta.json"
            positions = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (2, 1, 1)]
            q = math.sqrt(0.5)
            write_csv(estimates, ["stamp_s", "px", "py", "pz", "qw", "qx", "qy", "qz", "healthy", "initial_scan"],
                      [[i, *p, q, 0, 0, q, int(i != 2), int(i == 0)] for i, p in enumerate(positions)])
            # Body lever (1,0,0) rotated by the estimate is (0,1,0).
            # Reference frame is a 90-degree world rotation plus (5,6,7).
            world = [(5 - (p[1] + 1), 6 + p[0], 7 + p[2]) for p in positions]
            write_csv(reference, ["stamp_ns", "x", "y", "z"],
                      [[i * 1_000_000_000, *p] for i, p in enumerate(world)])
            metadata.write_text(json.dumps({"body_to_prism_translation_m": [1, 0, 0],
                                             "scan_timing": "rigid"}), encoding="utf-8")
            report, _ = COMPARISON.compare(estimates, reference, metadata, 1.1, 2.0)
            self.assertEqual(report["paired_samples"], 4)
            self.assertAlmostEqual(report["healthy_fraction_all_estimates"], 0.75)
            self.assertTrue(report["initial_scan_included"])
            self.assertLess(report["all_overlap"]["maximum_m"], 1e-8)
            self.assertIsNotNone(report["prefix_fit_holdout"])
            self.assertLess(report["prefix_fit_holdout"]["held_out"]["maximum_m"], 1e-8)

    def test_reference_gaps_are_not_interpolated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            estimates, reference, metadata = root / "estimate.csv", root / "prism.csv", root / "meta.json"
            positions = [(0, 0, 0), (1, 0, 0), (1, 1, 0), (2, 1, 1)]
            write_csv(estimates, ["stamp_s", "px", "py", "pz", "qw", "qx", "qy", "qz", "healthy"],
                      [[i, *p, 1, 0, 0, 0, 1] for i, p in enumerate(positions)])
            write_csv(reference, ["stamp_ns", "x", "y", "z"],
                      [[0, 0, 0, 0], [1_000_000_000, 1, 0, 0],
                       [2_000_000_000, 1, 1, 0], [4_000_000_000, 3, 1, 1]])
            metadata.write_text(json.dumps({"body_to_prism_translation_m": [0, 0, 0],
                                             "scan_timing": "rigid"}), encoding="utf-8")
            report, _ = COMPARISON.compare(estimates, reference, metadata, 1.1, 1.0)
            self.assertEqual(report["reference_gaps"], 1)
            self.assertEqual(report["paired_samples"], 3)
            self.assertIsNone(report["prefix_fit_holdout"])


if __name__ == "__main__":
    unittest.main()
