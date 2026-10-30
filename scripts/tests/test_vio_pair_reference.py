"""Behavioral tests for pairing estimator and reference trajectories."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / "tools/vio/pair_reference.py"


def write_rows(path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(fields)
        writer.writerows(rows)


class PairReferenceTest(unittest.TestCase):
    def test_interpolation_drops_uncovered_poses_without_extrapolation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            estimates = root / "estimate.csv"
            reference = root / "reference.csv"
            output = root / "paired.csv"
            summary = root / "summary.json"
            write_rows(estimates, ["stamp_s", "px", "py", "pz", "qw", "qx", "qy", "qz", "healthy"],
                       [[t, 0, 0, 0, 1, 0, 0, 0, 1] for t in (-0.5, 0.5, 1, 5, 10.5)])
            q = math.sqrt(0.5)
            write_rows(reference, ["stamp_s", "x", "y", "z", "qw", "qx", "qy", "qz"],
                       [[0, 0, 0, 0, 1, 0, 0, 0], [1, 2, 0, 0, q, 0, 0, q],
                        [10, 3, 0, 0, q, 0, 0, q]])
            result = subprocess.run([sys.executable, str(SCRIPT), str(estimates), str(reference),
                                     "--output", str(output), "--summary", str(summary),
                                     "--max-reference-gap-s", "2"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            stats = json.loads(summary.read_text(encoding="utf-8"))
            self.assertEqual((stats["selected_estimates"], stats["paired_estimates"],
                              stats["outside_reference"], stats["reference_gaps"]), (5, 2, 2, 1))
            with output.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertAlmostEqual(float(rows[0]["tx"]), 1)
            self.assertAlmostEqual(float(rows[0]["tqw"]), math.cos(math.pi / 8))
            self.assertAlmostEqual(float(rows[0]["tqz"]), math.sin(math.pi / 8))
            self.assertEqual(rows[0]["healthy"], "1")
            self.assertEqual(float(rows[1]["tx"]), 2)

    def test_rejects_out_of_order_or_nonunit_reference(self):
        for bad in ([[0, 0, 0, 0, 1, 0, 0, 0], [0, 1, 0, 0, 1, 0, 0, 0]],
                    [[0, 0, 0, 0, 1, 0, 0, 0], [1, 1, 0, 0, 2, 0, 0, 0]]):
            with self.subTest(bad=bad), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                estimate, reference = root / "estimate.csv", root / "reference.csv"
                write_rows(estimate, ["stamp_s", "px", "py", "pz", "qw", "qx", "qy", "qz"],
                           [[0, 0, 0, 0, 1, 0, 0, 0], [1, 0, 0, 0, 1, 0, 0, 0]])
                write_rows(reference, ["stamp_s", "x", "y", "z", "qw", "qx", "qy", "qz"], bad)
                output = root / "out.csv"
                result = subprocess.run([sys.executable, str(SCRIPT), str(estimate), str(reference),
                                         "--output", str(output), "--summary", str(root / "stats.json"),
                                         "--max-reference-gap-s", "2"], capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
